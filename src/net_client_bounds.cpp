/*
 * net_client_bounds.cpp — keep one dead client from freezing the bridge.
 *
 * Why this exists
 * ---------------
 * VRPN writes to each client with a plain blocking send() from the bridge's
 * one thread (vrpn_Endpoint_IP::send_pending_reports). When a client vanishes
 * without a FIN or RST reaching us — the ESP32 dropping off Wi-Fi, or
 * abandoning a connect whose SYN-ACK came back late over a congested link —
 * the connection stays ESTABLISHED here, its send buffer fills at 200 Hz, and
 * then send() blocks until the kernel gives up on the peer: tcp_retries2 = 15,
 * i.e. ~15 minutes. For that whole time the bridge publishes nothing to
 * ANYONE and never calls accept(), so the board's reconnects pile up in a
 * listen backlog of 1 (seen 2026-09-28: Recv-Q 2 on the listener, 70 kB
 * unacked on the dead endpoint, board stuck in "connecting").
 *
 * Every socket VRPN accepts therefore gets:
 *   TCP_USER_TIMEOUT  data unacked this long aborts the connection, so a dead
 *                     peer is dropped in ~2 s instead of ~15 min;
 *   SO_SNDTIMEO       send() never blocks the loop longer than this; VRPN
 *                     treats the failure as BROKEN and drops that endpoint
 *                     alone — the client reconnects, the others never notice;
 *   TCP_NODELAY       a pose is ~100 bytes and should leave now, not when
 *                     Nagle decides.
 * and listen() gets a real backlog, so a burst of reconnects is queued rather
 * than refused while the loop is busy.
 *
 * libvrpn is a static archive; `-Wl,--wrap=accept,--wrap=listen` (see
 * CMakeLists.txt) routes VRPN's own calls here without patching it, the same
 * way net_reuseaddr.cpp handles bind().
 */
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

namespace {

constexpr unsigned kUserTimeoutMs = 2000;  // > 10x a Wi-Fi hiccup, << 15 min
constexpr long     kSendTimeoutUs = 4000;  // under one tick at 200 Hz: a client
                                           // whose buffer is full is already
                                           // ~1 s behind, so waiting on it only
                                           // costs everyone else poses
constexpr int      kMinBacklog    = 16;

void bound_client(int fd)
{
    const unsigned uto = kUserTimeoutMs;
    ::setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &uto, sizeof(uto));
    const timeval tv{0, kSendTimeoutUs};
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    const int on = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
}

} // namespace

extern "C" {

int __real_accept(int fd, struct sockaddr *addr, socklen_t *len);
int __real_listen(int fd, int backlog);

int __wrap_accept(int fd, struct sockaddr *addr, socklen_t *len)
{
    const int c = __real_accept(fd, addr, len);
    if (c >= 0) bound_client(c);
    return c;
}

int __wrap_listen(int fd, int backlog)
{
    return __real_listen(fd, backlog < kMinBacklog ? kMinBacklog : backlog);
}

} // extern "C"
