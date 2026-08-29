/*
 * net_reuseaddr.cpp — set SO_REUSEADDR on every socket VRPN binds.
 *
 * Why this exists
 * ---------------
 * vrpn_Connection_IP's server constructor binds <NIC>:<port> for UDP and TCP
 * through its private open_socket(), which only sets SO_REUSEADDR on Android.
 * On Linux the TCP bind therefore fails with EADDRINUSE for the ~60 s that a
 * just-closed client connection spends in TIME-WAIT — exactly the window you
 * are in when you restart vicon_bridge while the ESP32 (or any VRPN client)
 * still had a session open. VRPN then leaves the connection in its BROKEN
 * state, vrpn_create_server_connection() still hands it back, and every
 * publish afterwards prints
 *
 *     vrpn_Connection::pack_message: Can't pack because the connection is broken
 *
 * forever, with no pose ever reaching a client.
 *
 * libvrpn is linked statically, so `-Wl,--wrap=bind` (added in CMakeLists.txt)
 * redirects VRPN's own bind() calls here without patching or rebuilding VRPN.
 * SO_REUSEADDR lets the listener bind over a TIME-WAIT leftover; it does NOT
 * let two live servers share the port, so a second vicon_bridge still fails
 * to bind and is reported as such by BridgeVicon::openLink().
 *
 * Note both sides of a restart need the option: Linux only ignores a
 * TIME-WAIT conflict when the socket that left it behind also had
 * SO_REUSEADDR set. The first restart after adopting this fix can still hit
 * one TIME-WAIT window, which BridgeVicon rides out by retrying the bind.
 */
#include <sys/socket.h>
#include <sys/types.h>

extern "C" {

int __real_bind(int fd, const struct sockaddr *addr, socklen_t len);

int __wrap_bind(int fd, const struct sockaddr *addr, socklen_t len)
{
    const int on = 1;
    /* Best-effort: a socket type that rejects the option still binds. */
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    return __real_bind(fd, addr, len);
}

} // extern "C"
