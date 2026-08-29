#include "BridgeVicon.h"
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>
#include <thread>
#include <map>
#include <set>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>

namespace {

using Clock = std::chrono::steady_clock;

/* Backoff between attempts to bind an output port that is still busy.
 * A TIME-WAIT leftover clears in ~60 s worst case; poll often enough that
 * the bridge comes back on its own the moment the port frees. */
constexpr double kRetryFirstSec = 0.5;
constexpr double kRetryMaxSec   = 5.0;

/* How long an established link may report "not okay" before it is torn down
 * and rebuilt. VRPN drops broken *endpoints* inside its own mainloop, so a
 * single bad reading is normal churn (a client that went away); a link that
 * stays down is the listening connection itself. Timed rather than counted
 * in ticks so it means the same thing at any output_frequency. */
constexpr double kDownSecBeforeRebuild = 1.0;

/* Split "Object@host" / "host:port" the way VRPN's own helpers do. */
std::string hostOf(const std::string& output_object)
{
    const size_t at = output_object.find('@');
    return at == std::string::npos ? output_object : output_object.substr(at + 1);
}

std::string nameOf(const std::string& output_object)
{
    const size_t at = output_object.find('@');
    return at == std::string::npos ? output_object : output_object.substr(0, at);
}

/* Explain why VRPN could not take the port, so the log says something more
 * useful than "connection is broken". The two causes look identical to
 * bind() (both EADDRINUSE), so they are told apart by connecting: a live
 * server accepts, a port held only by TIME-WAIT refuses. */
std::string diagnoseBindFailure(const std::string& host, int port)
{
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(port));
    a.sin_addr.s_addr = host.empty() ? INADDR_ANY : ::inet_addr(host.c_str());
    if (a.sin_addr.s_addr == INADDR_NONE)
        return "'" + host + "' is not an address on this machine";

    const int probe = ::socket(AF_INET, SOCK_STREAM, 0);
    if (probe < 0) return "cannot probe the port: " + std::string(std::strerror(errno));

    bool someone_listening = false;
    if (::connect(probe, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0)
        someone_listening = true;
    ::close(probe);

    if (someone_listening)
        return "another program (a second vicon_bridge?) is already serving "
               + host + ":" + std::to_string(port);

    /* Nothing is accepting, so the port is held by leftovers rather than a
     * live server — or the address itself is not up yet. */
    const int check = ::socket(AF_INET, SOCK_STREAM, 0);
    if (check >= 0) {
        const bool bound = ::bind(check, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0;
        const int err = errno;
        ::close(check);
        if (!bound && err == EADDRNOTAVAIL)
            return host + " is not configured on any interface right now";
    }
    return "the port is still held by the previous run's connections "
           "(TIME-WAIT, because a client was connected when it exited); "
           "it frees itself within ~60 s";
}

} // namespace

OutputViconTracker::OutputViconTracker(const std::string& name, vrpn_Connection* c)
    : vrpn_Tracker(name.c_str(), c) {
    current_pos[0] = current_pos[1] = current_pos[2] = 0.0;
    current_quat[0] = current_quat[1] = current_quat[2] = 0.0;
    current_quat[3] = 1.0; // Identity quaternion
}

void OutputViconTracker::updatePose(double x, double y, double z, double qx, double qy, double qz, double qw) {
    current_pos[0] = x;
    current_pos[1] = y;
    current_pos[2] = z;
    current_quat[0] = qx;
    current_quat[1] = qy;
    current_quat[2] = qz;
    current_quat[3] = qw;
}

bool OutputViconTracker::send_sim_pose() {
    struct timeval timestamp;
    vrpn_gettimeofday(&timestamp, NULL);

    pos[0] = current_pos[0];
    pos[1] = current_pos[1];
    pos[2] = current_pos[2];
    d_quat[0] = current_quat[0];
    d_quat[1] = current_quat[1];
    d_quat[2] = current_quat[2];
    d_quat[3] = current_quat[3];

    char msgbuf[1000];
    int len = encode_to(msgbuf);
    if (len <= 0) {
        std::cerr << "BridgeVicon: failed to encode tracker message\n";
        return false;
    }
    /* A failure here means the connection is broken; the caller decides
     * whether to rebuild it. Deliberately silent — BridgeVicon reports it
     * once per outage instead of once per 200 Hz tick. */
    return d_connection->pack_message(len, timestamp, position_m_id,
                                      d_sender_id, msgbuf,
                                      vrpn_CONNECTION_RELIABLE) >= 0;
}

bool OutputViconTracker::publish_pose() {
    const bool ok = send_sim_pose();
    server_mainloop();
    return ok;
}

void OutputViconTracker::mainloop() {
    publish_pose();
}

BridgeVicon::BridgeVicon(const BridgeConfig& config)
    : config(config), rng(std::random_device{}()),
      pos_noise_dist(0.0, config.pos_noise_stddev),
      att_noise_dist(0.0, config.att_noise_stddev) {

    // Validate rotation inputs
    if (config.rotation_preference != "quaternion" && config.rotation_preference != "matrix") {
        std::cerr << "Warning: Invalid rotation_preference '" << config.rotation_preference
                  << "'. Using 'quaternion'.\n";
        this->config.rotation_preference = "quaternion";
    }

    if (config.enable_rotation) {
        if (config.rotation_preference == "quaternion") {
            Eigen::Quaterniond q(config.quat_offset[3], config.quat_offset[0], config.quat_offset[1], config.quat_offset[2]);
            if (std::abs(q.norm() - 1.0) > 1e-6) {
                std::cerr << "Warning: Quaternion not unit length, normalizing.\n";
                q.normalize();
                this->config.quat_offset[0] = q.x();
                this->config.quat_offset[1] = q.y();
                this->config.quat_offset[2] = q.z();
                this->config.quat_offset[3] = q.w();
            }
        } else {
            Eigen::Matrix3d R;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R(i, j) = config.rot_matrix_offset[i * 3 + j];
            if (!R.isUnitary(1e-6)) {
                std::cerr << "Warning: Rotation matrix not unitary, using identity.\n";
                this->config.rot_matrix_offset[0] = 1.0; this->config.rot_matrix_offset[1] = 0.0; this->config.rot_matrix_offset[2] = 0.0;
                this->config.rot_matrix_offset[3] = 0.0; this->config.rot_matrix_offset[4] = 1.0; this->config.rot_matrix_offset[5] = 0.0;
                this->config.rot_matrix_offset[6] = 0.0; this->config.rot_matrix_offset[7] = 0.0; this->config.rot_matrix_offset[8] = 1.0;
            }
        }
    }

    const size_t n = config.objects.size();

    // Pre-size before registering callbacks so element pointers remain stable
    callback_data.resize(n);
    states.resize(n);
    output_trackers.assign(n, nullptr);
    link_of_object.assign(n, 0);

    const auto now = Clock::now();
    for (size_t i = 0; i < n; ++i) {
        states[i].last_input_time = now;
        callback_data[i] = {this, i};
    }
    next_tick = now;
    last_status = now;
    last_rates = now;

    // Group the objects by the port they publish on: one VRPN server
    // connection per host:port, however many trackers ride on it.
    std::map<std::string, size_t> link_index;
    for (size_t i = 0; i < n; ++i) {
        const auto& obj = config.objects[i];

        auto* tracker = new vrpn_Tracker_Remote(obj.input_object.c_str());
        tracker->register_change_handler(&callback_data[i], BridgeVicon::callback);
        input_trackers.push_back(tracker);

        const std::string conn_str =
            hostOf(obj.output_object) + ":" + std::to_string(config.vrpn_port);
        auto it = link_index.find(conn_str);
        if (it == link_index.end()) {
            it = link_index.emplace(conn_str, links.size()).first;
            OutputLink link;
            link.connection_str = conn_str;
            link.next_attempt = now;
            links.push_back(std::move(link));
        }
        link_of_object[i] = it->second;
        links[it->second].objects.push_back(i);

        std::cout << "BridgeVicon: Object " << (i + 1) << " — input: " << obj.input_object
                  << ", output: " << obj.output_object << "\n";
    }

    for (auto& link : links) openLink(link);
}

BridgeVicon::~BridgeVicon() {
    // Park every client at the identity pose before the sockets go away.
    for (size_t i = 0; i < output_trackers.size(); ++i)
        if (output_trackers[i]) sendZeroPose(i);
    for (auto& link : links)
        if (link.conn) link.conn->mainloop();

    for (auto* t : input_trackers) delete t;
    input_trackers.clear();
    for (auto& link : links) closeLink(link);
}

/* ------------------------------------------------------------------------
 *  Output links: open, close, and keep alive
 * --------------------------------------------------------------------- */

bool BridgeVicon::openLink(OutputLink& link)
{
    const size_t colon = link.connection_str.rfind(':');
    const std::string host = link.connection_str.substr(0, colon);
    const int port = std::stoi(link.connection_str.substr(colon + 1));

    // VRPN prints its own "can't bind address" pair on every attempt. Show it
    // once per outage and mute the repeats, so a long wait stays readable.
    const bool mute = link.attempts > 0;
    int saved_stderr = -1;
    if (mute) {
        saved_stderr = ::dup(STDERR_FILENO);
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) { ::dup2(devnull, STDERR_FILENO); ::close(devnull); }
    }
    vrpn_Connection* conn = vrpn_create_server_connection(link.connection_str.c_str());
    if (saved_stderr >= 0) {
        ::fflush(stderr);
        ::dup2(saved_stderr, STDERR_FILENO);
        ::close(saved_stderr);
    }

    // vrpn_create_server_connection() hands back a connection even when it
    // could not bind its port; without this check every publish afterwards
    // fails with "Can't pack because the connection is broken" forever.
    if (!conn || !conn->doing_okay()) {
        // vrpn_create_server_connection() hands back a reference and marks the
        // connection auto-delete; dropping the reference is how it is freed
        // (plain delete warns about the outstanding reference).
        if (conn) conn->removeReference();
        link.attempts++;
        if (!link.reported_down) {
            link.reported_down = true;
            link.down_reason = diagnoseBindFailure(host, port);
            std::cerr << "BridgeVicon: cannot serve " << link.connection_str << " — "
                      << link.down_reason << "\n"
                      << "BridgeVicon: retrying every "
                      << kRetryFirstSec << "-" << kRetryMaxSec
                      << " s; poses resume automatically once the port is free.\n";
        }
        const double delay = std::min(kRetryMaxSec, kRetryFirstSec * link.attempts);
        link.next_attempt = Clock::now() +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(delay));
        return false;
    }

    link.conn = conn;
    for (size_t i : link.objects)
        output_trackers[i] = new OutputViconTracker(nameOf(config.objects[i].output_object), conn);

    if (link.attempts || link.reported_down)
        std::cout << "BridgeVicon: " << link.connection_str << " is serving again"
                  << " (after " << link.attempts << " failed attempt"
                  << (link.attempts == 1 ? "" : "s") << ")\n";
    else
        std::cout << "BridgeVicon: serving " << link.connection_str << "\n";

    link.attempts = 0;
    link.down_since = Clock::time_point{};
    link.reported_down = false;
    link.down_reason.clear();
    return true;
}

void BridgeVicon::closeLink(OutputLink& link)
{
    // Trackers hold a pointer to the connection and unregister themselves in
    // their destructor, so they must go first.
    for (size_t i : link.objects) {
        delete output_trackers[i];
        output_trackers[i] = nullptr;
    }
    if (link.conn) link.conn->removeReference();
    link.conn = nullptr;
}

void BridgeVicon::retryLink(OutputLink& link, Clock::time_point now)
{
    if (!link.conn && now >= link.next_attempt) openLink(link);
}

void BridgeVicon::pumpLink(OutputLink& link, Clock::time_point now)
{
    if (!link.conn) return;

    // Flush whatever publish() packed this tick and accept new clients.
    link.conn->mainloop();

    // mainloop() has just dropped any endpoint that broke, so a connection
    // still reporting not-okay is the listening socket itself. Give it a
    // moment anyway — a client disconnecting mid-loop is not a failure.
    if (link.conn->doing_okay()) {
        link.down_since = Clock::time_point{};
        return;
    }
    if (link.down_since == Clock::time_point{}) link.down_since = now;
    if (std::chrono::duration<double>(now - link.down_since).count() < kDownSecBeforeRebuild)
        return;

    std::cerr << "BridgeVicon: " << link.connection_str
              << " went broken — rebuilding the server connection\n";
    closeLink(link);
    link.attempts = 0;
    link.reported_down = false;
    link.down_since = Clock::time_point{};
    link.next_attempt = now;
}

bool BridgeVicon::allLinksUp() const
{
    for (const auto& link : links)
        if (!link.conn) return false;
    return true;
}

/* ------------------------------------------------------------------------ */

void BridgeVicon::callback(void* userdata, const vrpn_TRACKERCB tdata) {
    TrackerCallbackData* cbd = static_cast<TrackerCallbackData*>(userdata);
    BridgeVicon* self = cbd->bridge;
    size_t idx = cbd->index;
    TrackerState& state = self->states[idx];

    Eigen::Vector3d pos(tdata.pos[0], tdata.pos[1], tdata.pos[2]);
    Eigen::Quaterniond quat(tdata.quat[3], tdata.quat[0], tdata.quat[1], tdata.quat[2]); // VRPN: qx, qy, qz, qw

    state.last_input_time = Clock::now();
    state.has_valid_data = true;
    state.input_count++;

    if (self->config.enable_latency) {
        state.pose_queue.push_back({state.last_input_time, pos, quat});
    } else {
        state.position = pos;
        state.quaternion = quat;
        self->applyNoise(state.position, state.quaternion);
        self->sendPose(idx);
    }
}

Eigen::Quaterniond BridgeVicon::eulerToQuaternion(double yaw, double pitch, double roll) {
    yaw = yaw * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    roll = roll * M_PI / 180.0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    return Eigen::Quaterniond(
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr
    );
}

Eigen::Quaterniond BridgeVicon::matrixToQuaternion(const Eigen::Matrix3d& R) {
    double qw = sqrt(1.0 + R(0,0) + R(1,1) + R(2,2)) / 2.0;
    double qx = (R(2,1) - R(1,2)) / (4.0 * qw);
    double qy = (R(0,2) - R(2,0)) / (4.0 * qw);
    double qz = (R(1,0) - R(0,1)) / (4.0 * qw);
    return Eigen::Quaterniond(qw, qx, qy, qz).normalized();
}

void BridgeVicon::applyNoise(Eigen::Vector3d& pos, Eigen::Quaterniond& quat) {
    if (config.enable_pos_noise) {
        pos += Eigen::Vector3d(pos_noise_dist(rng), pos_noise_dist(rng), pos_noise_dist(rng));
    }
    if (config.enable_att_noise) {
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
        euler += Eigen::Vector3d(att_noise_dist(rng), att_noise_dist(rng), att_noise_dist(rng));
        quat = eulerToQuaternion(euler(0), euler(1), euler(2));
    }
    if (config.enable_rotation) {
        if (config.rotation_preference == "quaternion") {
            Eigen::Quaterniond offset(config.quat_offset[3], config.quat_offset[0],
                                     config.quat_offset[1], config.quat_offset[2]);
            quat = offset * quat;
        } else {
            Eigen::Matrix3d R;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R(i, j) = config.rot_matrix_offset[i * 3 + j];
            Eigen::Quaterniond offset = matrixToQuaternion(R);
            quat = offset * quat;
        }
    }
}

void BridgeVicon::sendPose(size_t index) {
    if (!output_trackers[index]) return;   // link down; it will resume on its own
    TrackerState& state = states[index];
    output_trackers[index]->updatePose(
        state.position.x(), state.position.y(), state.position.z(),
        state.quaternion.x(), state.quaternion.y(), state.quaternion.z(), state.quaternion.w()
    );
}

void BridgeVicon::sendZeroPose(size_t index) {
    if (!output_trackers[index]) return;
    output_trackers[index]->updatePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
}

void BridgeVicon::publish(size_t index) {
    if (!output_trackers[index]) return;
    if (output_trackers[index]->publish_pose()) states[index].output_count++;
}

void BridgeVicon::updateRates(Clock::time_point now)
{
    const double dt = std::chrono::duration<double>(now - last_rates).count();
    if (dt < 1.0) return;
    last_rates = now;
    for (auto& st : states) {
        st.in_hz  = (st.input_count  - st.status_prev_in)  / dt;
        st.out_hz = (st.output_count - st.status_prev_out) / dt;
        st.status_prev_in  = st.input_count;
        st.status_prev_out = st.output_count;
    }
}

std::vector<ObjectStatus> BridgeVicon::status() const
{
    std::vector<ObjectStatus> out;
    out.reserve(states.size());
    for (size_t i = 0; i < states.size(); ++i) {
        const OutputLink& link = links[link_of_object[i]];
        ObjectStatus s;
        s.name             = nameOf(config.objects[i].output_object);
        s.input            = config.objects[i].input_object;
        s.output           = config.objects[i].output_object;
        s.in_hz            = states[i].in_hz;
        s.out_hz           = states[i].out_hz;
        s.in_total         = states[i].input_count;
        s.out_total        = states[i].output_count;
        s.link_key         = link.connection_str;
        s.link_up          = link.conn != nullptr;
        s.client_connected = link.conn && link.conn->connected();
        s.down_reason      = link.down_reason;
        out.push_back(std::move(s));
    }
    return out;
}

void BridgeVicon::reportStatus(Clock::time_point now)
{
    if (config.status_interval_sec <= 0) return;
    if (std::chrono::duration<double>(now - last_status).count() < config.status_interval_sec)
        return;
    last_status = now;

    std::ostringstream line;
    line << "BridgeVicon:";
    for (const auto& s : status()) {
        line << "  " << s.name
             << " in="  << static_cast<int>(s.in_hz  + 0.5) << "Hz"
             << " out=" << static_cast<int>(s.out_hz + 0.5) << "Hz ";
        if (!s.link_up)               line << "[port down]";
        else if (s.client_connected)  line << "[client connected]";
        else                          line << "[no client]";
    }
    std::cout << line.str() << std::endl;
}

void BridgeVicon::mainloop() {
    const auto now = Clock::now();

    // Bring back any output port that could not bind yet.
    for (auto& link : links) retryLink(link, now);

    const auto latency = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::milli>(config.latency_ms));

    for (size_t i = 0; i < input_trackers.size(); ++i) {
        input_trackers[i]->mainloop();

        TrackerState& state = states[i];

        if (config.enable_latency) {
            // Release everything that has now waited out the delay. Draining
            // by age (not one per tick) keeps the queue bounded no matter how
            // fast the source runs.
            bool released = false;
            while (!state.pose_queue.empty() &&
                   now - state.pose_queue.front().arrived >= latency) {
                state.position   = state.pose_queue.front().position;
                state.quaternion = state.pose_queue.front().quaternion;
                state.pose_queue.pop_front();
                released = true;
            }
            if (released) {
                applyNoise(state.position, state.quaternion);
                sendPose(i);
            }
        }

        const double idle_sec = std::chrono::duration<double>(now - state.last_input_time).count();
        if (!state.has_valid_data || idle_sec >= config.no_data_timeout_sec)
            sendZeroPose(i);

        // One publish per object per tick, whatever the source did. This
        // only packs into the endpoint buffers; pumpLink() below flushes
        // them in the same tick, so publishing adds no extra latency.
        publish(i);
    }

    for (auto& link : links) pumpLink(link, now);

    updateRates(now);
    reportStatus(now);

    // Sleep to an absolute deadline rather than a fixed duration so the
    // time spent doing VRPN work above doesn't push the loop period past
    // 1/output_frequency (that skew was why the tracker published at
    // ~193 Hz instead of the requested 200 Hz).
    auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.output_frequency));
    next_tick += period;

    auto after_work = Clock::now();
    if (next_tick < after_work) {
        // Fell behind by more than a full period; resync instead of
        // bursting through the backlog of missed ticks.
        next_tick = after_work;
    } else {
        std::this_thread::sleep_until(next_tick);
    }
}
