#ifndef BRIDGE_VICON_H
#define BRIDGE_VICON_H

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <Eigen/Dense>
#include <string>
#include <random>
#include <deque>
#include <chrono>
#include <vector>

struct ObjectPair {
    std::string input_object;
    std::string output_object;
    bool object_on = true;
};

struct BridgeConfig {
    std::vector<ObjectPair> objects;
    int output_frequency = 200; // Hz
    // Noise settings
    bool enable_pos_noise = false;
    double pos_noise_stddev = 0.01; // meters
    bool enable_att_noise = false;
    double att_noise_stddev = 0.01; // radians
    // Latency settings
    bool enable_latency = false;
    double latency_ms = 10.0; // milliseconds
    // Rotation adjustment
    bool enable_rotation = false;
    std::string rotation_preference = "quaternion"; // "quaternion" or "matrix"
    double quat_offset[4] = {0.0, 0.0, 0.0, 1.0}; // qx, qy, qz, qw (identity quaternion)
    double rot_matrix_offset[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // Identity matrix
    // Timeout for no input data (send zero pose if exceeded)
    double no_data_timeout_sec = 10.0; // seconds
    int vrpn_port = 3884;
    // Seconds between the one-line "still alive" reports (0 disables them).
    double status_interval_sec = 5.0;
};

class OutputViconTracker : public vrpn_Tracker {
public:
    OutputViconTracker(const std::string& name, vrpn_Connection* c);
    /// Publishes the pose held by updatePose() and runs the per-tracker
    /// server housekeeping. Returns false if the connection refused the
    /// message (i.e. it is broken). server_mainloop() is protected in
    /// vrpn_BaseClass, so callers go through here.
    bool publish_pose();
    void mainloop() override;
    void updatePose(double x, double y, double z, double qx, double qy, double qz, double qw);

private:
    bool send_sim_pose();

    double current_pos[3];
    double current_quat[4];
};

struct TrackerState {
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond quaternion{1, 0, 0, 0};
    /// Poses waiting out the configured artificial latency, each stamped with
    /// the time it arrived. Drained by age, so the queue cannot grow without
    /// bound when the input rate exceeds 1000/latency_ms.
    struct DelayedPose {
        std::chrono::steady_clock::time_point arrived;
        Eigen::Vector3d position;
        Eigen::Quaterniond quaternion;
    };
    std::deque<DelayedPose> pose_queue;
    std::chrono::steady_clock::time_point last_input_time;
    bool has_valid_data = false;
    unsigned long long input_count = 0;   // poses received from the source
    unsigned long long output_count = 0;  // poses published downstream
    unsigned long long status_prev_in = 0;   // counts at the last rate sample
    unsigned long long status_prev_out = 0;
    double in_hz = 0;                        // refreshed once a second
    double out_hz = 0;
};

/// Snapshot of one object for the console UI / status line.
struct ObjectStatus {
    std::string name;              // published tracker name
    std::string input;             // "Object@host" it reads from
    std::string output;            // "Object@host" it publishes as
    double in_hz = 0;              // measured over the last second
    double out_hz = 0;
    unsigned long long in_total = 0;
    unsigned long long out_total = 0;
    std::string link_key;          // "host:port" — objects sharing one server
                                   // connection share this, so the UI can
                                   // report a port problem once, not per object
    bool link_up = false;          // the output port is bound
    bool client_connected = false; // someone is subscribed
    std::string down_reason;       // why the port is unavailable, if it is
};

class BridgeVicon;

struct TrackerCallbackData {
    BridgeVicon* bridge;
    size_t index;
};

/**
 * One outgoing VRPN server connection (one host:port) plus the trackers
 * published on it. Kept as a unit because a connection that fails to bind —
 * or that breaks later — can only be repaired by rebuilding its trackers
 * along with it.
 */
struct OutputLink {
    std::string connection_str;              // "host:port" handed to VRPN
    vrpn_Connection* conn = nullptr;         // null while the link is down
    std::vector<size_t> objects;             // indices into config.objects
    int attempts = 0;                        // consecutive failed opens
    std::string down_reason;                 // why openLink() last failed
    std::chrono::steady_clock::time_point next_attempt;
    /// When the connection first reported not-okay (default = healthy).
    std::chrono::steady_clock::time_point down_since{};
    bool reported_down = false;              // rate-limits the failure log
};

class BridgeVicon {
public:
    BridgeVicon(const BridgeConfig& config);
    ~BridgeVicon();

    void mainloop();
    static void callback(void* userdata, const vrpn_TRACKERCB tdata);

    /// True once every configured output link has bound its port.
    bool allLinksUp() const;

    /// Per-object snapshot for display. Rates are the last full second.
    std::vector<ObjectStatus> status() const;

private:
    std::vector<vrpn_Tracker_Remote*> input_trackers;
    std::vector<OutputViconTracker*> output_trackers;  // null while its link is down
    std::vector<OutputLink> links;
    std::vector<size_t> link_of_object;                // object index -> links index
    std::vector<TrackerState> states;
    std::vector<TrackerCallbackData> callback_data;

    BridgeConfig config;
    std::chrono::steady_clock::time_point next_tick;
    std::chrono::steady_clock::time_point last_status;
    std::chrono::steady_clock::time_point last_rates;
    std::default_random_engine rng;
    std::normal_distribution<double> pos_noise_dist;
    std::normal_distribution<double> att_noise_dist;

    Eigen::Quaterniond eulerToQuaternion(double yaw, double pitch, double roll);
    Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& R);
    void applyNoise(Eigen::Vector3d& pos, Eigen::Quaterniond& quat);
    void sendPose(size_t index);
    void sendZeroPose(size_t index);
    void publish(size_t index);

    bool openLink(OutputLink& link);
    void closeLink(OutputLink& link);
    /// Retries a link whose port would not bind, once its backoff expires.
    void retryLink(OutputLink& link, std::chrono::steady_clock::time_point now);
    /// Flushes packed messages, accepts new clients, and rebuilds the link if
    /// the connection itself (not just a client) has gone bad.
    void pumpLink(OutputLink& link, std::chrono::steady_clock::time_point now);
    /// Recomputes in_hz/out_hz once a second, independent of how often
    /// anything asks to display them.
    void updateRates(std::chrono::steady_clock::time_point now);
    void reportStatus(std::chrono::steady_clock::time_point now);
};

#endif
