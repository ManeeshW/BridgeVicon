#ifndef BRIDGE_VICON_H
#define BRIDGE_VICON_H

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <Eigen/Dense>
#include <string>
#include <random>
#include <queue>
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
};

class OutputViconTracker : public vrpn_Tracker {
public:
    OutputViconTracker(const std::string& name, vrpn_Connection* c);
    void send_sim_pose();
    void mainloop() override;
    void updatePose(double x, double y, double z, double qx, double qy, double qz, double qw);

private:
    double current_pos[3];
    double current_quat[4];
};

struct TrackerState {
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond quaternion{1, 0, 0, 0};
    std::queue<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> pose_queue;
    std::chrono::steady_clock::time_point last_update;
    std::chrono::steady_clock::time_point last_input_time;
    bool has_valid_data = false;
};

class BridgeVicon;

struct TrackerCallbackData {
    BridgeVicon* bridge;
    size_t index;
};

class BridgeVicon {
public:
    BridgeVicon(const BridgeConfig& config);
    ~BridgeVicon();

    void mainloop();
    static void callback(void* userdata, const vrpn_TRACKERCB tdata);

private:
    std::vector<vrpn_Tracker_Remote*> input_trackers;
    std::vector<OutputViconTracker*> output_trackers;
    std::vector<vrpn_Connection*> output_connections;
    std::vector<TrackerState> states;
    std::vector<TrackerCallbackData> callback_data;

    BridgeConfig config;
    std::default_random_engine rng;
    std::normal_distribution<double> pos_noise_dist;
    std::normal_distribution<double> att_noise_dist;

    Eigen::Quaterniond eulerToQuaternion(double yaw, double pitch, double roll);
    Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& R);
    void applyNoise(Eigen::Vector3d& pos, Eigen::Quaterniond& quat);
    void sendPose(size_t index);
    void sendZeroPose(size_t index);
};

#endif
