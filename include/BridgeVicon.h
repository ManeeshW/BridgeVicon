#ifndef BRIDGE_VICON_H
#define BRIDGE_VICON_H

#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <Eigen/Dense>
#include <string>
#include <random>
#include <queue>
#include <chrono>

struct BridgeConfig {
    // Input Vicon settings
    std::string input_object = "OriginsX@192.168.10.1";
    // Output Vicon settings
    std::string output_object = "Quad@192.168.1.67";
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

class BridgeVicon {
public:
    BridgeVicon(const BridgeConfig& config);
    ~BridgeVicon();

    void mainloop();
    static void callback(void* userdata, const vrpn_TRACKERCB tdata);

private:
    vrpn_Tracker_Remote* input_tracker;
    OutputViconTracker* output_tracker;
    vrpn_Connection* output_connection;

    BridgeConfig config;
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    std::default_random_engine rng;
    std::normal_distribution<double> pos_noise_dist;
    std::normal_distribution<double> att_noise_dist;
    std::queue<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> pose_queue;
    std::chrono::steady_clock::time_point last_update;
    std::chrono::steady_clock::time_point last_input_time;
    bool has_valid_data = false;

    Eigen::Quaterniond eulerToQuaternion(double yaw, double pitch, double roll);
    Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& R);
    void applyNoise(Eigen::Vector3d& pos, Eigen::Quaterniond& quat);
    void sendPose();
    void sendZeroPose();
};

#endif