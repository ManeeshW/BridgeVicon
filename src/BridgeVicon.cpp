#include "BridgeVicon.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <thread>

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

void OutputViconTracker::send_sim_pose() {
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
    if (len > 0) {
        int result = d_connection->pack_message(len, timestamp, position_m_id,
                                               d_sender_id, msgbuf, vrpn_CONNECTION_RELIABLE);
        if (result < 0) {
            std::cerr << "Error: Failed to pack message for publishing\n";
        }
    } else {
        std::cerr << "Error: Failed to encode tracker message\n";
    }
}

void OutputViconTracker::mainloop() {
    send_sim_pose();
    server_mainloop();
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

    size_t n = config.objects.size();

    // Pre-size before registering callbacks so element pointers remain stable
    callback_data.resize(n);
    states.resize(n);

    auto now = std::chrono::steady_clock::now();
    for (size_t i = 0; i < n; ++i) {
        states[i].last_update = now;
        states[i].last_input_time = now;
        callback_data[i] = {this, i};
    }

    for (size_t i = 0; i < n; ++i) {
        const auto& obj = config.objects[i];

        auto* tracker = new vrpn_Tracker_Remote(obj.input_object.c_str());
        tracker->register_change_handler(&callback_data[i], BridgeVicon::callback);
        input_trackers.push_back(tracker);

        std::string connection_str = obj.output_object.substr(obj.output_object.find('@') + 1) + ":3883";
        auto* conn = vrpn_create_server_connection(connection_str.c_str());
        auto* out_tracker = new OutputViconTracker(obj.output_object.substr(0, obj.output_object.find('@')), conn);
        output_connections.push_back(conn);
        output_trackers.push_back(out_tracker);

        std::cout << "BridgeVicon: Object " << (i + 1) << " — input: " << obj.input_object
                  << ", output: " << obj.output_object << "\n";
    }
}

BridgeVicon::~BridgeVicon() {
    for (size_t i = 0; i < output_trackers.size(); ++i)
        sendZeroPose(i);
    for (auto* t : input_trackers) delete t;
    for (auto* t : output_trackers) delete t;
    for (auto* c : output_connections) delete c;
}

void BridgeVicon::callback(void* userdata, const vrpn_TRACKERCB tdata) {
    TrackerCallbackData* cbd = static_cast<TrackerCallbackData*>(userdata);
    BridgeVicon* self = cbd->bridge;
    size_t idx = cbd->index;
    TrackerState& state = self->states[idx];

    Eigen::Vector3d pos(tdata.pos[0], tdata.pos[1], tdata.pos[2]);
    Eigen::Quaterniond quat(tdata.quat[3], tdata.quat[0], tdata.quat[1], tdata.quat[2]); // VRPN: qx, qy, qz, qw

    state.last_input_time = std::chrono::steady_clock::now();
    state.has_valid_data = true;

    if (self->config.enable_latency) {
        state.pose_queue.push({pos, quat});
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
    TrackerState& state = states[index];
    output_trackers[index]->updatePose(
        state.position.x(), state.position.y(), state.position.z(),
        state.quaternion.x(), state.quaternion.y(), state.quaternion.z(), state.quaternion.w()
    );
}

void BridgeVicon::sendZeroPose(size_t index) {
    output_trackers[index]->updatePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    output_trackers[index]->mainloop(); // Force send
}

void BridgeVicon::mainloop() {
    auto now = std::chrono::steady_clock::now();

    for (size_t i = 0; i < input_trackers.size(); ++i) {
        input_trackers[i]->mainloop();
        output_trackers[i]->mainloop();
        output_connections[i]->mainloop();

        TrackerState& state = states[i];
        auto no_data_elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(now - state.last_input_time).count();

        if (!state.has_valid_data || no_data_elapsed_sec >= config.no_data_timeout_sec) {
            sendZeroPose(i);
        }

        if (config.enable_latency && !state.pose_queue.empty()) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_update).count();
            if (elapsed_ms >= config.latency_ms) {
                auto [pos, quat] = state.pose_queue.front();
                state.pose_queue.pop();
                state.position = pos;
                state.quaternion = quat;
                applyNoise(state.position, state.quaternion);
                sendPose(i);
                state.last_update = now;
            }
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / config.output_frequency));
}
