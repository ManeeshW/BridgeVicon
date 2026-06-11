#include "BridgeVicon.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>

BridgeConfig readConfig(const std::string& filename) {
    BridgeConfig config;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << ". Using default values.\n";
        return config;
    }

    std::string line, section;
    std::map<int, ObjectPair> object_map;
    std::string legacy_input, legacy_output;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line[0] == '[') {
            section = line.substr(1, line.find(']') - 1);
            continue;
        }

        std::istringstream iss(line);
        std::string key, value;
        std::getline(iss, key, '=');
        std::getline(iss, value);
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));

        try {
            if (section == "vicon") {
                if (key == "input_object") legacy_input = value;         // legacy single-object format
                else if (key == "output_object") legacy_output = value;  // legacy single-object format
                else if (key == "output_frequency") config.output_frequency = std::stoi(value);
                else if (key == "no_data_timeout_sec") config.no_data_timeout_sec = std::stod(value);
            } else if (section.size() > 7 && section.substr(0, 7) == "object_") {
                try {
                    int idx = std::stoi(section.substr(7));
                    if (key == "input_object") object_map[idx].input_object = value;
                    else if (key == "output_object") object_map[idx].output_object = value;
                } catch (...) {}
            } else if (section == "noise") {
                if (key == "enable_pos_noise") config.enable_pos_noise = (value == "true");
                else if (key == "pos_noise_stddev") config.pos_noise_stddev = std::stod(value);
                else if (key == "enable_att_noise") config.enable_att_noise = (value == "true");
                else if (key == "att_noise_stddev") config.att_noise_stddev = std::stod(value);
            } else if (section == "latency") {
                if (key == "enable_latency") config.enable_latency = (value == "true");
                else if (key == "latency_ms") config.latency_ms = std::stod(value);
            } else if (section == "rotation") {
                if (key == "enable_rotation") config.enable_rotation = (value == "true");
                else if (key == "rotation_preference") config.rotation_preference = value;
                else if (key == "quat_offset") {
                    std::istringstream vss(value);
                    std::string qval;
                    for (int i = 0; i < 4 && std::getline(vss, qval, ','); ++i)
                        config.quat_offset[i] = std::stod(qval);
                } else if (key == "rot_matrix_offset") {
                    std::istringstream vss(value);
                    std::string rval;
                    for (int i = 0; i < 9 && std::getline(vss, rval, ','); ++i)
                        config.rot_matrix_offset[i] = std::stod(rval);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing " << key << "=" << value << ": " << e.what() << "\n";
        }
    }

    if (!object_map.empty()) {
        for (auto& [idx, pair] : object_map)
            config.objects.push_back(pair);
    } else if (!legacy_input.empty() && !legacy_output.empty()) {
        config.objects.push_back({legacy_input, legacy_output});
    }

    return config;
}

int main(int argc, char* argv[]) {
    BridgeConfig config = readConfig("../config.ini");

    if (config.objects.empty()) {
        std::cerr << "Error: No objects configured. Add [object_1] sections to config.ini.\n";
        return -1;
    }
    for (size_t i = 0; i < config.objects.size(); ++i) {
        if (config.objects[i].input_object.empty() || config.objects[i].output_object.empty()) {
            std::cerr << "Error: Object " << (i + 1) << " has empty input or output name.\n";
            return -1;
        }
    }
    if (config.output_frequency <= 0) {
        std::cerr << "Error: Invalid frequency " << config.output_frequency << ". Using default 200 Hz.\n";
        config.output_frequency = 200;
    }

    BridgeVicon bridge(config);
    while (true) {
        bridge.mainloop();
    }

    return 0;
}
