#include "BridgeVicon.h"
#include "ConsoleUI.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <atomic>
#include <csignal>
#include <cstring>
#include <string>
#include <vector>
#include <unistd.h>
#include <limits.h>
#include <cstdio>

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop = true; }   // async-signal-safe: just a flag

/* Directory holding this executable, so the bridge finds its config.ini
 * however it was launched (./vicon_bridge, an absolute path, a symlink in
 * PATH, a systemd unit with a different working directory...). */
std::string exeDir()
{
    char buf[PATH_MAX];
    const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "";
    buf[n] = '\0';
    const std::string path(buf);
    const size_t slash = path.rfind('/');
    return slash == std::string::npos ? "" : path.substr(0, slash);
}

/* The old code hard-coded "../config.ini", which only worked when launched
 * from inside build/. Search the sensible places instead and say which file
 * was used. */
std::string findConfig(const char* explicit_path)
{
    std::vector<std::string> candidates;
    if (explicit_path && *explicit_path) candidates.push_back(explicit_path);
    if (const char* env = ::getenv("VICON_BRIDGE_CONFIG")) if (*env) candidates.push_back(env);
    candidates.push_back("config.ini");
    candidates.push_back("../config.ini");
    const std::string dir = exeDir();
    if (!dir.empty()) {
        candidates.push_back(dir + "/config.ini");
        candidates.push_back(dir + "/../config.ini");
    }
    for (const auto& c : candidates) {
        std::ifstream f(c);
        if (f.is_open()) return c;
    }
    return explicit_path && *explicit_path ? explicit_path : "config.ini";
}

} // namespace

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
        if (!value.empty()) value.erase(value.find_last_not_of(" \t\r\n") + 1);

        try {
            if (section == "vicon") {
                if (key == "input_object") legacy_input = value;         // legacy single-object format
                else if (key == "output_object") legacy_output = value;  // legacy single-object format
                else if (key == "output_frequency") config.output_frequency = std::stoi(value);
                else if (key == "no_data_timeout_sec") config.no_data_timeout_sec = std::stod(value);
                else if (key == "vrpn_port") config.vrpn_port = std::stoi(value);
                else if (key == "status_interval_sec") config.status_interval_sec = std::stod(value);
            } else if (section.size() > 7 && section.substr(0, 7) == "object_") {
                try {
                    int idx = std::stoi(section.substr(7));
                    if (key == "input_object") object_map[idx].input_object = value;
                    else if (key == "output_object") object_map[idx].output_object = value;
                    else if (key == "object_on") object_map[idx].object_on = (value == "true");
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
        for (auto& [idx, pair] : object_map) {
            if (pair.object_on)
                config.objects.push_back(pair);
            else
                std::cout << "BridgeVicon: object_" << idx << " (" << pair.input_object << ") disabled (object_on=false)\n";
        }
    } else if (!legacy_input.empty() && !legacy_output.empty()) {
        config.objects.push_back({legacy_input, legacy_output});
    }

    return config;
}

int main(int argc, char* argv[]) {
    /* Line-buffer stdout so `./vicon_bridge | tee log` and systemd journals
     * show progress live instead of in 4 KB bursts. */
    ::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::cout.setf(std::ios::unitbuf);

    const char* config_arg = nullptr;
    bool want_ui = true;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--plain" || a == "--no-ui") {
            want_ui = false;
        } else if (a == "-h" || a == "--help") {
            std::cout <<
                "usage: vicon_bridge [options] [config.ini]\n"
                "\n"
                "  Bridges a Vicon/VRPN object onto another network. With no config\n"
                "  path it looks for $VICON_BRIDGE_CONFIG, ./config.ini, ../config.ini,\n"
                "  then one next to the executable.\n"
                "\n"
                "options:\n"
                "  --plain, --no-ui   scrolling log instead of the live dashboard\n"
                "                     (also the default when stdout is not a terminal)\n"
                "  -h, --help         this message\n";
            return 0;
        } else if (!a.empty() && a[0] == '-') {
            std::cerr << "vicon_bridge: unknown option '" << a << "' (try --help)\n";
            return 2;
        } else if (!config_arg) {
            config_arg = argv[i];
        }
    }

    const std::string config_path = findConfig(config_arg);
    std::cout << "BridgeVicon: config " << config_path << "\n";
    BridgeConfig config = readConfig(config_path);

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

    /* Ctrl-C / SIGTERM must unwind normally: the destructor parks every
     * client at the identity pose and closes the listening sockets, which
     * is also what keeps the next run from tripping over its own leftovers. */
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);   /* a client vanishing must not kill us */

    /* The dashboard repaints one frame in place and swallows VRPN's own
     * chatter, so nothing scrolls. It declines automatically when stdout is
     * not a terminal, which keeps pipes and log files behaving as before. */
    ConsoleUI ui;
    if (want_ui && ui.begin()) {
        /* The periodic status line would land in the event pane and push
         * everything else out; the dashboard already shows the same numbers. */
        config.status_interval_sec = 0;
    }

    BridgeVicon bridge(config);

    auto next_frame = std::chrono::steady_clock::now();
    while (!g_stop) {
        bridge.mainloop();
        if (ui.active()) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_frame) {
                next_frame = now + std::chrono::milliseconds(100);   /* 10 fps */
                ui.pump();
                ui.render(bridge.status(), config_path, config.output_frequency);
            }
        }
    }

    ui.end();   /* restores the terminal and the real stdout/stderr */
    std::cout << "BridgeVicon: shutting down\n";

    return 0;
}
