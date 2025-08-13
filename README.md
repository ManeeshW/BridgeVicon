# Vicon Bridge

This project provides a C++ program that bridges a real Vicon motion capture system to a simulated Vicon output on a different network. It reads position and quaternion data from a Vicon system (e.g., `Origins@192.168.10.1`) and publishes it to another network (e.g., `Quad@192.168.1.67`) with configurable Gaussian noise, latency, frequency, and rotation adjustments (via quaternion or rotation matrix).

## Prerequisites

- **CMake** (>= 3.10)
- **C++ Compiler** (C++17 support, e.g., g++ or clang++)
- **Eigen3** (>= 3.3)
- **VRPN** (Virtual Reality Peripheral Network library)
- **Quat** (Quaternion library for VRPN)

### Installing Dependencies

#### macOS (using Homebrew)
```bash
brew install cmake eigen vrpn
```

#### Linux (using apt on Ubuntu/Debian)
```bash
sudo apt update
sudo apt install cmake g++ libeigen3-dev libvrpn-dev
```

If VRPN or Quat is not available, build from source:
```bash
git clone https://github.com/vrpn/vrpn.git
cd vrpn
git submodule update --init
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DVRPN_BUILD_QUATLIB=ON
make -j$(nproc)
sudo make install
```

## Setup and Installation

1. **Create a Build Directory**:
   ```bash
   mkdir build && cd build
   ```

2. **Run CMake**:
   ```bash
   cmake ..
   ```

3. **Build the Project**:
   ```bash
   make
   ```

This will compile the `vicon_bridge` executable.

## Configuration

The program uses a `config.ini` file to specify input/output objects and parameters. The default configuration is:
```
[vicon]
input_object=Origins@192.168.10.1
output_object=Quad@192.168.1.67
output_frequency=200

[noise]
enable_pos_noise=false
pos_noise_stddev=0.01
enable_att_noise=false
att_noise_stddev=0.01

[latency]
enable_latency=false
latency_ms=10.0

[rotation]
enable_rotation=false
rotation_preference=quaternion
quat_offset=0.0,0.0,0.0,1.0
rot_matrix_offset=1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0
```

Edit `config.ini` in the build directory to change settings:
- `input_object`: Vicon object to read from (e.g., `Origins@192.168.10.1`).
- `output_object`: Simulated Vicon object to publish to (e.g., `Quad@192.168.1.67`).
- `output_frequency`: Output frequency in Hz.
- `enable_pos_noise`: Enable Gaussian noise for position (true/false).
- `pos_noise_stddev`: Standard deviation for position noise (meters).
- `enable_att_noise`: Enable Gaussian noise for attitude (true/false).
- `att_noise_stddev`: Standard deviation for attitude noise (radians).
- `enable_latency`: Enable artificial latency (true/false).
- `latency_ms`: Latency in milliseconds.
- `enable_rotation`: Enable rotation offset (true/false).
- `rotation_preference`: Use "quaternion" or "matrix" for rotation offset.
- `quat_offset`: Quaternion offset [qx, qy, qz, qw] (default: identity [0, 0, 0, 1]).
- `rot_matrix_offset`: Rotation matrix offset [r11, r12, r13, r21, r22, r23, r31, r32, r33] (default: identity).

**Note**: If `rotation_preference=quaternion`, `quat_offset` is used; if `matrix`, `rot_matrix_offset` is used. The quaternion is normalized, and the rotation matrix is validated to be unitary.

## Usage

Run the executable from the build directory:
```bash
./vicon_bridge
```

This reads from the input Vicon system, applies configured transformations (noise, latency, rotation), and publishes to the output network.

## Directory Structure
```
vicon_bridge/
  ├── include/
  │   └── BridgeVicon.h
  ├── src/
  │   ├── main.cpp
  │   └── BridgeVicon.cpp
  ├── config.ini
  ├── CMakeLists.txt
  └── README.md
```

## Troubleshooting
- **VRPN/Quat not found**: Ensure libraries are in `/opt/homebrew/lib` (macOS) or `/usr/local/lib` (Linux).
- **Config file errors**: Verify `config.ini` exists with valid entries (e.g., correct number of values for `quat_offset` or `rot_matrix_offset`).
- **Network issues**: Ensure the Vicon system and output network are accessible at the specified IPs.
- **Rotation errors**: Ensure `quat_offset` is a valid unit quaternion and `rot_matrix_offset` is a valid rotation matrix.