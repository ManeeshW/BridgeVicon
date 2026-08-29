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

The program uses a `config.ini` file to specify input/output objects and parameters.
It is looked up in this order, so the bridge runs from anywhere:
`./vicon_bridge <path>` → `$VICON_BRIDGE_CONFIG` → `./config.ini` →
`../config.ini` → next to the executable. The chosen file is printed at startup.

```
[vicon]
output_frequency=200
no_data_timeout_sec=10.0
vrpn_port=3883
status_interval_sec=5.0

# One [object_N] section per tracked object. Objects that publish to the same
# host share a single VRPN server connection on vrpn_port.
[object_1]
input_object=OriginsX@100.118.37.119
output_object=OriginsX@192.168.11.2
object_on=true

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
- `input_object`: Vicon object to read from (e.g., `OriginsX@100.118.37.119`).
- `output_object`: Simulated Vicon object to publish to (e.g., `OriginsX@192.168.11.2`).
- `object_on`: Set `false` to skip an object without deleting its section.
- `output_frequency`: Output frequency in Hz.
- `vrpn_port`: TCP/UDP port the bridge serves on (clients connect here).
- `no_data_timeout_sec`: Publish the identity pose after this long with no input.
- `status_interval_sec`: Seconds between the one-line health reports (0 = off).
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
./vicon_bridge              # or: ./vicon_bridge /path/to/config.ini
```

On a terminal this brings up a live dashboard that repaints one frame in place —
**nothing scrolls**, so the numbers you care about stay on the same line:

```
vicon_bridge  3 objects                                            up 0:14:02
------------------------------------------------------------------------------
OBJECT     SOURCE                 OUTPUT               IN      OUT     CLIENT
Alpha      Alpha@192.168.10.1     Alpha@192.168.11.2   200 Hz  200 Hz  connected
BravoRelay Bravo@192.168.10.1     BravoRelay@10.0.0.5  200 Hz  200 Hz  waiting
Charlie    Charlie@192.168.10.1   Charlie@192.168.11.2 200 Hz  200 Hz  connected
------------------------------------------------------------------------------
publishing at 200 Hz   ·   2/3 with a client   ·   168420 in / 168420 out
config ../config.ini
------------------------------------------------------------------------------
events   (37 routine VRPN notes muted)
01:10:16 BridgeVicon: serving 192.168.11.2:3883
------------------------------------------------------------------------------
Ctrl-C to quit   ·   --plain for scrolling log output
```

`IN` is the rate arriving from the source, `OUT` the rate actually published, and
`CLIENT` says whether anything is subscribed to *that object's* output port —
objects publishing to different hosts have independent ports, so they report
independently. The table sizes itself to the objects and to the terminal, and the
event pane shrinks (or disappears) on a short window rather than pushing the
table off screen.

VRPN narrates every client connect and disconnect on stdout/stderr, which is what
used to make the terminal roll. The UI captures both streams into the bounded event
pane: repeats collapse into one line with a `(xN)` counter, and the notes VRPN
itself labels harmless (`check_vrpn_cookie(): VRPN Note: …`, `this is normal when a
connection is dropped`) are muted down to the counter in the pane title.

```bash
./vicon_bridge --plain      # scrolling log instead, with nothing filtered
./vicon_bridge --help
```

`--plain` prints one health line per interval, as before:

```
BridgeVicon:  OriginsX in=200Hz out=200Hz [client connected]
```

The dashboard also steps aside automatically when stdout is not a terminal, so
`./vicon_bridge > run.log`, pipes and systemd units keep their plain output.

`Ctrl-C` shuts down cleanly in either mode: clients are parked at the identity
pose, the sockets are closed and the terminal is restored.

### Restarting while a client is connected

Restarting the bridge while the ESP32 (or any VRPN client) has a session open used to
leave it permanently broken — see [Troubleshooting](#troubleshooting). It now rebinds
immediately, and if the port genuinely is not available the bridge says why and keeps
retrying until it frees, resuming on its own:

```
BridgeVicon: cannot serve 192.168.11.2:3883 — the port is still held by the
             previous run's connections (TIME-WAIT, ...); it frees itself within ~60 s
BridgeVicon: retrying every 0.5-5 s; poses resume automatically once the port is free.
BridgeVicon: 192.168.11.2:3883 is serving again (after 4 failed attempts)
```

## Directory Structure
```
vicon_bridge/
  ├── include/
  │   ├── BridgeVicon.h
  │   └── ConsoleUI.h
  ├── src/
  │   ├── main.cpp
  │   ├── BridgeVicon.cpp
  │   ├── ConsoleUI.cpp       # the in-place dashboard (and stdout/stderr capture)
  │   └── net_reuseaddr.cpp   # SO_REUSEADDR on VRPN's listening sockets
  ├── config.ini
  ├── CMakeLists.txt
  └── README.md
```

## Troubleshooting

- **`Failed to pack message for publishing` / `pack_message: Can't pack because the
  connection is broken`, repeating forever, with no client ever receiving a pose.**
  This is a *bind failure at startup*, not a runtime fault: VRPN could not take
  `vrpn_port`, marked the connection BROKEN, and
  `vrpn_create_server_connection()` handed it back anyway. The usual cause is
  restarting the bridge while a client was connected — the just-closed client
  socket sits in `TIME-WAIT` on the same port for ~60 s, and VRPN's
  `open_socket()` binds without `SO_REUSEADDR` on every platform except Android.
  Two things now prevent it:
  `src/net_reuseaddr.cpp` sets `SO_REUSEADDR` on VRPN's own `bind()` calls (via
  the linker's `--wrap`, so VRPN itself needs no patching), and the bridge checks
  `doing_okay()` after creating each connection and retries instead of publishing
  into a dead one. If you still see the message, you are running a build from
  before this fix.
- **`another program (a second vicon_bridge?) is already serving …`**: exactly what
  it says — one process per port. Stop the other one; this instance takes over by
  itself within a few seconds.
- **VRPN/Quat not found**: Ensure libraries are in `/opt/homebrew/lib` (macOS) or `/usr/local/lib` (Linux).
- **Config file errors**: Verify `config.ini` exists with valid entries (e.g., correct number of values for `quat_offset` or `rot_matrix_offset`).
- **Network issues**: Ensure the Vicon system and output network are accessible at the specified IPs.
- **Rotation errors**: Ensure `quat_offset` is a valid unit quaternion and `rot_matrix_offset` is a valid rotation matrix.