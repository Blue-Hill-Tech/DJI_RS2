# DJI RS2/RS3 Gimbal ZMQ Follower

Real-time gimbal follower that receives Euler angles via ZMQ and controls the DJI RS2/RS3 gimbal.

## Hardware Requirements

- DJI RS2 or RS3 Gimbal
- CANable Pro (or similar USB-CAN adapter)
- Linux computer with SocketCAN support

## Software Dependencies

### 1. Install Python packages

```bash
pip3 install python-can zmq numpy icecream pandas pyserial playsound
```

### 2. Install CAN utilities (if not already installed)

```bash
sudo apt-get install can-utils
```

## CAN Bus Setup

### Initialize CAN interface

Run these commands before starting the follower:

```bash
# Set CAN transmit queue length (important for high-frequency communication)
sudo ifconfig can0 txqueuelen 5000

# Bring up CAN interface with 1Mbps bitrate
sudo ip link set can0 up type can bitrate 1000000
```

Or use the provided script:

```bash
sudo ./can-up
```

### Verify CAN interface

```bash
ip link show can0
```

You should see `state UP` in the output.

## Usage

### 1. Start the gimbal follower

```bash
python3 follow_zmq_v8
```

The script will:
- Listen on `tcp://0.0.0.0:5555` for ZMQ messages
- Control the gimbal based on received Euler angles
- Print real-time status with delay information

### 2. Send data from another device (e.g., Windows)

```python
import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect("tcp://192.168.1.56:5555")  # Replace with Linux IP

while True:
    yaw, pitch, roll = 0.0, 0.0, 0.0  # Your Euler angles
    timestamp = time.time()
    socket.send_string(f"{yaw},{pitch},{roll},{timestamp}")
    time.sleep(0.01)  # 100Hz
```

### Data Format

```
yaw,pitch,roll,timestamp
```

- **yaw**: Yaw angle in degrees
- **pitch**: Pitch angle in degrees  
- **roll**: Roll angle in degrees
- **timestamp**: Unix timestamp (optional, for delay calculation)

Example: `10.5,-5.2,3.0,1764795277.393`

## Versions

| Version | Description |
|---------|-------------|
| v4 | Basic dual-thread with timestamp |
| v5 | Print all messages |
| v6 | Add delay calculation |
| v7 | Event-driven, 500Hz control loop |
| v8 | Async logger (recommended) |
| v9 | With PID controller |

## Troubleshooting

### CAN interface not found

```bash
# Check if CANable is connected
lsusb | grep -i can

# Load CAN modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
```

### Permission denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

### High latency

- Use wired Ethernet instead of WiFi
- Ensure both machines have synchronized clocks (NTP)
- Use v8 with async logger

## Stop the follower

Press `Ctrl+C` to stop. The gimbal will return to home position automatically.
