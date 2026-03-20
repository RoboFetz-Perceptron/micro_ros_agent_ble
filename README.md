# Custom Micro-ROS Agent for BLE Transports

This package allows us to communicate with a micro-ROS client over BLE. We use the [SimpleBLE](https://github.com/simpleble/simpleble) library for BLE connections.

Note that you need to add the following dependencies:
```bash
sudo apt install libbluetooth-dev bluez libdbus-1-dev
```

Rosdep _should_ also work:
```bash
rosdep install --from-paths ws01/src --ignore-src -y
```

Note that this package depends on the [modified Micro-ROS-Agent](https://github.com/Geibinger/micro-ROS-Agent). Make sure to clone and build this package first (e.g. clone and build it in ws00, then put this custom BLE agent into ws01).

To execute the agent, simply run:
```bash
ros2 run micro_ros_agent_ble micro_ros_agent_ble --dev <ble_device_name>
```

Or with the launch file:

```bash
ros2 launch micro_ros_agent_ble agent_ble.launch.py device_name:=<ble_device_name>
```

Replace `<ble_device_name>` with the, duh, BLE advertising name of your bluetooth device. Then, the agent will scan for a device with that name and try to connect to it.

Launch parameters: `device_name`, `scan_timeout`, `verbose` (0-6), `reconnect_delay`, `adapter`.

To list available Bluetooth adapters:
```bash
ros2 run micro_ros_agent_ble micro_ros_agent_ble --list-adapters
```

The data transfer happens in a NUS [Nordic Uart Service](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html) style configuration, with the typical uuids:
```python
        SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
        RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
        TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
```
If you write the firmware on your own, make sure your transport also uses these UUIDs!

## BLE Debugging

1) Try to restart your adapter:
```bash
sudo hciconfig hci1 down && sleep 1 && sudo hciconfig hci1 up
```

(replace hci1 with your actual adapter, you can check the connected ones using hciconfig)

### Supervision Timeout

The firmware automatically requests a 3-second BLE supervision timeout via L2CAP after connecting. This ensures faster disconnect detection across all Bluetooth adapters (some adapters default to as low as 420ms, which can cause drops during entity creation).

To manually check or override the supervision timeout for an existing connection:
```bash
# Find the connection handle:
sudo hcitool -i hci0 con
# Output: < LE F0:F5:BD:09:F1:5E handle 64 state 1 lm PERIPHERAL

# Update supervision timeout to 3s (300 * 10ms):
sudo hcitool -i hci0 lecup --handle 0x0040 --min 24 --max 40 --latency 0 --timeout 300
```

### Monitoring BLE Traffic

Use `btmon` to capture and analyze BLE traffic (useful for diagnosing connection issues):
```bash
# Live monitoring:
sudo btmon -i hci0

# Capture to file for later analysis:
sudo btmon -i hci0 --write /tmp/btmon_capture.btsnoop

# View capture in Wireshark:
wireshark /tmp/btmon_capture.btsnoop
```

Key things to look for in btmon output:
- **Supervision timeout**: In `LE Connection Complete` or `LE Connection Update Complete` events (value in units of 10ms, e.g., 42 = 420ms, 300 = 3s)
- **Disconnect reason**: `0x08` = connection timeout (supervision timeout expired), `0x13` = remote user terminated, `0x22` = LL response timeout
- **MTU**: In `ATT Exchange MTU Request/Response`

### Adapter Info

```bash
# List adapters:
ros2 run micro_ros_agent_ble micro_ros_agent_ble --list-adapters

# Check adapter capabilities (LE ACL buffer count matters for BLE throughput):
sudo hcitool -i hci0 cmd 0x08 0x0002  # LE Read Buffer Size

# Reset a stuck adapter:
sudo hciconfig hci0 down && sleep 1 && sudo hciconfig hci0 up
```

### Common Issues

- **"Device not found"**: Make sure the ESP32 is powered on and advertising. Check `sudo btmon -i hci0` to see if advertisements are received.
- **Connection drops during entity creation**: The adapter's default supervision timeout may be too short. Check with btmon and increase if needed (see above).
- **Agent can't reconnect**: Reset the Bluetooth adapter with `hciconfig down/up` (see above) and restart the ESP32.

That's it, goodbye and thank's for the fish!