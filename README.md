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

Launch parameters: `device_name`, `scan_timeout`, `verbose` (0-6), `reconnect_delay`.

The data transfer happens in a NUS [Nordic Uart Service](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html) style configuration, with the typical uuids:
```python
        SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
        RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
        TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
```
If you write the firmware on your own, make sure your transport also uses thes uuids!

That's it, goodbye and thank's for the fish!