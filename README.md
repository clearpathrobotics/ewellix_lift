# Ewellix Driver

ROS2 driver for the Ewellix TLT lifts.

## Setup
Add user to `dialout` group to ensure the driver has permissions to access the serial device.
```
sudo usermod -a -G dialout UserName
```

## Ewellix Node Usage
Run the driver:
```
ros2 run ewellix_driver ewellix_node
```

Parameters:
  - `port`: Path to device. By default, `/dev/ttyUSB0`.
  - `baud`: Baud rate, on most TLT it should be set to `38400`
  - `timeout`: Timeout in milliseconds to wait for response from SCU. By default, `1000` ms.
  - `joint_count`: Number of actuators on tower. By default, `2`.
  - `conversion`: Encoder ticks per meter. Varies depending on lift model. On the 500mm model, `3225` encoder ticks per meter.
  - `rated_effort`: Rated force of lift. On the 500mm model, `2000` N.
  - `tolerance`: Distance from commanded position to consider within bounds. By default `0.005` meters.
  - `frequency`: Publishing loop rate. By default, `10 Hz`.

Running the driver with parameters:
```
ros2 run ewellix_driver ewellix_node --ros-args -p port:=/dev/ttyUSB0
```

Moving the lift:
```
ros2 topic pub /command std_msgs/msg/Int32 "data: 0"
```

## Ewellix Hardware Interface Usage

## References
1. [Ewellix serial control unit (SCU) installation, operation, and maintenance manual.](https://medialibrary.ewellix.com/asset/16223)
2. [Ewellix RS232 interface for serial control unit.](https://medialibrary.ewellix.com/asset/16222)
3. [Ewellix TLT lifting columns](https://medialibrary.ewellix.com/asset/16207)
