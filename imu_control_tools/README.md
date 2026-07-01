# E2BOX/E2BDX IMU control tools

Standalone tools for enabling or disabling the magnetometer and checking the
remaining gyroscope/accelerometer axes in real time. These tools live outside
the ROS 2 workspace and do not require ROS 2.

## Requirements

```bash
sudo apt install python3-serial
chmod +x disable_imu_magnetometer.py
```

The default serial settings are `/dev/ttyUSB0` and `115200` baud.

## Magnetometer OFF

```bash
./disable_imu_magnetometer.py --state off
```

This sends `<sem0>` and succeeds only when the IMU returns `<ok>`.

## Magnetometer ON

```bash
./disable_imu_magnetometer.py --state on
```

This sends `<sem1>` and succeeds only when the IMU returns `<ok>`.

## Live 6-axis check

```bash
./disable_imu_magnetometer.py --monitor
```

Monitor mode disables magnetometer fusion and magnetometer raw output, enables
gyroscope XYZ and accelerometer XYZ output, and displays the six values in the
terminal. Rotate or tilt the IMU to verify that each axis changes. Press
`Ctrl+C` to stop.

To select another device or baud rate:

```bash
./disable_imu_magnetometer.py \
  --port /dev/ttyUSB1 \
  --baudrate 115200 \
  --monitor
```

Only one process can use the serial port at a time. Stop the ROS 2 EBIMU driver
before running this tool, or run the OFF command before starting the driver.
