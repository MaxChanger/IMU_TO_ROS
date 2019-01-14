# **mpu6050_to_imu_ROS**

This project was modified by [fsteinhardt/mpu6050_serial_to_imu](https://github.com/fsteinhardt/mpu6050_serial_to_imu). This project if you want to get IMU data through MPU6050, You need an additional Arduino development board.



## **mpu6050_serial_to_imu** 

This package use Arduino and MPU6050(GY-521) to publish imu data in ROS.

You can see more details in [mpu6050_serial_to_imu/README.md](https://github.com/MaxChanger/mpu6050_to_imu_ROS/blob/master/mpu6050_serial_to_imu/README.md)

Besides, I add the libraries (MPU6050 I2Cdev I2CMaster) in `mpu6050_serial_to_imu/arduino/MPU6050/`.

So you can copy them to the libraries of Arduino IDE directly.

If you have finished all the work , you can run this command to see the 3D model in rviz.

```shell
roslaunch mpu6050_serial_to_imu demo.launch
```

And use this to see the topic published by the node.

```shell
rostopic echo /imu/data
```



## mpu6050_serial_ttl_to_imu

This package use USB_TTL and MPU6050(GY-61) to publish imu data in ROS.

Correctly link USB-TTL module and MPU6050 module. `VCC-VCC, GND-GND, TX-RX, RX-TX`

If you have finished all the work , you can run this command to see the 3D model in rviz.

```shell
roslaunch mpu6050_serial_ttl_to_imu demo.launch
```

And use this to see the topic published by the node.

```shell
rostopic echo /imu/data
```



