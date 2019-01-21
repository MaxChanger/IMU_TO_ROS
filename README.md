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



## sanchi_amov

The driver for sanchi company's IMU, I use 100D2. 

```shell
# start the imu
roslaunch sanchi_amov imu_100D2.launch
```

We should pay more attention to the units. Is the unit of data published by IMU consistent with the unit used by ROS?

```c++
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

```

