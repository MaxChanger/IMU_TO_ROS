#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request &,
						  std_srvs::Empty::Response &)
{
	ROS_INFO("Zero Orientation Set.");
	zero_orientation_set = false;
	return true;
}

int main(int argc, char **argv)
{
	serial::Serial ser;
	std::string port;
	std::string tf_parent_frame_id;
	std::string tf_frame_id;
	std::string frame_id;
	double time_offset_in_seconds;
	bool broadcast_tf;
	double linear_acceleration_stddev;
	double angular_velocity_stddev;
	double orientation_stddev;
	uint8_t last_received_message_number;
	bool received_message = false;
	int data_packet_start;

	tf::Quaternion orientation;
	tf::Quaternion zero_orientation;

	ros::init(argc, argv, "mpu6050_serial_ttl_to_imu_node");

	ros::NodeHandle private_node_handle("~");
	private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");  // USB端口
	private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
	private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
	private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
	private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
	private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);

	// Standard Deviation,StdDev,标准偏差指标,目的是用来衡量**的波动性
	private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
	private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
	private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

	ros::NodeHandle nh("imu");
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
	ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
	ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation); // 定义了一个服务器

	ros::Rate r(200); // 200 hz

	sensor_msgs::Imu imu; // 标准数据类型 可见 http://docs.ros.org/jade/api/sensor_msgs/html/msg/Imu.html

	/**
	 * 	Header header
		geometry_msgs/Quaternion orientation
		float64[9] orientation_covariance # Row major about x, y, z axes

		geometry_msgs/Vector3 angular_velocity
		float64[9] angular_velocity_covariance # Row major about x, y, z axes

		geometry_msgs/Vector3 linear_acceleration
		float64[9] linear_acceleration_covariance # Row major x, y z 
	**/
	imu.linear_acceleration_covariance[0] = linear_acceleration_stddev; // 线性加速度协方差 = 线性加速度标准差?
	imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
	imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

	imu.angular_velocity_covariance[0] = angular_velocity_stddev;
	imu.angular_velocity_covariance[4] = angular_velocity_stddev;
	imu.angular_velocity_covariance[8] = angular_velocity_stddev;

	imu.orientation_covariance[0] = orientation_stddev;
	imu.orientation_covariance[4] = orientation_stddev;
	imu.orientation_covariance[8] = orientation_stddev;

	/**
	 *  Header header		# timestamp is the time the temperature was measured
                         	# frame_id is the location of the temperature reading

 		float64 temperature     # Measurement of the Temperature in Degrees Celsius

 		float64 variance        # 0 is interpreted as variance unknown 方差
 	**/
	sensor_msgs::Temperature temperature_msg;
	temperature_msg.variance = 0; //方差

	static tf::TransformBroadcaster tf_br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));

	std::string input;
	std::string read;

	while (ros::ok())
	{
		try
		{
			if (ser.isOpen()) // 端口打开
			{
				// read string from serial device 从串行设备读取字符串
				if (ser.available())
				{
					read = ser.read(ser.available());
					// ROS_INFO("read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
					input += read;
					// ROS_WARN_STREAM(" Wait read ");
					// 读到的字符串长度>=33  输入中可能有完整的包
					while (input.length() >= 33) // while there might be a complete package in input
					{
						//parse for data packets 解析数据包
						data_packet_start = input.find("\x55\x51"); // 找开头的标记位
						if (data_packet_start != std::string::npos)
						{
							ROS_DEBUG("found possible start of data packet at position %d", data_packet_start);
							if ((input.length() >= data_packet_start + 33) && (input.compare(data_packet_start + 11, 2, "\x55\x52") == 0) && (input.compare(data_packet_start + 22, 2, "\x55\x53") == 0)) //check if positions 26,27 exist, then test values
							{
								// ROS_ERROR_STREAM(" Found ");

								// for (int i = 0; i < 33; i++)
								// {
								// 	// out put the origin data read from mpu6050, can compare with the software---cutecom
								// 	ROS_INFO("0x%x ", (0xff & (char)input[data_packet_start + i]));
								// }

								// get acelerometer values   获取加速度计值
								// https://www.jianshu.com/p/69dd18638b8e
								int16_t ax = (((0xff & (char)input[data_packet_start + 3]) << 8) | 0xff & (char)input[data_packet_start + 2]);
								int16_t ay = (((0xff & (char)input[data_packet_start + 5]) << 8) | 0xff & (char)input[data_packet_start + 4]);
								int16_t az = (((0xff & (char)input[data_packet_start + 7]) << 8) | 0xff & (char)input[data_packet_start + 6]);
								// calculate accelerations in m/s²
								double axf = ax / 32768.0 * 16 * 9.8;
								double ayf = ay / 32768.0 * 16 * 9.8;
								double azf = az / 32768.0 * 16 * 9.8;
								/**
								 * 采用和陀螺仪同样的计算方法，当AFS_SEL=3时，数字-32767对应-16g，32767对应16g。把32767除以16，就可以得到2048， 即我们说的灵敏度。把从加速度计读出的数字除以2048，就可以换算成加速度的数值。举个例子，如果我们从加速度计读到的数字是1000，那么对应的加速度数据是1000/2048=0.49g。g为加速度的单位，重力加速度定义为1g, 等于9.8米每平方秒。
								 * 
								 * 也就是说说直接从imu中读取到的并不是真正的加速度值，而是一个比例值，相当于几倍的重力加速度g
								 * 
								 **/


								// get gyro values 获得陀螺值
								int16_t gx = (((0xff & (char)input[data_packet_start + 14]) << 8) | 0xff & (char)input[data_packet_start + 13]);
								int16_t gy = (((0xff & (char)input[data_packet_start + 16]) << 8) | 0xff & (char)input[data_packet_start + 15]);
								int16_t gz = (((0xff & (char)input[data_packet_start + 18]) << 8) | 0xff & (char)input[data_packet_start + 17]);
								// calculate rotational velocities in rad/s
								// without the last factor the velocities were too small ????
								// http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
								// FIFO frequency 100 Hz -> factor 10 ?
								// seems 25 is the right factor
								//TODO: check / test if rotational velocities are correct
								double gxf = gx * (2000.0 / 32768.0) * (M_PI / 180.0) * 25.0;
								double gyf = gy * (2000.0 / 32768.0) * (M_PI / 180.0) * 25.0;
								double gzf = gz * (2000.0 / 32768.0) * (M_PI / 180.0) * 25.0; // ????  What is 25???

								ROS_DEBUG("seems to be a real data package: long enough and found end characters");
								
								// get quaternion values 获取四元数值
								// 先读取出来的是 roll pitch 和 yaw三个角度 之后使用ros中的一个转化 转化为四元数
								int16_t roll = (((0xff & (char)input[data_packet_start + 25]) << 8) | 0xff & (char)input[data_packet_start + 24]);
								int16_t pitch = (((0xff & (char)input[data_packet_start + 27]) << 8) | 0xff & (char)input[data_packet_start + 26]);
								int16_t yaw = (((0xff & (char)input[data_packet_start + 29]) << 8) | 0xff & (char)input[data_packet_start + 28]);

								double rollf = roll / 32768.0 * 180 * (M_PI / 180.0);
								double pitchf = pitch / 32768.0 * 180 * (M_PI / 180.0);
								double yawf = yaw / 32768.0 * 180 * (M_PI / 180.0);
								// ROS_INFO("setEuler rollf = %f  pitchf = %f yawf = %f ",rollf, pitchf, yawf);
								tf::Matrix3x3 obs_mat;
								obs_mat.setEulerYPR(yawf, pitchf, rollf);
								tf::Quaternion orientation;
								obs_mat.getRotation(orientation);
								// tf::Quaternion orientation;
								// orientation.setEuler(yawf,pitchf,rollf);

								if (!zero_orientation_set)
								{
									zero_orientation = orientation;
									zero_orientation_set = true;
								}

								//http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
								tf::Quaternion differential_rotation;
								differential_rotation = zero_orientation.inverse() * orientation;

								// get temperature
								int16_t temperature = (((0xff & (char)input[data_packet_start + 22]) << 8) | 0xff & (char)input[data_packet_start + 23]);
								double temperature_in_C = (temperature / 340.0) + 36.25;
								ROS_DEBUG_STREAM("Temperature [in C] " << temperature_in_C);

								// uint8_t received_message_number = input[data_packet_start + 25];
								// ROS_DEBUG("received message number: %i", received_message_number);

								// if (received_message) // can only check for continuous numbers if already received at least one packet
								// {
								// 	uint8_t message_distance = received_message_number - last_received_message_number;
								// 	if (message_distance > 1)
								// 	{
								// 		ROS_WARN_STREAM("Missed " << message_distance - 1 << " MPU6050 data packets from arduino.");
								// 	}
								// }
								// else
								// {
								// 	received_message = true;
								// }
								// last_received_message_number = received_message_number;

								// calculate measurement time
								ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

								// publish imu message
								imu.header.stamp = measurement_time;
								imu.header.frame_id = frame_id;

								quaternionTFToMsg(differential_rotation, imu.orientation); // 把tf四元数转化为geomsg四元数

								imu.angular_velocity.x = gxf; // 角速度
								imu.angular_velocity.y = gyf;
								imu.angular_velocity.z = gzf;

								imu.linear_acceleration.x = axf; // 线加速度
								imu.linear_acceleration.y = ayf;
								imu.linear_acceleration.z = azf;

								imu_pub.publish(imu);

								// publish temperature message
								temperature_msg.header.stamp = measurement_time;
								temperature_msg.header.frame_id = frame_id;
								temperature_msg.temperature = temperature_in_C;

								imu_temperature_pub.publish(temperature_msg);

								// publish tf transform
								if (broadcast_tf)
								{
									transform.setRotation(differential_rotation);
									tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
								}
								input.erase(0, data_packet_start + 33); // delete everything up to and including the processed packet
							}
							else
							{
								if (input.length() >= data_packet_start + 33)
								{
									input.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
								}
								else
								{
									// do not delete start character, maybe complete package has not arrived yet
									input.erase(0, data_packet_start);
								}
							}
						}
						else
						{
							// no start character found in input, so delete everything
							input.clear();
						}
					}
				}
			}
			else
			{
				// try and open the serial port
				try
				{
					ser.setPort(port);
					ser.setBaudrate(115200);
					serial::Timeout to = serial::Timeout::simpleTimeout(1000);
					ser.setTimeout(to);
					ser.open();
				}
				catch (serial::IOException &e)
				{
					ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
					ros::Duration(5).sleep();
				}

				if (ser.isOpen())
				{
					ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
				}
			}
		}
		catch (serial::IOException &e)
		{
			ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
			ser.close();
		}
		ros::spinOnce();
		r.sleep();
	}
}
