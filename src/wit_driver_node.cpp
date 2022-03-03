#include "ros/ros.h"
#include <serial/serial.h> //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
// #include <std_msgs/Empty.h>

#include "JY901.h"

serial::Serial ser; //声明串口对象
//回调函数
// void write_callback(const std_msgs::String::ConstPtr &msg)
// {
//     ROS_INFO_STREAM("Writing to serial port" << msg->data);
//     ser.write(msg->data); //发送串口数据
// }
int main(int argc, char **argv)
{
    std::string port;
    int baudrate;
    int pub_rate;
    bool pub_mag;
    std::string imu_topic;
    std::string mag_topic;
    std::string imu_frame;

    //初始化节点
    ros::init(argc, argv, "imu");
    ROS_INFO("Init IMU Node.");
    //声明节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("port", port, "ttyS0");
    port = "/dev/" + port;
    nh_private.param<int>("baudrate", baudrate, 9600);
    nh_private.param<int>("publish_rate", pub_rate, 20);
    nh_private.param<bool>("publish_mag", pub_mag, true);
    nh_private.param<std::string>("imu_topic", imu_topic, "imu_data");
    nh_private.param<std::string>("mag_topic", mag_topic, "mag_data");
    nh_private.param<std::string>("imu_frame", imu_frame, "imu_link");

    ROS_INFO_STREAM("port : " << port);
    ROS_INFO_STREAM("baudrate : " << baudrate);
    ROS_INFO_STREAM("publish_rate : " << pub_rate);
    ROS_INFO_STREAM("publish_mag : " << (pub_mag ? "true" : "false"));
    ROS_INFO_STREAM("imu_topic : " << imu_topic);
    ROS_INFO_STREAM("mag_topic : " << mag_topic);
    ROS_INFO_STREAM("imu_frame : " << imu_frame);

    //发布主题
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>(mag_topic, 1000);

    CJY901 imu = CJY901();
    try
    {
        //设置串口属性，并打开串口
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(500);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    // ser.flush();
    int size;
    //指定循环的频率
    ros::Rate loop_rate(pub_rate);
    while (ros::ok())
    {
        int count = ser.available();
        if (count != 0)
        {
            ROS_INFO_ONCE("Data received from serial port.");
            // std::string result;
            // result = ser.read(ser.available());
            // // result.data = ser.readline(500, "\n");
            // ROS_INFO("\nRead %d byte:", result.size());
            // for (int i = 0; i < result.size(); i++)
            //     printf("0x%.2X ", result.data()[i]);
            // continue;
            // imu.CopeSerialData(const_cast<char *>(result.data()), result.size());

            int num;
            unsigned char read_buf[count];
            num = ser.read(read_buf, count);
            // result.data = ser.readline(500, "\n");
            // ROS_INFO("\nRead %d byte:", num);
            // for (int i = 0; i < num; i++)
            //     printf("0x%.2X ", read_buf[i]);
            // continue;
            //imu.CopeSerialData((char *)read_buf, num);
            imu.FetchData((char *)read_buf, num);
            // ROS_INFO("IMU Data : Quaternion{ x: %f  y: %f  z: %f  w: %f} ", imu.quat.x, imu.quat.y, imu.quat.z, imu.quat.w);
            sensor_msgs::Imu imu_data;

            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = imu_frame;

            imu_data.linear_acceleration.x = imu.acc.x;
            imu_data.linear_acceleration.y = imu.acc.y;
            imu_data.linear_acceleration.z = imu.acc.z;
            imu_data.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            imu_data.angular_velocity.x = imu.gyro.x;
            imu_data.angular_velocity.y = imu.gyro.y;
            imu_data.angular_velocity.z = imu.gyro.z;
            imu_data.linear_acceleration_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            imu_data.orientation.x = imu.quat.x;
            imu_data.orientation.y = imu.quat.y;
            imu_data.orientation.z = imu.quat.z;
            imu_data.orientation.w = imu.quat.w;
            imu_data.orientation_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

            imu_pub.publish(imu_data);

            if (pub_mag)
            {
                sensor_msgs::MagneticField mag_data;

                mag_data.header.stamp = imu_data.header.stamp;
                mag_data.header.frame_id = imu_data.header.frame_id;

                mag_data.magnetic_field.x = imu.mag.x;
                mag_data.magnetic_field.y = imu.mag.y;
                mag_data.magnetic_field.z = imu.mag.z;
                mag_data.magnetic_field_covariance = {1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6};

                mag_pub.publish(mag_data);
            }
        }

        //处理ROS的信息，比如订阅消息,并调用回调函数
        //ros::spinOnce();
        loop_rate.sleep();
    }
}