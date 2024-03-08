#include <ros.h>
#include <rosserial.h>
#include "geometry_msgs/TwistWithCovariance.h"
#include "std_msgs/UInt16.h"

void vel_callback(const geometry_msgs::Twist &msg)
{
	linearvelocity_x = msg.linear.x;
	linearvelocity_y = msg.linear.y;
	angularvelocity = msg.angular.z;
}

void led_callback(const std_msgs::UInt16 &msg)
{
	LED_state = msg.data;
}

//----------------------------definition of ros----------------------------
ros::NodeHandle nh;
geometry_msgs::Twist real_speed;
ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", vel_callback);
ros::Subscriber<std_msgs::UInt16> led_sub("/cmd_led", led_callback);
ros::Publisher odom_pub("Toposition", &real_speed);
double odom_vel[3];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->flush();
}

void Rosserial_Init(void)
{
    nh.initNode();
    nh.advertise(odom_pub);
    nh.subscribe(vel_sub);
    nh.subscribe(led_sub);
}

void Rosserial_Spin(void)
{
		nh.spinOnce();
}

bool Rosserial_Checkconfigstate(void)
{
		return nh.config_state();
}

void Rosserial_GetHardware(void)
{
		nh.getHardware()->init();
}

void odom_store(void)
{
		real_speed.linear.x =  odom_vel[0];
		real_speed.linear.y = odom_vel[1];
		real_speed.angular.z = odom_vel[2];
}

void odom_publish(void)
{
		odom_pub.publish(&real_speed);
		nh.spinOnce();
}
