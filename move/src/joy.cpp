
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopCerus
{
public:
  TeleopCerus();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  int linear, angular, strafe,save;
  double l_scale, a_scale, s_scale, s_save;
  ros::Publisher vel_pub;
  ros::Subscriber joy_sub;

};


TeleopCerus::TeleopCerus():
  linear(1),
  angular(2),
  strafe(3),
  save(4)
{

  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("axis_strafe", strafe, strafe);
  nh.param("axis_save", save, save);
  nh.param("scale_angular", a_scale, a_scale);
  nh.param("scale_linear", l_scale, l_scale);
  nh.param("scale_strafe", s_scale, s_scale);
  nh.param("scale_save", s_save, s_save);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1,true);


  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopCerus::joyCallback, this);

}

void TeleopCerus::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale*joy->axes[angular];
  twist.linear.x = l_scale*joy->axes[linear];  
  twist.linear.y = s_scale*joy->axes[strafe];
  twist.angular.x = s_save*joy->axes[save];
  vel_pub.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  TeleopCerus teleop_cerus;

  ros::spin();
}

