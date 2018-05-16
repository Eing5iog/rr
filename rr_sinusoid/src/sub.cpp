#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <iomanip>

bool due = true;

void printLline(const sensor_msgs::JointState::ConstPtr& msg)
{
  const int NUM_JOINTS = 3;
  if (due == true)
  {
    for (int i = 0; i < NUM_JOINTS - 1; i++)
      std::cout << msg->name[i] << " : " << std::setw(13) << msg->position[i] << "\t,\t";
    std::cout << msg->name[NUM_JOINTS - 1] << " : ";
    std::cout << std::setw(13) << msg->position[NUM_JOINTS - 1] << std::endl;
  }
  due = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sub");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joint_states", 10, printLline);
  const int frequency = 70;	//limit printing frequency to reasonable
  ros::Rate loop_rate(frequency);

  while (ros::ok())
  {
    due = true;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
