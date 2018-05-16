#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <vector>
#include <string>
#include <cmath>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("sinus", 10);

  const int frequency = 1000;
  ros::Rate loop_rate(frequency);

  const size_t NUM_TRAJ_POINTS = 1;
  const size_t NUM_JOINTS = 3;
  std::vector<std::string> jointNames(NUM_JOINTS);
  jointNames[0] = "shoulder";
  jointNames[1] = "elbow";
  jointNames[2] = "wrist";

  ros::Duration instant(0.0);
  const ros::Duration period(1.0/((float)frequency));

  const double sinPeriod = 10.0;
  const double pi = 3.14159;
  const double phaseShift[NUM_JOINTS - 1] = {pi/3.0, 2.0*pi/3.0};
  const double phaseStep = pi*2.0/sinPeriod/(double)frequency;
  double phase[NUM_JOINTS] = {0.0, phaseShift[0], phaseShift[1]};

  while (ros::ok())
  {
    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = jointNames;
    msg.points.resize(NUM_TRAJ_POINTS);
    msg.points[0].positions.resize(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; i++)
    {
      phase[i] += phaseStep;
      if (phase[i] > 2.0*pi) phase[i] -= 2.0*pi;
    }

    msg.points[0].time_from_start = instant;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      msg.points[0].positions[i] = std::sin(phase[i]);
    }

    pub.publish(msg);

    ros::spinOnce();
    instant += period;

    loop_rate.sleep();
  }

  return 0;
}
