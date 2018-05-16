#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <queue>
#include <thread>
#include <chrono>
#include <ratio>
#include <mutex>

class TimedState: public sensor_msgs::JointState
{
public:
  std::chrono::time_point<std::chrono::high_resolution_clock> sendTime;
};

std::queue<TimedState> stateQueue;
std::mutex stateQueueMutex;

void postpone(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  TimedState state;
  state.sendTime = std::chrono::high_resolution_clock::now() + std::chrono::duration<int, std::milli>(100);
  state.name = msg->joint_names;
  state.position = msg->points[0].positions;

  stateQueueMutex.lock();
  stateQueue.push(state);
  stateQueueMutex.unlock();
}

void publishQueue()
{
  ROS_INFO("publishQueue thread has started.");

  int argc = 0;	//fictitious
  char **argv = nullptr;	//fictitious
  ros::init(argc, argv, "subpub");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

  while (true)
  {
    stateQueueMutex.lock();
    bool stateQueueEmpty = stateQueue.empty();
    stateQueueMutex.unlock();

    if (!stateQueueEmpty)
    {
      stateQueueMutex.lock();
      auto sendTime = stateQueue.front().sendTime;
      stateQueueMutex.unlock();
      if (sendTime < std::chrono::high_resolution_clock::now())
      {
	sensor_msgs::JointState msg;

	stateQueueMutex.lock();
	msg.name = stateQueue.front().name;
	msg.position = stateQueue.front().position;
	stateQueue.pop();
	stateQueueMutex.unlock();

	pub.publish(msg);
      } else
      {
        std::this_thread::sleep_until<std::chrono::high_resolution_clock>(sendTime);
      }
    } else
    {
//      ROS_INFO("The stateQueue is empty. Waiting for it is filled.");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subpub");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sinus", 10, postpone);

  std::thread pq(publishQueue);

  ros::spin();

//  pq.join();
  return 0;
}
