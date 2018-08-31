#include <subscription_notifier/subscription_notifier.h>
#include <std_msgs/Float64.h>

void callback(const boost::shared_ptr<std_msgs::Float64 const>& msg)
{
  ROS_WARN("data+1=%f",msg->data+1);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "subscription_notifier");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  
  ros_helper::SubscriptionNotifier<std_msgs::Float64> sub(nh,"topic",1,&callback);
  
  if (!sub.waitForANewData(ros::Duration(10)))
    return -1;
  ros::Duration(1).sleep();
  ros::spinOnce();
  ROS_INFO("received = %f", sub.getData().data);
  return 0;
}
  