# Subscription notifier is an helper class to add simple features to ROS Subscribers (www.stiima.cnr.it)

The repository contains the implementation of hardware interfaces and controller manager developed by the Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing, of the National Research Council of Italy (CNR-STIIMA).

## SubscriptionNotifier Class
> _SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size)_
creates an Subscriber instance of Topic _topic_ with a queue of _queue_size_ elements.

> _SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, boost::function<void(const boost::shared_ptr<T const>& msg)> callback)_
      subscribe a topic and provide basic utilities (new messages received, wait for a new message). *callback* function is called when the message is processed. 

> _isANewDataAvailable()_ check is there is a data that is not already read.

> _waitForANewData(const ros::Duration& timeout=ros::Duration(10.0))_ wait _timeout_ for receiving a new data, if no data are received return false;

> _getData()_ returns the last available data, and marks it as already read.

> _setAdvancedCallback(boost::function<void(const boost::shared_ptr<T const>& msg)> callback)_  set a used-defined callback, use boost bind for methods.

## Example of usage

```c++
ros::NodeHandle nh;
ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_sub(  nh, "joint_states", 10);
if (!js_sub._waitForANewData())
  return 0;
while (ros::ok())
{
  if (js_sub._isANewDataAvailable())
  {
    // do something...
  }
  ros::Duration(0.001).sleep();
  ros::spinOnce();
}
return 0;
```


_Software License Agreement (BSD License)_    
_Copyright (c) 2010, National Research Council of Italy, Institute of Intelligent Industrial Technologies and Systems for Advanced Manufacturing_    
_All rights reserved._