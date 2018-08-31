#ifndef subscription_notifier_20180622_0746
#define subscription_notifier_20180622_0746
#include <ros/ros.h>
#include <boost/function.hpp>
#include <mutex>

namespace ros_helper
{
  
  /**
   * \brief 
   * SubscriptionNotifier<T>
   * subscribe a topic and provide basic utilities (new messages received, wait for a new message).
   */
  template<typename T>  class SubscriptionNotifier
  {
    
    protected:
    ros::Subscriber   m_sub;
    ros::NodeHandle   m_nh;
    bool              m_new_data;
    std::string       m_topic;
    unsigned long int m_msg_counter;
    std::mutex        m_mtx;
    T                 data;
    
    boost::function<void(const boost::shared_ptr<T const>& msg)> m_callback;
    
    /*
     * void callback(const boost::shared_ptr<T const>& msg);    
     * SubscriptionNotifier callback
     */
    void callback(const boost::shared_ptr<T const>& msg);    
    
  public:
    /*
     * SubscriptionNotifier<T>( ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size)
     * subscribe a topic and provide basic utilities (new messages received, wait for a new message).
     */
    SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size);
    
    /*
     * SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, boost::function<void(const boost::shared_ptr<T const>& msg)> callback)
     * subscribe a topic and provide basic utilities (new messages received, wait for a new message). *callback* function is called when the message is processed. 
     */
    SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, boost::function<void(const boost::shared_ptr<T const>& msg)> callback);
    
    /*
     * void setAdvancedCallback(boost::function<void(const boost::shared_ptr<T const>& msg)> callback);
     * set a used-defined callback, use boost bind for methods.
     */
    void setAdvancedCallback(boost::function<void(const boost::shared_ptr<T const>& msg)> callback);
    
    /*
     * bool isANewDataAvailable();
     * check is there is a data that is not already read.
     */
    bool              isANewDataAvailable();
    
    /*
     * bool waitForANewData(const ros::Duration& timeout);
     * wait *timeout* for receiving a new data, if no data are received return false;
     */
    bool waitForANewData(const ros::Duration& timeout=ros::Duration(10.0));
    
    
    /*
     * T  getData();
     * returns the last available data, and marks it as already read.
     */
    T  getData();
  }; 
  
  
  template<typename T> 
  SubscriptionNotifier<T>::SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, boost::function<void(const boost::shared_ptr<T const>& msg)> callback):
  SubscriptionNotifier<T>(nh,topic,queue_size)
  {
    setAdvancedCallback(callback);
  }
  
  template<typename T> 
  SubscriptionNotifier<T>::SubscriptionNotifier(  ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size)
  {
    m_nh=nh;
    m_topic=topic;
    m_sub = m_nh.subscribe<T>(topic,queue_size,&ros_helper::SubscriptionNotifier<T>::callback,this); 
    m_new_data=false;
    m_msg_counter=0;
    
    ROS_DEBUG("[%s] create SubscriptionNotifier!\n",m_topic.c_str());
  }
  
  template<typename T>
  void SubscriptionNotifier<T>::setAdvancedCallback(boost::function<void(const boost::shared_ptr<T const>& msg)> callback)
  {
    m_callback=callback;
  }
  
  template<typename T> 
  void SubscriptionNotifier<T>::callback(const boost::shared_ptr<T const>& msg)
  {
    m_mtx.lock();
    data=*msg;
    m_new_data=true;
    m_msg_counter++;
    m_mtx.unlock();
    
    ROS_INFO("[%s] first message received!\n",m_topic.c_str());
    ROS_DEBUG("[MsgReceived %s] new message received!\n",m_topic.c_str());
    if (m_callback)
      m_callback(msg);
  }
  
  template<typename T> 
  T SubscriptionNotifier<T>::getData()
  {
    m_mtx.lock();
    T tmp=data;
    m_new_data=false;
    m_mtx.unlock();
    return tmp;
  }
  
  template<typename T> 
  bool SubscriptionNotifier<T>::isANewDataAvailable()
  {
    return m_new_data;
  }
  
  template<typename T> 
  bool SubscriptionNotifier<T>::waitForANewData(const ros::Duration& timeout)
  {
    ros::Rate loopRate(100);
    ros::Time init_time = ros::Time::now();
    while((ros::Time::now()-init_time)<timeout)
    {
      if (this->isANewDataAvailable())
      {
        ROS_INFO("[%s] wait %5.4f seconds to receive a new message !\n",m_topic.c_str(), (ros::Time::now()-init_time).toSec());
        return true;
      }
      loopRate.sleep();
      ros::spinOnce();
    }
    
    ROS_ERROR("[%s] timeout (%5.4f seconds) on receiving a new message !\n",m_topic.c_str(),(ros::Time::now()-init_time).toSec());
    return false;
  }   
  
  
  
}; 

#endif