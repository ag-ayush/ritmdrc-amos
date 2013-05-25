#ifndef DYNAMIC_PARAM_H
#define DYNAMIC_PARAM_H

#include <ros/ros.h>

template <typename Msg, typename Val>
class DynamicParam {
public:
  virtual const Val& get()=0;
  virtual void set(const Val& v)=0;

  virtual bool init(ros::NodeHandle* nh, std::string topic)=0;
  virtual void init(ros::NodeHandle* nh, std::string topic, const Val& def)=0;
    
protected:
  ros::NodeHandle *n;
  ros::Publisher pub;
  ros::Subscriber sub;
  std::string topicName;
  
  void initTopics(ros::NodeHandle* nh, std::string topic) {
    n = nh;
    topicName = topic;
    pub = nh->advertise<Msg>(topic, 1, true);
    sub = nh->subscribe(topic, 1, &DynamicParam<Msg, Val>::paramCallback, this);
  }
  
  virtual void updateVal()=0;
  
  virtual void paramCallback(const Msg& msg)=0;
};

//The definition generation macro for primitives
#define PRIMITIVE_PARAM_DEF(TYPENAME, MSG, TYPE) \
\
class TYPENAME##Param : DynamicParam<std_msgs::MSG, TYPE> {\
public:\
  virtual bool init(ros::NodeHandle* nh, std::string topic);\
  virtual void init(ros::NodeHandle* nh, std::string topic, const TYPE& def);\
  \
  virtual const TYPE& get();\
  virtual void set(const TYPE& v);\
  \
protected:\
  \
  virtual void updateVal();\
  virtual void paramCallback(const std_msgs::MSG& msg);\
  \
private:\
  TYPE val;\
};

//The implementation generation macro for primitive data types
#define PRIMITIVE_PARAM_IMPL(TYPENAME, MSG, TYPE)\
bool TYPENAME##Param::init(ros::NodeHandle* nh, std::string topic) {\
  initTopics(nh, topic);\
  return nh->getParam(topic, val);\
}\
\
void TYPENAME##Param::init(ros::NodeHandle* nh, std::string topic, const TYPE& def) {\
  initTopics(nh, topic);\
  \
    if (!nh->hasParam(topic)) {\
        set(def);\
    } else {\
        nh->getParam(topic, val);\
    }\
}\
\
const TYPE& TYPENAME##Param::get() {\
  return val;\
}\
\
void TYPENAME##Param::set(const TYPE& v) {\
  val = v;\
  updateVal();\
}\
\
void TYPENAME##Param::updateVal() {\
  std_msgs::MSG msg;\
  msg.data = val;\
  pub.publish(msg);\
  n->setParam(topicName, val);\
}\
\
void TYPENAME##Param::paramCallback(const std_msgs::MSG& msg) {\
  val = msg.data;\
}

#endif
