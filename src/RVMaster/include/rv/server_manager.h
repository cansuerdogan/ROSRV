#ifndef RVCPP_SERVER_MANAGER_H
#define RVCPP_SERVER_MANAGER_H

#include "XmlRpcValue.h"
#include "rv/monitor.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "ros/common.h"
#include "rv/callInfo.h"

namespace rv
{

class ServerManager;
typedef boost::shared_ptr<ServerManager> ServerManagerPtr;

class ROSCPP_DECL ServerManager
{
public:
  static const ServerManagerPtr& instance();
  ServerManager();
  ~ServerManager();
  Monitor* getMonitor(std::string topic);
  void start();
  void shutdown();
  void publish(std::string topic, ros::SerializedMessage message);
  ros::PublicationPtr getOrCreatePubPtr(std::string topic);
  void requestTopicCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool testxmlCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  void pubUpdateCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool registerSubscriberCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool unregisterSubscriberCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool registerPublisherCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool unregisterPublisherCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool lookupNodeCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getRVStateCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getMonitorsCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool monitorControlCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  void monitorControl(std::string monitor,bool enable);

  bool getSystemStateCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getPidCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getUriCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getPublishedTopicsCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getTopicTypesCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool registerServiceCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool unregisterServiceCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool lookupServiceCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool subscribeParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool unsubscribeParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool deleteParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool getParamNamesCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool setParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool searchParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool hasParamCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  void createInterceptors();
private:
  bool requestTopic(const std::string &topic, XmlRpc::XmlRpcValue &protos, XmlRpc::XmlRpcValue &ret); 
  volatile bool shutting_down_;
  boost::mutex shutting_down_mutex_;

map<string,Monitor*> monitorMap;
map<string,ros::PublicationPtr> pubptrMap;
uint32_t real_ros_port;
uint32_t rv_ros_port;
string rv_ros_host;
};

}//namespace rv

#endif

