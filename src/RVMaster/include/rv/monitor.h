#ifndef RVCPP_MONITOR_H
#define RVCPP_MONITOR_H

#include "rv/xmlrpc_manager.h"
#include "rv/connection_manager.h"
#include "rv/subscription.h"
#include "ros/callback_queue.h"
#include "ros/publication.h"
#include <set>

using namespace std;

namespace rv
{

class XMLRPCCallWrapper;
typedef boost::shared_ptr<XMLRPCCallWrapper> XMLRPCCallWrapperPtr;

class Monitor
{

public:
   Monitor(XmlRpc::XmlRpcValue& params,std::string hostname, std::string monitorname);
   ~Monitor();
  string getMonitorXmlRpcUri();

  bool bind(const std::string& function_name, const XMLRPCFunc& cb);

  XmlRpcClient* getXMLRPCClient(const std::string& host, const int port, const std::string& uri);
  void releaseXMLRPCClient(XmlRpcClient* c);

  void addASyncConnection(const ASyncXMLRPCConnectionPtr& conn);
  void removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn);
  void shutdown();

  void addSubscriber(std::string ip);
  int removeSubscriber(std::string ip);

  void setPubUris(vector<string> &uris);
  ros::PublicationPtr getMonitorPublicationPtr();

  SubscriptionPtr getMonitorSubscriptionPtr();

  void getPidCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue &result); 
  void requestTopicCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue &result); 
  void pubUpdateCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  bool shutdownCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);
  //bool getMasterUriCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);  
  bool getBusInfoCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result);  

  void monitorThreadFunc();
  void processPublishQueues();
  bool isShuttingDown() { return shutting_down_; }

  string getMonitorName();
  int getMonitorXmlRpcPort();
  int getMonitorTcpPort();

private:

string m_topic;
string m_uri;
string m_host;
int m_port;
int tpc_port;
string m_name;

vector<string> pub_uris;
vector<string> sub_ips;

ros::PublicationPtr pub_ptr;
SubscriptionPtr sub_ptr;
ros::CallbackQueuePtr cb_queue;

  typedef std::vector<CachedXmlRpcClient> V_CachedXmlRpcClient;
  V_CachedXmlRpcClient clients_;
  boost::mutex clients_mutex_;

bool shutting_down_;
XmlRpcServer m_server;
ConnectionManagerPtr connection_manager_;

  S_ASyncXMLRPCConnection added_connections_;
  boost::mutex added_connections_mutex_;
  S_ASyncXMLRPCConnection removed_connections_;
  boost::mutex removed_connections_mutex_;

  S_ASyncXMLRPCConnection connections_;
  struct FunctionInfo
  {
    std::string name;
    XMLRPCFunc function;
    XMLRPCCallWrapperPtr wrapper;
  };
  typedef std::map<std::string, FunctionInfo> M_StringToFuncInfo;
  boost::mutex functions_mutex_;
  M_StringToFuncInfo functions_;

};

}//namespace rv

#endif
