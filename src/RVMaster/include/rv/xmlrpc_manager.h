
#ifndef RVCPP_XMLRPC_MANAGER_H
#define RVCPP_XMLRPC_MANAGER_H

#include <string>
#include <set>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "ros/common.h"

#include "rv/XmlRpc.h"
#include "rv/callInfo.h"

#include <ros/time.h>

namespace rv
{

namespace xmlrpc
{
/**
 * \brief internal
 */
XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response);
XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response);
XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response);
}

class XMLRPCCallWrapper;
typedef boost::shared_ptr<XMLRPCCallWrapper> XMLRPCCallWrapperPtr;

class ROSCPP_DECL ASyncXMLRPCConnection : public boost::enable_shared_from_this<ASyncXMLRPCConnection>
{
public:
  virtual ~ASyncXMLRPCConnection() {}

  virtual void addToDispatch(XmlRpcDispatch* disp) = 0;
  virtual void removeFromDispatch(XmlRpcDispatch* disp) = 0;

  virtual bool check() = 0;
};
typedef boost::shared_ptr<ASyncXMLRPCConnection> ASyncXMLRPCConnectionPtr;
typedef std::set<ASyncXMLRPCConnectionPtr> S_ASyncXMLRPCConnection;

class ROSCPP_DECL CachedXmlRpcClient
{
public:
  CachedXmlRpcClient(XmlRpcClient *c)
  : in_use_(false)
  , client_(c)
  {
  }

  bool in_use_;
  ros::WallTime last_use_time_; // for reaping
  XmlRpcClient* client_;

  static const ros::WallDuration s_zombie_time_; // how long before it is toasted
};

class XMLRPCManager;
typedef boost::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;

typedef boost::function<void(XmlRpc::XmlRpcValue&, ClientInfo&, XmlRpc::XmlRpcValue&)> XMLRPCFunc;

class ROSCPP_DECL XMLRPCManager
{
public:
  static const XMLRPCManagerPtr& instance();

  XMLRPCManager();
  ~XMLRPCManager();

  /** @brief Validate an XML/RPC response
   *
   * @param method The RPC method that was invoked.
   * @param response The resonse that was received.
   * @param payload The payload that was received.
   *
   * @return true if validation succeeds, false otherwise.
   *
   * @todo Consider making this private.
   */
  bool validateXmlrpcResponse(const std::string& method, 
			      XmlRpc::XmlRpcValue &response, XmlRpc::XmlRpcValue &payload);

  /**
   * @brief Get the xmlrpc server URI of this node
   */
  inline const std::string& getServerURI() const { return uri_; }
  inline uint32_t getServerPort() const { return port_; }

  XmlRpcClient* getXMLRPCClient(const std::string& host, const int port, const std::string& uri);
  void releaseXMLRPCClient(XmlRpcClient* c);

  void addASyncConnection(const ASyncXMLRPCConnectionPtr& conn);
  void removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn);

  bool bind(const std::string& function_name, const XMLRPCFunc& cb);
  void unbind(const std::string& function_name);

  void start();
  void shutdown();

  bool isShuttingDown() { return shutting_down_; }

private:
  void serverThreadFunc();

  std::string uri_;
  int port_;
  boost::thread server_thread_;

#if defined(__APPLE__)
  // OSX has problems with lots of concurrent xmlrpc calls
  boost::mutex xmlrpc_call_mutex_;
#endif
  XmlRpcServer server_;
  typedef std::vector<CachedXmlRpcClient> V_CachedXmlRpcClient;
  V_CachedXmlRpcClient clients_;
  boost::mutex clients_mutex_;

  bool shutting_down_;

  ros::WallDuration master_retry_timeout_;

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

  volatile bool unbind_requested_;
};

}

#endif

