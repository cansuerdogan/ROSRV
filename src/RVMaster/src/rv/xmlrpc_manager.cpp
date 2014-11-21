
#include "rv/xmlrpc_manager.h"
#include <rv/callInfo.h>

#include "ros/network.h"

#include "ros/param.h"
#include "ros/assert.h"
#include "ros/common.h"
#include "ros/file_log.h"
#include "ros/io.h"


namespace rv
{
//rvmaster uri
uint32_t rv_port = 11311; 
std::string rv_host;
std::string rv_uri;


std::string getRVHost()
{
return rv_host;
}

namespace xmlrpc
{
XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = code;
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = response;
  return v;
}

XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = XmlRpc::XmlRpcValue(response);
  return v;
}
}
class XMLRPCCallWrapper : public XmlRpcServerMethod2
{
public:
  XMLRPCCallWrapper(const std::string& function_name, const XMLRPCFunc& cb, XmlRpcServer *s)
  : XmlRpcServerMethod2(function_name, s)
  , name_(function_name)
  , func_(cb)
  { }

  void execute(XmlRpc::XmlRpcValue &params, rv::ClientInfo &ci, XmlRpc::XmlRpcValue &result)
  {
	//ROS_INFO("XMLRPCCALL WRAPPER");

    func_(params, ci, result);
  }

private:
  std::string name_;
  XMLRPCFunc func_;
};

void getPid(const XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue& result)
{
  result = xmlrpc::responseInt(1, "", (int)getpid());
}

const ros::WallDuration CachedXmlRpcClient::s_zombie_time_(30.0); // reap after 30 seconds

XMLRPCManagerPtr g_xmlrpc_manager;
boost::mutex g_xmlrpc_manager_mutex;
const XMLRPCManagerPtr& XMLRPCManager::instance()
{
  if (!g_xmlrpc_manager)
  {
    boost::mutex::scoped_lock lock(g_xmlrpc_manager_mutex);
    if (!g_xmlrpc_manager)
    {
      g_xmlrpc_manager.reset(new XMLRPCManager);
    }
  }

  return g_xmlrpc_manager;
}

XMLRPCManager::XMLRPCManager()
: port_(0)
, shutting_down_(false)
, unbind_requested_(false)
{
}

XMLRPCManager::~XMLRPCManager()
{
ROS_INFO("shutdown");  
shutdown();
}

void XMLRPCManager::start()
{
  shutting_down_ = false;

    char *master_uri_env = NULL;
    #ifdef _MSC_VER
      _dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
    #else
      master_uri_env = getenv("ROS_MASTER_URI");
    #endif
    if (!master_uri_env)
    {
       port_ = 11311;
    }
    else
    {
    rv_uri = master_uri_env;
        // Split URI into
  if (!ros::network::splitURI(rv_uri, rv_host, rv_port))
  {
    ROS_FATAL( "Couldn't parse the master URI [%s] into a host:port pair.", rv_uri.c_str());
    ROS_BREAK();
  } 
   port_ = rv_port;
    }



#ifdef _MSC_VER
    // http://msdn.microsoft.com/en-us/library/ms175774(v=vs.80).aspx
    free(master_uri_env);
#endif

  port_ = 11311;

  bind("getPid", getPid);

//  ROS_INFO("reach point debug1");
  bool bound = server_.bindAndListen(port_);
  (void) bound;
  ROS_ASSERT(bound);
  port_ = server_.get_port();
  ROS_ASSERT(port_ != 0);


//  ROS_INFO("reach point debug2");
  std::stringstream ss;
  ss << "http://" << rv_host << ":" << port_ << "/";
  uri_ = ss.str();


  ROS_INFO("listen at uri %s",uri_.c_str());

  server_thread_ = boost::thread(boost::bind(&XMLRPCManager::serverThreadFunc, this));
}

void XMLRPCManager::shutdown()
{
  if (shutting_down_)
  {
    return;
  }

  shutting_down_ = true;
  server_thread_.join();

  server_.close();

  // kill the last few clients that were started in the shutdown process
  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       i != clients_.end(); ++i)
  {
    for (int wait_count = 0; i->in_use_ && wait_count < 10; wait_count++)
    {
      ROSCPP_LOG_DEBUG("waiting for xmlrpc connection to finish...");
      ros::WallDuration(0.01).sleep();
    }

    i->client_->close();
    delete i->client_;
  }

  clients_.clear();

  boost::mutex::scoped_lock lock(functions_mutex_);
  functions_.clear();

  {
    S_ASyncXMLRPCConnection::iterator it = connections_.begin();
    S_ASyncXMLRPCConnection::iterator end = connections_.end();
    for (; it != end; ++it)
    {
      (*it)->removeFromDispatch(server_.get_dispatch());
    }
  }

  connections_.clear();

  {
    boost::mutex::scoped_lock lock(added_connections_mutex_);
    added_connections_.clear();
  }

  {
    boost::mutex::scoped_lock lock(removed_connections_mutex_);
    removed_connections_.clear();
  }
}

bool XMLRPCManager::validateXmlrpcResponse(const std::string& method, XmlRpc::XmlRpcValue &response,
                                    XmlRpc::XmlRpcValue &payload)
{
  if (response.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return an array",
        method.c_str());
    return false;
  }
  if (response.size() != 3)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a 3-element array",
        method.c_str());
    return false;
  }
  if (response[0].getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a int as the 1st element",
        method.c_str());
    return false;
  }
  int status_code = response[0];
  if (response[1].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] didn't return a string as the 2nd element",
        method.c_str());
    return false;
  }
  std::string status_string = response[1];
  if (status_code != 1)
  {
    ROSCPP_LOG_DEBUG("XML-RPC call [%s] returned an error (%d): [%s]",
        method.c_str(), status_code, status_string.c_str());
    return false;
  }
  payload = response[2];
  return true;
}

void XMLRPCManager::serverThreadFunc()
{
  ros::disableAllSignalsInThisThread();


  //ROS_INFO("here1");
  while(!shutting_down_)
  {

  //ROS_INFO("here2");
    {
      boost::mutex::scoped_lock lock(added_connections_mutex_);
      S_ASyncXMLRPCConnection::iterator it = added_connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = added_connections_.end();
      for (; it != end; ++it)
      {
        (*it)->addToDispatch(server_.get_dispatch());
        connections_.insert(*it);

      }

      added_connections_.clear();
    }


    // Update the XMLRPC server, blocking for at most 100ms in select()
    {
      boost::mutex::scoped_lock lock(functions_mutex_);
      server_.work(0.1);
    }

    while (unbind_requested_)
    {
      ros::WallDuration(0.01).sleep();
    }

    if (shutting_down_)
    {
      return;
    }

    {
      S_ASyncXMLRPCConnection::iterator it = connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = connections_.end();
      for (; it != end; ++it)
      {
        if ((*it)->check())
        {
          removeASyncConnection(*it);
        }
      }
    }

    {
      boost::mutex::scoped_lock lock(removed_connections_mutex_);
      S_ASyncXMLRPCConnection::iterator it = removed_connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = removed_connections_.end();
      for (; it != end; ++it)
      {
        (*it)->removeFromDispatch(server_.get_dispatch());
        connections_.erase(*it);
      }

      removed_connections_.clear();
    }
  }
}

XmlRpcClient* XMLRPCManager::getXMLRPCClient(const std::string &host, const int port, const std::string &uri)
{
  // go through our vector of clients and grab the first available one

  XmlRpcClient *c = NULL;

  boost::mutex::scoped_lock lock(clients_mutex_);

  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       !c && i != clients_.end(); )
  {
    if (!i->in_use_)
    {
      // see where it's pointing
      if (i->client_->getHost() == host &&
          i->client_->getPort() == port &&
          i->client_->getUri()  == uri)
      {
        // hooray, it's pointing at our destination. re-use it.
        c = i->client_;
        i->in_use_ = true;
        i->last_use_time_ = ros::WallTime::now();
        break;
      }
      else if (i->last_use_time_ + CachedXmlRpcClient::s_zombie_time_ < ros::WallTime::now())
      {
        // toast this guy. he's dead and nobody is reusing him.
        delete i->client_;
        i = clients_.erase(i);
      }
      else
      {
        ++i; // move along. this guy isn't dead yet.
      }
    }
    else
    {
      ++i;
    }
  }

  if (!c)
  {
    // allocate a new one
    c = new XmlRpcClient(host.c_str(), port, uri.c_str());
    CachedXmlRpcClient mc(c);
    mc.in_use_ = true;
    mc.last_use_time_ = ros::WallTime::now();
    clients_.push_back(mc);
    //ROS_INFO("%d xmlrpc clients allocated\n", xmlrpc_clients.size());
  }
  // ONUS IS ON THE RECEIVER TO UNSET THE IN_USE FLAG
  // by calling releaseXMLRPCClient
  return c;
}

void XMLRPCManager::releaseXMLRPCClient(XmlRpcClient *c)
{
  boost::mutex::scoped_lock lock(clients_mutex_);

  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       i != clients_.end(); ++i)
  {
    if (c == i->client_)
    {
      i->in_use_ = false;
      break;
    }
  }
}

void XMLRPCManager::addASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(added_connections_mutex_);
  added_connections_.insert(conn);
}

void XMLRPCManager::removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(removed_connections_mutex_);
  removed_connections_.insert(conn);
}

bool XMLRPCManager::bind(const std::string& function_name, const XMLRPCFunc& cb)
{
  boost::mutex::scoped_lock lock(functions_mutex_);
  if (functions_.find(function_name) != functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.function = cb;
  info.wrapper.reset(new XMLRPCCallWrapper(function_name, cb, &server_));
  functions_[function_name] = info;

  return true;
}

void XMLRPCManager::unbind(const std::string& function_name)
{
  unbind_requested_ = true;
  boost::mutex::scoped_lock lock(functions_mutex_);
  functions_.erase(function_name);
  unbind_requested_ = false;
}
} //namespace rv
