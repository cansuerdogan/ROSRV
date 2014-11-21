#include "ros/console.h"
#include "ros/network.h"
#include "ros/common.h"
#include "ros/time.h"
#include "ros/file_log.h"
#include "rvmonitor.h"
#include <unistd.h>

#include "rv/monitor.h"
#include "rv/xmlrpc_manager.h"
#include "rv/acctrl_manager.h"
#include "rv/connection_manager.h"
#include "ros/publication.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_udp.h"
#include "rv/subscription.h"
#include "ros/transport_hints.h"
#include "ros/init.h"
#include "ros/poll_manager.h"
#include <ros/ros.h>
#include <boost/function.hpp>

using namespace std;

namespace rv
{

class XMLRPCCallWrapper : public XmlRpcServerMethod2
{
public:
  XMLRPCCallWrapper(const std::string& function_name, const XMLRPCFunc& cb, XmlRpcServer *s)
  : XmlRpcServerMethod2(function_name, s)
  , name_(function_name)
  , func_(cb)
  { }

  void execute(XmlRpc::XmlRpcValue &params, ClientInfo &ci, XmlRpc::XmlRpcValue &result)
  {
	//ROS_INFO("XMLRPCCALL WRAPPER");

    func_(params, ci, result);
  }

private:
  std::string name_;
  XMLRPCFunc func_;
};

void Monitor::addSubscriber(std::string ip)
{
   sub_ips.push_back(ip);
  ROS_INFO("monitor add subscriber %s",ip.c_str());
}

int Monitor::removeSubscriber(std::string ip)
{
std::vector<std::string>::iterator it = std::find(sub_ips.begin(),sub_ips.end(),ip);
if(it!=sub_ips.end())
    sub_ips.erase(it);

    return sub_ips.size();
}



SubscriptionPtr Monitor::getMonitorSubscriptionPtr()
{
return sub_ptr;
}
ros::PublicationPtr Monitor::getMonitorPublicationPtr()
{
return pub_ptr;
}

void Monitor::setPubUris(vector<string> &uris)
{
       pub_uris = uris;
      //send request to all the publishers??
      sub_ptr->pubUpdate(pub_uris);
}

string Monitor::getMonitorName()
{
   return m_name;
}
int Monitor::getMonitorXmlRpcPort()
{
   return m_port;
}

int Monitor::getMonitorTcpPort()
{
return connection_manager_->getTCPPort();
}

Monitor::Monitor(XmlRpc::XmlRpcValue &params,string hostname,string monitorname)
{
   m_host = hostname;
   m_name = monitorname;

   string node_name = params[0];
   string topic = params[1];
   string m_datatype = params[2];
   string sub_uri = params[3];

   m_topic = topic;

   ros::Time::init();

   
//   ros::AdvertiseOptions ops_pub;
   ros::SubscribeOptions ops_sub;
   
   RVMonitor *rvmonitor =  new RVMonitor(topic,ops_sub);

//   ops_pub.callback_queue = ops_sub.callback_queue; 
 
//pub_ptr = ros::PublicationPtr(new ros::Publication(ops_pub.topic, ops_pub.datatype, ops_pub.md5sum, ops_pub.message_definition, ops_pub.queue_size, ops_pub.latch, ops_pub.has_header)); 
 
//ros::SubscriberCallbacksPtr callbacks(new ros::SubscriberCallbacks(ops_pub.connect_cb, ops_pub.disconnect_cb,  ops_pub.tracked_object, ops_pub.callback_queue)); 
//pub_ptr->addCallbacks(callbacks); 

pub_ptr = rv::ServerManager::instance()->getOrCreatePubPtr(topic);

//bind xmlrpc call
bind("requestTopic", boost::bind(&Monitor::requestTopicCallback, this, _1, _2,_3));
bind("publisherUpdate", boost::bind(&Monitor::pubUpdateCallback, this, _1, _2,_3));
bind("shutdown", boost::bind(&Monitor::shutdownCallback, this, _1, _2,_3));
bind("getBusInfo", boost::bind(&Monitor::getBusInfoCallback, this, _1, _2,_3));
bind("getPid", boost::bind(&Monitor::getPidCallback, this, _1, _2,_3));

//startXmlRpcServer
  shutting_down_ = false;
  m_port = 0;

int port = rv::acctrl::getNewPort();  
while(!m_server.bindAndListen(port))
{
port=rv::acctrl::getNewPort();
}
  //(void) bound;
  //ROS_ASSERT(bound);
  m_port = m_server.get_port();
  ROS_ASSERT(m_port != 0);

  std::stringstream ss;
  ss << "http://" << m_host << ":" << m_port << "/";
  m_uri = ss.str();

  ROS_INFO("monitor uri is %s", m_uri.c_str());

//connection_manager_ = ConnectionManager::instance();
connection_manager_.reset(new ConnectionManager);
connection_manager_->start();
ros::PollManagerPtr poll_manager = connection_manager_->getPollManagerPtr();
poll_manager->addPollThreadListener(boost::bind(&Monitor::processPublishQueues, this));
 
ops_sub.transport_hints = ros::TransportHints();
cb_queue.reset(new ros::CallbackQueue);
ops_sub.callback_queue = (ros::CallbackQueueInterface*)(cb_queue.get());

SubscriptionPtr s(new Subscription(ops_sub.topic, ops_sub.md5sum, ops_sub.datatype, ops_sub.transport_hints,poll_manager));
  s->addCallback(ops_sub.helper, ops_sub.md5sum, ops_sub.callback_queue, ops_sub.queue_size,
                 ops_sub.tracked_object, ops_sub.allow_concurrent_callbacks);

sub_ptr = s;

}

void Monitor::processPublishQueues()
{
    //ROS_INFO("processing publish queues");
    pub_ptr->processPublishQueue();
}

Monitor::~Monitor()
{

    //ROS_INFO("monitor shutdown");
shutdown();
}
bool Monitor::bind(const std::string& function_name, const XMLRPCFunc& cb)
{
  boost::mutex::scoped_lock lock(functions_mutex_);
  if (functions_.find(function_name) != functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.function = cb;
  info.wrapper.reset(new XMLRPCCallWrapper(function_name, cb, &m_server));
  functions_[function_name] = info;

  return true;
}

string Monitor::getMonitorXmlRpcUri()
{
return m_uri;
}


bool Monitor::getBusInfoCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result)
{
string node_name = params[0];

ROS_INFO("Node %s trying to getBusInfo from monitor %s", node_name.c_str(), m_name.c_str());

std::string command = "getBusInfo";

if(acctrl::isCommandAllowed(command,node_name,ci.ip))
{

  result[0] = 1;
  result[1] = std::string("");
  XmlRpc::XmlRpcValue info;
//  getBusInfo(response);

// force these guys to be arrays, even if we don't populate them
  info.setSize(0);
  pub_ptr->getInfo(info);
  sub_ptr->getInfo(info);

  result[2] = info;

ROS_INFO("Node %s successfully getBusInfo from monitor %s", node_name.c_str(),m_name.c_str());
return true;
}
else
{
ROS_WARN("Node %s is not able to getBusInfo from monitor %s due to access control!",node_name.c_str(),m_name.c_str());

//return bad result to the publisher??

result = rv::xmlrpc::responseInt(-1,"Access Control",0);

return false;
}

}
bool Monitor::shutdownCallback(XmlRpc::XmlRpcValue& params,  ClientInfo &ci, XmlRpc::XmlRpcValue &result)
{
string node_name = params[0];

ROS_INFO("Node %s trying to shutdown monitor %s", node_name.c_str(), m_name.c_str());

std::string command = "shutdown";

if(acctrl::isCommandAllowed(command,node_name,ci.ip))
{

shutdown();
ROS_INFO("Node %s successfully shutdown monitor %s", node_name.c_str(),m_name.c_str());
result = rv::xmlrpc::responseInt(1,"monitor shutdown",1);
return true;
}
else
{
ROS_WARN("Node %s is not able to shutdown monitor %s due to access control!",node_name.c_str(),m_name.c_str());

//return bad result to the publisher??

result = rv::xmlrpc::responseInt(0,"Access Control",0);

return false;
}

}


void Monitor::shutdown()
{
  if (shutting_down_)
  {
    return;
  }

ROS_INFO("Monitor at TCP port %d was shutdown",connection_manager_->getTCPPort());

  shutting_down_ = true;

return;

  m_server.close();



  // kill the last few clients that were started in the shutdown process
  for (V_CachedXmlRpcClient::iterator i = clients_.begin();
       i != clients_.end(); ++i)
  {
    for (int wait_count = 0; i->in_use_ && wait_count < 10; wait_count++)
    {
      //ROSCPP_LOG_DEBUG("waiting for xmlrpc connection to finish...");
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
      (*it)->removeFromDispatch(m_server.get_dispatch());
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

ROS_INFO("Monitor at TCP port %d was shutdown.",connection_manager_->getTCPPort());

}
void Monitor::monitorThreadFunc()
{

  ros::disableAllSignalsInThisThread();

  while(!shutting_down_)
  {

//test publish message
//why no message is received??
cb_queue->callAvailable(ros::WallDuration(0.1));

//for testing only
/*
std_msgs::String msg;
std::stringstream ss;
ss << "test monitor pub";
msg.data = ss.str();

//do serialize the message
ros::SerializedMessage m = ros::serialization::serializeMessage(msg);

//require publicatioPtr which requires adversed_topics..
pub_ptr->publish(m);
*/

//ros::PollManager::instance()->getPollSet().signal();


//ROS_INFO("monitor publish msg: %s", msg.data.c_str());
    {
      boost::mutex::scoped_lock lock(added_connections_mutex_);
      S_ASyncXMLRPCConnection::iterator it = added_connections_.begin();
      S_ASyncXMLRPCConnection::iterator end = added_connections_.end();
      

for (; it != end; ++it)
      {
        (*it)->addToDispatch(m_server.get_dispatch());
        connections_.insert(*it);
      }

      added_connections_.clear();
    }

    // Update the XMLRPC server, blocking for at most 100ms in select()
    {
      boost::mutex::scoped_lock lock(functions_mutex_);
      

     m_server.work(0.1);
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
        (*it)->removeFromDispatch(m_server.get_dispatch());
        connections_.erase(*it);
      }

      removed_connections_.clear();
    }
  }

}

void Monitor::getPidCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue& result)
{
  result = xmlrpc::responseInt(1, "", (int)getpid());
}
void Monitor::requestTopicCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue& ret)
{
   string topic = params[1];
   XmlRpc::XmlRpcValue protos=params[2];

   //ROS_INFO("Requesting topic: %s", topic.c_str());

  for (int proto_idx = 0; proto_idx < protos.size(); proto_idx++)
  {
    XmlRpc::XmlRpcValue proto = protos[proto_idx]; // save typing
    if (proto.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
    	ROSCPP_LOG_DEBUG( "requestTopic protocol list was not a list of lists");
      return;
    }

    if (proto[0].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
    	ROSCPP_LOG_DEBUG( "requestTopic received a protocol list in which a sublist " \
                 "did not start with a string");
      return;
    }

    string proto_name = proto[0];
    if (proto_name == string("TCPROS"))
    {
      XmlRpc::XmlRpcValue tcpros_params;
      tcpros_params[0] = string("TCPROS");
      tcpros_params[1] = m_host;
      tcpros_params[2] = int(connection_manager_->getTCPPort());
      ret[0] = int(1);
      ret[1] = string();
      ret[2] = tcpros_params;

      ROS_INFO("TCP connection at port: %d", connection_manager_->getTCPPort());
    }
    /*else if (proto_name == string("UDPROS"))
    {
      if (proto.size() != 5 ||
          proto[1].getType() != XmlRpcValue::TypeBase64 ||
          proto[2].getType() != XmlRpcValue::TypeString ||
          proto[3].getType() != XmlRpcValue::TypeInt ||
          proto[4].getType() != XmlRpcValue::TypeInt)
      {
      	ROSCPP_LOG_DEBUG("Invalid protocol parameters for UDPROS");
        return false;
      }
      std::vector<char> header_bytes = proto[1];
      boost::shared_array<uint8_t> buffer = boost::shared_array<uint8_t>(new uint8_t[header_bytes.size()]);
      memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
      ros::Header h;
      string err;
      if (!h.parse(buffer, header_bytes.size(), err))
      {
      	ROSCPP_LOG_DEBUG("Unable to parse UDPROS connection header: %s", err.c_str());
        return false;
      }

      ros::PublicationPtr pub_ptr = lookupPublication(topic);
      if(!pub_ptr)
      {
      	ROSCPP_LOG_DEBUG("Unable to find advertised topic %s for UDPROS connection", topic.c_str());
        return false;
      }

      std::string host = proto[2];
      int port = proto[3];

      ros::M_string m;

      std::string error_msg;
      if (!pub_ptr->validateHeader(h, error_msg))
      {
        ROSCPP_LOG_DEBUG("Error validating header from [%s:%d] for topic [%s]: %s", host.c_str(), port, topic.c_str(), error_msg.c_str());
        return false;
      }

      int max_datagram_size = proto[4];
      int conn_id = connection_manager_->getNewConnectionID();
      ros::TransportUDPPtr transport = connection_manager_->getUDPServerTransport()->createOutgoing(host, port, conn_id, max_datagram_size);
      connection_manager_->udprosIncomingConnection(transport, h);

      XmlRpcValue udpros_params;
      udpros_params[0] = string("UDPROS");
      udpros_params[1] = getRVHost();
      udpros_params[2] = connection_manager_->getUDPServerTransport()->getServerPort();
      udpros_params[3] = conn_id;
      udpros_params[4] = max_datagram_size;
      m["topic"] = topic;
      m["md5sum"] = pub_ptr->getMD5Sum();
      m["type"] = pub_ptr->getDataType();
      m["callerid"] = m_node_name;
      m["message_definition"] = pub_ptr->getMessageDefinition();
      boost::shared_array<uint8_t> msg_def_buffer;
      uint32_t len;
      ros::Header::write(m, msg_def_buffer, len);
      XmlRpcValue v(msg_def_buffer.get(), len);
      udpros_params[5] = v;
      ret[0] = int(1);
      ret[1] = string();
      ret[2] = udpros_params;
      return true;
    }*/
    else
    {
      ROSCPP_LOG_DEBUG( "an unsupported protocol was offered: [%s]",
          proto_name.c_str());
    }
  }

  ROSCPP_LOG_DEBUG( "Currently, roscpp only supports TCPROS. The caller to " \
             "requestTopic did not support TCPROS, so there are no " \
             "protocols in common.");
}

/*
void Monitor::paramUpdateCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue &result)
{
  result[0] = 1;
  result[1] = std::string("");
  result[2] = 0;
  string param1 = params[1];
  string param2 = params[2];
 
  ros::param::update((std::string)params[1], params[2]);

ROS_INFO("parameter UPDATE %s -> %s", param1.c_str(), param2.c_str());


}*/
void Monitor::pubUpdateCallback(XmlRpc::XmlRpcValue& params, ClientInfo &ci, XmlRpc::XmlRpcValue &result)
{
for (int i = 0; i < params[2].size(); i++)
  {
      pub_uris.push_back(params[2][i]);
  }

ROS_INFO("publisher UPDATE %s", pub_uris[0].c_str());
sub_ptr->pubUpdate(pub_uris);


result = rv::xmlrpc::responseInt(1,"",0);
}

XmlRpcClient* Monitor::getXMLRPCClient(const std::string &host, const int port, const std::string &uri)
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

void Monitor::releaseXMLRPCClient(XmlRpcClient *c)
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



void Monitor::addASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(added_connections_mutex_);
  added_connections_.insert(conn);
}
void Monitor::removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(removed_connections_mutex_);
  removed_connections_.insert(conn);
}

}//namespace rv
