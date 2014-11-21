#include "rv/connection_manager.h"
#include "rv/acctrl_manager.h"
#include "ros/poll_manager.h"
#include "ros/connection.h"
#include "rv/transport_subscriber_link.h"
#include "ros/service_client_link.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_udp.h"
#include "ros/file_log.h"
#include "ros/network.h"

#include <ros/assert.h>

namespace rv
{

ConnectionManagerPtr g_connection_manager;
boost::mutex g_connection_manager_mutex;
const ConnectionManagerPtr& ConnectionManager::instance()
{
  if (!g_connection_manager)
  {
    boost::mutex::scoped_lock lock(g_connection_manager_mutex);
    if (!g_connection_manager)
    {
      g_connection_manager.reset(new ConnectionManager);
    }
  }

  return g_connection_manager;
}

ConnectionManager::ConnectionManager()
: connection_id_counter_(0)
{
}

ConnectionManager::~ConnectionManager()
{
  shutdown();
}
ros::PollManagerPtr ConnectionManager::getPollManagerPtr()
{
return poll_manager_;
}

void ConnectionManager::start()
{
  //poll_manager_ = ros::PollManager::instance();
   poll_manager_.reset(new ros::PollManager); 

   poll_manager_->start();



  poll_conn_ = poll_manager_->addPollThreadListener(boost::bind(&ConnectionManager::removeDroppedConnections, 
								this));

  // Bring up the TCP listener socket
  tcpserver_transport_ = ros::TransportTCPPtr(new ros::TransportTCP(&poll_manager_->getPollSet()));
  int port = rv::acctrl::getNewPort();
  while (!tcpserver_transport_->listen(port, 
				    MAX_TCPROS_CONN_QUEUE, 
				    boost::bind(&ConnectionManager::tcprosAcceptConnection, this, _1)))
  {
    //ROS_FATAL("Listen on port [%d] failed", ros::network::getTCPROSPort());
    //ROS_BREAK();
     port = rv::acctrl::getNewPort();
  }
     //ROS_INFO("test: tcp connection started at tcp port: %d",ros::network::getTCPROSPort());

  // Bring up the UDP listener socket
  udpserver_transport_ = ros::TransportUDPPtr(new ros::TransportUDP(&poll_manager_->getPollSet()));
  if (!udpserver_transport_->createIncoming(0, true))
  {
    ROS_FATAL("Listen failed");
    ROS_BREAK();
  }
}

void ConnectionManager::shutdown()
{
  if (udpserver_transport_)
  {
    udpserver_transport_->close();
    udpserver_transport_.reset();
  }

  if (tcpserver_transport_)
  {
    tcpserver_transport_->close();
    tcpserver_transport_.reset();
  }

  poll_manager_->removePollThreadListener(poll_conn_);

  clear(ros::Connection::Destructing);
}

void ConnectionManager::clear(ros::Connection::DropReason reason)
{
  ros::S_Connection local_connections;
  {
    boost::mutex::scoped_lock conn_lock(connections_mutex_);
    local_connections.swap(connections_);
  }

  for(ros::S_Connection::iterator itr = local_connections.begin();
      itr != local_connections.end();
      itr++)
  {
    const ros::ConnectionPtr& conn = *itr;
    conn->drop(reason);
  }

  boost::mutex::scoped_lock dropped_lock(dropped_connections_mutex_);
  dropped_connections_.clear();
}

uint32_t ConnectionManager::getTCPPort()
{
  return tcpserver_transport_->getServerPort();
}

uint32_t ConnectionManager::getUDPPort()
{
  return udpserver_transport_->getServerPort();
}

uint32_t ConnectionManager::getNewConnectionID()
{
  boost::mutex::scoped_lock lock(connection_id_counter_mutex_);
  uint32_t ret = connection_id_counter_++;
  return ret;
}

void ConnectionManager::addConnection(const ros::ConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(connections_mutex_);

  connections_.insert(conn);
  conn->addDropListener(boost::bind(&ConnectionManager::onConnectionDropped, this, _1));
}

void ConnectionManager::onConnectionDropped(const ros::ConnectionPtr& conn)
{
  boost::mutex::scoped_lock lock(dropped_connections_mutex_);
  dropped_connections_.push_back(conn);
}

void ConnectionManager::removeDroppedConnections()
{
  ros::V_Connection local_dropped;
  {
    boost::mutex::scoped_lock dropped_lock(dropped_connections_mutex_);
    dropped_connections_.swap(local_dropped);
  }

  boost::mutex::scoped_lock conn_lock(connections_mutex_);

  ros::V_Connection::iterator conn_it = local_dropped.begin();
  ros::V_Connection::iterator conn_end = local_dropped.end();
  for (;conn_it != conn_end; ++conn_it)
  {
    const ros::ConnectionPtr& conn = *conn_it;
    connections_.erase(conn);
  }
}

void ConnectionManager::udprosIncomingConnection(const ros::TransportUDPPtr& transport, ros::Header& header)
{
  std::string client_uri = ""; // TODO: transport->getClientURI();
  ROSCPP_LOG_DEBUG("UDPROS received a connection from [%s]", client_uri.c_str());

  ros::ConnectionPtr conn(new ros::Connection());
  addConnection(conn);

  conn->initialize(transport, true, NULL);
  onConnectionHeaderReceived(conn, header);
}

void ConnectionManager::tcprosAcceptConnection(const ros::TransportTCPPtr& transport)
{
  std::string client_uri = transport->getClientURI();
  ROSCPP_LOG_DEBUG("TCPROS received a connection from [%s]", client_uri.c_str());

  ros::ConnectionPtr conn(new ros::Connection());
  addConnection(conn);

  conn->initialize(transport, true, boost::bind(&ConnectionManager::onConnectionHeaderReceived, this, _1, _2));
}

bool ConnectionManager::onConnectionHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header)
{

//ROS_INFO("client TCP connection received!");

  bool ret = false;
  std::string val;
  if (header.getValue("topic", val))
  {
    ROSCPP_LOG_DEBUG("Connection: Creating TransportSubscriberLink for topic [%s] connected to [%s]", 
		     val.c_str(), conn->getRemoteString().c_str());

    TransportSubscriberLinkPtr sub_link(new TransportSubscriberLink());
    sub_link->initialize(conn);
    ret = sub_link->handleHeader(header);



  }
  else if (header.getValue("service", val))
  {
    ROSCPP_LOG_DEBUG("Connection: Creating ServiceClientLink for service [%s] connected to [%s]", 
		     val.c_str(), conn->getRemoteString().c_str());

    //does not handle service

    //ServiceClientLinkPtr link(new ServiceClientLink());
    //link->initialize(conn);
    //ret = link->handleHeader(header);
  }
  else
  {
  	ROSCPP_LOG_DEBUG("Got a connection for a type other than 'topic' or 'service' from [%s].  Fail.", 
			 conn->getRemoteString().c_str());
    return false;
  }

  return ret;
}

}//rv namespace

