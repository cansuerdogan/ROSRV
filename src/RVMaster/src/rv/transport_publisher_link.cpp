#include <ros/platform.h>  // platform dependendant requirements

#include "rv/transport_publisher_link.h"
#include "rv/subscription.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "rv/connection_manager.h"
#include "ros/file_log.h"
#include "ros/poll_manager.h"
#include "ros/transport/transport_tcp.h"
#include "ros/timer_manager.h"
#include "ros/callback_queue.h"
#include "ros/internal_timer_manager.h"

#include <boost/bind.hpp>

#include <sstream>

namespace rv
{

static ros::CallbackQueuePtr g_internal_callback_queue;

TransportPublisherLink::TransportPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const ros::TransportHints& transport_hints)
: PublisherLink(parent, xmlrpc_uri, transport_hints)
, retry_timer_handle_(-1)
, needs_retry_(false)
, dropping_(false)
{
}

TransportPublisherLink::~TransportPublisherLink()
{
  dropping_ = true;

  if (retry_timer_handle_ != -1)
  {
    ros::getInternalTimerManager()->remove(retry_timer_handle_);
  }

  connection_->drop(ros::Connection::Destructing);
}

bool TransportPublisherLink::initialize(const ros::ConnectionPtr& connection)
{
  connection_ = connection;
  connection_->addDropListener(boost::bind(&TransportPublisherLink::onConnectionDropped, this, _1, _2));

  if (connection_->getTransport()->requiresHeader())
  {
    connection_->setHeaderReceivedCallback(boost::bind(&TransportPublisherLink::onHeaderReceived, this, _1, _2));

    SubscriptionPtr parent = parent_.lock();

    ros::M_string header;
    header["topic"] = parent->getName();
    header["md5sum"] = parent->md5sum();
    header["callerid"] = ros::this_node::getName();
    header["type"] = parent->datatype();
    header["tcp_nodelay"] = transport_hints_.getTCPNoDelay() ? "1" : "0";
    connection_->writeHeader(header, boost::bind(&TransportPublisherLink::onHeaderWritten, this, _1));
  }
  else
  {
    connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));
  }

  return true;
}

void TransportPublisherLink::drop()
{
  dropping_ = true;
  connection_->drop(ros::Connection::Destructing);

  if (SubscriptionPtr parent = parent_.lock())
  {
    parent->removePublisherLink(shared_from_this());
  }
}

void TransportPublisherLink::onHeaderWritten(const ros::ConnectionPtr& conn)
{
  // Do nothing
}

bool TransportPublisherLink::onHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header)
{
  ROS_ASSERT(conn == connection_);

  if (!setHeader(header))
  {
    drop();
    return false;
  }

  if (retry_timer_handle_ != -1)
  {
    ros::getInternalTimerManager()->remove(retry_timer_handle_);
    retry_timer_handle_ = -1;
  }

  connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));

  return true;
}

void TransportPublisherLink::onMessageLength(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  if (retry_timer_handle_ != -1)
  {
    ros::getInternalTimerManager()->remove(retry_timer_handle_);
    retry_timer_handle_ = -1;
  }

  if (!success)
  {
    if (connection_)
      connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));
    return;
  }

  ROS_ASSERT(conn == connection_);
  ROS_ASSERT(size == 4);

  uint32_t len = *((uint32_t*)buffer.get());

  if (len > 1000000000)
  {
    ROS_ERROR("a message of over a gigabyte was " \
                "predicted in tcpros. that seems highly " \
                "unlikely, so I'll assume protocol " \
                "synchronization is lost.");
    drop();
    return;
  }

  connection_->read(len, boost::bind(&TransportPublisherLink::onMessage, this, _1, _2, _3, _4));
}

void TransportPublisherLink::onMessage(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success)
{
  if (!success && !conn)
    return;

  ROS_ASSERT(conn == connection_);

  if (success)
  {
    handleMessage(ros::SerializedMessage(buffer, size), true, false);
  }

  if (success || !connection_->getTransport()->requiresHeader())
  {
    connection_->read(4, boost::bind(&TransportPublisherLink::onMessageLength, this, _1, _2, _3, _4));
  }
}

void TransportPublisherLink::onRetryTimer(const ros::WallTimerEvent&)
{
  if (dropping_)
  {
    return;
  }

  if (needs_retry_ && ros::WallTime::now() > next_retry_)
  {
    retry_period_ = std::min(retry_period_ * 2, ros::WallDuration(20));
    needs_retry_ = false;
    SubscriptionPtr parent = parent_.lock();
    // TODO: support retry on more than just TCP
    // For now, since UDP does not have a heartbeat, we do not attempt to retry
    // UDP connections since an error there likely means some invalid operation has
    // happened.
    if (connection_->getTransport()->getType() == std::string("TCPROS"))
    {
      std::string topic = parent ? parent->getName() : "unknown";

      ros::TransportTCPPtr old_transport = boost::dynamic_pointer_cast<ros::TransportTCP>(connection_->getTransport());
      ROS_ASSERT(old_transport);
      const std::string& host = old_transport->getConnectedHost();
      int port = old_transport->getConnectedPort();

      ROSCPP_LOG_DEBUG("Retrying connection to [%s:%d] for topic [%s]", host.c_str(), port, topic.c_str());

ROS_INFO("here in transport publisher link");

      ros::TransportTCPPtr transport(new ros::TransportTCP(&ros::PollManager::instance()->getPollSet()));
      if (transport->connect(host, port))
      {
        ros::ConnectionPtr connection(new ros::Connection);
        connection->initialize(transport, false, ros::HeaderReceivedFunc());
        initialize(connection);

        ConnectionManager::instance()->addConnection(connection);
      }
      else
      {
        ROSCPP_LOG_DEBUG("connect() failed when retrying connection to [%s:%d] for topic [%s]", host.c_str(), port, topic.c_str());
      }
    }
    else if (parent)
    {
      parent->removePublisherLink(shared_from_this());
    }
  }
}

ros::CallbackQueuePtr getInternalCallbackQueue()
{
  if (!g_internal_callback_queue)
  {
    g_internal_callback_queue.reset(new ros::CallbackQueue);
  }

  return g_internal_callback_queue;
}

void TransportPublisherLink::onConnectionDropped(const ros::ConnectionPtr& conn, ros::Connection::DropReason reason)
{
  if (dropping_)
  {
    return;
  }

  ROS_ASSERT(conn == connection_);

  SubscriptionPtr parent = parent_.lock();

  if (reason == ros::Connection::TransportDisconnect)
  {
    std::string topic = parent ? parent->getName() : "unknown";

    ROSCPP_LOG_DEBUG("Connection to publisher [%s] to topic [%s] dropped", connection_->getTransport()->getTransportInfo().c_str(), topic.c_str());

    ROS_ASSERT(!needs_retry_);
    needs_retry_ = true;
    next_retry_ = ros::WallTime::now() + retry_period_;

    if (retry_timer_handle_ == -1)
    {
      retry_period_ = ros::WallDuration(0.1);
      next_retry_ = ros::WallTime::now() + retry_period_;
      retry_timer_handle_ = ros::getInternalTimerManager()->add(ros::WallDuration(retry_period_),
          boost::bind(&TransportPublisherLink::onRetryTimer, this, _1), getInternalCallbackQueue().get(),
          ros::VoidConstPtr(), false);
    }
    else
    {
      ros::getInternalTimerManager()->setPeriod(retry_timer_handle_, retry_period_);
    }
  }
  else
  {
    drop();
  }
}

void TransportPublisherLink::handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy)
{
  stats_.bytes_received_ += m.num_bytes;
  stats_.messages_received_++;

  SubscriptionPtr parent = parent_.lock();

  if (parent)
  {
    stats_.drops_ += parent->handleMessage(m, ser, nocopy, getConnection()->getHeader().getValues(), shared_from_this());
  }
}

std::string TransportPublisherLink::getTransportType()
{
  return connection_->getTransport()->getType();
}

} // namespace rv


