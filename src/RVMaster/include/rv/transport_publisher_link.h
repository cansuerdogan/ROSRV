#ifndef RVCPP_TRANSPORT_PUBLISHER_LINK_H
#define RVCPP_TRANSPORT_PUBLISHER_LINK_H

#include "ros/common.h"
#include "rv/publisher_link.h"
#include "ros/connection.h"

namespace rv
{
class Header;
class Message;
class Subscription;
typedef boost::shared_ptr<Subscription> SubscriptionPtr;
typedef boost::weak_ptr<Subscription> SubscriptionWPtr;
class Connection;
typedef boost::shared_ptr<ros::Connection> ConnectionPtr;

class WallTimerEvent;

/**
 * \brief Handles a connection to a single publisher on a given topic.  Receives messages from a publisher
 * and hands them off to its parent Subscription
 */
class ROSCPP_DECL TransportPublisherLink : public PublisherLink
{
public:
  TransportPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const ros::TransportHints& transport_hints);
  virtual ~TransportPublisherLink();

  //
  bool initialize(const ros::ConnectionPtr& connection);

  const ConnectionPtr& getConnection() { return connection_; }

  virtual std::string getTransportType();
  virtual void drop();

private:
  void onConnectionDropped(const ros::ConnectionPtr& conn, ros::Connection::DropReason reason);
  bool onHeaderReceived(const ros::ConnectionPtr& conn, const ros::Header& header);

  /**
   * \brief Handles handing off a received message to the subscription, where it will be deserialized and called back
   */
  virtual void handleMessage(const ros::SerializedMessage& m, bool ser, bool nocopy);

  void onHeaderWritten(const ros::ConnectionPtr& conn);
  void onMessageLength(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);
  void onMessage(const ros::ConnectionPtr& conn, const boost::shared_array<uint8_t>& buffer, uint32_t size, bool success);

  void onRetryTimer(const ros::WallTimerEvent&);

  ros::ConnectionPtr connection_;

  int32_t retry_timer_handle_;
  bool needs_retry_;
  ros::WallDuration retry_period_;
  ros::WallTime next_retry_;
  bool dropping_;
};
typedef boost::shared_ptr<TransportPublisherLink> TransportPublisherLinkPtr;

} // namespace rv

#endif // RVCPP_TRANSPORT_PUBLISHER_LINK_H




