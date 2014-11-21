#ifndef RVCPP_TRANSPORT_SUBSCRIBER_LINK_H
#define RVCPP_TRANSPORT_SUBSCRIBER_LINK_H

#include "ros/common.h"
#include "ros/subscriber_link.h"

#include <boost/signals/connection.hpp>

namespace rv
{

/**
 * \brief SubscriberLink handles broadcasting messages to a single subscriber on a single topic
 */
class ROSCPP_DECL TransportSubscriberLink : public ros::SubscriberLink
{
public:
  TransportSubscriberLink();
  virtual ~TransportSubscriberLink();

  //
  bool initialize(const ros::ConnectionPtr& connection);
  bool handleHeader(const ros::Header& header);

  const ros::ConnectionPtr& getConnection() { return connection_; }

  virtual void enqueueMessage(const ros::SerializedMessage& m, bool ser, bool nocopy);
  virtual void drop();
  virtual std::string getTransportType();

private:
  void onConnectionDropped(const ros::ConnectionPtr& conn);

  void onHeaderWritten(const ros::ConnectionPtr& conn);
  void onMessageWritten(const ros::ConnectionPtr& conn);
  void startMessageWrite(bool immediate_write);

  bool writing_message_;
  bool header_written_;

  ros::ConnectionPtr connection_;
  boost::signals::connection dropped_conn_;

  std::queue<ros::SerializedMessage> outbox_;
  boost::mutex outbox_mutex_;
  bool queue_full_;
};
typedef boost::shared_ptr<TransportSubscriberLink> TransportSubscriberLinkPtr;

} // namespace rv

#endif // RVCPP_TRANSPORT_SUBSCRIBER_LINK_H

