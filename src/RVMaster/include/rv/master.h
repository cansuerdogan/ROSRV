
#ifndef ROSCPP_MASTER_H
#define ROSCPP_MASTER_H

#include "ros/forwards.h"
#include "XmlRpcValue.h"
#include "ros/common.h"

namespace rv
{

/**
 * \brief Contains functions which allow you to query information about the master
 */
namespace master
{


ROSCPP_DECL void init(const ros::M_string& remappings);


/** @brief Execute an XMLRPC call on the master
 *
 * @param method The RPC method to invoke
 * @param request The arguments to the RPC call
 * @param response [out] The resonse that was received.
 * @param payload [out] The payload that was received.
 * @param wait_for_master Whether or not this call should loop until it can contact the master
 *
 * @return true if call succeeds, false otherwise.
 */
ROSCPP_DECL bool execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master);

/** @brief Get the hostname where the master runs.
 *
 * @return The master's hostname, as a string
 */
ROSCPP_DECL const std::string& getHost();
/** @brief Get the port where the master runs.
 *
 * @return The master's port.
 */
ROSCPP_DECL uint32_t getPort();
/**
 * \brief Get the full URI to the master (eg. http://host:port/)
 */
ROSCPP_DECL const std::string& getURI();

/** @brief Check whether the master is up
 *
 * This method tries to contact the master.  You can call it any time
 * after ros::init has been called.  The intended usage is to check
 * whether the master is up before trying to make other requests
 * (subscriptions, advertisements, etc.).
 *
 * @returns true if the master is available, false otherwise.
 */
ROSCPP_DECL bool check();

/**
 * \brief Contains information retrieved from the master about a topic
 */
struct ROSCPP_DECL TopicInfo
{
  TopicInfo() {}
  TopicInfo(const std::string& _name, const std::string& _datatype /*, const std::string& _md5sum*/)
  : name(_name)
  , datatype(_datatype)
  //, md5sum(_md5sum)
  {}
  std::string name;        ///< Name of the topic
  std::string datatype;    ///< Datatype of the topic

  // not possible yet unfortunately (master does not have this information)
  //std::string md5sum;      ///< md5sum of the topic
};
typedef std::vector<TopicInfo> V_TopicInfo;

/** @brief Get the list of topics that are being published by all nodes.
 *
 * This method communicates with the master to retrieve the list of all
 * currently advertised topics.
 *
 * @param topics A place to store the resulting list.  Each item in the
 * list is a pair <string topic, string type>.  The type is represented
 * in the format "package_name/MessageName", and is also retrievable
 * through message.__getDataType() or MessageName::__s_getDataType().
 *
 * @return true on success, false otherwise (topics not filled in)
 */
ROSCPP_DECL bool getTopics(V_TopicInfo& topics);

/**
 * \brief Retreives the currently-known list of nodes from the master
 */
ROSCPP_DECL bool getNodes(ros::V_string& nodes);

/**
 * @brief Set the max time this node should spend looping trying to connect to the master
 * @param The timeout.  A negative value means infinite
 */
ROSCPP_DECL void setRetryTimeout(ros::WallDuration timeout);

} // namespace master

} // namespace rv

#endif

