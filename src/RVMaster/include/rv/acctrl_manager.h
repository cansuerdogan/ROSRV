#ifndef RVCPP_ACCTRL_MANAGER_H
#define RVCPP_ACCTRL_MANAGER_H

#include "ros/forwards.h"
#include "ros/common.h"

using namespace std;

namespace rv
{
namespace acctrl
{
/** read access control configuration files*/
ROSCPP_DECL void init();

/* return a new port that is legal*/
ROSCPP_DECL int getNewPort();

/** return true if the hostname is allowed to execute the command*/
ROSCPP_DECL bool isCommandAllowed(const std::string& command, const std::string& nodename, const std::string& ip);

/** return true if the hostname is allowed to subscribe to the topic*/
ROSCPP_DECL bool isSubscriberAllowed(const std::string& topic, const std::string& nodename,const std::string& ip);

/** return true if the hostname is allowed to publish to the topic*/
ROSCPP_DECL bool isPublisherAllowed(const std::string& topic, const std::string& nodename, const std::string& ip);
} //namespace acctrl
}//namespace rv

#endif

