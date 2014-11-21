#ifndef RVCPP_RVMONITOR_H
#define RVCPP_RVMONITOR_H

#include "rv/xmlrpc_manager.h"
#include "rv/connection_manager.h"
#include "rv/server_manager.h"
#include "rv/subscription.h"
#include "ros/publication.h"
#include "std_msgs/String.h"
#include "ros/subscribe_options.h"
#include "ros/advertise_options.h"
#include "ros/callback_queue.h"
#include <rosgraph_msgs/Log.h>
#include <boost/scoped_ptr.hpp>
#include <ros/serialization.h>

#include "std_msgs/String.h"

namespace rv
{
    class RVMonitor
    {
        public:
            RVMonitor(std::string topic, ros::SubscribeOptions &ops_sub);
            ~RVMonitor();

            void monitorCallback_perfEval(const std_msgs::String::ConstPtr& monitored_msg);

        private:
            std::string topic_name;
            boost::shared_ptr<rv::ServerManager> server_manager;

    };
}

#endif
