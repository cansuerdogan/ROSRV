#include "rvmonitor.h"

using namespace std;
namespace rv
{
    // Declarations of shared variables
    

    namespace monitor
    {
        std::set<std::string> monitorTopics;
        std::set<std::string> allMonitors;
        std::set<std::string> enabledMonitors;
        std::map<std::string,std::string> topicsAndTypes;

        void initMonitorTopics()
        {
            monitorTopics.insert("/chatter");
            topicsAndTypes["/chatter"] = "std_msgs/String";

            allMonitors.insert("perf");

        }

        void initAdvertiseOptions(std::string topic, ros::AdvertiseOptions &ops_pub)
        {
            if (topic == "/chatter") {
                ops_pub.init<std_msgs::String>(topic, 1000);
            }
        }

    }

    RVMonitor::RVMonitor(string topic, ros::SubscribeOptions &ops_sub)
    {
        topic_name = topic;
        server_manager = rv::ServerManager::instance();

        if (topic == "/chatter") {
            ops_sub.init<std_msgs::String>(topic, 1000, boost::bind(&RVMonitor::monitorCallback_perfEval, this, _1));
        }
    }

    void RVMonitor::monitorCallback_perfEval(const std_msgs::String::ConstPtr& monitored_msg)
    {

        std_msgs::String rv_msg;
        rv_msg.data = monitored_msg->data;



        if(monitor::enabledMonitors.find("perf") != monitor::enabledMonitors.end())
        {
		ROS_INFO("intercepted");
	}


        ros::SerializedMessage serializedMsg = ros::serialization::serializeMessage(rv_msg);
        server_manager->publish(topic_name, serializedMsg);
    }


}

