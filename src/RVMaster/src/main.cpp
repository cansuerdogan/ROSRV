#include "ros/console.h"
#include "rv/xmlrpc_manager.h"
#include "rv/server_manager.h"
#include "rv/master.h"

#include "ros/duration.h"
#include <string>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{

boost::shared_ptr<rv::XMLRPCManager> xmlrpc_manager_ = rv::XMLRPCManager::instance();
boost::shared_ptr<rv::ServerManager> server_manager_ = rv::ServerManager::instance();

//rv::master::init(argc,argv,"rvmaster");
ros::M_string remappings;
rv::master::init(remappings);

//for testing only
//xmlrpc_manager_->bind("testxml",boost::bind(&rv::ServerManager::testxmlCallback,server_manager_,_1,_2,_3));

//--- ros master api

//subscriber and publisher
xmlrpc_manager_->bind("registerSubscriber",boost::bind(&rv::ServerManager::registerSubscriberCallback,server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("unregisterSubscriber",boost::bind(&rv::ServerManager::unregisterSubscriberCallback,server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("registerPublisher",boost::bind(&rv::ServerManager::registerPublisherCallback,server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("unregisterPublisher",boost::bind(&rv::ServerManager::unregisterPublisherCallback,server_manager_,_1,_2,_3));
//xmlrpc_manager_->bind("publisherUpdate", boost::bind(&rv::ServerManager::pubUpdateCallback, server_manager_, _1, _2,_3));

//other commands
xmlrpc_manager_->bind("lookupNode", boost::bind(&rv::ServerManager::lookupNodeCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getSystemState", boost::bind(&rv::ServerManager::getSystemStateCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getPid", boost::bind(&rv::ServerManager::getPidCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getUri", boost::bind(&rv::ServerManager::getUriCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getPublishedTopics", boost::bind(&rv::ServerManager::getPublishedTopicsCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getTopicTypes", boost::bind(&rv::ServerManager::getTopicTypesCallback, server_manager_,_1,_2,_3));

//service
xmlrpc_manager_->bind("registerService", boost::bind(&rv::ServerManager::registerServiceCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("unregisterService", boost::bind(&rv::ServerManager::unregisterServiceCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("lookupService", boost::bind(&rv::ServerManager::lookupServiceCallback, server_manager_,_1,_2,_3));

//parameter server
xmlrpc_manager_->bind("unsubscribeParam", boost::bind(&rv::ServerManager::unsubscribeParamCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("subscribeParam", boost::bind(&rv::ServerManager::subscribeParamCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("deleteParam", boost::bind(&rv::ServerManager::deleteParamCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getParam", boost::bind(&rv::ServerManager::getParamCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getParamNames", boost::bind(&rv::ServerManager::getParamNamesCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("setParam", boost::bind(&rv::ServerManager::setParamCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("searchParam", boost::bind(&rv::ServerManager::searchParamCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("hasParam", boost::bind(&rv::ServerManager::hasParamCallback, server_manager_,_1,_2,_3));

//for debugging
xmlrpc_manager_->bind("getRVState", boost::bind(&rv::ServerManager::getRVStateCallback, server_manager_,_1,_2,_3));
xmlrpc_manager_->bind("getMonitors", boost::bind(&rv::ServerManager::getMonitorsCallback, server_manager_,_1,_2,_3));

//for controlling monitors
xmlrpc_manager_->bind("monitorControl", boost::bind(&rv::ServerManager::monitorControlCallback, server_manager_,_1,_2,_3));

xmlrpc_manager_->start();
server_manager_->start();

ros::WallDuration(7*24*3600).sleep();
}
