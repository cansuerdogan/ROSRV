#include "rv/acctrl_manager.h"
#include "rv/monitor.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>

#include <boost/thread/mutex.hpp>
#include <boost/config.hpp>
#include <boost/program_options/detail/config_file.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace pod = boost::program_options::detail;

namespace rv
{
namespace monitor
{
extern std::set<std::string> monitorTopics;

}

namespace acctrl
{

typedef std::map<std::string, ros::S_string> M_Acctrl;
boost::mutex ports_mutex_;

M_Acctrl map_cmds, map_subs, map_pubs, map_nodes;
bool enabled = false;
ros::S_string sub_defaults, pub_defaults, cmd_defaults, node_defaults;

int port_start, port_end, port_current;

int getNewPort()
{
  boost::mutex::scoped_lock lock(ports_mutex_);

   if(port_current<port_end)
   {
	port_current++; 
   }
   else
   {
       port_current = port_start;
   }

   return port_current;

}
void init()
{

//Just for test
/*

ros::S_string names;

names.insert("localhost");
names.insert("fslwork");
//names.insert("fslexp");

map_subs["/chatter"] = names;
map_pubs["/chatter"] = names;
*/

//read from configuration files

char* config_file_path = getenv("ACCESS_POLICY_PATH");

if(config_file_path)
  { 
    //std::string config_file_name = ros_workspace;
    //config_file_name = config_file_name+"/access-policy.cfg";
    std::ifstream config(config_file_path);
    if(!config)
    {
        std::cerr<<"Error of access policy file: "<<config_file_path<<std::endl;
    }
    else
    {
     enabled = true;

    ROS_INFO("---- access policies -----");
    
    std::set<std::string> options;
    options.insert("*");
    
    try
    {     

         M_Acctrl map_groups;

        for (pod::config_file_iterator i(config, options), e ; i != e; ++i)
        {
            std::string fullkey = i->string_key;
            std::string fullvalue = i->value[0];

            ROS_INFO("%s %s",fullkey.c_str(),fullvalue.c_str());
            std::vector<std::string> key_strs, value_strs;
            boost::split(key_strs, fullkey, boost::is_any_of("."));
            boost::split(value_strs, fullvalue, boost::is_any_of(" "));
            if(key_strs.size()==2)
            {

                 if(key_strs[0]=="ports"||key_strs[0]=="Ports") 
                 {
                       //std::string range = key_strs[1];
                      for(int k=0;k<value_strs.size();k++)
                      {
                            std::vector<std::string> port_strs;
                            boost::split(port_strs, value_strs[k], boost::is_any_of("-"));
                            
                            if(port_strs.size()==2)
                            {//start-end
                                port_current=port_start=boost::lexical_cast<int>(port_strs[0]);
                                port_end=boost::lexical_cast<int>(port_strs[1]);
                            }
                            else
                            {//handle single ports
                            }
                      }
                 }
                 else if(key_strs[0]=="monitor"||key_strs[0]=="Monitor") 
                 {
                       std::string name = key_strs[1];
                      if(name=="topic")
                      for(int k=0;k<value_strs.size();k++){
                          rv::monitor::monitorTopics.insert(value_strs[k]);
                          //rv::monitor::monitorTopics.insert(value_strs[k]+"/hmac");
                      }
                 }
                 else if(key_strs[0]=="groups"||key_strs[0]=="Groups") 
                 {
                       std::string group = key_strs[1];
                      for(int k=0;k<value_strs.size();k++)
                          map_groups[group].insert(value_strs[k]);
                 }
                 else if(key_strs[0]=="Commands") 
                 {
                      std::string cmd = key_strs[1];
                      for(int k=0;k<value_strs.size();k++)
                      {
                           M_Acctrl::iterator it = map_groups.find(value_strs[k]);
                          if(it!=map_groups.end())
                          {
                               for(ros::S_string::iterator si = it->second.begin();si!=it->second.end();si++) 
                               {    
                                   if(cmd=="default")
                                      cmd_defaults.insert(*si);
                                   else    
                                      map_cmds[cmd].insert(*si);
                                   //ROS_INFO("Commands-- %s %s",cmd.c_str(),(*si).c_str());
                               } 
                          }
                          else
                          {      
                               if(cmd=="default")
                                   cmd_defaults.insert(value_strs[k]); 
                               else
                                   map_cmds[cmd].insert(value_strs[k]);
                          } 
                     }
                 }
                 else if(key_strs[0]=="Nodes") 
                 {
                       std::string node = key_strs[1];
                      for(int k=0;k<value_strs.size();k++)
                      {
                           M_Acctrl::iterator it = map_groups.find(value_strs[k]);
                          if(it!=map_groups.end())
                          {
                               for(ros::S_string::iterator si = it->second.begin();si!=it->second.end();si++) 
                               {    
                                   if(node=="default")
                                       node_defaults.insert(*si);
                                   else
                                       map_nodes[node].insert(*si);
                               } 
                          }
                          else
                          {
                              if(node=="default")
                                  node_defaults.insert(value_strs[k]);
                              else
                                  map_nodes[node].insert(value_strs[k]);
                          }
                     }
                 }
                 else if(key_strs[0]=="Publishers") 
                 {
                       std::string topic = key_strs[1];
                      for(int k=0;k<value_strs.size();k++)
                      {
                           M_Acctrl::iterator it = map_groups.find(value_strs[k]);
                          if(it!=map_groups.end())
                          {
                               for(ros::S_string::iterator si = it->second.begin();si!=it->second.end();si++) 
                               {    
                                   if(topic=="default")
                                       pub_defaults.insert(*si);
                                   else
                                       map_pubs[topic].insert(*si);
                                   //ROS_INFO("Publishers-- %s %s",topic.c_str(),(*si).c_str());
                               } 
                          }
                          else
                          {
                              if(topic=="default")
                                  pub_defaults.insert(value_strs[k]);
                              else
                                  map_pubs[topic].insert(value_strs[k]);
                          }
                     }
                 }
                 else if(key_strs[0]=="Subscribers") 
                 {
                       std::string topic = key_strs[1];
                      for(int k=0;k<value_strs.size();k++)
                      {
                           M_Acctrl::iterator it = map_groups.find(value_strs[k]);
                          if(it!=map_groups.end())
                          {
                               for(ros::S_string::iterator si = it->second.begin();si!=it->second.end();si++) 
                               {    
                                   if(topic=="default")
                                       sub_defaults.insert(*si);
                                   else
                                       map_subs[topic].insert(*si);
                                   //ROS_INFO("Subscribers-- %s %s",topic.c_str(),(*si).c_str());
                               } 
                          }
                          else
                          {    
                               if(topic=="default")
                                   sub_defaults.insert(value_strs[k]);
                               else
                                   map_subs[topic].insert(value_strs[k]);
                          }
                     }
                 }
             }
        }
    }
    catch(std::exception& e)    
    {
        std::cerr<<"Exception: "<<e.what()<<std::endl;
    }
   }
  }

if(!enabled)
std::cerr<<"NO ACCESS CONTROL\n";



}

bool prefix(const std::string& a, const std::string& b) {
  if (a.size() > b.size()) {
    //return a.substr(0,b.size()) == b;
	return false;
  }
  else {
    return b.substr(0,a.size()) == a;
  }
}
bool isNodeAllowed(const std::string& name, const std::string& ip)
{


   //ROS_INFO("isNodeAllowed %s %s",name.c_str(),ip.c_str());

    if(node_defaults.find(ip)!=node_defaults.end())
        return true;

    for(M_Acctrl::iterator it = map_nodes.begin();it!=map_nodes.end();++it)
    {
       if(prefix(it->first,name))
       {
         ros::S_string ips = map_nodes[it->first];
         if(ips.find(ip)!=ips.end())
            return true;
       }
    }
    
    return false;  
}

bool isCommandAllowed(const std::string& command, const std::string& name, const std::string& ip)
{

if(!enabled)
   return true;

if(!isNodeAllowed(name,ip))
return false;

if(cmd_defaults.find(ip)!=cmd_defaults.end())
return true;

if(map_cmds.find(command) !=map_cmds.end())
{
ros::S_string ips = map_cmds[command];
if(ips.find(ip)!= ips.end())
return true;
}

return false;
} 

bool isSubscriberAllowed(const std::string& topic, const std::string& name, const std::string& ip)
{

if(!enabled) 
return true;

if(!isNodeAllowed(name,ip))
return false;

if(sub_defaults.find(ip)!=sub_defaults.end())
return true;

for(M_Acctrl::iterator it = map_subs.begin();it!=map_subs.end();++it)
{
   if(prefix(it->first,topic))
   {
       ros::S_string ips = map_subs[it->first];
       if(ips.find(ip)!=ips.end())
            return true;
   }
}
return false;
} 

bool isPublisherAllowed(const std::string& topic, const std::string& name, const std::string& ip)
{

if(!enabled) 
return true;

if(!isNodeAllowed(name,ip))
return false;

if(pub_defaults.find(ip)!=pub_defaults.end())
return true;

for(M_Acctrl::iterator it = map_pubs.begin();it!=map_pubs.end();++it)
{
   if(prefix(it->first,topic))   //it->first==topic||(topic.substr(topic.length()-5)!="/hmac"&&
   {
       ros::S_string ips = map_pubs[it->first];
       if(ips.find(ip)!=ips.end())
            return true;
   }
}

return false;
} 
}//namespace acctrl

}//namespace rv
