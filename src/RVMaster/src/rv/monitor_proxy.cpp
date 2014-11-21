#include <ros/ros.h>
#include <msg_auth/msg_auth.h>
#include <landshark_msgs/SpoofStatus.h>
#include <landshark_msgs/SpoofStatusList.h>
#include "cmu_z_test_monitor/AnomalyMsg.h"

class AttackMonitorProxy
{
public:
  /**
   * Constructor
   */
  AttackMonitorProxy();

  /**
   * Destructor
   */
  ~AttackMonitorProxy();

  /**
   * Initialize the communication process
   */
  void Initialize();

private:
  void ProcessAttackMonitorMsg(const cmu_z_test_monitor::AnomalyMsgConstPtr& pAnomalyMsg);

  ros::NodeHandle m_NodeHandle;
  ros::Publisher m_SpoofStatusPublisher;
  ros::Subscriber m_SpoofStatusSubscriber;

  bool m_UseMsgAuth;
  msg_auth::MsgAuth m_MsgAuth;
  msg_auth::Publisher m_SpoofStatusAuthPublisher;

  std::vector<unsigned int> m_MonitoredSensors;
  landshark_msgs::SpoofStatusList m_NotSpoofedStatusList;
  landshark_msgs::SpoofStatusList m_CurrentSpoofStatusList;
  ros::Duration m_MaxUpdatePeriod;
};

AttackMonitorProxy::AttackMonitorProxy()
  : m_NodeHandle("~")
  , m_UseMsgAuth(true)
  , m_MsgAuth(m_NodeHandle)
{
  m_MonitoredSensors.push_back(landshark_msgs::SpoofStatus::WHEEL_ENCODER);
  m_MonitoredSensors.push_back(landshark_msgs::SpoofStatus::GPS);

  for (size_t i = 0; i < m_MonitoredSensors.size(); ++i)
  {
    landshark_msgs::SpoofStatus status;
    status.id = m_MonitoredSensors[i];
    status.spoofing_confidence = 0.0;
    status.monitor_name = std::string("cmu");

    m_NotSpoofedStatusList.status.push_back(status);
  }

  m_CurrentSpoofStatusList = m_NotSpoofedStatusList;
  m_CurrentSpoofStatusList.header.stamp = ros::Time::now();
}

AttackMonitorProxy::~AttackMonitorProxy()
{}

void AttackMonitorProxy::Initialize()
{
  double processRate, maxUpdatePeriod;
  std::string inputTopic, outputTopic;

  m_NodeHandle.param("process_rate", processRate, 10.0);
  m_NodeHandle.param("update_period_duration", maxUpdatePeriod, 2.0);
  m_NodeHandle.param("input_topic", inputTopic, std::string("/landshark/ExecutionMonitor/AnomalyMonitor"));
  m_NodeHandle.param("output_topic", outputTopic, std::string("/landshark_spoofing/status"));
  m_NodeHandle.param("use_msg_auth", m_UseMsgAuth, true);

  m_MaxUpdatePeriod = ros::Duration(maxUpdatePeriod);

  m_SpoofStatusSubscriber = m_NodeHandle.subscribe<cmu_z_test_monitor::AnomalyMsg>(inputTopic, 1, &AttackMonitorProxy::ProcessAttackMonitorMsg, this);

  if (m_UseMsgAuth)
  {
    m_SpoofStatusAuthPublisher = m_MsgAuth.advertise<landshark_msgs::SpoofStatusList>(outputTopic, 1);
  }
  else
  {
    m_SpoofStatusPublisher = m_NodeHandle.advertise<landshark_msgs::SpoofStatusList>(outputTopic, 1);
  }

  ros::Rate rate(processRate);

  while (m_NodeHandle.ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void AttackMonitorProxy::ProcessAttackMonitorMsg(const cmu_z_test_monitor::AnomalyMsgConstPtr& pAnomalyMsg)
{
  m_CurrentSpoofStatusList = m_NotSpoofedStatusList;
  m_CurrentSpoofStatusList.header.stamp = ros::Time::now();

  if (pAnomalyMsg->anomaly_detected)
  {
    for (unsigned int i = 0; i < m_MonitoredSensors.size(); ++i)
    {
      m_CurrentSpoofStatusList.status[i].spoofing_confidence = pAnomalyMsg->anomaly_value;
      m_CurrentSpoofStatusList.status[i].description = std::string("Odo/GPS") + pAnomalyMsg->anomaly_info;
    }

    if (m_UseMsgAuth)
    {
      m_SpoofStatusAuthPublisher.publish(m_CurrentSpoofStatusList);
    }
    else
    {
      m_SpoofStatusPublisher.publish(m_CurrentSpoofStatusList);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CmuAttackMonitorProxy");

  AttackMonitorProxy proxy;
  proxy.Initialize();

  return 0;
}
