#include "mrta/log.h"
#include "mrta/robot_monitor.h"
#include "mrta/robot.h"
#include "mrta/system.h"
#include "rqt_mrta/config/architecture/topic.h"
#include "utilities/message_subscriber_registry.h"

namespace mrta
{
RobotMonitor::RobotMonitor(System* system,
                           utilities::MessageSubscriberRegistry* registry,
                           Monitor::Config* config, int state)
    : Monitor(system, registry, config), state_(state)
{
  QList<Robot*> robots(system->getRobots());
  for (size_t index(0); index < robots.count(); index++)
  {
    add(robots[index]);
  }
  connect(system, SIGNAL(added(const QString&)), this, SLOT(add(const QString&)));
}

RobotMonitor::~RobotMonitor()
{
  ROS_INFO_STREAM("[~RobotMonitor]");
}

void RobotMonitor::update(const QString &id)
{
  Monitor::update(id, state_);
}

void RobotMonitor::add(const QString& id) { add(getSystem()->getRobot(id)); }

void RobotMonitor::add(Robot* robot)
{
  if (robot)
  {
    addStateMonitor(robot->getId(), robot->getStateMonitor());
  }
}
}
