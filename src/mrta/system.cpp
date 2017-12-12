#include "mrta/allocation.h"
#include "mrta/problem.h"
#include "mrta/robot.h"
#include "mrta/robot_monitor.h"
#include "mrta/task.h"
#include "mrta/system.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "utilities/exception.h"
#include "utilities/message_subscriber_registry.h"

namespace mrta
{
System::System(QObject* parent, ApplicationConfig* application_config,
               ArchitectureConfig* architecture_config,
               utilities::MessageSubscriberRegistry* registry)
    : QObject(parent), application_config_(application_config),
      architecture_config_(architecture_config),
      problem_(new Problem(this,
                           application_config->getApplication()->getName(),
                           architecture_config))
{
  if (!application_config_)
  {
    throw utilities::Exception(
        "The application configuration must not be null.");
  }
  if (!architecture_config_)
  {
    throw utilities::Exception(
        "The architecture configuration must not be null.");
  }
  monitors_.append(new RobotMonitor(
      this, registry,
      architecture_config->getArchitecture()->getRobots()->getBusyRobots()->getTopic(),
      Robot::Busy));
  monitors_.append(new RobotMonitor(
      this, registry,
      architecture_config->getArchitecture()->getRobots()->getIdleRobots()->getTopic(),
      Robot::Idle));
  connect(problem_, SIGNAL(taskStateChanged(const QString&, int)), this,
          SIGNAL(changed()));
  connect(problem_, SIGNAL(taskStateChanged(const QString&, int)), this,
          SIGNAL(taskStateChanged(const QString&, int)));
  connect(problem_, SIGNAL(allocationStateChanged(const QString&, int)), this,
          SIGNAL(changed()));
  connect(problem_, SIGNAL(allocationStateChanged(const QString&, int)), this,
          SIGNAL(allocationStateChanged(const QString&, int)));
  rqt_mrta::config::application::Robots* configs =
      application_config_->getApplication()->getRobots();
  for (size_t index(0); index < configs->count(); index++)
  {
    addRobot(configs->getRobot(index));
  }
}

System::~System()
{
  ROS_INFO_STREAM("[~System] before");
  application_config_ = NULL;
  architecture_config_ = NULL;
  for (size_t index(0); index < monitors_.count(); index++)
  {
    if (monitors_[index])
    {
      delete monitors_[index];
      monitors_[index] = NULL;
    }
  }
  monitors_.clear();
  if (problem_)
  {
    delete problem_;
    problem_ = NULL;
  }
  for (RobotMap::iterator it(robots_.begin()); it != robots_.end(); it++)
  {
    if (it.value())
    {
      delete it.value();
      robots_[it.key()] = NULL;
    }
  }
  robots_.clear();
  ROS_INFO_STREAM("[~System] after");
}

Robot* System::getRobot(const QString& id)
{
  return robots_.contains(id) ? robots_[id] : NULL;
}

Task* System::getTask(const QString& id) { return problem_->getTask(id); }

Allocation* System::getAllocation(const QString& id)
{
  return problem_->getAllocation(id);
}

QList<Robot*> System::getRobots() const { return robots_.values(); }

QList<Task*> System::getTasks() const { return problem_->getTasks(); }

QList<Allocation*> System::getAllocations() const
{
  return problem_->getAllocations();
}

void System::setRegistry(utilities::MessageSubscriberRegistry* registry)
{
  for (size_t index(0); index < monitors_.count(); index++)
  {
    monitors_[index]->setRegistry(registry);
  }
}

Robot* System::addRobot(RobotConfig* config)
{
  if (robots_.contains(config->getId()))
  {
    return robots_[config->getId()];
  }
  Robot* robot = new Robot(this, config);
  robots_[robot->getId()] = robot;
  emit added(robot->getId());
  emit changed();
}

void System::robotStateChanged(int state)
{
  Robot* robot = static_cast<Robot*>(sender());
  if (robot)
  {
    emit robotStateChanged(robot->getId(), state);
    emit changed();
  }
}
}
