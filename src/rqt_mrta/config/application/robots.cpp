#include <QStringList>
#include "rqt_mrta/config/application/robots.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Robots::Robots(QObject* parent) : AbstractConfig(parent) {}

Robots::~Robots()
{
  for (size_t index(0); index < robots_.count(); index++)
  {
    if (robots_[index])
    {
      delete robots_[index];
      robots_[index] = NULL;
    }
  }
  robots_.clear();
}

size_t Robots::count() const { return robots_.count(); }

Robot* Robots::getRobot(size_t index) const
{
  return index < robots_.count() ? robots_[index] : NULL;
}

Robot* Robots::addRobot()
{
  Robot* robot = new Robot(this);
  robots_.append(robot);
  connect(robot, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(robot, SIGNAL(idChanged(const QString&)), this,
          SLOT(robotIdChanged(const QString&)));
  connect(robot, SIGNAL(taskIdChanged(size_t, const QString&)), this,
          SLOT(taskIdChanged(size_t, const QString&)));
  connect(robot, SIGNAL(added(size_t)), this, SLOT(taskAdded(size_t)));
  connect(robot, SIGNAL(removed(const QString&)), this,
          SLOT(taskRemoved(const QString&)));
  connect(robot, SIGNAL(cleared()), this, SLOT(tasksCleared()));
  connect(robot, SIGNAL(destroyed()), this, SLOT(robotDestroyed()));
  emit added(robots_.count() - 1);
  emit changed();
  return robot;
}

void Robots::removeRobot(Robot* robot) { removeRobot(robots_.indexOf(robot)); }

void Robots::removeRobot(size_t index)
{
  if (index >= 0 && index < robots_.count())
  {
    QString robot_id(robots_[index]->getId());
    robots_.remove(index);
    emit removed(robot_id);
    emit changed();
  }
}

void Robots::clearRobots()
{
  if (!robots_.isEmpty())
  {
    for (size_t index(0); index < robots_.count(); index++)
    {
      if (robots_[index])
      {
        delete robots_[index];
        robots_[index] = NULL;
      }
    }
    robots_.clear();
    emit cleared();
    emit changed();
  }
}

bool Robots::contains(const QString& id) const
{
  for (size_t index(0); index < robots_.count(); index++)
  {
    if (robots_[index]->getId() == id)
    {
      return true;
    }
  }
  return false;
}

bool Robots::isEmpty() const { return robots_.isEmpty(); }

void Robots::save(QSettings& settings) const
{
  settings.beginGroup("robots");
  for (size_t index(0); index < robots_.count(); ++index)
  {
    settings.beginGroup("robot_" + QString::number(index));
    robots_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Robots::load(QSettings& settings)
{
  settings.beginGroup("robots");
  QStringList groups(settings.childGroups());
  size_t index(0);
  for (QStringList::iterator it(groups.begin()); it != groups.end(); it++)
  {
    Robot* robot =
        index < robots_.count() ? robot = robots_[index] : addRobot();
    settings.beginGroup(*it);
    robot->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < robots_.count())
  {
    removeRobot(index);
  }
}

void Robots::reset() { clearRobots(); }

void Robots::write(QDataStream& stream) const
{
  for (size_t index(0); index < robots_.count(); ++index)
  {
    robots_[index]->write(stream);
  }
}

void Robots::read(QDataStream& stream)
{
  for (size_t index(0); index < robots_.count(); index++)
  {
    robots_[index]->read(stream);
  }
}

Robots& Robots::operator=(const Robots& config)
{
  while (robots_.count() < config.robots_.count())
  {
    addRobot();
  }
  while (robots_.count() > config.robots_.count())
  {
    removeRobot(robots_.count() - 1);
  }
  for (size_t index(0); index < robots_.count(); ++index)
  {
    *robots_[index] = *config.robots_[index];
  }
  return *this;
}

QString Robots::validate() const
{
  if (robots_.isEmpty())
  {
    return "Enter the system robots.";
  }
  QString validation;
  for (size_t i(0); i < robots_.count(); i++)
  {
    validation = robots_[i]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

void Robots::taskIdChanged(size_t task_index, const QString& task_id)
{
  int index(robots_.indexOf(static_cast<Robot*>(sender())));
  if (index != -1)
  {
    emit taskIdChanged(index, task_index, task_id);
    emit changed();
  }
}

void Robots::taskAdded(size_t task_index)
{
  int index(robots_.indexOf(static_cast<Robot*>(sender())));
  if (index != -1)
  {
    emit taskAdded(index, task_index);
    emit changed();
  }
}

void Robots::taskRemoved(const QString& task_id)
{
  int index(robots_.indexOf(static_cast<Robot*>(sender())));
  if (index != -1)
  {
    emit taskRemoved(index, task_id);
    emit changed();
  }
}

void Robots::tasksCleared()
{
  int index(robots_.indexOf(static_cast<Robot*>(sender())));
  if (index != -1)
  {
    emit tasksCleared(index);
    emit changed();
  }
}

void Robots::robotIdChanged(const QString& robot_id)
{
  int index(robots_.indexOf(static_cast<Robot*>(sender())));
  if (index != -1)
  {
    emit robotIdChanged(index, robot_id);
    emit changed();
  }
}

void Robots::robotDestroyed()
{
  Robot* robot = static_cast<Robot*>(sender());
  int index(robots_.indexOf(robot));
  if (index >= 0)
  {
    QString robot_id(robot->getId());
    robots_.remove(index);
    emit removed(robot_id);
    emit changed();
  }
}
}
}
}
