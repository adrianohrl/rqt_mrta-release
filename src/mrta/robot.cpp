#include "mrta/history.h"
#include "mrta/robot.h"
#include "mrta/sample_holder.h"
#include "mrta/state_monitor.h"
#include "mrta/system.h"
#include "mrta/task.h"
#include "rqt_mrta/config/application/robot.h"

namespace mrta
{
Robot::Robot(System* parent, Config* config)
    : QObject(parent), config_(NULL), state_(Offline),
      monitor_(new StateMonitor(this, ros::Duration(5.0), STATE_COUNT)),
      history_(new History(this))
{
  setConfig(config);
  connect(monitor_, SIGNAL(updated(size_t, bool)), this,
          SLOT(monitorUpdated(size_t, bool)));
}

Robot::Robot(const Robot& robot)
    : QObject(robot.parent()), config_(NULL), id_(robot.id_), tasks_(robot.tasks_),
      state_(robot.state_), history_(robot.history_), monitor_(robot.monitor_)
{
  setConfig(robot.config_);
}

Robot::~Robot()
{
  ROS_INFO_STREAM("[~Robot] before");
  config_ = NULL;
  if (history_)
  {
    delete history_;
    history_ = NULL;
  }
  if (monitor_)
  {
    delete monitor_;
    monitor_ = NULL;
  }
  tasks_.clear();
  ROS_INFO_STREAM("[~Robot] after");
}

Robot::Config* Robot::getConfig() const { return config_; }

void Robot::setConfig(Robot::Config* config)
{
  if (config != config_)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      disconnect(config_, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
      disconnect(config_, SIGNAL(idChanged(const QString&)), this,
                 SLOT(setId(const QString&)));
    }
    config_ = config;
    if (config_)
    {
      connect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(config_, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
      connect(config_, SIGNAL(idChanged(const QString&)), this,
              SLOT(setId(const QString&)));
      setId(config_->getId());
      clearTasks();
      for (size_t index(0); index < config->getTasks()->count(); index++)
      {
        addTask(new Task(this, config->getTasks()->getTask(index)));
      }
    }
  }
}

QString Robot::getId() const { return id_; }

Robot::Type Robot::getType() const { return type_; }

Robot::State Robot::getState() const { return state_; }

History* Robot::getHistory() const { return history_; }

StateMonitor* Robot::getStateMonitor() const { return monitor_; }

void Robot::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

void Robot::setState(Robot::State state)
{
  if (state != state_)
  {
    state_ = state;
    Log::Severity severity;
    switch (state_)
    {
    case Idle:
      severity = Log::Info;
      emit idle();
      break;
    case Busy:
      severity = Log::Info;
      emit busy();
      break;
    case Offline:
      severity = Log::Warn;
      emit offline();
      break;
    }
    history_->log(ros::Time::now(), Log::Robot, severity, id_, state_);
    emit stateChanged(state);
    emit changed();
  }
}

size_t Robot::count() const { return tasks_.count(); }

Task* Robot::getTask(int index) const { return tasks_[index]; }

void Robot::addTask(Task* task)
{
  tasks_.append(task);
  connect(task, SIGNAL(changed()), this, SLOT(changed()));
  connect(task, SIGNAL(changed()), this, SLOT(taskChanged()));
  connect(task, SIGNAL(idChanged(const QString&)), this,
          SLOT(taskIdChanged(const QString&)));
  connect(task, SIGNAL(destroyed()), this, SLOT(taskDestroyed()));
  emit added(tasks_.count() - 1);
  emit changed();
}

void Robot::removeTask(Task* task)
{
  size_t index(tasks_.indexOf(task));
  QString task_id(task->getId());
  tasks_.remove(index);
  emit removed(task_id);
  emit changed();
}

void Robot::clearTasks() { tasks_.clear(); }

Robot& Robot::operator=(const Robot& robot)
{
  id_ = robot.id_;
  tasks_ = robot.tasks_;
  emit changed();
}

void Robot::configDestroyed() { config_ = NULL; }

void Robot::taskDestroyed()
{
  Task* task = static_cast<Task*>(sender());
  int index(tasks_.indexOf(task));
  if (index != -1)
  {
    QString task_id(task->getId());
    tasks_.remove(index);
    emit removed(task_id);
    emit changed();
  }
}

void Robot::monitorUpdated(size_t index, bool up_to_date)
{
  if (up_to_date)
  {
    setState(static_cast<State>(index));
  }
  else if (index == Busy)
  {
    bool idle(monitor_->getSampleHolder(Idle)->isUpToDate());
    setState(idle ? Idle : Offline);
  }
}
}
