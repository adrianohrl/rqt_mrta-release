#include "mrta/task.h"
#include "rqt_mrta/config/application/task.h"

namespace mrta
{
Task::Task(QObject* parent) : QObject(parent), config_(NULL) {}

Task::Task(QObject* parent, Task::Config* config)
    : QObject(parent), config_(NULL)
{
  setConfig(config);
}

Task::Task(const Task& task)
    : QObject(task.parent()), config_(NULL), id_(task.id_)
{
  setConfig(task.config_);
}

Task::~Task() {
  config_ = NULL;
  ROS_INFO_STREAM("[~Task]");
}

Task::Config* Task::getConfig() const { return config_; }

void Task::setConfig(Task::Config* config)
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
    }
  }
}

QString Task::getId() const { return id_; }

void Task::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

Task& Task::operator=(const Task& task)
{
  id_ = task.id_;
  emit changed();
}

void Task::configDestroyed() { config_ = NULL; }
}
