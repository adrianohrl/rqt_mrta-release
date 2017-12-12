#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Robot::Robot(QObject* parent)
    : AbstractConfig(parent), tasks_(new Tasks(this))
{
  connect(tasks_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(tasks_, SIGNAL(taskIdChanged(size_t, const QString&)), this,
          SLOT(taskChanged(size_t, const QString&)));
  connect(tasks_, SIGNAL(added(size_t)), this, SLOT(taskAdded(size_t)));
  connect(tasks_, SIGNAL(removed(const QString&)), this,
          SLOT(taskRemoved(const QString&)));
  connect(tasks_, SIGNAL(cleared()), this, SLOT(tasksCleared()));
}

Robot::~Robot()
{
  if (tasks_)
  {
    delete tasks_;
    tasks_ = NULL;
  }
}

QString Robot::getId() const { return id_; }

Tasks* Robot::getTasks() const { return tasks_; }

void Robot::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

void Robot::save(QSettings& settings) const
{
  settings.setValue("id", id_);
  tasks_->save(settings);
}

void Robot::load(QSettings& settings)
{
  setId(settings.value("id").toString());
  tasks_->load(settings);
}

void Robot::reset()
{
  setId("");
  tasks_->reset();
}

void Robot::write(QDataStream& stream) const
{
  stream << id_;
  tasks_->write(stream);
}

void Robot::read(QDataStream& stream)
{
  QString id;
  stream >> id;
  setId(id);
  tasks_->read(stream);
}

Robot& Robot::operator=(const Robot& config)
{
  setId(config.id_);
  *tasks_ = *config.tasks_;
  return *this;
}

QString Robot::validate() const
{
  if (id_.isEmpty())
  {
    return "The robot id must not be empty.";
  }
  if (id_.contains(' '))
  {
    return "The robot id must not contain <space>.";
  }
  return tasks_->validate();
}

void Robot::taskChanged(size_t task_index, const QString& task_id)
{
  emit taskIdChanged(task_index, task_id);
}

void Robot::taskAdded(size_t task_index) { emit added(task_index); }

void Robot::taskRemoved(const QString& task_id) { emit removed(task_id); }

void Robot::tasksCleared() { emit cleared(); }
}
}
}
