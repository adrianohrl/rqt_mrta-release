#include <QStringList>
#include "rqt_mrta/config/application/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Tasks::Tasks(QObject* parent) : AbstractConfig(parent) {}

Tasks::~Tasks()
{
  for (size_t index(0); index < tasks_.count(); index++)
  {
    if (tasks_[index])
    {
      delete tasks_[index];
      tasks_[index] = NULL;
    }
  }
  tasks_.clear();
}

size_t Tasks::count() const { return tasks_.count(); }

Task* Tasks::getTask(size_t index) const
{
  return index < tasks_.count() ? tasks_[index] : NULL;
}

Task* Tasks::addTask()
{
  Task* task = new Task(this);
  tasks_.append(task);
  connect(task, SIGNAL(changed()), this, SLOT(taskChanged()));
  connect(task, SIGNAL(idChanged(const QString&)), this,
          SLOT(taskIdChanged(const QString&)));
  connect(task, SIGNAL(destroyed()), this, SLOT(taskDestroyed()));
  emit added(tasks_.count() - 1);
  emit changed();
  return task;
}

void Tasks::removeTask(Task* task) { removeTask(tasks_.indexOf(task)); }

void Tasks::removeTask(size_t index)
{
  if (index >= 0 && index < tasks_.count())
  {
    QString task_id(tasks_[index]->getId());
    tasks_.remove(index);
    emit removed(task_id);
    emit changed();
  }
}

void Tasks::clearTasks()
{
  if (!tasks_.isEmpty())
  {
    for (size_t i(0); i < tasks_.count(); ++i)
    {
      if (tasks_[i])
      {
        delete tasks_[i];
        tasks_[i] = NULL;
      }
    }
    tasks_.clear();
    emit cleared();
    emit changed();
  }
}

bool Tasks::contains(const QString& id) const
{
  for (size_t index(0); index < tasks_.count(); index++)
  {
    if (tasks_[index]->getId() == id)
    {
      return true;
    }
  }
  return false;
}

bool Tasks::isEmpty() const { return tasks_.isEmpty(); }

void Tasks::save(QSettings& settings) const
{
  settings.beginGroup("tasks");
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    settings.beginGroup("task_" + QString::number(index));
    tasks_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Tasks::load(QSettings& settings)
{
  settings.beginGroup("tasks");
  QStringList groups(settings.childGroups());
  size_t index(0);
  for (QStringList::iterator it(groups.begin()); it != groups.end(); ++it)
  {
    Task* task = index < tasks_.count() ? task = tasks_[index] : addTask();
    settings.beginGroup(*it);
    task->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < tasks_.count())
  {
    removeTask(index);
  }
}

void Tasks::reset() { clearTasks(); }

void Tasks::write(QDataStream& stream) const
{
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    tasks_[index]->write(stream);
  }
}

void Tasks::read(QDataStream& stream)
{
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    tasks_[index]->read(stream);
  }
}

Tasks& Tasks::operator=(const Tasks& config)
{
  while (tasks_.count() < config.tasks_.count())
  {
    addTask();
  }
  while (tasks_.count() > config.tasks_.count())
  {
    removeTask(tasks_.count() - 1);
  }
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    *tasks_[index] = *config.tasks_[index];
  }
  return *this;
}

QString Tasks::validate() const
{
  if (tasks_.isEmpty())
  {
    return "Enter the robot tasks.";
  }
  QString validation;
  for (size_t i(0); i < tasks_.count(); i++)
  {
    validation = tasks_[i]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

void Tasks::taskChanged()
{
  for (size_t index(0); index < tasks_.count(); ++index)
  {
    if (tasks_[index] == sender())
    {
      emit taskChanged(index);
      break;
    }
  }
  emit changed();
}

void Tasks::taskIdChanged(const QString& task_id)
{
  int index(tasks_.indexOf(static_cast<Task*>(sender())));
  if (index != -1)
  {
    emit taskIdChanged(index, task_id);
    emit changed();
  }
}

void Tasks::taskDestroyed()
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
}
}
}
