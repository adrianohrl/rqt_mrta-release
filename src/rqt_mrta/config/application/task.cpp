#include "rqt_mrta/config/application/task.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Task::Task(QObject* parent) : AbstractConfig(parent) {}

Task::~Task() {}

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

void Task::save(QSettings& settings) const { settings.setValue("id", id_); }

void Task::load(QSettings& settings) { setId(settings.value("id").toString()); }

void Task::reset() { setId(""); }

void Task::write(QDataStream& stream) const { stream << id_; }

void Task::read(QDataStream& stream)
{
  QString id;
  stream >> id;
  setId(id);
}

Task& Task::operator=(const Task& config)
{
  setId(config.id_);
  return *this;
}

QString Task::validate() const
{
  if (id_.isEmpty())
  {
    return "The task id must not be empty.";
  }
  if (id_.contains(' '))
  {
    return "The task id must not contain <space>.";
  }
  return "";
}
}
}
}
