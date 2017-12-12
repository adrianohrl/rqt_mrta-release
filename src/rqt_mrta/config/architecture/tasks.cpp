#include "rqt_mrta/config/architecture/tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Tasks::Tasks(QObject* parent)
    : AbstractConfig(parent), incoming_tasks_(new IncomingTasks(this))
{
  connect(incoming_tasks_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Tasks::~Tasks()
{
  if (incoming_tasks_)
  {
    delete incoming_tasks_;
    incoming_tasks_ = NULL;
  }
}

QString Tasks::getType() const { return type_; }

IncomingTasks* Tasks::getIncomingTasks() const { return incoming_tasks_; }

void Tasks::setType(const QString& type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(type);
    emit changed();
  }
}

void Tasks::save(QSettings& settings) const
{
  settings.beginGroup("tasks");
  settings.setValue("type", type_);
  incoming_tasks_->save(settings);
  settings.endGroup();
}

void Tasks::load(QSettings& settings)
{
  settings.beginGroup("tasks");
  setType(settings.value("type").toString());
  incoming_tasks_->load(settings);
  settings.endGroup();
}

void Tasks::reset()
{
  setType("");
  incoming_tasks_->reset();
}

void Tasks::write(QDataStream& stream) const
{
  stream << type_;
  incoming_tasks_->write(stream);
}

void Tasks::read(QDataStream& stream)
{
  QString type;
  stream >> type;
  setType(type);
  incoming_tasks_->read(stream);
}

Tasks& Tasks::operator=(const Tasks& config)
{
  setType(config.type_);
  *incoming_tasks_ = *config.incoming_tasks_;
  return *this;
}
}
}
}
