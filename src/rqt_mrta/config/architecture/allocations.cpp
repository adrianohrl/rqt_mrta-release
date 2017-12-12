#include "rqt_mrta/config/architecture/allocations.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Allocations::Allocations(QObject* parent)
    : AbstractConfig(parent), allocated_tasks_(new AllocatedTasks(this))
{
  connect(allocated_tasks_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Allocations::~Allocations()
{
  if (allocated_tasks_)
  {
    delete allocated_tasks_;
    allocated_tasks_ = NULL;
  }
}

QString Allocations::getType() const
{
  return type_;
}

AllocatedTasks* Allocations::getAllocatedTasks() const
{
  return allocated_tasks_;
}

void Allocations::setType(const QString &type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(type);
    emit changed();
  }
}

void Allocations::save(QSettings& settings) const
{
  settings.beginGroup("allocations");
  settings.setValue("type", type_);
  allocated_tasks_->save(settings);
  settings.endGroup();
}

void Allocations::load(QSettings& settings)
{
  settings.beginGroup("allocations");
  setType(settings.value("type").toString());
  allocated_tasks_->load(settings);
  settings.endGroup();
}

void Allocations::reset()
{
  setType("");
  allocated_tasks_->reset();
}

void Allocations::write(QDataStream& stream) const
{
  stream << type_;
  allocated_tasks_->write(stream);
}

void Allocations::read(QDataStream& stream)
{
  QString type;
  stream >> type;
  setType(type);
  allocated_tasks_->read(stream);
}

Allocations& Allocations::operator=(const Allocations& config)
{
  setType(config.type_);
  *allocated_tasks_ = *config.allocated_tasks_;
  return *this;
}
}
}
}
