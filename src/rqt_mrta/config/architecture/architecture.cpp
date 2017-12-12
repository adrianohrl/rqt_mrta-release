#include "rqt_mrta/config/architecture/architecture.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Architecture::Architecture(QObject* parent)
    : AbstractConfig(parent), allocations_(new Allocations(this)),
      robots_(new Robots(this)), tasks_(new Tasks(this))
{
  connect(allocations_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(robots_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(tasks_, SIGNAL(changed()), this,SIGNAL(changed()));
}

Architecture::~Architecture()
{
  if (allocations_)
  {
    delete allocations_;
    allocations_ = NULL;
  }
  if (robots_)
  {
    delete robots_;
    robots_ = NULL;
  }
  if (tasks_)
  {
    delete tasks_;
    tasks_ = NULL;
  }
}

Allocations* Architecture::getAllocations() const { return allocations_; }

QString Architecture::getName() const { return name_; }

Robots* Architecture::getRobots() const { return robots_; }

Tasks* Architecture::getTasks() const { return tasks_; }

void Architecture::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void Architecture::save(QSettings& settings) const
{
  settings.beginGroup("architecture");
  settings.setValue("name", name_);
  allocations_->save(settings);
  robots_->save(settings);
  tasks_->save(settings);
  settings.endGroup();
}

void Architecture::load(QSettings& settings)
{
  settings.beginGroup("architecture");
  setName(settings.value("name").toString());
  allocations_->load(settings);
  robots_->load(settings);
  tasks_->load(settings);
  settings.endGroup();
}

void Architecture::reset()
{
  setName("");
  allocations_->reset();
  robots_->reset();
  tasks_->reset();
}

void Architecture::write(QDataStream& stream) const
{
  stream << name_;
  allocations_->write(stream);
  robots_->write(stream);
  tasks_->write(stream);
}

void Architecture::read(QDataStream& stream)
{
  QString name;
  stream >> name;
  setName(name);
  allocations_->read(stream);
  robots_->read(stream);
  tasks_->read(stream);
}

Architecture& Architecture::operator=(const Architecture& config)
{
  setName(config.name_);
  *allocations_ = *config.allocations_;
  *robots_ = *config.robots_;
  *tasks_ = *config.tasks_;
  return *this;
}
}
}
}
