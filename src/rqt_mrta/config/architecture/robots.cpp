#include "rqt_mrta/config/architecture/robots.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Robots::Robots(QObject* parent)
    : AbstractConfig(parent), busy_robots_(new BusyRobots(this)),
      idle_robots_(new IdleRobots(this))
{
  connect(busy_robots_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(idle_robots_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Robots::~Robots()
{
  if (busy_robots_)
  {
    delete busy_robots_;
    busy_robots_ = NULL;
  }
  if (idle_robots_)
  {
    delete idle_robots_;
    idle_robots_ = NULL;
  }
}

QString Robots::getType() const { return type_; }

QString Robots::getConfigId() const { return config_id_; }

QString Robots::getLaunchId() const { return launch_id_; }

BusyRobots* Robots::getBusyRobots() const { return busy_robots_; }

IdleRobots* Robots::getIdleRobots() const { return idle_robots_; }

void Robots::setType(const QString& type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(type);
    emit changed();
  }
}

void Robots::setConfigId(const QString& config_id)
{
  if (config_id != config_id_)
  {
    config_id_ = config_id;
    emit configIdChanged(config_id);
    emit changed();
  }
}

void Robots::setLaunchId(const QString& launch_id)
{
  if (launch_id != launch_id_)
  {
    launch_id_ = launch_id;
    emit launchIdChanged(launch_id);
    emit changed();
  }
}

void Robots::save(QSettings& settings) const
{
  settings.beginGroup("robots");
  settings.setValue("type", type_);
  settings.setValue("config_id", config_id_);
  settings.setValue("launch_id", launch_id_);
  busy_robots_->save(settings);
  idle_robots_->save(settings);
  settings.endGroup();
}

void Robots::load(QSettings& settings)
{
  settings.beginGroup("robots");
  setType(settings.value("type").toString());
  setConfigId(settings.value("config_id").toString());
  setLaunchId(settings.value("launch_id").toString());
  busy_robots_->load(settings);
  idle_robots_->load(settings);
  settings.endGroup();
}

void Robots::reset()
{
  setType("");
  setConfigId("");
  setLaunchId("");
  busy_robots_->reset();
  idle_robots_->reset();
}

void Robots::write(QDataStream& stream) const
{
  stream << type_;
  stream << config_id_;
  stream << launch_id_;
  busy_robots_->write(stream);
  idle_robots_->write(stream);
}

void Robots::read(QDataStream& stream)
{
  QString type;
  stream >> type;
  setType(type);
  QString config_id;
  stream >> config_id;
  setConfigId(config_id);
  QString launch_id;
  stream >> launch_id;
  setLaunchId(launch_id);
  busy_robots_->read(stream);
  idle_robots_->read(stream);
}

Robots& Robots::operator=(const Robots& config)
{
  setType(config.type_);
  setConfigId(config.config_id_);
  setLaunchId(config.launch_id_);
  *busy_robots_ = *config.busy_robots_;
  *idle_robots_ = *config.idle_robots_;
  return *this;
}
}
}
}
