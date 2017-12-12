#include "rqt_mrta/config/architecture/topic.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Topic::Topic(QObject* parent) : AbstractConfig(parent) {}

Topic::~Topic() {}

QString Topic::getName() const { return name_; }

QString Topic::getType() const { return type_; }

size_t Topic::getQueueSize() const { return queue_size_; }

QString Topic::getField() const { return field_; }

ros::Duration Topic::getTimeout() const { return timeout_; }

ros::Duration Topic::getHorizon() const { return horizon_; }

void Topic::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void Topic::setType(const QString& type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(type);
    emit changed();
  }
}

void Topic::setQueueSize(const size_t& queue_size)
{
  if (queue_size != queue_size_)
  {
    queue_size_ = queue_size;
    emit queueSizeChanged(queue_size);
    emit changed();
  }
}

void Topic::setField(const QString& field)
{
  if (field != field_)
  {
    field_ = field;
    emit fieldChanged(field);
    emit changed();
  }
}

void Topic::setTimeout(const ros::Duration& timeout)
{
  if (timeout != timeout_)
  {
    timeout_ = timeout;
    emit timeoutChanged(timeout);
    emit changed();
  }
}

void Topic::setHorizon(const ros::Duration& horizon)
{
  if (horizon != horizon_)
  {
    horizon_ = horizon;
    emit horizonChanged(horizon);
    emit changed();
  }
}

void Topic::save(QSettings& settings) const
{
  settings.beginGroup("topic");
  settings.setValue("name", name_);
  settings.setValue("type", type_);
  settings.setValue("queue_size", (quint64)queue_size_);
  settings.setValue("field", field_);
  settings.setValue("timeout", timeout_.toSec());
  settings.setValue("horizon", horizon_.toSec());
  settings.endGroup();
}

void Topic::load(QSettings& settings)
{
  settings.beginGroup("topic");
  setName(settings.value("name").toString());
  setType(settings.value("type").toString());
  setQueueSize(settings.value("queue_size").toULongLong());
  setField(settings.value("field").toString());
  setTimeout(ros::Duration(settings.value("timeout", 2.0).toDouble()));
  setHorizon(ros::Duration(settings.value("horizon", 5.0).toDouble()));
  settings.endGroup();
}

void Topic::reset()
{
  setName("");
  setType("");
  setQueueSize(10);
  setField("");
  setTimeout(ros::Duration(2.0));
  setHorizon(ros::Duration(5.0));
}

void Topic::write(QDataStream& stream) const
{
  stream << name_;
  stream << type_;
  stream << (quint64)queue_size_;
  stream << field_;
  stream << timeout_.toSec();
  stream << horizon_.toSec();
}

void Topic::read(QDataStream& stream)
{
  QString name, type, field;
  quint64 queue_size;
  double timeout, horizon;
  stream >> name;
  setName(name);
  stream >> type;
  setType(type);
  stream >> queue_size;
  setQueueSize(queue_size);
  stream >> field;
  setField(field);
  stream >> timeout;
  setTimeout(ros::Duration(timeout));
  stream >> horizon;
  setHorizon(ros::Duration(horizon));
}

Topic& Topic::operator=(const Topic& config)
{
  setName(config.name_);
  setType(config.type_);
  setQueueSize(config.queue_size_);
  setField(config.field_);
  setTimeout(config.timeout_);
  setHorizon(config.horizon_);
  return *this;
}
}
}
}
