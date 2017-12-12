#include "mrta/monitor.h"
#include "mrta/sample_holder.h"
#include "mrta/state_monitor.h"
#include "mrta/system.h"
#include "rqt_mrta/config/architecture/topic.h"
#include "utilities/message_subscriber_registry.h"

namespace mrta
{
Monitor::Monitor(System* system, utilities::MessageSubscriberRegistry* registry,
                 Config* config)
    : QObject(system), subscriber_(NULL), registry_(NULL), config_(NULL)
{
  connect(this, SIGNAL(changed()), this, SLOT(updateSubscriber()));
  setRegistry(registry);
  setConfig(config);
}

Monitor::~Monitor()
{
  ROS_INFO_STREAM("[~Monitor] before");
  config_ = NULL;
  registry_ = NULL;
  unsubscribe();
  if (subscriber_)
  {
    delete subscriber_;
    subscriber_ = NULL;
  }
  monitors_.clear();
  ROS_INFO_STREAM("[~Monitor] after");
}

Monitor::Config* Monitor::getConfig() const { return config_; }

QString Monitor::getName() const { return name_; }

QString Monitor::getType() const { return type_; }

size_t Monitor::getQueueSize() const { return queue_size_; }

QString Monitor::getField() const { return field_; }

ros::Duration Monitor::getTimeout() const { return timeout_; }

ros::Duration Monitor::getHorizon() const { return horizon_; }

System* Monitor::getSystem() const { return static_cast<System*>(parent()); }

void Monitor::setConfig(Monitor::Config* config)
{
  if (config != config_)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(nameChanged(const QString&)), this,
                 SLOT(setName(const QString&)));
      disconnect(config_, SIGNAL(typeChanged(const QString&)), this,
                 SLOT(setType(const QString&)));
      disconnect(config_, SIGNAL(queueSizeChanged(const size_t&)), this,
                 SLOT(setQueueSize(const size_t&)));
      disconnect(config_, SIGNAL(fieldChanged(const QString&)), this,
                 SLOT(setField(const QString&)));
      disconnect(config_, SIGNAL(timeoutChanged(const ros::Duration&)), this,
                 SLOT(setTimeout(const ros::Duration&)));
      disconnect(config_, SIGNAL(horizonChanged(const ros::Duration&)), this,
                 SLOT(setHorizon(const ros::Duration&)));
    }
    config_ = config;
    if (config_)
    {
      connect(config_, SIGNAL(nameChanged(const QString&)), this,
              SLOT(setName(const QString&)));
      connect(config_, SIGNAL(typeChanged(const QString&)), this,
              SLOT(setType(const QString&)));
      connect(config_, SIGNAL(queueSizeChanged(const size_t&)), this,
              SLOT(setQueueSize(const size_t&)));
      connect(config_, SIGNAL(fieldChanged(const QString&)), this,
              SLOT(setField(const QString&)));
      connect(config_, SIGNAL(timeoutChanged(const ros::Duration&)), this,
              SLOT(setTimeout(const ros::Duration&)));
      connect(config_, SIGNAL(horizonChanged(const ros::Duration&)), this,
              SLOT(setHorizon(const ros::Duration&)));
      setName(config_->getName());
      setType(config_->getType());
      setQueueSize(config_->getQueueSize());
      setField(config_->getField());
      setTimeout(config_->getTimeout());
      setHorizon(config_->getHorizon());
    }
    emit changed();
  }
}

void Monitor::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit changed();
  }
}

void Monitor::setType(const QString& type)
{
  if (type != type_)
  {
    type_ = type;
    emit changed();
  }
}

void Monitor::setQueueSize(const size_t& queue_size)
{
  if (queue_size != queue_size_)
  {
    queue_size_ = queue_size;
    emit changed();
  }
}

void Monitor::setField(const QString& field)
{
  if (field != field_)
  {
    field_ = field;
    emit changed();
  }
}

void Monitor::setTimeout(const ros::Duration& timeout)
{
  if (timeout != timeout_)
  {
    timeout_ = timeout;
    for (StateMonitorMap::iterator it(monitors_.begin()); it != monitors_.end();
         it++)
    {
      monitors_[it.key()]->setTimeout(timeout);
    }
    emit changed();
  }
}

void Monitor::setHorizon(const ros::Duration& horizon)
{
  if (horizon != horizon_)
  {
    horizon_ = horizon;
    emit changed();
  }
}

void Monitor::setRegistry(utilities::MessageSubscriberRegistry* registry)
{
  if (registry != registry_)
  {
    registry_ = registry;
    if (subscriber_)
    {
      subscriber_->setRegistry(registry);
    }
  }
}

void Monitor::addStateMonitor(const QString& id, StateMonitor* monitor)
{
  if (!monitors_.contains(id))
  {
    monitors_.insert(id, monitor);
  }
}

void Monitor::update(const QString& id, int state)
{
  if (monitors_.contains(id))
  {
    monitors_[id]->update(state);
  }
}

void Monitor::updateSubscriber()
{
  unsubscribe();
  subscribe();
}

bool Monitor::subscribe()
{
  if (name_.isEmpty() || type_.isEmpty() || field_.isEmpty() || !registry_)
  {
    return false;
  }
  subscriber_ =
      new utilities::MessageFieldSubscriber(this, type_, field_, registry_);
  subscriber_->subscribe(name_, queue_size_);
  if (subscriber_->isSubscribed())
  {
    connect(
        subscriber_,
        SIGNAL(received(const variant_topic_tools::BuiltinVariant&)), this,
        SLOT(subscriberReceived(const variant_topic_tools::BuiltinVariant&)));
  }
  else
  {
    delete subscriber_;
    subscriber_ = NULL;
  }
  return subscriber_;
}

bool Monitor::unsubscribe()
{
  if (!subscriber_)
  {
    return false;
  }
  disconnect(
      subscriber_, SIGNAL(received(const variant_topic_tools::BuiltinVariant&)),
      this,
      SLOT(subscriberReceived(const variant_topic_tools::BuiltinVariant&)));
  if (subscriber_->isSubscribed())
  {
    subscriber_->unsubscribe();
  }
  delete subscriber_;
  subscriber_ = NULL;
  return true;
}

void Monitor::subscriberReceived(
    variant_topic_tools::BuiltinVariant field_value)
{
  QString id(QString::fromStdString(field_value.getValue<std::string>()));
  update(id);
  emit received(id);
}
}
