#include "utilities/message_field.h"
#include "utilities/message_field_subscriber.h"
#include "utilities/message_subscriber_registry.h"

namespace utilities
{
MessageFieldSubscriber::MessageFieldSubscriber(
    QObject* parent, const QString& type, const QString& field,
    MessageSubscriberRegistry* registry)
    : QObject(parent), subscribed_(false),
      field_(new MessageField(this, type, field)), registry_(registry)
{
}

MessageFieldSubscriber::~MessageFieldSubscriber()
{
  registry_ = NULL;
  if (field_)
  {
    delete field_;
    field_ = NULL;
  }
}

variant_topic_tools::BuiltinVariant
MessageFieldSubscriber::getCurrentFieldValue() const
{
  return current_field_value_;
}

MessageSubscriberRegistry* MessageFieldSubscriber::getRegistry() const
{
  return registry_;
}

void MessageFieldSubscriber::setRegistry(MessageSubscriberRegistry* registry)
{
  if (registry != registry_)
  {
    QString subscribed_topic(subscribed_topic_);
    if (registry_)
    {
      unsubscribe();
    }
    registry_ = registry;
    if (registry_ && !subscribed_topic.isEmpty())
    {
      subscribe(subscribed_topic, 10);
    }
  }
}

bool MessageFieldSubscriber::isSubscribed() const
{
  return !subscribed_topic_.isEmpty();
}

void MessageFieldSubscriber::subscribe(const QString& topic,
                                       const size_t& queue_size)
{
  if (isSubscribed())
  {
    unsubscribe();
  }
  if (field_ && registry_)
  {
    MessageBroker::PropertyMap properties;
    properties[MessageSubscriber::QueueSize] =
        QVariant::fromValue<quint64>(queue_size);
    if (registry_->subscribe(topic, this, SLOT(subscriberMessageReceived(
                                              const QString&, const Message&)),
                             properties))
    {
      subscribed_topic_ = topic;
    }
    if (isSubscribed())
    {
      emit subscribed();
    }
  }
}

void MessageFieldSubscriber::unsubscribe()
{
  if (isSubscribed())
  {
    registry_->unsubscribe(subscribed_topic_, this);
  }
  subscribed_topic_.clear();
  emit unsubscribed();
}

void MessageFieldSubscriber::subscriberMessageReceived(const QString& topic,
                                                       const Message& message)
{
  try
  {
    variant_topic_tools::BuiltinVariant field_value(
        field_->getFieldValue(message));
    emit received(field_value);
  }
  catch (const ros::Exception& exception)
  {
    ROS_ERROR_STREAM(
        "ROS Exception caught while getting the value of the message field: "
        << exception.what());
  }
}
}
