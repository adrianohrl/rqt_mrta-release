#include <QMetaType>
#include "utilities/message_subscriber_registry.h"

namespace utilities
{
MessageSubscriberRegistry::MessageSubscriberRegistry(QObject* parent,
                                                     const ros::NodeHandle& nh)
    : MessageBroker(parent), nh_(nh)
{
}

MessageSubscriberRegistry::~MessageSubscriberRegistry() {}

const ros::NodeHandle& MessageSubscriberRegistry::getNodeHandle() const
{
  return nh_;
}

bool MessageSubscriberRegistry::subscribe(const QString& topic,
                                          QObject* receiver, const char* method,
                                          const PropertyMap& properties,
                                          Qt::ConnectionType type)
{
  iterator it(subscribers_.find(topic));
  size_t queue_size(100);
  if (properties.contains(MessageSubscriber::QueueSize))
  {
    queue_size = properties[MessageSubscriber::QueueSize].toULongLong();
  }
  MessageSubscriber* subscriber = it != subscribers_.end() ? *it : NULL;
  if (!subscriber)
  {
    subscriber = new MessageSubscriber(this, getNodeHandle());
    subscribers_.insert(topic, subscriber);
    subscriber->setQueueSize(queue_size);
    subscriber->setTopic(topic);
    connect(subscriber, SIGNAL(aboutToBeDestroyed()), this,
            SLOT(subscriberAboutToBeDestroyed()));
  }
  else if (subscriber->getQueueSize() < queue_size)
  {
    subscriber->setQueueSize(queue_size);
  }
  return receiver->connect(
      subscriber, SIGNAL(messageReceived(const QString&, const Message&)),
      method, type);
}

bool MessageSubscriberRegistry::unsubscribe(const QString& topic,
                                            QObject* receiver,
                                            const char* method)
{
  iterator it = subscribers_.find(topic);
  MessageSubscriber* subscriber = it != subscribers_.end() ? *it : NULL;
  return subscriber &&
         subscriber->disconnect(
             SIGNAL(messageReceived(const QString&, const Message&)), receiver,
             method);
}

void MessageSubscriberRegistry::subscriberAboutToBeDestroyed()
{
  for (iterator it = subscribers_.begin(); it != subscribers_.end(); it++)
  {
    if (*it == static_cast<MessageSubscriber*>(sender()))
    {
      subscribers_.erase(it);
      break;
    }
  }
}
}
