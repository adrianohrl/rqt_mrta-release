#include <QApplication>
#include "utilities/message_event.h"
#include "utilities/message_subscriber.h"
#include <variant_topic_tools/MessageType.h>

namespace utilities
{
MessageSubscriber::MessageSubscriber(QObject* parent, const ros::NodeHandle& nh)
    : QObject(parent), nh_(nh), queue_size_(100)
{
}

MessageSubscriber::~MessageSubscriber() {}

const ros::NodeHandle& MessageSubscriber::getNodeHandle() const { return nh_; }

const QString& MessageSubscriber::getTopic() const { return topic_; }

void MessageSubscriber::setTopic(const QString& topic)
{
  if (topic != topic_)
  {
    topic_ = topic;
    if (subscriber_)
    {
      unsubscribe();
      subscribe();
    }
  }
}

void MessageSubscriber::setQueueSize(size_t queue_size)
{
  if (queue_size != queue_size_)
  {
    queue_size_ = queue_size;
    if (subscriber_)
    {
      unsubscribe();
      subscribe();
    }
  }
}

size_t MessageSubscriber::getQueueSize() const { return queue_size_; }

size_t MessageSubscriber::getNumPublishers() const
{
  return subscriber_.getNumPublishers();
}

bool MessageSubscriber::isValid() const { return subscriber_; }

bool MessageSubscriber::event(QEvent* event)
{
  if (event->type() == MessageEvent::Type)
  {
    MessageEvent* message_event = static_cast<MessageEvent*>(event);
    emit messageReceived(message_event->getTopic(), message_event->getMessage());
    return true;
  }
  return QObject::event(event);
}

void MessageSubscriber::subscribe()
{
  variant_topic_tools::MessageType type;

  subscriber_ =
      type.subscribe(nh_, topic_.toStdString(), queue_size_,
                     boost::bind(&MessageSubscriber::callback, this, _1, _2));
  if (subscriber_)
  {
    emit subscribed(topic_);
  }
}

void MessageSubscriber::unsubscribe()
{
  if (subscriber_)
  {
    subscriber_.shutdown();
    QApplication::removePostedEvents(this, MessageEvent::Type);
    emit unsubscribed(topic_);
  }
}

void MessageSubscriber::callback(
    const variant_topic_tools::MessageVariant& variant,
    const ros::Time& receiptTime)
{
  Message message;
  message.setReceiptTime(receiptTime);
  message.setVariant(variant);
  MessageEvent* messageEvent = new MessageEvent(topic_, message);
  QApplication::postEvent(this, messageEvent);
}

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
void MessageSubscriber::connectNotify(const QMetaMethod& signal)
{
  if (signal == QMetaMethod::fromSignal(&MessageSubscriber::messageReceived) &&
      !subscriber_)
  {
    subscribe();
  }
}

void MessageSubscriber::disconnectNotify(const QMetaMethod& signal)
{
  if (!receivers(QMetaObject::normalizedSignature(
          SIGNAL(messageReceived(const QString&, const Message&)))))
  {
    if (subscriber_)
    {
      unsubscribe();
    }
    emit aboutToBeDestroyed();
    deleteLater();
  }
}
#else
void MessageSubscriber::connectNotify(const char* signal)
{
  if ((QByteArray(signal) ==
       QMetaObject::normalizedSignature(
           SIGNAL(messageReceived(const QString&, const Message&)))) &&
      !subscriber_)
  {
    subscribe();
  }
}

void MessageSubscriber::disconnectNotify(const char* signal)
{
  if (!receivers(QMetaObject::normalizedSignature(
          SIGNAL(messageReceived(const QString&, const Message&)))))
  {
    if (subscriber_)
    {
      unsubscribe();
    }
    emit aboutToBeDestroyed();
    deleteLater();
  }
}
#endif
}
