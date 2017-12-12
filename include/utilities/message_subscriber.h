#ifndef _UTILITIES_MESSAGE_SUBSCRIBER_H_
#define _UTILITIES_MESSAGE_SUBSCRIBER_H_

#include <QMap>
#include <QObject>
#include <QString>
#include <QMetaMethod>
#include <ros/node_handle.h>
#include "utilities/message.h"
#include <variant_topic_tools/Subscriber.h>

namespace utilities
{
class MessageSubscriber : public QObject
{
  Q_OBJECT
public:
  enum Property
  {
    QueueSize
  };
  MessageSubscriber(QObject* parent = 0,
                    const ros::NodeHandle& nh = ros::NodeHandle("~"));
  ~MessageSubscriber();
  const ros::NodeHandle& getNodeHandle() const;
  void setTopic(const QString& topic);
  const QString& getTopic() const;
  void setQueueSize(size_t queue_size);
  size_t getQueueSize() const;
  size_t getNumPublishers() const;
  bool isValid() const;
  bool event(QEvent* event);

signals:
  void subscribed(const QString& topic);
  void messageReceived(const QString& topic, const Message& message);
  void unsubscribed(const QString& topic);
  void aboutToBeDestroyed();

private:
  ros::NodeHandle nh_;
  QString topic_;
  size_t queue_size_;
  variant_topic_tools::Subscriber subscriber_;
  void subscribe();
  void unsubscribe();
  void callback(const variant_topic_tools::MessageVariant& variant,
                const ros::Time& receipt_time);
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  void connectNotify(const QMetaMethod& signal);
  void disconnectNotify(const QMetaMethod& signal);
#else
  void connectNotify(const char* signal);
  void disconnectNotify(const char* signal);
#endif
};
}

#endif // _UTILITIES_MESSAGE_SUBSCRIBER_H_
