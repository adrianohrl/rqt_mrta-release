#ifndef _UTILITIES_MESSAGE_FIELD_SUBSCRIBER_H_
#define _UTILITIES_MESSAGE_FIELD_SUBSCRIBER_H_

#include <QObject>
#include <variant_topic_tools/BuiltinVariant.h>

namespace utilities
{
class Message;
class MessageField;
class MessageSubscriberRegistry;

class MessageFieldSubscriber : public QObject
{
  Q_OBJECT
public:
  MessageFieldSubscriber(QObject* parent, const QString& type,
                         const QString& field, MessageSubscriberRegistry* registry = NULL);
  virtual ~MessageFieldSubscriber();
  variant_topic_tools::BuiltinVariant getCurrentFieldValue() const;
  MessageSubscriberRegistry* getRegistry() const;
  void setRegistry(MessageSubscriberRegistry* registry);
  bool isSubscribed() const;
  void subscribe(const QString &topic, const size_t& queue_size);
  void unsubscribe();

signals:
  void subscribed();
  void received(const variant_topic_tools::BuiltinVariant& field_value);
  void unsubscribed();

private:
  bool subscribed_;
  QString subscribed_topic_;
  MessageField* field_;
  variant_topic_tools::BuiltinVariant current_field_value_;
  MessageSubscriberRegistry* registry_;

private slots:
  void subscriberMessageReceived(const QString& topic, const Message& message);
};
}

#endif // _UTILITIES_MESSAGE_FIELD_SUBSCRIBER_H_
