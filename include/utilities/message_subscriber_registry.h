#ifndef _UTILITIES_MESSAGE_SUBSCRIBER_REGISTRY_H_
#define _UTILITIES_MESSAGE_SUBSCRIBER_REGISTRY_H_

#include <QMap>
#include <QString>
#include "utilities/message_broker.h"
#include "utilities/message_subscriber.h"

namespace utilities
{
class MessageSubscriberRegistry : public MessageBroker
{
  Q_OBJECT
public:
  MessageSubscriberRegistry(
      QObject* parent = NULL,
      const ros::NodeHandle& nh = ros::NodeHandle("~"));
  virtual ~MessageSubscriberRegistry();
  const ros::NodeHandle& getNodeHandle() const;
  bool subscribe(const QString& topic, QObject* receiver, const char* method,
                 const PropertyMap& properties = PropertyMap(),
                 Qt::ConnectionType type = Qt::AutoConnection);
  bool unsubscribe(const QString& topic, QObject* receiver,
                   const char* method = 0);

private:
  typedef QMap<QString, MessageSubscriber*>::iterator iterator;
  typedef QMap<QString, MessageSubscriber*>::const_iterator const_iterator;
  ros::NodeHandle nh_;
  QMap<QString, MessageSubscriber*> subscribers_;

private slots:
  void subscriberAboutToBeDestroyed();
};
}

#endif // _UTILITIES_MESSAGE_SUBSCRIBER_REGISTRY_H_
