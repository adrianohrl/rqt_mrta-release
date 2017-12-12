#ifndef _UTILITIES_MESSAGE_BROKER_H_
#define _UTILITIES_MESSAGE_BROKER_H_

#include <QObject>
#include <QMap>
#include <QVariant>
#include "utilities/message.h"

namespace utilities
{
class MessageBroker : public QObject
{
Q_OBJECT
public:
  typedef QMap<int, QVariant> PropertyMap;
  MessageBroker(QObject* parent = NULL);
  virtual ~MessageBroker();
  virtual bool subscribe(const QString& topic, QObject* receiver,
    const char* method, const PropertyMap& properties = PropertyMap(),
    Qt::ConnectionType type = Qt::AutoConnection) = 0;
  virtual bool unsubscribe(const QString& topic, QObject* receiver,
    const char* method = NULL) = 0;
};
}

#endif // _UTILITIES_MESSAGE_BROKER_H_
