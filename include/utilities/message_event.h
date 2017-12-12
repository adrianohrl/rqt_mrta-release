#ifndef _UTILITIES_MESSAGE_EVENT_H_
#define _UTILITIES_MESSAGE_EVENT_H_

#include <QEvent>
#include <QString>
#include "utilities/message.h"

namespace utilities
{
class MessageEvent : public QEvent
{
public:
  static const QEvent::Type Type;
  MessageEvent(const QString& topic, const Message& message);
  virtual ~MessageEvent();
  const QString& getTopic() const;
  const Message& getMessage() const;

private:
  QString topic_;
  Message message_;
};
}

#endif // _UTILITIES_MESSAGE_EVENT_H_
