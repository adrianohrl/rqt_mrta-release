#include "utilities/message_event.h"

namespace utilities
{
const QEvent::Type MessageEvent::Type =
    static_cast<QEvent::Type>(QEvent::registerEventType());

MessageEvent::MessageEvent(const QString& topic, const Message& message)
    : QEvent(Type), topic_(topic), message_(message)
{
}

MessageEvent::~MessageEvent() {}

const QString& MessageEvent::getTopic() const { return topic_; }

const Message& MessageEvent::getMessage() const { return message_; }
}
