#include <QMetaType>
#include "utilities/message_broker.h"

namespace utilities
{
MessageBroker::MessageBroker(QObject* parent) : QObject(parent)
{
  qRegisterMetaType<Message>("Message");
}

MessageBroker::~MessageBroker() {}
}
