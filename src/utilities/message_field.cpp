#include "utilities/message_field.h"
#include "utilities/message.h"
#include <variant_topic_tools/MessageVariant.h>

namespace utilities
{

MessageField::MessageField(QObject* parent, const QString& type,
                           const QString& field)
    : QObject(parent), type_(type), field_(field)
{
  variant_topic_tools::MessageDefinition definition;
  definition.load(type_.toStdString());
  if (!definition.hasField(field_.toStdString()))
  {
    throw ros::Exception("The " + type.toStdString() +
                         " message does not have the " + field_.toStdString() +
                         " field.");
  }
}

MessageField::~MessageField() {}

QString MessageField::getType() const { return type_; }

QString MessageField::getField() const { return field_; }

void MessageField::setType(const QString& type)
{
  variant_topic_tools::MessageDefinition definition;
  definition.load(type.toStdString());
  type_ = type;
}

void MessageField::setField(const QString& field)
{
  variant_topic_tools::MessageDefinition definition;
  definition.load(type_.toStdString());
  if (!definition.hasField(field.toStdString()))
  {
    throw ros::Exception("The " + type_.toStdString() +
                         " message does not have the " + field.toStdString() +
                         " field.");
  }
  field_ = field;
}

variant_topic_tools::BuiltinVariant
MessageField::getFieldValue(const Message& message)
{
  std::string type(message.getVariant().getType().getIdentifier());
  if (type != type_.toStdString())
  {
    throw ros::Exception(type + " does not match to " + type_.toStdString() +
                         ".");
  }
  return message.getVariant().getMember(field_.toStdString());
}
}
