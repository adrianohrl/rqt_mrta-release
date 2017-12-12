#ifndef _UTITLITIES_MESSAGE_FIELD_H_
#define _UTITLITIES_MESSAGE_FIELD_H_

#include <QObject>
#include <variant_topic_tools/BuiltinVariant.h>

namespace utilities
{
class Message;

class MessageField : public QObject
{
  Q_OBJECT
public:
  MessageField(QObject* parent, const QString& type, const QString& field);
  virtual ~MessageField();
  QString getType() const;
  QString getField() const;
  void setType(const QString& type);
  void setField(const QString& field);
  variant_topic_tools::BuiltinVariant
  getFieldValue(const Message& message);

private:
  QString type_;
  QString field_;
};
}

#endif // _UTITLITIES_MESSAGE_FIELD_H_
