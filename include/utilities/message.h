#ifndef _UTILITIES_MESSAGE_H_
#define _UTILITIES_MESSAGE_H_

#include <ros/time.h>
#include <variant_topic_tools/MessageVariant.h>

namespace utilities
{
class Message
{
public:
  Message();
  Message(const Message& src);
  virtual ~Message();
  void setReceiptTime(const ros::Time& receipt_time);
  const ros::Time& getReceiptTime() const;
  void setVariant(const variant_topic_tools::MessageVariant& variant);
  const variant_topic_tools::MessageVariant& getVariant() const;
  bool isEmpty() const;
  void clear();

private:
  ros::Time receipt_time_;
  variant_topic_tools::MessageVariant variant_;
};
}

#endif // _UTILITIES_MESSAGE_H_
