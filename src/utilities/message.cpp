#include "utilities/message.h"

namespace utilities
{
Message::Message() {}

Message::Message(const Message& src)
    : receipt_time_(src.receipt_time_), variant_(src.variant_)
{
}

Message::~Message() {}

void Message::setReceiptTime(const ros::Time& receipt_time)
{
  receipt_time_ = receipt_time;
}

const ros::Time& Message::getReceiptTime() const { return receipt_time_; }

void Message::setVariant(const variant_topic_tools::MessageVariant& variant)
{
  variant_ = variant;
}

const variant_topic_tools::MessageVariant& Message::getVariant() const
{
  return variant_;
}

bool Message::isEmpty() const { return variant_.isEmpty(); }

void Message::clear()
{
  receipt_time_ = ros::Time();
  variant_.clear();
}
}
