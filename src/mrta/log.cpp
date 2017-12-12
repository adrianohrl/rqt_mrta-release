#include "mrta/log.h"
#include "mrta/history.h"
#include <ros/console.h>

namespace mrta
{
Log::Log(History* history, const ros::Time& timestamp, Log::Type type,
         Severity severity, const QString& id, int state)
    : QObject(history), timestamp_(timestamp), type_(type), severity_(severity),
      id_(id), state_(state)
{
}

Log::Log(const Log& log)
    : QObject(log.parent()), timestamp_(log.timestamp_), type_(log.type_),
      severity_(log.severity_), id_(log.id_), state_(log.state_)
{
}

Log::~Log()
{
  ROS_INFO_STREAM("[~Log]");
}

ros::Time Log::getTimestamp() const { return timestamp_; }

Log::Type Log::getType() const { return type_; }

Log::Severity Log::getSeverity() const { return severity_; }

QString Log::getId() const { return id_; }

int Log::getState() const { return state_; }

QString Log::toString() const
{
  return "[" + QString::number(timestamp_.toSec()) + "] [" + QString::number(type_) +
         "] [" + QString::number(severity_) + "] id: " + id_ + ", state: " +
      QString::number(state_);
}

const char *Log::toCString() const
{
  return toString().toStdString().c_str();
}
}
