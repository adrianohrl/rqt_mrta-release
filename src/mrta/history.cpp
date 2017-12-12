#include "mrta/history.h"
#include <ros/console.h>

namespace mrta
{
History::History(QObject *parent) : QObject(parent) {}

History::~History()
{
  ROS_INFO_STREAM("[~History] before");
  for (size_t index(0); index < logs_.count(); index++)
  {
    if (logs_[index])
    {
      delete logs_[index];
      logs_[index] = NULL;
    }
  }
  logs_.clear();
  ROS_INFO_STREAM("[~History] after");
}

QList<Log*> History::getLogs() const { return logs_; }

void History::log(const ros::Time& timestamp, Log::Type type,
                  Log::Severity severity, const QString& id, int state)
{
  Log* log = new Log(this, timestamp, type, severity, id, state);
  switch (severity)
  {
  case Log::Debug:
    ROS_DEBUG("%s", log->toCString());
    break;
  case Log::Info:
    ROS_INFO("%s", log->toCString());
    break;
  case Log::Warn:
    ROS_WARN("%s", log->toCString());
    break;
  case Log::Error:
    ROS_ERROR("%s", log->toCString());
    break;
  case Log::Fatal:
    ROS_FATAL("%s", log->toCString());
    break;
  }
  logs_.append(log);
}
}
