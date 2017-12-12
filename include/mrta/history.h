#ifndef _MRTA_ROBOT_MONITOR_H_
#define _MRTA_ROBOT_MONITOR_H_

#include "mrta/log.h"
#include <QList>
#include <QObject>
#include <ros/time.h>

namespace mrta
{
class History : public QObject
{
  Q_OBJECT
public:
  History(QObject* parent);
  virtual ~History();
  QList<Log*> getLogs() const;
  void log(const ros::Time& timestamp, Log::Type type, Log::Severity severity,
           const QString& id, int state);

private:
  typedef QList<Log*> LogList;
  LogList logs_;
};
}

#endif // _MRTA_ROBOT_MONITOR_H_
