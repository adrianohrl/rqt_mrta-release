#ifndef _MRTA_ROBOT_MONITOR_H_
#define _MRTA_ROBOT_MONITOR_H_

#include "mrta/monitor.h"

namespace mrta
{
class Robot;
class RobotMonitor : public Monitor
{
  Q_OBJECT
public:
  RobotMonitor(System* system, utilities::MessageSubscriberRegistry* registry,
               Config* config, int state);
  virtual ~RobotMonitor();

protected:
  void update(const QString& id);

protected slots:
  void add(const QString& id);

private:
  int state_;
  void add(Robot* robot);
};
}

#endif // _MRTA_ROBOT_MONITOR_H_
