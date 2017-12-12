#ifndef _MRTA_STATE_MONITOR_H_
#define _MRTA_STATE_MONITOR_H_

#include <QVector>
#include <QObject>
#include <ros/duration.h>

namespace mrta
{
class SampleHolder;
class StateMonitor : public QObject
{
  Q_OBJECT
public:
  StateMonitor(QObject* parent, const ros::Duration& timeout,
               size_t count_states);
  virtual ~StateMonitor();
  SampleHolder* getSampleHolder(size_t index) const;
  void setTimeout(ros::Duration timeout);
  void update(size_t index);

signals:
  void changed();
  void updated(size_t index, bool up_to_date);

private:
  QVector<SampleHolder*> sample_holders_;

private slots:
  void updated(bool up_to_date);
};
}

#endif // _MRTA_STATE_MONITOR_H_
