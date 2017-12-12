#ifndef _MRTA_SAMPLE_HOLDER_H_
#define _MRTA_SAMPLE_HOLDER_H_

#include <QTimer>
#include <QObject>
#include <ros/duration.h>

namespace mrta
{
class StateMonitor;
class SampleHolder : public QObject {
  Q_OBJECT
public:
  SampleHolder(StateMonitor* monitor, const ros::Duration& timeout);
  ~SampleHolder();
  bool isUpToDate() const;
  ros::Duration getTimeout() const;
  void setTimeout(const ros::Duration& timeout);
  void update();

signals:
  void changed();
  void updated(bool up_to_date);
  void timeoutChanged(const ros::Duration& timeout);

private:
  bool up_to_date_;
  ros::Duration timeout_;
  QTimer* timer_;
  void setUpToDate(bool up_to_date);

private slots:
  void expired();
};
}

#endif // _MRTA_SAMPLE_HOLDER_H_
