#include "mrta/sample_holder.h"
#include "mrta/state_monitor.h"
#include <ros/console.h>

namespace mrta
{
StateMonitor::StateMonitor(QObject* parent, const ros::Duration& timeout,
                           size_t count_states)
    : QObject(parent)
{
  for (size_t index(0); index < count_states; index++)
  {
    SampleHolder* sample_holder = new SampleHolder(this, timeout);
    connect(sample_holder, SIGNAL(changed()), this, SIGNAL(changed()));
    connect(sample_holder, SIGNAL(updated(bool)), this, SLOT(updated(bool)));
    sample_holders_.append(sample_holder);
  }
}

StateMonitor::~StateMonitor()
{
  ROS_INFO_STREAM("[~StateMonitor] before");
  for (size_t index(0); index < sample_holders_.count(); index++)
  {
    if (sample_holders_[index])
    {
      delete sample_holders_[index];
      sample_holders_[index] = NULL;
    }
  }
  sample_holders_.clear();
  ROS_INFO_STREAM("[~StateMonitor] after");
}

SampleHolder* StateMonitor::getSampleHolder(size_t index) const
{
  return index >= 0 && index < sample_holders_.count() ? sample_holders_[index]
                                                         : NULL;
}

void StateMonitor::setTimeout(ros::Duration timeout)
{
  for (size_t index(0); index < sample_holders_.count(); index++)
  {
    sample_holders_[index]->setTimeout(timeout);
  }
}

void StateMonitor::update(size_t index)
{
  if (index >= 0 && index < sample_holders_.count())
  {
    sample_holders_[index]->update();
  }
}

void StateMonitor::updated(bool up_to_date)
{
  int index(sample_holders_.indexOf(static_cast<SampleHolder*>(sender())));
  if (index != -1)
  {
    emit updated(index, up_to_date);
    emit changed();
  }
}
}
