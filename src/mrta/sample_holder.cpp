#include "mrta/sample_holder.h"
#include "mrta/state_monitor.h"
#include <ros/console.h>

namespace mrta
{
SampleHolder::SampleHolder(StateMonitor* monitor, const ros::Duration& timeout)
    : QObject(monitor), timer_(new QTimer(this))
{
  connect(timer_, SIGNAL(timeout()), this, SLOT(expired()));
  setUpToDate(false);
  setTimeout(timeout);
}

SampleHolder::~SampleHolder()
{
  ROS_INFO_STREAM("[~SampleHolder] before");
  timer_->stop();
  if (timer_)
  {
    delete timer_;
    timer_ = NULL;
  }
  ROS_INFO_STREAM("[~SampleHolder] after");
}

bool SampleHolder::isUpToDate() const { return up_to_date_; }

ros::Duration SampleHolder::getTimeout() const { return timeout_; }

void SampleHolder::setUpToDate(bool up_to_date)
{
  timer_->stop();
  if (up_to_date)
  {
    timer_->start(1e3 * timeout_.toSec());
  }
  if (up_to_date != up_to_date_)
  {
    up_to_date_ = up_to_date;
    emit updated(up_to_date);
    emit changed();
  }
}

void SampleHolder::setTimeout(const ros::Duration& timeout)
{
  if (timeout != timeout_)
  {
    timer_->stop();
    if (up_to_date_)
    {
      timer_->start(1e3 * timeout_.toSec());
    }
    timeout_ = timeout;
    emit timeoutChanged(timeout);
    emit changed();
  }
}

void SampleHolder::update() { setUpToDate(true); }

void SampleHolder::expired() { setUpToDate(false); }
}
