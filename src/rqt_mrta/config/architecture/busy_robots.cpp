#include "rqt_mrta/config/architecture/busy_robots.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
BusyRobots::BusyRobots(QObject* parent)
    : AbstractConfig(parent), topic_(new Topic(this))
{
  connect(topic_, SIGNAL(changed()), this,
          SLOT(topicChanged()));
}

BusyRobots::~BusyRobots()
{
  if (topic_)
  {
    delete topic_;
    topic_ = NULL;
  }
}

Topic* BusyRobots::getTopic() const
{
  return topic_;
}

void BusyRobots::save(QSettings& settings) const
{
  settings.beginGroup("busy_robots");
  topic_->save(settings);
  settings.endGroup();
}

void BusyRobots::load(QSettings& settings)
{
  settings.beginGroup("busy_robots");
  topic_->load(settings);
  settings.endGroup();
}

void BusyRobots::reset()
{
  topic_->reset();
}

void BusyRobots::write(QDataStream& stream) const
{
  topic_->write(stream);
}

void BusyRobots::read(QDataStream& stream)
{
  topic_->read(stream);
}

BusyRobots& BusyRobots::operator=(const BusyRobots& config)
{
  *topic_ = *config.topic_;
  return *this;
}

void BusyRobots::topicChanged() { emit changed(); }
}
}
}
