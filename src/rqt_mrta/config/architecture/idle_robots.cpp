#include "rqt_mrta/config/architecture/idle_robots.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
IdleRobots::IdleRobots(QObject* parent)
    : AbstractConfig(parent), topic_(new Topic(this))
{
  connect(topic_, SIGNAL(changed()), this,
          SLOT(topicChanged()));
}

IdleRobots::~IdleRobots()
{
  if (topic_)
  {
    delete topic_;
    topic_ = NULL;
  }
}

Topic* IdleRobots::getTopic() const
{
  return topic_;
}

void IdleRobots::save(QSettings& settings) const
{
  settings.beginGroup("idle_robots");
  topic_->save(settings);
  settings.endGroup();
}

void IdleRobots::load(QSettings& settings)
{
  settings.beginGroup("idle_robots");
  topic_->load(settings);
  settings.endGroup();
}

void IdleRobots::reset()
{
  topic_->reset();
}

void IdleRobots::write(QDataStream& stream) const
{
  topic_->write(stream);
}

void IdleRobots::read(QDataStream& stream)
{
  topic_->read(stream);
}

IdleRobots& IdleRobots::operator=(const IdleRobots& config)
{
  *topic_ = *config.topic_;
  return *this;
}

void IdleRobots::topicChanged() { emit changed(); }
}
}
}
