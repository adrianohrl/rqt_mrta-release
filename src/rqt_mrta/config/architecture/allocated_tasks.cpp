#include "rqt_mrta/config/architecture/allocated_tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
AllocatedTasks::AllocatedTasks(QObject* parent)
    : AbstractConfig(parent), topic_(new Topic(this))
{
  connect(topic_, SIGNAL(changed()), this,
          SLOT(topicChanged()));
}

AllocatedTasks::~AllocatedTasks()
{
  if (topic_)
  {
    delete topic_;
    topic_ = NULL;
  }
}

Topic* AllocatedTasks::getTopic() const
{
  return topic_;
}

void AllocatedTasks::save(QSettings& settings) const
{
  settings.beginGroup("allocated_tasks");
  topic_->save(settings);
  settings.endGroup();
}

void AllocatedTasks::load(QSettings& settings)
{
  settings.beginGroup("allocated_tasks");
  topic_->load(settings);
  settings.endGroup();
}

void AllocatedTasks::reset()
{
  topic_->reset();
}

void AllocatedTasks::write(QDataStream& stream) const
{
  topic_->write(stream);
}

void AllocatedTasks::read(QDataStream& stream)
{
  topic_->read(stream);
}

AllocatedTasks& AllocatedTasks::operator=(const AllocatedTasks& config)
{
  *topic_ = *config.topic_;
  return *this;
}

void AllocatedTasks::topicChanged() { emit changed(); }
}
}
}
