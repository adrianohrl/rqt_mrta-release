#include "mrta/architecture.h"
#include "mrta/architecture_config.h"
#include "utilities/exception.h"

namespace mrta
{
ArchitectureConfig::ArchitectureConfig(QObject* parent)
    : AbstractConfig(parent), architecture_(NULL)
{
}

Architecture* ArchitectureConfig::getArchitecture() const
{
  return architecture_;
}

void ArchitectureConfig::setArchitecture(Architecture* architecture)
{
  architecture_ = architecture;
}

void ArchitectureConfig::load(QSettings& settings)
{
  if (!architecture_)
  {
    throw utilities::Exception("Architecture must not be NULL.");
  }
  settings.beginGroup("architecture");
  architecture_->setName(settings.value("name").toString());
  architecture_->setAllocationType(Taxonomy::getAllocationType(
      settings.value("allocations/type", "").toString()));
  architecture_->setRobotType(Taxonomy::getRobotType(
      settings.value("robots/type", "").toString()));
  architecture_->setTaskType(Taxonomy::getTaskType(
      settings.value("tasks/type", "").toString()));
  settings.endGroup();
}
}
