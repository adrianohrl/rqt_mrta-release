#include "mrta/architecture.h"
#include "mrta/architecture_config.h"
#include <QFileInfo>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "utilities/simple_xml_settings.h"

namespace mrta
{
Architecture::Architecture(QObject* parent, Config* config) : QObject(parent)
{
  setAllocationType(Taxonomy::getAllocationType(
      config->getArchitecture()->getAllocations()->getType()));
  setRobotType(Taxonomy::getRobotType(
      config->getArchitecture()->getRobots()->getType()));
  setTaskType(
      Taxonomy::getTaskType(config->getArchitecture()->getTasks()->getType()));
}

Architecture::Architecture(QObject* parent, const QString& package,
                           const QString& config_file_path)
    : QObject(parent), package_(package), config_file_path_(config_file_path)
{
  ArchitectureConfig config;
  config.setArchitecture(this);
  if (!config_file_path.isEmpty())
  {
    QFileInfo file_info(config_file_path);
    if (file_info.isReadable())
    {
      QSettings settings(config_file_path, utilities::SimpleXmlSettings::format);
      if (settings.status() == QSettings::NoError)
      {
        settings.beginGroup("rqt_mrta");
        config.load(settings);
        settings.endGroup();
      }
    }
  }
}

Architecture::~Architecture()
{
  ROS_INFO_STREAM("[~Architecture]");
}

QString Architecture::getPackage() const { return package_; }

QString Architecture::getConfigFilePath() const { return config_file_path_; }

QString Architecture::getName() const { return name_; }

Taxonomy::AllocationType Architecture::getAllocationType() const
{
  return allocation_type_;
}

Taxonomy::RobotType Architecture::getRobotType() const { return robot_type_; }

Taxonomy::TaskType Architecture::getTaskType() const { return task_type_; }

void Architecture::setName(const QString& name) { name_ = name; }

void Architecture::setAllocationType(const Taxonomy::AllocationType& type)
{
  allocation_type_ = type;
}

void Architecture::setRobotType(const Taxonomy::RobotType& type)
{
  robot_type_ = type;
}

void Architecture::setTaskType(const Taxonomy::TaskType& type)
{
  task_type_ = type;
}

bool Architecture::belongs(const Taxonomy::AllocationType& allocation_type,
                           const Taxonomy::RobotType& robot_type,
                           const Taxonomy::TaskType& task_type) const
{
  return (allocation_type_ == allocation_type ||
          allocation_type == Taxonomy::UnknownAllocationType) &&
         (robot_type_ == robot_type ||
          robot_type == Taxonomy::UnknownRobotType) &&
         (task_type_ == task_type || task_type == Taxonomy::UnknownTaskType);
}

QString Architecture::toString() const { return package_; }

bool Architecture::operator==(const QString& package) const
{
  return package_ == package;
}

bool Architecture::operator==(const Architecture& architecture) const
{
  return package_ == architecture.package_;
}

bool Architecture::isValid(Taxonomy::AllocationType type) const
{
  return allocation_type_ == type;
}

bool Architecture::isValid(Taxonomy::RobotType type) const
{
  return robot_type_ == type;
}

bool Architecture::isValid(Taxonomy::TaskType type) const
{
  return task_type_ == type;
}
}
