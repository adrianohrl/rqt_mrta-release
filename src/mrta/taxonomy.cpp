#include "mrta/taxonomy.h"
#include "mrta/architecture.h"

namespace mrta
{
Taxonomy::AllocationType Taxonomy::getAllocationType(const QString& type)
{
  if (type == "instantaneous_assignment" || type == "IA" || type == "ia" ||
      type == "INSTANTANEOUS_ASSIGNMENT" || type == "InstantaneousAssignment" ||
      type == "Instantaneous Assignment" ||
      type == "Instantaneous Assignment (IA)")
  {
    return Taxonomy::InstantaneousAssignment;
  }
  else if (type == "time_extended_assignment" || type == "TA" || type == "ta" ||
           type == "TIME_EXTENDED_ASSIGNMENT" ||
           type == "TimeExtendedAssignment" ||
           type == "Time-Extended Assignment" ||
           type == "Time-Extended Assignment (TA)")
  {
    return Taxonomy::TimeExtendedAssignment;
  }
  return Taxonomy::UnknownAllocationType;
}

Taxonomy::RobotType Taxonomy::getRobotType(const QString& type)
{
  if (type == "single_task" || type == "ST" || type == "st" ||
      type == "SINGLE_TASK" || type == "SingleTask" || type == "Single Task" ||
      type == "Single Task (ST)")
  {
    return Taxonomy::SingleTask;
  }
  else if (type == "multi_task" || type == "MT" || type == "mt" ||
           type == "MULTI_TASK" || type == "MultiTask" ||
           type == "Multi Task" || type == "Multi Task (MT)")
  {
    return Taxonomy::MultiTask;
  }
  return Taxonomy::UnknownRobotType;
}

Taxonomy::TaskType Taxonomy::getTaskType(const QString& type)
{
  if (type == "single_robot" || type == "SR" || type == "sr" ||
      type == "SINGLE_ROBOT" || type == "SingleRobot" ||
      type == "Single Robot" || type == "Single Robot (SR)")
  {
    return Taxonomy::SingleRobot;
  }
  else if (type == "multi_robot" || type == "MR" || type == "mr" ||
           type == "MULTI_ROBOT" || type == "MultiRobot" ||
           type == "Multi Robot" || type == "Multi Robot (MR)")
  {
    return Taxonomy::MultiRobot;
  }
  return Taxonomy::UnknownTaskType;
}

QString Taxonomy::toQString(const Taxonomy::AllocationType& type)
{
  return type == Taxonomy::InstantaneousAssignment
             ? "IA"
             : type == Taxonomy::TimeExtendedAssignment ? "TA" : "";
}

QString Taxonomy::toQString(const Taxonomy::RobotType& type)
{
  return type == Taxonomy::SingleTask ? "ST"
                                      : type == Taxonomy::MultiTask ? "MT" : "";
}

QString Taxonomy::toQString(const Taxonomy::TaskType& type)
{
  return type == Taxonomy::SingleRobot ? "SR" : type == Taxonomy::MultiRobot
                                                    ? "MR"
                                                    : "";
}

QString Taxonomy::toQString(const Architecture& architecture)
{
  return Taxonomy::toQString(architecture.getRobotType()) + "-" +
         Taxonomy::toQString(architecture.getTaskType()) + "-" +
         Taxonomy::toQString(architecture.getAllocationType());
}

std::string Taxonomy::toString(const Taxonomy::AllocationType& type)
{
  return Taxonomy::toQString(type).toStdString();
}

std::string Taxonomy::toString(const Taxonomy::RobotType& type)
{
  return Taxonomy::toQString(type).toStdString();
}

std::string Taxonomy::toString(const Taxonomy::TaskType& type)
{
  return Taxonomy::toQString(type).toStdString();
}

std::string Taxonomy::toString(const Architecture& architecture)
{
  return Taxonomy::toQString(architecture).toStdString();
}

const char* Taxonomy::toCString(const Taxonomy::AllocationType& type)
{
  return Taxonomy::toString(type).c_str();
}

const char* Taxonomy::toCString(const Taxonomy::RobotType& type)
{
  return Taxonomy::toString(type).c_str();
}

const char* Taxonomy::toCString(const Taxonomy::TaskType& type)
{
  return Taxonomy::toString(type).c_str();
}

const char* Taxonomy::toCString(const Architecture& architecture)
{
  return Taxonomy::toString(architecture).c_str();
}
}
