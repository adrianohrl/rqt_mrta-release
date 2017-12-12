#ifndef _MRTA_TAXONOMY_H_
#define _MRTA_TAXONOMY_H_

#include <QString>
#include <ros/console.h>

namespace mrta
{
class Architecture;
class Taxonomy
{
public:
  enum AllocationType
  {
    UnknownAllocationType,
    InstantaneousAssignment,
    TimeExtendedAssignment
  };
  enum RobotType
  {
    UnknownRobotType,
    SingleTask,
    MultiTask
  };
  enum TaskType
  {
    UnknownTaskType,
    SingleRobot,
    MultiRobot
  };
  static AllocationType getAllocationType(const QString& type);
  static RobotType getRobotType(const QString& type);
  static TaskType getTaskType(const QString& type);
  static QString toQString(const AllocationType& type);
  static QString toQString(const RobotType& type);
  static QString toQString(const TaskType& type);
  static QString toQString(const Architecture& architecture);
  static std::string toString(const AllocationType& type);
  static std::string toString(const RobotType& type);
  static std::string toString(const TaskType& type);
  static std::string toString(const Architecture& architecture);
  static const char* toCString(const AllocationType& type);
  static const char* toCString(const RobotType& type);
  static const char* toCString(const TaskType& type);
  static const char* toCString(const Architecture& architecture);
};
}

#endif // _MRTA_TAXONOMY_H_
