#ifndef _MRTA_PROBLEM_H_
#define _MRTA_PROBLEM_H_

#include <QMap>
#include <QVector>
#include <QObject>
#include "mrta/taxonomy.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class RqtMrtaArchitecture;
}
}
}

namespace mrta
{
typedef rqt_mrta::config::architecture::RqtMrtaArchitecture ArchitectureConfig;
class Allocation;
class Architecture;
class Robot;
class System;
class Task;
class Problem : public QObject
{
  Q_OBJECT
public:
  Problem(System* system, const QString& name, ArchitectureConfig* architecture_config);
  virtual ~Problem();
  QString getName() const;
  Architecture* getArchitecture() const;
  Task* getTask(const QString& id) const;
  Allocation* getAllocation(const QString& id) const;
  QList<Task*> getTasks() const;
  QList<Allocation*> getAllocations() const;
  void setArchitecture(Architecture* architecture);
  Allocation* addAllocation(Task* task, const QVector<Robot*>& robots);
  bool isValid(Taxonomy::AllocationType type) const;
  bool isValid(Taxonomy::RobotType type) const;
  bool isValid(Taxonomy::TaskType type) const;

signals:
  void changed();
  void taskStateChanged(const QString& task, int state);
  void allocationAdded(const QString& id);
  void allocationStateChanged(const QString& id, int state);

private:
  typedef QMap<QString, Task*> TaskMap;
  typedef QMap<QString, Allocation*> AllocationMap;
  QString name_;
  System* system_;
  Architecture* architecture_;
  TaskMap tasks_;
  AllocationMap allocations_;

private:
  void taskStateChanged(int state);
  void allocationStateChanged(int state);
};
}

#endif // _MRTA_PROBLEM_H_
