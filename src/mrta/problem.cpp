#include "mrta/allocation.h"
#include "mrta/architecture.h"
#include "mrta/problem.h"
#include "mrta/robot.h"
#include "mrta/system.h"
#include "mrta/task.h"

namespace mrta
{
Problem::Problem(System* system, const QString& name,
                 ArchitectureConfig* architecture_config)
    : QObject(system), name_(name), architecture_(NULL)
{
  setArchitecture(new Architecture(this, architecture_config));
}

Problem::~Problem()
{
  setParent(NULL);
  ROS_INFO_STREAM("[~Problem] before");
  if (architecture_)
  {
    delete architecture_;
    architecture_ = NULL;
  }
  for (AllocationMap::iterator it(allocations_.begin());
       it != allocations_.end(); it++)
  {
    if (it.value())
    {
      delete it.value();
      allocations_[it.key()] = NULL;
    }
  }
  allocations_.clear();
  for (TaskMap::iterator it(tasks_.begin()); it != tasks_.end(); it++)
  {
    if (it.value())
    {
      delete it.value();
      tasks_[it.key()] = NULL;
    }
  }
  tasks_.clear();
  ROS_INFO_STREAM("[~Problem] after");
}

QString Problem::getName() const { return name_; }

Architecture* Problem::getArchitecture() const { return architecture_; }

Task* Problem::getTask(const QString& id) const
{
  return tasks_.contains(id) ? tasks_[id] : NULL;
}

Allocation* Problem::getAllocation(const QString& id) const
{
  return allocations_.contains(id) ? allocations_[id] : NULL;
}

QList<Task*> Problem::getTasks() const { return tasks_.values(); }

QList<Allocation*> Problem::getAllocations() const
{
  return allocations_.values();
}

void Problem::setArchitecture(Architecture* architecture)
{
  if (architecture != architecture_)
  {
    if (architecture_)
    {
      delete architecture_;
    }
    architecture_ = architecture;
    emit changed();
  }
}

Allocation* Problem::addAllocation(Task* task, const QVector<Robot*>& robots)
{
  Allocation* allocation = new Allocation(this, task, robots);
  allocations_[allocation->getId()] = allocation;
  connect(allocation, SIGNAL(stateChanged(int)), this,
          SLOT(allocationStateChanged(int)));
  emit allocationAdded(allocation->getId());
  return allocation;
}

bool Problem::isValid(Taxonomy::AllocationType type) const
{
  return architecture_->isValid(type);
}

bool Problem::isValid(Taxonomy::RobotType type) const
{
  return architecture_->isValid(type);
}

bool Problem::isValid(Taxonomy::TaskType type) const
{
  return architecture_->isValid(type);
}

void Problem::taskStateChanged(int state)
{
  Task* task = static_cast<Task*>(sender());
  if (task)
  {
    emit taskStateChanged(task->getId(), state);
    emit changed();
  }
}

void Problem::allocationStateChanged(int state)
{
  Allocation* allocation = static_cast<Allocation*>(sender());
  if (allocation)
  {
    emit allocationStateChanged(allocation->getId(), state);
    emit changed();
  }
}
}
