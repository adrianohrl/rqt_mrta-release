#ifndef _MRTA_ALLOCATION_H_
#define _MRTA_ALLOCATION_H_

#include <QVector>
#include <QObject>
#include "mrta/taxonomy.h"

namespace mrta
{
class Problem;
class Robot;
class Task;
class Allocation : public QObject
{
  Q_OBJECT
public:
  enum State
  {
    OnHold,
    Assigned,
    Done
  };
  typedef Taxonomy::AllocationType Type;
  Allocation(Problem* problem, Task* task,
             const QVector<Robot*>& robots = QVector<Robot*>());
  virtual ~Allocation();
  QString getId() const;
  Task* getTask() const;
  QVector<Robot*> getRobots() const;
  void setRobots(const QVector<Robot*>& robots);
  void setState(State state);

signals:
  void changed();
  void stateChanged(int state);

private:
  QString id_;
  Type type_;
  State state_;
  Task* task_;
  QVector<Robot*> robots_;

private slots:
  void robotStateChanged(int state);
  void taskStateChanged(int state);
};
}

#endif // _MRTA_ALLOCATION_H_
