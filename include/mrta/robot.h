#ifndef _MRTA_ROBOT_H_
#define _MRTA_ROBOT_H_

#include <QMap>
#include <QVector>
#include <QObject>
#include "mrta/taxonomy.h"

namespace rqt_mrta
{
namespace config
{
class ParamsArray;
namespace application
{
class Robot;
}
}
}

namespace mrta
{
class History;
class StateMonitor;
class System;
class Task;
class Robot : public QObject
{
  Q_OBJECT
public:
  typedef QList<Task*>::iterator iterator;
  typedef QList<Task*>::const_iterator const_iterator;
  typedef rqt_mrta::config::application::Robot Config;
  typedef Taxonomy::RobotType Type;
  enum State
  {
    Idle,
    Busy,
    Offline,
    STATE_COUNT
  };
  Robot(System* parent = NULL, Config* config = NULL);
  Robot(const Robot& robot);
  virtual ~Robot();
  Config* getConfig() const;
  QString getId() const;
  Type getType() const;
  State getState() const;
  History* getHistory() const;
  StateMonitor* getStateMonitor() const;
  void setState(State state);
  void setConfig(Config* config);
  size_t count() const;
  Task* getTask(int index) const;
  void addTask(Task* task);
  void removeTask(Task* task);
  void clearTasks();
  Robot& operator=(const Robot& robot);

public slots:
  void setId(const QString& id);

signals:
  void changed();
  void idChanged(const QString& id);
  void stateChanged(int state);
  void idle();
  void busy();
  void offline();
  void added(size_t index);
  void removed(const QString& task_id);
  void taskChanged();
  void taskIdChanged(const QString& task_id);

private:
  QString id_;
  Type type_;
  State state_;
  QVector<Task*> tasks_;
  Config* config_;
  History* history_;
  StateMonitor* monitor_;

private slots:
  void configDestroyed();
  void taskDestroyed();
  void monitorUpdated(size_t index, bool up_to_date);
};
}

#endif // _MRTA_ROBOT_H_
