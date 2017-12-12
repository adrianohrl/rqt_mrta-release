#ifndef _MRTA_SYSTEM_H_
#define _MRTA_SYSTEM_H_

#include <QMap>
#include <QObject>
#include <QVector>
#include "mrta/taxonomy.h"

namespace utilities
{
class MessageSubscriberRegistry;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;
class RqtMrtaApplication;
}

namespace architecture
{
class RqtMrtaArchitecture;
}
}
}

namespace mrta
{
typedef rqt_mrta::config::application::Robot RobotConfig;
typedef rqt_mrta::config::application::RqtMrtaApplication ApplicationConfig;
typedef rqt_mrta::config::architecture::RqtMrtaArchitecture ArchitectureConfig;
class Allocation;
class Monitor;
class Problem;
class Robot;
class RobotMonitor;
class Task;
class System : public QObject
{
  Q_OBJECT
public:
  System(QObject* parent, ApplicationConfig* application_config,
         ArchitectureConfig* architecture_config,
         utilities::MessageSubscriberRegistry* registry);
  virtual ~System();
  Robot* getRobot(const QString& id);
  Task* getTask(const QString& id);
  Allocation* getAllocation(const QString& id);
  QList<Robot*> getRobots() const;
  QList<Task*> getTasks() const;
  QList<Allocation*> getAllocations() const;
  void setRegistry(utilities::MessageSubscriberRegistry* registry);

signals:
  void changed();
  void added(const QString& id);
  void robotStateChanged(const QString& id, int state);
  void taskStateChanged(const QString& id, int state);
  void allocationStateChanged(const QString& id, int state);

private:
  typedef QMap<QString, Robot*> RobotMap;
  RobotMap robots_;
  Problem* problem_;
  ApplicationConfig* application_config_;
  ArchitectureConfig* architecture_config_;
  QVector<Monitor*> monitors_;
  Robot* addRobot(RobotConfig* config);

private slots:
  void robotStateChanged(int state);
};
}

#endif // _MRTA_SYSTEM_H_
