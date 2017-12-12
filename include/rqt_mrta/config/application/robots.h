#ifndef _RQT_MRTA_APPLICATION_CONFIG_ROBOTS_H_
#define _RQT_MRTA_APPLICATION_CONFIG_ROBOTS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/robot.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;

class Robots : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Robots(QObject* parent = NULL);
  virtual ~Robots();
  size_t count() const;
  Robot* getRobot(size_t index) const;
  Robot* addRobot();
  void removeRobot(Robot* robot);
  void removeRobot(size_t index);
  void clearRobots();
  bool contains(const QString& id) const;
  bool isEmpty() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Robots& operator=(const Robots& config);
  QString validate() const;

signals:
  void added(size_t robot_index);
  void removed(const QString& robot_id);
  void cleared();
  void robotIdChanged(size_t index, const QString& robot_id);
  void taskIdChanged(size_t robot_index, size_t task_index,
                     const QString& task_id);
  void taskAdded(size_t robot_index, size_t task_index);
  void taskRemoved(size_t robot_index, const QString& task_id);
  void tasksCleared(size_t robot_index);

private:
  QVector<Robot*> robots_;

private slots:
  void robotIdChanged(const QString& robot_id);
  void taskIdChanged(size_t task_index, const QString& task_id);
  void taskAdded(size_t task_index);
  void taskRemoved(const QString& task_id);
  void tasksCleared();
  void robotDestroyed();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_ROBOTS_H_
