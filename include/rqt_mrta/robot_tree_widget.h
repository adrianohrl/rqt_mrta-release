#ifndef _RQT_MRTA_ROBOTS_TREE_WIDGET_H_
#define _RQT_MRTA_ROBOTS_TREE_WIDGET_H_

#include <QList>
#include <QTreeWidget>

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robots;
class Robot;
class Tasks;
class Task;
}
}

typedef config::application::Robots RobotsConfig;
typedef config::application::Robot RobotConfig;
typedef config::application::Tasks TasksConfig;
typedef config::application::Task TaskConfig;

class RobotTreeWidget : public QTreeWidget
{
  Q_OBJECT
public:
  enum Type
  {
    None,
    Robot,
    Task
  };
  RobotTreeWidget(QWidget* parent = NULL);
  virtual ~RobotTreeWidget();
  RobotsConfig* getConfig() const;
  void setConfig(RobotsConfig* config);
  QStringList getTasks() const;
  QString getCurrentId() const;
  void setCurrentId(const QString& id);
  void addRobot();
  void addTask();
  QString validate() const;
  void clear();

signals:
  void changed();
  void robotAdded(const QString& robot_id);
  void taskAdded(const QString& task_id);
  void robotSelected(const QString& robot_id);
  void taskSelected(const QString& task_id);
  void selected(Type type, const QString& id);

private:
  RobotsConfig* config_;
  union Selection
  {
    RobotConfig* robot_;
    TaskConfig* task_;
  };
  Type current_type_;
  Selection current_;
  void addRobot(RobotConfig* robot);
  void addTask(TaskConfig* task, QTreeWidgetItem* parent);

private slots:
  void currentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
  void robotsConfigChanged();
  void configRobotIdChanged(size_t robot_index, const QString& robot_id);
  void configRobotAdded(size_t robot_index);
  void configRobotRemoved(const QString& robot_id);
  void configRobotsCleared();
  void configTaskIdChanged(size_t robot_index, size_t task_index,
                           const QString& task_id);
  void configTaskAdded(size_t robot_index, size_t task_index);
  void configTaskRemoved(size_t robot_index, const QString& task_id);
  void configTasksCleared(size_t robot_index);
};
}

#endif // _RQT_MRTA_ROBOTS_TREE_WIDGET_H_
