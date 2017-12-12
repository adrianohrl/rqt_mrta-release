#include "rqt_mrta/config/application/robots.h"
#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/tasks.h"
#include "rqt_mrta/config/application/task.h"
#include "rqt_mrta/robot_tree_widget.h"

#include <ros/console.h>

Q_DECLARE_METATYPE(rqt_mrta::config::application::Robots*)
Q_DECLARE_METATYPE(rqt_mrta::config::application::Robot*)
Q_DECLARE_METATYPE(rqt_mrta::config::application::Tasks*)
Q_DECLARE_METATYPE(rqt_mrta::config::application::Task*)

namespace rqt_mrta
{
RobotTreeWidget::RobotTreeWidget(QWidget* parent)
    : QTreeWidget(parent), config_(NULL), current_type_(None)
{
  current_.robot_ = NULL;
  setColumnCount(1);
  headerItem()->setText(0, "");
  connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
          this, SLOT(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
}

RobotTreeWidget::~RobotTreeWidget() { config_ = NULL; }

RobotsConfig* RobotTreeWidget::getConfig() const { return config_; }

void RobotTreeWidget::setConfig(RobotsConfig* config)
{
  if (config_ != config)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(changed()), this, SLOT(robotsConfigChanged()));
      disconnect(config_, SIGNAL(robotIdChanged(size_t, const QString&)), this,
                 SLOT(configRobotIdChanged(size_t, const QString&)));
      disconnect(config_, SIGNAL(added(size_t)), this,
                 SLOT(configRobotAdded(size_t)));
      disconnect(config_, SIGNAL(removed(const QString&)), this,
                 SLOT(configRobotRemoved(const QString&)));
      disconnect(config_, SIGNAL(cleared()), this, SLOT(configRobotsCleared()));
      disconnect(config_, SIGNAL(taskIdChanged(size_t, size_t, const QString&)),
                 this,
                 SLOT(configTaskIdChanged(size_t, size_t, const QString&)));
      disconnect(config_, SIGNAL(taskAdded(size_t, size_t)), this,
                 SLOT(configTaskAdded(size_t, size_t)));
      disconnect(config_, SIGNAL(taskRemoved(size_t, const QString&)), this,
                 SLOT(configTaskRemoved(size_t, const QString&)));
      disconnect(config_, SIGNAL(tasksCleared(size_t)), this,
                 SLOT(configTasksCleared(size_t)));
      clear();
    }
    config_ = config;
    if (config_)
    {
      connect(config_, SIGNAL(changed()), this, SLOT(robotsConfigChanged()));
      connect(config_, SIGNAL(robotIdChanged(size_t, const QString&)), this,
              SLOT(configRobotIdChanged(size_t, const QString&)));
      connect(config_, SIGNAL(added(size_t)), this,
              SLOT(configRobotAdded(size_t)));
      connect(config_, SIGNAL(removed(const QString&)), this,
              SLOT(configRobotRemoved(const QString&)));
      connect(config_, SIGNAL(cleared()), this, SLOT(configRobotsCleared()));
      connect(config_, SIGNAL(taskIdChanged(size_t, size_t, const QString&)),
              this, SLOT(configTaskIdChanged(size_t, size_t, const QString&)));
      connect(config_, SIGNAL(taskAdded(size_t, size_t)), this,
              SLOT(configTaskAdded(size_t, size_t)));
      connect(config_, SIGNAL(taskRemoved(size_t, const QString&)), this,
              SLOT(configTaskRemoved(size_t, const QString&)));
      connect(config_, SIGNAL(tasksCleared(size_t)), this,
              SLOT(configTasksCleared(size_t)));
      for (size_t i(0); i < config_->count(); i++)
      {
        configRobotAdded(i);
      }
    }
  }
}

QStringList RobotTreeWidget::getTasks() const
{
  QStringList tasks;
  for (size_t i(0); i < config_->count(); i++)
  {
    RobotConfig* robot = config_->getRobot(i);
    for (size_t j(0); j < robot->getTasks()->count(); j++)
    {
      QString task_id(robot->getTasks()->getTask(j)->getId());
      if (!tasks.contains(task_id))
      {
        tasks.append(task_id);
      }
    }
  }
  return tasks;
}

QString RobotTreeWidget::getCurrentId() const
{
  return current_type_ == Robot
             ? current_.robot_->getId()
             : current_type_ == Task ? current_.task_->getId() : "";
}

void RobotTreeWidget::setCurrentId(const QString& id)
{
  if (current_type_ == Robot)
  {
    current_.robot_->setId(id);
  }
  else if (current_type_ == Task)
  {
    current_.task_->setId(id);
  }
}

void RobotTreeWidget::addRobot()
{
  if (config_)
  {
    RobotConfig* robot = config_->addRobot();
    int i(0);
    while (config_->contains("robot" + QString::number(++i)));
    robot->setId("robot" + QString::number(i));
  }
}

void RobotTreeWidget::addRobot(RobotConfig* robot)
{
  QTreeWidgetItem* root = invisibleRootItem();
  QTreeWidgetItem* item = new QTreeWidgetItem(root);
  item->setText(0, robot->getId());
  item->setData(0, Qt::UserRole, QVariant::fromValue<RobotConfig*>(robot));
  root->addChild(item);
  emit robotAdded(robot->getId());
  emit changed();
  for (size_t j(0); j < robot->getTasks()->count(); j++)
  {
    addTask(robot->getTasks()->getTask(j), item);
  }
  setCurrentItem(item);
}

void RobotTreeWidget::addTask()
{
  QTreeWidgetItem* current = currentItem();
  if (current)
  {
    QTreeWidgetItem* parent = current->parent();
    QTreeWidgetItem* robot_item = !parent ? current : parent;
    RobotConfig* robot =
        robot_item->data(0, Qt::UserRole).value<RobotConfig*>();
    TaskConfig* task = robot->getTasks()->addTask();
    int i(0);
    while (robot->getTasks()->contains("task" + QString::number(++i)));
    task->setId("task" + QString::number(i));
  }
}

void RobotTreeWidget::addTask(TaskConfig* task, QTreeWidgetItem* parent)
{
  QTreeWidgetItem* item = new QTreeWidgetItem(parent);
  item->setText(0, task->getId());
  item->setData(0, Qt::UserRole, QVariant::fromValue<TaskConfig*>(task));
  parent->addChild(item);
  setCurrentItem(item);
  emit taskAdded(task->getId());
  emit changed();
}

void RobotTreeWidget::robotsConfigChanged() { emit changed(); }

void RobotTreeWidget::configRobotIdChanged(size_t robot_index,
                                           const QString& robot_id)
{
  QTreeWidgetItem* item = invisibleRootItem()->child(robot_index);
  item->setText(0, robot_id);
  emit changed();
}

void RobotTreeWidget::configRobotAdded(size_t index)
{
  addRobot(config_->getRobot(index));
}

void RobotTreeWidget::configRobotRemoved(const QString& robot_id) {}

void RobotTreeWidget::configRobotsCleared() { clear(); }

void RobotTreeWidget::configTaskIdChanged(size_t robot_index, size_t task_index,
                                          const QString& task_id)
{
  QTreeWidgetItem* item =
      invisibleRootItem()->child(robot_index)->child(task_index);
  item->setText(0, task_id);
  emit changed();
}

void RobotTreeWidget::configTaskAdded(size_t robot_index, size_t task_index)
{
  TaskConfig* task =
      config_->getRobot(robot_index)->getTasks()->getTask(task_index);
  addTask(task, invisibleRootItem()->child(robot_index));
}

void RobotTreeWidget::configTaskRemoved(size_t robot_index,
                                        const QString& task_id)
{
}

void RobotTreeWidget::configTasksCleared(size_t robot_index) {}

QString RobotTreeWidget::validate() const
{
  return config_ ? config_->validate() : "";
}

void RobotTreeWidget::clear()
{
  QTreeWidget::clear();
  current_type_ = None;
  current_.robot_ = NULL;
  emit selected(None, "");
  emit changed();
}

void RobotTreeWidget::currentItemChanged(QTreeWidgetItem* current,
                                         QTreeWidgetItem* previous)
{
  if (!current)
  {
    current_type_ = None;
    current_.robot_ = NULL;
    emit selected(None, "");
    return;
  }
  else if (!current->parent())
  {
    current_type_ = Robot;
    current_.robot_ = current->data(0, Qt::UserRole).value<RobotConfig*>();
    emit robotSelected(current_.robot_->getId());
    emit selected(Robot, current_.robot_->getId());
  }
  else if (!current->parent()->parent())
  {
    current_type_ = Task;
    current_.task_ = current->data(0, Qt::UserRole).value<TaskConfig*>();
    emit taskSelected(current_.task_->getId());
    emit selected(Task, current_.task_->getId());
  }
}
}
