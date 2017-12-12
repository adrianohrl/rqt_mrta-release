#include "mrta/robot.h"
#include "mrta/task.h"
#include <QSettings>
#include <ros/console.h>
#include <ros/package.h>
#include "rqt_mrta/config/application/robots.h"
#include "rqt_mrta/config/application/robot.h"
#include "rqt_mrta/config/application/tasks.h"
#include "rqt_mrta/config/application/task.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/define_robots_widget.h"
#include "rqt_mrta/ui_define_robots_widget.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
DefineRobotsWidget::DefineRobotsWidget(QWidget* parent, RqtMrtaApplicationConfig* application_config)
    : QWidget(parent), ui_(new Ui::DefineRobotsWidget()),
      application_config_(NULL), tasks_model_(new QStringListModel(this))
{
  if (!application_config)
  {
    throw utilities::Exception(
        "[DefineApplicationWidget] The application config must not be NULL.");
  }
  ui_->setupUi(this);
  ui_->id_line_edit->setEnabled(false);
  ui_->tasks_list_view->setModel(tasks_model_);
  QIcon new_icon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/new.png")));
  ui_->new_robot_command_link_button->setIcon(new_icon);
  ui_->new_task_command_link_button->setIcon(new_icon);
  ui_->new_task_command_link_button->setEnabled(false);
  connect(ui_->id_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(idChanged(const QString&)));
  connect(ui_->new_robot_command_link_button, SIGNAL(clicked()), this,
          SLOT(newRobotButtonClicked()));
  connect(ui_->new_task_command_link_button, SIGNAL(clicked()), this,
          SLOT(newTaskButtonClicked()));
  connect(ui_->robot_tree_widget, SIGNAL(robotSelected(const QString&)),
          this, SLOT(robotSelected(const QString&)));
  connect(ui_->robot_tree_widget, SIGNAL(taskSelected(const QString&)),
          this, SLOT(taskSelected(const QString&)));
  setApplicationConfig(application_config);
}

DefineRobotsWidget::~DefineRobotsWidget()
{
  application_config_ = NULL;
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

RqtMrtaApplicationConfig* DefineRobotsWidget::getApplicationConfig() const
{
  return application_config_;
}

void DefineRobotsWidget::setApplicationConfig(RqtMrtaApplicationConfig* config)
{
  if (application_config_ != config)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      ui_->robot_tree_widget->setConfig(NULL);
      ui_->new_robot_command_link_button->setEnabled(false);
      ui_->new_robot_command_link_button->setEnabled(false);
    }
    application_config_ = config;
    if (application_config_)
    {
      connect(application_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      ui_->robot_tree_widget->setConfig(application_config_->getApplication()->getRobots());
      ui_->new_robot_command_link_button->setEnabled(true);
    }
  }
}

QString DefineRobotsWidget::validate() const
{
  QString tip(ui_->robot_tree_widget->validate());
  ui_->robot_tree_widget->setToolTip(tip);
  return tip;
}

void DefineRobotsWidget::idChanged(const QString &id)
{
  ui_->robot_tree_widget->setCurrentId(id);
}

void DefineRobotsWidget::newRobotButtonClicked()
{
  ui_->robot_tree_widget->addRobot();
}

void DefineRobotsWidget::newTaskButtonClicked()
{
  ui_->robot_tree_widget->addTask();
}

/*void DefineRobotsWidget::newTaskAdded(const QString &task_id)
{
  tasks_model_->insertRow(tasks_model_->rowCount());
  QModelIndex index = tasks_model_->index(tasks_model_->rowCount() - 1);
  tasks_model_->setData(index, task_id);
}*/

void DefineRobotsWidget::robotSelected(const QString &robot_id)
{
  ui_->id_line_edit->setText(robot_id);
  ui_->new_task_command_link_button->setEnabled(true);
  ui_->id_line_edit->setEnabled(true);
}

void DefineRobotsWidget::taskSelected(const QString &task_id)
{
  ui_->id_line_edit->setText(task_id);
  ui_->new_task_command_link_button->setEnabled(false);
  ui_->id_line_edit->setEnabled(true);
}
}
