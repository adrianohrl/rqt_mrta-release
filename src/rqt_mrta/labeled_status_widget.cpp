#include "mrta/robot.h"
#include "mrta/task.h"
#include "rqt_mrta/labeled_status_widget.h"
#include "rqt_mrta/ui_labeled_status_widget.h"

namespace rqt_mrta
{
LabeledStatusWidget::LabeledStatusWidget(QWidget* parent, mrta::Robot* robot)
    : QWidget(parent), ui_(new Ui::LabeledStatusWidget())
{
  ui_->setupUi(this);
  connect(robot, SIGNAL(idle()), this, SLOT(setGreen()));
  connect(robot, SIGNAL(busy()), this, SLOT(setYellow()));
  connect(robot, SIGNAL(offline()), this, SLOT(setGray()));
  connect(robot, SIGNAL(idChanged(const QString&)), this,
          SLOT(setLabel(const QString&)));
  switch (robot->getState())
  {
  case mrta::Robot::Idle:
    setGreen();
    break;
  case mrta::Robot::Busy:
    setYellow();
    break;
  case mrta::Robot::Offline:
    setGray();
    break;
  }
  setLabel(robot->getId());
}

LabeledStatusWidget::LabeledStatusWidget(QWidget* parent, mrta::Task* task) {}

LabeledStatusWidget::~LabeledStatusWidget()
{
  ROS_INFO_STREAM("[~LabeledStatusWidget] before ...");
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
  ROS_INFO_STREAM("[~LabeledStatusWidget] after ...");
}

void LabeledStatusWidget::setGreen()
{
  ui_->status_widget->setCurrentRole(StatusWidget::Green, "");
}

void LabeledStatusWidget::setYellow()
{
  ui_->status_widget->setCurrentRole(StatusWidget::Yellow, "");
}

void LabeledStatusWidget::setRed()
{
  ui_->status_widget->setCurrentRole(StatusWidget::Red, "");
}

void LabeledStatusWidget::setBlue()
{
  ui_->status_widget->setCurrentRole(StatusWidget::Blue, "");
}

void LabeledStatusWidget::setGray()
{
  ui_->status_widget->setCurrentRole(StatusWidget::Gray, "");
}

void LabeledStatusWidget::setLabel(const QString& label)
{
  ui_->label->setText(label);
}

void LabeledStatusWidget::objectDestroyed()
{
  ui_->label->setText("");
  ui_->status_widget->setCurrentRole(StatusWidget::None, "");
}
}
