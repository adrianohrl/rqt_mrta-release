#include "mrta/architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/define_architecture_widget.h"
#include "rqt_mrta/ui_define_architecture_widget.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
DefineArchitectureWidget::DefineArchitectureWidget(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaArchitectureConfig* architecture_config)
    : QWidget(parent), ui_(new Ui::DefineArchitectureWidget()),
      application_config_(NULL), architecture_config_(NULL)
{
  if (!application_config)
  {
    throw utilities::Exception(
        "[DefineApplicationWidget] The application config must not be NULL.");
  }
  if (!architecture_config)
  {
    throw utilities::Exception(
        "[DefineApplicationWidget] The architecture config must not be NULL.");
  }
  ui_->setupUi(this);
  connect(ui_->allocations_type_combo_box, SIGNAL(currentIndexChanged(int)),
          this, SLOT(setFilterAllocationType()));
  connect(ui_->robots_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(setFilterRobotType()));
  connect(ui_->tasks_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(setFilterTaskType()));
  connect(ui_->architectures_combo_box, SIGNAL(unknownAchitecture()), this,
          SLOT(unknownAchitecture()));
  connect(ui_->architectures_combo_box,
          SIGNAL(currentArchitectureChanged(mrta::Architecture*)), this,
          SLOT(currentArchitectureChanged(mrta::Architecture*)));
  setApplicationConfig(application_config);
  setArchitectureConfig(architecture_config);
}

DefineArchitectureWidget::~DefineArchitectureWidget()
{
  architecture_config_ = NULL;
  application_config_ = NULL;
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

RqtMrtaArchitectureConfig*
DefineArchitectureWidget::getArchitectureConfig() const
{
  return architecture_config_;
}

RqtMrtaApplicationConfig* DefineArchitectureWidget::getApplicationConfig() const
{
  return application_config_;
}

void DefineArchitectureWidget::setArchitectureConfig(
    RqtMrtaArchitectureConfig* config)
{
  if (architecture_config_ != config)
  {
    if (architecture_config_)
    {
      disconnect(architecture_config_, SIGNAL(changed()), this, SIGNAL(changed()));
    }
    architecture_config_ = config;
    if (architecture_config_)
    {
      connect(config, SIGNAL(changed()), this, SIGNAL(changed()));
    }
  }
}

void DefineArchitectureWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  if (application_config_ != config)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this, SIGNAL(changed()));
    }
    application_config_ = config;
    if (application_config_)
    {
      connect(config, SIGNAL(changed()), this, SIGNAL(changed()));
    }
  }
}

void DefineArchitectureWidget::setFilterAllocationType()
{
  mrta::Taxonomy::AllocationType allocation_type(
      mrta::Taxonomy::getAllocationType(
          ui_->allocations_type_combo_box->currentText()));
  ui_->architectures_combo_box->setFilterAllocationType(allocation_type);
}

void DefineArchitectureWidget::setFilterRobotType()
{
  mrta::Taxonomy::RobotType robot_type(
      mrta::Taxonomy::getRobotType(ui_->robots_type_combo_box->currentText()));
  ui_->architectures_combo_box->setFilterRobotType(robot_type);
}

void DefineArchitectureWidget::setFilterTaskType()
{
  mrta::Taxonomy::TaskType task_type(
      mrta::Taxonomy::getTaskType(ui_->tasks_type_combo_box->currentText()));
  ui_->architectures_combo_box->setFilterTaskType(task_type);
}

void DefineArchitectureWidget::unknownAchitecture()
{
  ui_->taxonomy_label->setText("");
  application_config_->getApplication()->setArchitecturePackage("");
}

void DefineArchitectureWidget::currentArchitectureChanged(
    mrta::Architecture* architecture)
{
  ui_->taxonomy_label->setText(
      architecture ? mrta::Taxonomy::toQString(*architecture) : "");
  application_config_->getApplication()->setArchitecturePackage(
      architecture ? architecture->getPackage() : "");
}
}
