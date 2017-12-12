#include "mrta/taxonomy.h"
#include "rqt_mrta/config/architecture/architecture.h"
#include "rqt_mrta/config/architecture/allocations.h"
#include "rqt_mrta/config/architecture/robots.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/architecture/tasks.h"
#include "rqt_mrta/register_architecture_widget.h"
#include "rqt_mrta/ui_register_architecture_widget.h"

namespace rqt_mrta
{
RegisterArchitectureWidget::RegisterArchitectureWidget(QWidget* parent,
                                                       Config* config)
    : QWidget(parent), ui_(new Ui::RegisterArchitectureWidget()), config_(NULL)
{
  ui_->setupUi(this);
  setConfig(config);
  connect(ui_->allocations_type_combo_box, SIGNAL(currentIndexChanged(int)),
          this, SLOT(robotTypeChanged()));
  connect(ui_->robots_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(taskTypeChanged()));
  connect(ui_->tasks_type_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(allocationTypeChanged()));
  connect(ui_->architecture_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(packageChanged(const QString&)));
}

RegisterArchitectureWidget::~RegisterArchitectureWidget()
{
  config_ = NULL;
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

Config* RegisterArchitectureWidget::getConfig() const { return config_; }

void RegisterArchitectureWidget::setConfig(Config* config)
{
  if (config != config_)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
    }
    config_ = config;
    if (config_)
    {
      ArchitectureConfig* architecture_config = config_->getArchitecture();
      RobotsConfig* robots_config = architecture_config->getRobots();
      TasksConfig* tasks_config = architecture_config->getTasks();
      AllocationsConfig* allocations_config =
          architecture_config->getAllocations();
      connect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(config_, SIGNAL(architecturePackageChanged(const QString&)), this,
              SLOT(configPackageChanged(const QString&)));
      connect(robots_config, SIGNAL(typeChanged(const QString&)), this,
              SLOT(configRobotsTypeChanged(const QString&)));
      connect(tasks_config, SIGNAL(typeChanged(const QString&)), this,
              SLOT(configTasksTypeChanged(const QString&)));
      connect(allocations_config, SIGNAL(typeChanged(const QString&)), this,
              SLOT(configAllocationsTypeChanged(const QString&)));
      configPackageChanged(config_->getArchitecturePackage());
      configRobotsTypeChanged(robots_config->getType());
      configTasksTypeChanged(tasks_config->getType());
      configAllocationsTypeChanged(allocations_config->getType());
    }
  }
}

QString RegisterArchitectureWidget::validate() const
{
  if (!config_)
  {
    return "The ArchitectureConfig must be given.";
  }
  if (config_->getArchitecturePackage().isEmpty())
  {
    return "The architecture package must be given.";
  }
  if (config_->getArchitecture()->getRobots()->getType().isEmpty())
  {
    return "The robots type must be selected.";
  }
  if (config_->getArchitecture()->getTasks()->getType().isEmpty())
  {
    return "The tasks type must be selected.";
  }
  if (config_->getArchitecture()->getAllocations()->getType().isEmpty())
  {
    return "The allocations type must be selected.";
  }
  return "";
}

void RegisterArchitectureWidget::packageChanged(const QString& package)
{
  config_->setArchitecturePackage(package);
}

void RegisterArchitectureWidget::robotTypeChanged()
{
  mrta::Taxonomy::RobotType type(
      mrta::Taxonomy::getRobotType(ui_->robots_type_combo_box->currentText()));
  config_->getArchitecture()->getRobots()->setType(
      mrta::Taxonomy::toQString(type));
}

void RegisterArchitectureWidget::taskTypeChanged()
{
  mrta::Taxonomy::TaskType type(
      mrta::Taxonomy::getTaskType(ui_->tasks_type_combo_box->currentText()));
  config_->getArchitecture()->getTasks()->setType(
      mrta::Taxonomy::toQString(type));
}

void RegisterArchitectureWidget::allocationTypeChanged()
{
  mrta::Taxonomy::AllocationType type(mrta::Taxonomy::getAllocationType(
      ui_->allocations_type_combo_box->currentText()));
  config_->getArchitecture()->getAllocations()->setType(
      mrta::Taxonomy::toQString(type));
}

void RegisterArchitectureWidget::configPackageChanged(const QString& package)
{
  ui_->architecture_line_edit->setText(package);
}

void RegisterArchitectureWidget::configRobotsTypeChanged(const QString& type)
{
  ui_->robots_type_combo_box->setCurrentIndex(mrta::Taxonomy::getRobotType(type));
}

void RegisterArchitectureWidget::configTasksTypeChanged(const QString& type)
{
  ui_->tasks_type_combo_box->setCurrentIndex(mrta::Taxonomy::getTaskType(type));
}

void RegisterArchitectureWidget::configAllocationsTypeChanged(
    const QString& type)
{
  ui_->allocations_type_combo_box->setCurrentIndex(mrta::Taxonomy::getAllocationType(type));
}
}
