#include <QDir>
#include <QPushButton>
#include <QVBoxLayout>
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/define_application_wizard_page.h"
#include "rqt_mrta/define_architecture_wizard_page.h"
#include "rqt_mrta/define_robots_wizard_page.h"
#include "rqt_mrta/define_parameters_wizard_page.h"
#include <ros/console.h>
#include <ros/package.h>
#include "utilities/exception.h"

namespace rqt_mrta
{
NewApplicationWizard::NewApplicationWizard(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    Qt::WindowFlags flags)
    : QWizard(parent, flags), application_config_(application_config),
      package_config_(new RqtMrtaApplicationPackageConfig(this)),
      architecture_config_(new RqtMrtaArchitectureConfig(this)),
      summary_label_(new QLabel(this))
{
  if (!application_config_)
  {
    throw utilities::Exception(
        "The application configuration must not be null.");
  }
  setPage(DefineApplication, new DefineApplicationWizardPage(this));
  setPage(DefineArchitecture, new DefineArchitectureWizardPage(this));
  setPage(DefineRobots, new DefineRobotsWizardPage(this));
  setPage(DefineParameters, new DefineParametersWizardPage(this));
  setPage(Summary, createSummaryPage());
  setWindowTitle("New application ...");
  connect(this, SIGNAL(currentIdChanged(int)), this, SLOT(summary()));
  connect(this, SIGNAL(accepted()), this, SLOT(generate()));
  connect(this, SIGNAL(rejected()), this, SLOT(resetConfig()));
}

NewApplicationWizard::~NewApplicationWizard()
{
  application_config_ = NULL;
  if (architecture_config_)
  {
    delete architecture_config_;
    architecture_config_ = NULL;
  }
  if (package_config_)
  {
    delete package_config_;
    package_config_ = NULL;
  }
}

RqtMrtaApplicationConfig* NewApplicationWizard::getApplicationConfig() const
{
  return application_config_;
}

RqtMrtaApplicationPackageConfig* NewApplicationWizard::getPackageConfig() const
{
  return package_config_;
}

RqtMrtaArchitectureConfig* NewApplicationWizard::getArchitectureConfig() const
{
  return architecture_config_;
}

void NewApplicationWizard::generate()
{
  package_config_->addRunDepend(
      application_config_->getApplication()->getArchitecturePackage());
  if (package_config_->createPackage())
  {
    QString package_url(package_config_->getUrl());
    application_config_->setApplicationPackage(package_config_->getName());
    application_config_->setApplicationPackageUrl(package_url);
    ROS_INFO_STREAM(
        "Created package ["
        << application_config_->getApplicationPackage().toStdString() << "] @ ["
        << package_url.toStdString()
        << "].");
    application_config_->save();
    application_config_->getConfigs()->saveAsYaml(package_url);
    application_config_->getLaunches()->saveAsLaunch(package_url);
  }
}

NewApplicationWizardPage* NewApplicationWizard::createSummaryPage()
{
  NewApplicationWizardPage* page =
      new NewApplicationWizardPage(this, "Summary");
  page->setWidget(summary_label_);
  return page;
}

void NewApplicationWizard::summary()
{
  if (currentId() != Summary)
  {
    return;
  }
  QString summary;
  if (!package_config_->workspaceExists())
  {
    summary += "A ROS workspace will be created in:\n\n\t" +
               package_config_->getWorkspaceUrl() + "\n";
  }
  summary += "\nA ROS package will be created in:\n\n\t" +
             package_config_->getUrl() + "\n";
  summary += "\nThe following files and folder will be generated inside this "
             "ROS package:\n";
  QStringList list;
  list.append(package_config_->willBeGenerated());
  list.append(application_config_->getConfigs()->willBeGenerated());
  list.append(application_config_->getLaunches()->willBeGenerated());
  list.sort();
  for (size_t index(0); index < list.count(); index++)
  {
    summary += "\n\t" + list[index];
  }
  summary_label_->setText(summary);
}

void NewApplicationWizard::resetConfig()
{
  application_config_->reset();
  package_config_->reset();
  architecture_config_->reset();
}
}
