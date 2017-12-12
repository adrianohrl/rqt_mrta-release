#include "mrta/architecture.h"
#include <QFileInfo>
#include <QFileDialog>
#include <QSettings>
#include <ros/console.h>
#include <ros/package.h>
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/application/rqt_mrta_package.h"
#include "rqt_mrta/define_application_widget.h"
#include "rqt_mrta/ui_define_application_widget.h"
#include "utilities/exception.h"
#include "utilities/xml_settings.h"

namespace rqt_mrta
{
DefineApplicationWidget::DefineApplicationWidget(
    QWidget* parent, RqtMrtaApplicationConfig* application_config,
    RqtMrtaApplicationPackageConfig* metapackage_config)
    : QWidget(parent), ui_(new Ui::DefineApplicationWidget()),
      application_config_(NULL), package_config_(NULL)
{
  if (!application_config)
  {
    throw utilities::Exception(
        "[DefineApplicationWidget] The application config must not be NULL.");
  }
  if (!metapackage_config)
  {
    throw utilities::Exception(
        "[DefineApplicationWidget] The metapackage config must not be NULL.");
  }
  rp_.setQuiet(true);
  std::vector<std::string> search_path;
  rp_.getSearchPathFromEnv(search_path);
  rp_.crawl(search_path, true);
  ui_->setupUi(this);
  ui_->workspace_push_button->setEnabled(false);
  connect(ui_->name_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(nameChanged(const QString&)));
  connect(ui_->package_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(packageChanged(const QString&)));
  connect(ui_->workspace_push_button, SIGNAL(clicked()), this,
          SLOT(workspaceBrowserButtonClicked()));
  connect(ui_->workspace_package_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(workspaceUrlChanged(const QString&)));
  connect(ui_->version_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(versionChanged(const QString&)));
  connect(ui_->description_plain_text_edit, SIGNAL(textChanged()), this,
          SLOT(descriptionChanged()));
  connect(ui_->maintainer_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(maintainerChanged(const QString&)));
  connect(ui_->maintainer_email_line_edit, SIGNAL(textChanged(const QString&)),
          this, SLOT(maintainerEmailChanged(const QString&)));
  connect(ui_->license_line_edit, SIGNAL(textChanged(const QString&)), this,
          SLOT(licenseChanged(const QString&)));
  connect(ui_->run_depends_plain_text_edit, SIGNAL(textChanged()), this,
          SLOT(runDependsChanged()));
  setApplicationConfig(application_config);
  setMetapackageConfig(metapackage_config);
}

DefineApplicationWidget::~DefineApplicationWidget()
{
  application_config_ = NULL;
  package_config_ = NULL;
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
}

RqtMrtaApplicationConfig* DefineApplicationWidget::getApplicationConfig() const
{
  return application_config_;
}

RqtMrtaApplicationPackageConfig*
DefineApplicationWidget::getMetapackageConfig() const
{
  return package_config_;
}

void DefineApplicationWidget::setApplicationConfig(
    RqtMrtaApplicationConfig* config)
{
  if (application_config_ != config)
  {
    if (application_config_)
    {
      disconnect(application_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      disconnect(application_config_, SIGNAL(applicationPackageChanged(const QString&)),
                 this, SLOT(configPackageChanged(const QString&)));
      disconnect(application_config_->getApplication(),
                 SIGNAL(nameChanged(const QString&)), this,
                 SLOT(configNameChanged(const QString&)));
    }
    application_config_ = config;
    if (application_config_)
    {
      connect(application_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(application_config_, SIGNAL(applicationPackageChanged(const QString&)), this,
              SLOT(configPackageChanged(const QString&)));
      connect(application_config_->getApplication(),
              SIGNAL(nameChanged(const QString&)), this,
              SLOT(configNameChanged(const QString&)));
      configNameChanged(application_config_->getApplication()->getName());
      configPackageChanged(application_config_->getApplicationPackage());
      if (package_config_ &&
          package_config_->getDescription().isEmpty())
      {
        configDescriptionChanged(
            application_config_->getApplication()->getName());
      }
    }
  }
}

void DefineApplicationWidget::setMetapackageConfig(
    RqtMrtaApplicationPackageConfig* config)
{
  if (package_config_ != config)
  {
    if (package_config_)
    {
      disconnect(package_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      disconnect(package_config_,
                 SIGNAL(workspaceUrlChanged(const QString&)), this,
                 SLOT(configWorkspaceUrlChanged(const QString&)));
      disconnect(package_config_, SIGNAL(nameChanged(const QString&)), this,
                 SLOT(configPackageChanged(const QString&)));
      disconnect(package_config_, SIGNAL(versionChanged(const QString&)),
                 this, SLOT(configVersionChanged(const QString&)));
      disconnect(package_config_,
                 SIGNAL(descriptionChanged(const QString&)), this,
                 SLOT(configDescriptionChanged(const QString&)));
      disconnect(package_config_, SIGNAL(maintainerChanged(const QString&)),
                 this, SLOT(configMaintainerChanged(const QString&)));
      disconnect(package_config_,
                 SIGNAL(maintainerEmailChanged(const QString&)), this,
                 SLOT(configMaintainerEmailChanged(const QString&)));
      disconnect(package_config_, SIGNAL(licenseChanged(const QString&)),
                 this, SLOT(configLicenseChanged(const QString&)));
      disconnect(package_config_,
                 SIGNAL(runDependsChanged(const QStringList&)), this,
                 SLOT(configRunDependsChanged(const QStringList&)));
    }
    package_config_ = config;
    if (package_config_)
    {
      connect(package_config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(package_config_, SIGNAL(workspaceUrlChanged(const QString&)),
              this, SLOT(configWorkspaceUrlChanged(const QString&)));
      connect(package_config_, SIGNAL(nameChanged(const QString&)), this,
              SLOT(configPackageChanged(const QString&)));
      connect(package_config_, SIGNAL(versionChanged(const QString&)), this,
              SLOT(configVersionChanged(const QString&)));
      connect(package_config_, SIGNAL(descriptionChanged(const QString&)),
              this, SLOT(configDescriptionChanged(const QString&)));
      connect(package_config_, SIGNAL(maintainerChanged(const QString&)),
              this, SLOT(configMaintainerChanged(const QString&)));
      connect(package_config_,
              SIGNAL(maintainerEmailChanged(const QString&)), this,
              SLOT(configMaintainerEmailChanged(const QString&)));
      connect(package_config_, SIGNAL(licenseChanged(const QString&)), this,
              SLOT(configLicenseChanged(const QString&)));
      connect(package_config_,
              SIGNAL(runDependsChanged(const QStringList&)), this,
              SLOT(configRunDependsChanged(const QStringList&)));
      configPackageChanged(package_config_->getName());
      configWorkspaceUrlChanged(package_config_->getWorkspaceUrl());
      configVersionChanged(package_config_->getVersion());
      configDescriptionChanged(package_config_->getDescription());
      configMaintainerChanged(package_config_->getMaintainer());
      configMaintainerEmailChanged(package_config_->getMaintainerEmail());
      configLicenseChanged(package_config_->getLicense());
      configRunDependsChanged(package_config_->getRunDepends());
    }
  }
  ui_->workspace_push_button->setEnabled(package_config_);
}

void DefineApplicationWidget::configWorkspaceUrlChanged(const QString& url)
{
  ui_->workspace_package_line_edit->setText(url);
  if (url.isEmpty())
  {
    ui_->workspace_status_widget->setCurrentRole(
        StatusWidget::Error, "The workspace must be given.");
  }
  else if (package_config_->workspaceExists())
  {
    ui_->workspace_status_widget->setCurrentRole(StatusWidget::Okay);
  }
  else
  {
    ui_->workspace_status_widget->setCurrentRole(
        StatusWidget::Warn, "The given directory is not a ROS workspace.");
  }
}

void DefineApplicationWidget::configNameChanged(const QString& name)
{
  ui_->name_line_edit->setText(name);
}

void DefineApplicationWidget::configPackageChanged(const QString& package)
{
  ui_->package_line_edit->setText(package);
  if (package.isEmpty())
  {
    ui_->package_status_widget->setCurrentRole(
        StatusWidget::Error, "The package name must not be empty.");
  }
  else if (package.contains(' '))
  {
    ui_->package_status_widget->setCurrentRole(
        StatusWidget::Error, "The package name must not be empty.");
  }
  else
  {
    ui_->package_status_widget->setCurrentRole(
        StatusWidget::Busy, "Verifying if the input package exists...");
    std::string package_path;
    rp_.find(package.toStdString(), package_path);
    if (package_path.empty())
    {
      ui_->package_status_widget->setCurrentRole(StatusWidget::Okay);
    }
    else
    {
      ui_->package_status_widget->setCurrentRole(
          StatusWidget::Error, "The input package already exists.");
    }
  }
}

void DefineApplicationWidget::configVersionChanged(const QString& version)
{
  ui_->version_line_edit->setText(version);
}

void DefineApplicationWidget::configDescriptionChanged(
    const QString& description)
{
  ui_->description_plain_text_edit->setPlainText(description);
  QTextCursor cursor(ui_->description_plain_text_edit->textCursor());
  cursor.movePosition(QTextCursor::End);
  ui_->description_plain_text_edit->setTextCursor(cursor);
}

void DefineApplicationWidget::configMaintainerChanged(const QString& name)
{
  ui_->maintainer_line_edit->setText(name);
}

void DefineApplicationWidget::configMaintainerEmailChanged(const QString& email)
{
  ui_->maintainer_email_line_edit->setText(email);
}

void DefineApplicationWidget::configLicenseChanged(const QString& license)
{
  ui_->license_line_edit->setText(license);
}

void DefineApplicationWidget::configRunDependsChanged(
    const QStringList& depends)
{
  QString text;
  if (!depends.isEmpty())
  {
    text += depends[0];
    for (size_t i(1); i < depends.count(); i++)
    {
      text += " " + depends[i];
    }
  }
  ui_->run_depends_plain_text_edit->setPlainText(text);
  QTextCursor cursor(ui_->run_depends_plain_text_edit->textCursor());
  cursor.movePosition(QTextCursor::End);
  ui_->run_depends_plain_text_edit->setTextCursor(cursor);
}

void DefineApplicationWidget::workspaceBrowserButtonClicked()
{
  QFileDialog dialog(this, "Select the workspace directory", QDir::homePath());
  dialog.setAcceptMode(QFileDialog::AcceptSave);
  dialog.setFileMode(QFileDialog::Directory);
  dialog.setOption(QFileDialog::ShowDirsOnly);
  dialog.setLabelText(QFileDialog::Accept, "Choose");
  if (dialog.exec() == QDialog::Accepted)
  {
    package_config_->setWorkspaceUrl(dialog.selectedFiles().first());
  }
}

void DefineApplicationWidget::workspaceUrlChanged(const QString& url)
{
  if (package_config_)
  {
    package_config_->setWorkspaceUrl(url);
  }
}

void DefineApplicationWidget::nameChanged(const QString& name)
{
  if (application_config_)
  {
    application_config_->getApplication()->setName(name);
  }
}

void DefineApplicationWidget::packageChanged(const QString& package)
{
  if (application_config_)
  {
    application_config_->setApplicationPackage(package);
  }
  if (package_config_)
  {
    package_config_->setName(package);
  }
}

void DefineApplicationWidget::versionChanged(const QString& version)
{
  if (package_config_)
  {
    package_config_->setVersion(version);
  }
}

void DefineApplicationWidget::descriptionChanged()
{
  if (package_config_)
  {
    package_config_->setDescription(
        ui_->description_plain_text_edit->toPlainText());
  }
}

void DefineApplicationWidget::maintainerChanged(const QString& name)
{
  if (package_config_)
  {
    package_config_->setMaintainer(name);
  }
}

void DefineApplicationWidget::maintainerEmailChanged(const QString& email)
{
  if (package_config_)
  {
    package_config_->setMaintainerEmail(email);
  }
}

void DefineApplicationWidget::licenseChanged(const QString& license)
{
  if (package_config_)
  {
    package_config_->setLicense(license);
  }
}

void DefineApplicationWidget::runDependsChanged()
{
  QString depends(ui_->run_depends_plain_text_edit->toPlainText());
  depends.replace('\n', ' ');
  if (package_config_)
  {
    package_config_->clearRunDepends();
    QStringList run_depends(depends.split(' '));
    run_depends.removeAll("");
    for (size_t i(0); i < run_depends.count(); i++)
    {
      package_config_->addRunDepend(run_depends[i]);
    }
    if (run_depends.count() == package_config_->countRunDepends())
    {
      ui_->run_depends_status_widget->setCurrentRole(StatusWidget::Okay);
    }
    else
    {
      ui_->run_depends_status_widget->setCurrentRole(
          StatusWidget::Warn,
          "Some run dependency(ies) is(are) not ROS package(s).");
    }
  }
}
}
