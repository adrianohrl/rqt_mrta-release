#include <QInputDialog>
#include "mrta/system.h"
#include <ros/package.h>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/labeled_status_widget.h"
#include "rqt_mrta/new_application_wizard.h"
#include "rqt_mrta/new_architecture_wizard.h"
#include "rqt_mrta/rqt_mrta_widget.h"
#include "rqt_mrta/ui_rqt_mrta_widget.h"
#include "utilities/exception.h"
#include "utilities/message_subscriber_registry.h"

namespace rqt_mrta
{
RqtMrtaWidget::RqtMrtaWidget(QWidget* parent,
                             const qt_gui_cpp::PluginContext& context)
    : QWidget(parent), ui_(new Ui::RqtMrtaWidget()),
      architecture_config_(new RqtMrtaArchitectureConfig(this)),
      application_config_(new RqtMrtaApplicationConfig(this)),
      registry_(new utilities::MessageSubscriberRegistry(this)),
      context_(context), loader_("rqt_gui", "rqt_gui_cpp::Plugin"),
      system_(NULL)
{
  ui_->setupUi(this);
  ui_->runtime_tab_widget->setCurrentIndex(0);
  ui_->architecture_tab->activateWindow();
  ui_->new_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/new.png"))));
  ui_->open_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/open.png"))));
  ui_->launch_application_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/run.png"))));
  ui_->new_architecture_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/new.png"))));
  ui_->open_architecture_push_button->setIcon(QIcon(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/open.png"))));
  connect(ui_->new_application_push_button, SIGNAL(clicked()), this,
          SLOT(newApplicationPushButtonClicked()));
  connect(ui_->open_application_push_button, SIGNAL(clicked()), this,
          SLOT(openApplicationPushButtonClicked()));
  connect(ui_->new_architecture_push_button, SIGNAL(clicked()), this,
          SLOT(newArchitecturePushButtonClicked()));
  connect(ui_->open_architecture_push_button, SIGNAL(clicked()), this,
          SLOT(openArchitecturePushButtonClicked()));
}

RqtMrtaWidget::~RqtMrtaWidget()
{
  clear();
  if (ui_)
  {
    delete ui_;
    ui_ = NULL;
  }
  if (system_)
  {
    delete system_;
    system_ = NULL;
  }
  if (architecture_config_)
  {
    delete architecture_config_;
    architecture_config_ = NULL;
  }
  if (application_config_)
  {
    delete application_config_;
    application_config_ = NULL;
  }
  if (registry_)
  {
    delete registry_;
    registry_ = NULL;
  }
}

void RqtMrtaWidget::newApplicationPushButtonClicked()
{
  RqtMrtaApplicationConfig config;
  NewApplicationWizard wizard(this, &config);
  if (wizard.exec() == QWizard::Accepted)
  {
    loadSystem(&config);
  }
}

void RqtMrtaWidget::openApplicationPushButtonClicked()
{
  QMap<QString, QString> applications(findPlugins("application"));
  if (applications.isEmpty())
  {
    showMessage("Error while loading architecture ...",
                "There isn't any application registered yet.");
    return;
  }
  QString package(
      askItem("Opening application ...", "Application:", applications.keys()));
  if (package.isEmpty())
  {
    return;
  }
  loadApplication(applications[package]);
}

void RqtMrtaWidget::newArchitecturePushButtonClicked()
{
  RqtMrtaArchitectureConfig config;
  NewArchitectureWizard wizard(this, &config);
  if (wizard.exec() == QWizard::Accepted)
  {
    std::string url(
        ros::package::getPath(config.getArchitecturePackage().toStdString()));
    loadArchitecture(QString::fromStdString(url));
  }
}

void RqtMrtaWidget::openArchitecturePushButtonClicked()
{
  QMap<QString, QString> architectures(findPlugins("architecture"));
  if (architectures.isEmpty())
  {
    showMessage("Error while loading architecture ...",
                "There isn't any architecture registered yet.");
    return;
  }
  QString package(askItem("Opening architecture ...", "Architecture:",
                          architectures.keys()));
  if (package.isEmpty())
  {
    return;
  }
  loadArchitecture(architectures[package]);
}

void RqtMrtaWidget::loadApplication(const QString& url)
{
  RqtMrtaApplicationConfig application_config(this);
  try
  {
    application_config.load(url);
  }
  catch (const utilities::Exception& e)
  {
    showMessage("Error while loading application ...", e.what());
  }
  loadSystem(&application_config);
}

void RqtMrtaWidget::loadArchitecture(const QString& url)
{
  QString architecture_url(url);
  if (architecture_url.isEmpty())
  {
    QString package(
        application_config_->getApplication()->getArchitecturePackage());
    if (package.isEmpty())
    {
      showMessage("Error while loading architecture ...",
                  "The architecture package name must not be empty.");
      return;
    }
    QMap<QString, QString> architectures(findPlugins("architecture"));
    if (architectures.isEmpty())
    {
      showMessage("Error while loading architecture ...",
                  "There isn't any architecture registered yet.");
      return;
    }
    if (!architectures.contains(package))
    {
      showMessage("Error while loading architecture ...",
                  "The given package does not have the architecture "
                  "configuration file.");
      return;
    }
    architecture_url = architectures[package];
  }
  try
  {
    architecture_config_->load(architecture_url);
  }
  catch (const utilities::Exception e)
  {
    showMessage("Error while loading architecture ...", e.what());
    return;
  }
  for (size_t index(0); index < architecture_config_->getWidgets()->count();
       index++)
  {
    QString plugin_name(
        architecture_config_->getWidgets()->getWidget(index)->getPluginName());
    try
    {
      PluginPtr external_plugin(
          loader_.createInstance(plugin_name.toStdString()));
      external_plugin->initPlugin(context_);
      external_plugins_.push_back(external_plugin);
    }
    catch (pluginlib::PluginlibException& e)
    {
      showMessage("Error while loading architecture ...",
                  "The " + plugin_name +
                      " plugin failed to load for some reason. Error: " +
                      e.what());
    }
  }
}

void RqtMrtaWidget::clear()
{
  for (size_t index(0); index < external_plugins_.count(); index++)
  {
    external_plugins_[index]->shutdownPlugin();
  }
  context_.closePlugin();
  external_plugins_.clear();
  ui_->robots_list_widget->clear();
  for (size_t index(1); index < ui_->runtime_tab_widget->count(); index++)
  {
    ui_->runtime_tab_widget->removeTab(index);
  }
  if (system_)
  {
    delete system_;
    system_ = NULL;
  }
}

void RqtMrtaWidget::loadSystem(RqtMrtaApplicationConfig* application_config)
{
  clear();
  if (application_config)
  {
    *application_config_ = *application_config;
  }
  loadArchitecture();
  system_ = new mrta::System(this, application_config_, architecture_config_,
                             registry_);
  QList<mrta::Robot*> robots(system_->getRobots());
  for (size_t index(0); index < robots.count(); index++)
  {
    LabeledStatusWidget* widget = new LabeledStatusWidget(this, robots[index]);
    QListWidgetItem* item = new QListWidgetItem(ui_->robots_list_widget);
    item->setSizeHint(widget->sizeHint());
    ui_->robots_list_widget->addItem(item);
    ui_->robots_list_widget->setItemWidget(item, widget);
  }
}

void RqtMrtaWidget::showMessage(const QString& title, const QString& message,
                                QMessageBox::Icon icon) const
{
  switch (icon)
  {
  case QMessageBox::NoIcon:
  case QMessageBox::Information:
    ROS_INFO("%s", message.toStdString().c_str());
    break;
  case QMessageBox::Warning:
    ROS_WARN("%s", message.toStdString().c_str());
    break;
  case QMessageBox::Critical:
    ROS_ERROR("%s", message.toStdString().c_str());
    break;
  }
  QMessageBox msg_box;
  msg_box.setText(title);
  msg_box.setInformativeText(message);
  msg_box.setIcon(icon);
  msg_box.exec();
}

QMap<QString, QString>
RqtMrtaWidget::findPlugins(const QString& attribute) const
{
  typedef std::map<std::string, std::string> StdMap;
  StdMap map;
  ros::package::getPlugins("rqt_mrta", attribute.toStdString(), map, true);
  QMap<QString, QString> applications;
  for (StdMap::const_iterator it(map.begin()); it != map.end(); it++)
  {
    ROS_WARN_STREAM("[RqtMrtaWidget::findPlugins] " << it->first << ": "
                                                    << it->second);
    applications[QString::fromStdString(it->first)] =
        QString::fromStdString(it->second);
  }
  return applications;
}

QString RqtMrtaWidget::askItem(const char* title, const char* label,
                               const QStringList& items)
{
  bool ok(false);
  QString item(
      QInputDialog::getItem(this, tr(title), tr(label), items, 0, false, &ok));
  return ok ? item : "";
}
}
