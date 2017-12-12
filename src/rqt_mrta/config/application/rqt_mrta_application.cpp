#include <QFileInfo>
#include <ros/console.h>
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "utilities/exception.h"
#include "utilities/simple_xml_settings.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
RqtMrtaApplication::RqtMrtaApplication(QObject* parent)
    : AbstractConfig(parent), application_(new Application(this)),
      configs_(new Configs(this)), launches_(new Launches(this))
{
  reset();
  connect(application_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(configs_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(launches_, SIGNAL(changed()), this, SIGNAL(changed()));
}

RqtMrtaApplication::~RqtMrtaApplication()
{
  if (application_)
  {
    delete application_;
    application_ = NULL;
  }
  if (configs_)
  {
    delete configs_;
    configs_ = NULL;
  }
  if (launches_)
  {
    delete launches_;
    launches_ = NULL;
  }
}

QString RqtMrtaApplication::getApplicationPackage() const { return package_; }

QString RqtMrtaApplication::getApplicationPackageUrl() const { return url_; }

Application* RqtMrtaApplication::getApplication() const { return application_; }

Configs* RqtMrtaApplication::getConfigs() const { return configs_; }

Launches *RqtMrtaApplication::getLaunches() const
{
  return launches_;
}

void RqtMrtaApplication::setApplicationPackage(const QString& package)
{
  if (package != package_)
  {
    package_ = package;
    emit applicationPackageChanged(package);
    emit changed();
  }
}

void RqtMrtaApplication::setApplicationPackageUrl(const QString& url)
{
  if (url != url_)
  {
    ROS_ERROR_STREAM("[RqtMrtaApplication::setApplicationPackageUrl] url: " << url.toStdString());
    url_ = url;
    emit applicationPackageUrlChanged(url);
    emit changed();
  }
}

void RqtMrtaApplication::save() const { save("rqt_mrta.xml"); }

void RqtMrtaApplication::save(const QString& filename) const
{
  if (url_.isEmpty() || filename.isEmpty())
  {
    ROS_ERROR(
        "[RqtMrtaApplication] unable to save application rqt_mrta.xml file.");
    return;
  }
  QString url(url_ + "/" + filename);
  QSettings settings(url, utilities::SimpleXmlSettings::format);
  if (settings.isWritable())
  {
    settings.clear();
    save(settings);
    settings.sync();
    if (settings.status() != QSettings::NoError)
    {
      ROS_ERROR("This application configuration file cannot be saved.");
      return;
    }
    ROS_INFO_STREAM("Saved application configuration file ["
                    << url.toStdString() << "].");
  }
}

void RqtMrtaApplication::save(QSettings& settings) const
{
  settings.beginGroup("rqt_mrta");
  //settings.setValue("@format", "application");
  application_->save(settings);
  configs_->save(settings);
  launches_->save(settings);
  settings.endGroup();
}

void RqtMrtaApplication::load(const QString& url)
{
  QFileInfo file_info(url);
  if (!file_info.isReadable())
  {
    return;
  }
  QSettings settings(url, utilities::SimpleXmlSettings::format);
  if (settings.status() != QSettings::NoError)
  {
    ROS_ERROR("This application configuration file cannot be loaded.");
    return;
  }
  load(settings);
  ROS_INFO_STREAM("Loaded application configuration file ["
                  << url.toStdString() << "].");
}

void RqtMrtaApplication::load(QSettings& settings)
{
  settings.beginGroup("rqt_mrta");
  QString type(settings.value("@format").toString());
  /*if (type.isEmpty())
  {
    throw utilities::Exception("The <rqt_mrta> tag in the input xml file must "
                               "have an attribute named 'format'.");
  }
  if (type != "application")
  {
    throw utilities::Exception(
        "The 'format' attribute of the <rqt_mrta> tag in "
        "the input xml file must be valued as "
        "'application' to be loaded as an application "
        "configuration file.");
  }*/
  application_->load(settings);
  configs_->load(settings);
  launches_->load(settings);
  settings.endGroup();
}

void RqtMrtaApplication::reset()
{
  setApplicationPackage("");
  setApplicationPackageUrl("");
  application_->reset();
  configs_->reset();
  launches_->reset();
}

void RqtMrtaApplication::write(QDataStream& stream) const
{
  application_->write(stream);
  configs_->write(stream);
  launches_->write(stream);
}

void RqtMrtaApplication::read(QDataStream& stream)
{
  application_->read(stream);
  configs_->read(stream);
  launches_->read(stream);
}

RqtMrtaApplication& RqtMrtaApplication::
operator=(const RqtMrtaApplication& config)
{
  setApplicationPackage(config.package_);
  setApplicationPackageUrl(config.url_);
  *application_ = *config.application_;
  *configs_ = *config.configs_;
  *launches_ = *config.launches_;
  return *this;
}
}
}
}
