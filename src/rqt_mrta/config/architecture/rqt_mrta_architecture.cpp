#include <QFileInfo>
#include <ros/console.h>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"
#include "utilities/exception.h"
#include "utilities/simple_xml_settings.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
RqtMrtaArchitecture::RqtMrtaArchitecture(QObject* parent)
    : AbstractConfig(parent), architecture_(new Architecture(this)),
      configs_(new Configs(this)), launches_(new Launches(this)), widgets_(new Widgets(this))
{
  reset();
  connect(architecture_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(configs_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(launches_, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(widgets_, SIGNAL(changed()), this, SIGNAL(changed()));
}

RqtMrtaArchitecture::~RqtMrtaArchitecture()
{
  if (architecture_)
  {
    delete architecture_;
    architecture_ = NULL;
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
  if (widgets_)
  {
    delete widgets_;
    widgets_ = NULL;
  }
}

QString RqtMrtaArchitecture::getArchitecturePackage() const
{
  return package_;
}

QString RqtMrtaArchitecture::getArchitecturePackageUrl() const
{
  return url_;
}

Architecture* RqtMrtaArchitecture::getArchitecture() const
{
  return architecture_;
}

Configs* RqtMrtaArchitecture::getConfigs() const { return configs_; }

Launches *RqtMrtaArchitecture::getLaunches() const
{
  return launches_;
}

Widgets* RqtMrtaArchitecture::getWidgets() const { return widgets_; }

void RqtMrtaArchitecture::setArchitecturePackage(const QString &package)
{
  if (package != package_)
  {
    package_ = package;
    emit architecturePackageChanged(package);
    emit changed();
  }
}

void RqtMrtaArchitecture::setArchitecturePackageUrl(const QString &url)
{
  if (url != url_)
  {
    url_ = url;
    emit architecturePackageUrlChanged(url);
    emit changed();
  }
}

void RqtMrtaArchitecture::save(const QString& url) const
{
  if (url.isEmpty())
  {
    return;
  }
  QSettings settings(url, utilities::SimpleXmlSettings::format);
  if (settings.isWritable())
  {
    settings.clear();
    save(settings);
    settings.sync();
    if (settings.status() != QSettings::NoError)
    {
      ROS_ERROR("This architecture configuration file cannot be saved.");
      return;
    }
    ROS_INFO_STREAM("Saved architecture configuration file ["
                    << url.toStdString() << "].");
  }
}

void RqtMrtaArchitecture::save(QSettings& settings) const
{
  settings.beginGroup("rqt_mrta");
  //settings.setValue("@format", "architecture");
  architecture_->save(settings);
  configs_->save(settings);
  launches_->save(settings);
  widgets_->save(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::load(const QString& url)
{
  QFileInfo file_info(url);
  if (!file_info.isReadable())
  {
    return;
  }
  QSettings settings(url, utilities::SimpleXmlSettings::format);
  if (settings.status() != QSettings::NoError)
  {
    ROS_ERROR_STREAM("The given file is not well formatted ["
                     << url.toStdString() << "].");
    return;
  }
  load(settings);
  ROS_INFO_STREAM("Loaded architecture configuration file ["
                  << url.toStdString() << "].");
}

void RqtMrtaArchitecture::load(QSettings& settings)
{
  settings.beginGroup("rqt_mrta");
  QString type(settings.value("@format").toString());
  /*if (type.isEmpty())
  {
    throw utilities::Exception("The <rqt_mrta> tag in the input xml file must "
                               "have an attribute named 'format'.");
  }
  if (type != "architecture")
  {
    throw utilities::Exception("The 'format' attribute of the <rqt_mrta> tag in "
                               "the input xml file must be valued as "
                               "'architecture' to be loaded as an architecture "
                               "configuration file.");
  }*/
  architecture_->load(settings);
  configs_->load(settings);
  launches_->load(settings);
  widgets_->load(settings);
  settings.endGroup();
}

void RqtMrtaArchitecture::reset()
{
  setArchitecturePackage("");
  setArchitecturePackageUrl("");
  architecture_->reset();
  configs_->reset();
  launches_->reset();
  widgets_->reset();
}

void RqtMrtaArchitecture::write(QDataStream& stream) const
{
  architecture_->write(stream);
  configs_->write(stream);
  launches_->write(stream);
  widgets_->write(stream);
}

void RqtMrtaArchitecture::read(QDataStream& stream)
{
  architecture_->read(stream);
  configs_->read(stream);
  launches_->read(stream);
  widgets_->read(stream);
}

RqtMrtaArchitecture& RqtMrtaArchitecture::
operator=(const RqtMrtaArchitecture& config)
{
  setArchitecturePackage(config.package_);
  setArchitecturePackageUrl(config.url_);
  *architecture_ = *config.architecture_;
  *configs_ = *config.configs_;
  *launches_ = *config.launches_;
  *widgets_ = *config.widgets_;
  return *this;
}
}
}
}
