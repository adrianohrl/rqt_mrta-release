#include "rqt_mrta/config/application/application.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
Application::Application(QObject *parent)
  : AbstractConfig(parent), robots_(new Robots(this))
{
  connect(robots_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Application::~Application()
{
  if (robots_)
  {
    delete robots_;
    robots_ = NULL;
  }
}

QString Application::getName() const
{
  return name_;
}

QString Application::getArchitecturePackage() const
{
  return architecture_;
}

Robots *Application::getRobots() const
{
  return robots_;
}

void Application::setName(const QString &name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void Application::setArchitecturePackage(const QString &package)
{
  if (package != architecture_)
  {
    architecture_ = package;
    emit architecturePackageChanged(package);
    emit changed();
  }
}

void Application::save(QSettings &settings) const
{
  settings.beginGroup("application");
  settings.setValue("name", name_);
  settings.setValue("architecture", architecture_);
  robots_->save(settings);
  settings.endGroup();
}

void Application::load(QSettings &settings)
{
  settings.beginGroup("application");
  setName(settings.value("name").toString());
  setArchitecturePackage(settings.value("architecture").toString());
  robots_->load(settings);
  settings.endGroup();
}

void Application::reset()
{
  setName("");
  setArchitecturePackage("");
  robots_->reset();
}

void Application::write(QDataStream &stream) const
{
  stream << name_;
  stream << architecture_;
  robots_->write(stream);
}

void Application::read(QDataStream &stream)
{
  QString name, architecture;
  stream >> name;
  setName(name);
  stream >> architecture;
  setArchitecturePackage(architecture);
  robots_->read(stream);
}

Application &Application::operator=(const Application &config)
{
  setName(config.name_);
  setArchitecturePackage(config.architecture_);
  return *this;
}
}
}
}
