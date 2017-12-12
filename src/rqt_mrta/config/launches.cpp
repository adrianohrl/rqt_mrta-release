#include <QDir>
#include <QStringList>
#include "rqt_mrta/config/application/robots.h"
#include "rqt_mrta/config/launches.h"
#include "rqt_mrta/config/launch.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
namespace config
{
Launches::Launches(QObject* parent) : AbstractConfig(parent) {}

Launches::~Launches()
{
  for (size_t index(0); index < launches_.count(); index++)
  {
    if (launches_[index])
    {
      delete launches_[index];
      launches_[index] = NULL;
    }
  }
  launches_.clear();
}

Launch* Launches::getLaunch(size_t index) const
{
  return index < launches_.count() ? launches_[index] : NULL;
}

Launch *Launches::getLaunch(const QString &id) const
{
  for (size_t index(0); index < launches_.count(); index)
  {
    if (launches_[index]->getId() == id)
    {
      return launches_[index];
    }
  }
  return NULL;
}

Launch* Launches::addLaunch()
{
  Launch* launch = new Launch(this);
  launches_.append(launch);
  connect(launch, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(launch, SIGNAL(idChanged(const QString&)), this,
          SLOT(launchIdChanged(const QString&)));
  connect(launch, SIGNAL(destroyed()), this, SLOT(launchDestroyed()));
  emit added(launches_.count() - 1);
  emit changed();
  return launch;
}

void Launches::removeLaunch(Launch* launch)
{
  removeLaunch(launches_.indexOf(launch));
}

void Launches::removeLaunch(size_t index)
{
  if (index >= 0 && index < launches_.count())
  {
    QString name(launches_[index]->getId());
    launches_.remove(index);
    emit removed(name);
    emit changed();
  }
}

void Launches::clearLaunches()
{
  if (!launches_.isEmpty())
  {
    for (size_t index(0); index < launches_.count(); index++)
    {
      if (launches_[index])
      {
        delete launches_[index];
        launches_[index] = NULL;
      }
    }
    launches_.clear();
    emit cleared();
    emit changed();
  }
}

bool Launches::contains(const QString& id) const
{
  for (size_t index(0); index < launches_.count(); index++)
  {
    if (launches_[index]->getId() == id)
    {
      return true;
    }
  }
  return false;
}

size_t Launches::count() const { return launches_.count(); }

bool Launches::isEmpty() const { return launches_.isEmpty(); }

QString Launches::validate() const
{
  QString validation;
  for (size_t index(0); index < launches_.count(); index++)
  {
    validation = launches_[index]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

QStringList Launches::willBeGenerated() const
{
  QStringList list;
  for (size_t index(0); index < launches_.count(); index++)
  {
    list.append("launch/" + launches_[index]->getId() + ".launch");
  }
  return list;
}

void Launches::saveAsLaunch(const QString& package_url) const
{
  QDir package(package_url);
  if (!package.exists())
  {
    throw utilities::Exception("Inexistent package.");
  }
  if (package.cd("launch"))
  {
    package.cd("..");
  }
  else if (!package.mkdir("launch"))
  {
    throw utilities::Exception("Unable to create the launch folder.");
  }
  for (size_t index(0); index < launches_.count(); index++)
  {
    /*QString validation(launches_[index]->validate());
    if (!validation.isEmpty())
    {
      ROS_ERROR("%s", validation.toStdString().c_str());
      continue;
    }*/
    launches_[index]->add("package", package.dirName());
    launches_[index]->saveAsLaunch(package.path() + "/launch/" +
                                   launches_[index]->getId());
  }
}

void Launches::save(QSettings& settings) const
{
  settings.beginGroup("launches");
  for (size_t index(0); index < launches_.count(); index++)
  {
    settings.beginGroup("launch_" + QString::number(index));
    launches_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Launches::load(QSettings& settings)
{
  settings.beginGroup("launches");
  QStringList groups(settings.childGroups());
  size_t index(0);
  for (QStringList::iterator it(groups.begin()); it != groups.end(); it++)
  {
    Launch* launch =
        index < launches_.count() ? launch = launches_[index] : addLaunch();
    settings.beginGroup(*it);
    launch->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < launches_.count())
  {
    removeLaunch(index);
  }
}

void Launches::reset() { clearLaunches(); }

void Launches::write(QDataStream& stream) const
{
  for (size_t index(0); index < launches_.count(); index++)
  {
    launches_[index]->write(stream);
  }
}

void Launches::read(QDataStream& stream)
{
  for (size_t index(0); index < launches_.count(); index++)
  {
    launches_[index]->read(stream);
  }
}

Launches& Launches::operator=(const Launches& config)
{
  while (launches_.count() < config.launches_.count())
  {
    addLaunch();
  }
  while (launches_.count() > config.launches_.count())
  {
    removeLaunch(launches_.count() - 1);
  }
  for (size_t index(0); index < launches_.count(); ++index)
  {
    *launches_[index] = *config.launches_[index];
  }
  return *this;
}

void Launches::setLaunches(const Launches& launches,
                           const application::Robots& robots,
                           const QString& robots_launch_id)
{
  clearLaunches();
  Launch* robots_launch = launches.getLaunch(robots_launch_id);
  for (size_t i(0); i < launches.count(); i++)
  {
    Launch* template_launch = launches.getLaunch(i);
    bool is_robot_template = robots_launch && template_launch == robots_launch;
    size_t count = is_robot_template ? robots.count() : 1;
    for (size_t j(0); j < count; j++)
    {
      Launch* launch = addLaunch();
      *launch = *template_launch;
      if (is_robot_template)
      {
        launch->add("robot_id", robots.getRobot(j)->getId());
        launch->setId(robots.getRobot(j)->getId() + "_" + launch->getId());
      }
    }
  }
}

void Launches::launchIdChanged(const QString& id)
{
  int index(launches_.indexOf(static_cast<Launch*>(sender())));
  if (index != -1)
  {
    emit launchIdChanged(index, id);
    emit changed();
  }
}

void Launches::launchDestroyed()
{
  Launch* launch = static_cast<Launch*>(sender());
  int index(launches_.indexOf(launch));
  if (index >= 0)
  {
    QString id(launch->getId());
    launches_.remove(index);
    emit removed(id);
    emit changed();
  }
}
}
}
