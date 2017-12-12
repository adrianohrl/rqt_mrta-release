#include <QStringList>
#include "rqt_mrta/config/includes.h"
#include "rqt_mrta/config/include.h"

namespace rqt_mrta
{
namespace config
{
Includes::Includes(QObject* parent) : AbstractConfig(parent) {}

Includes::~Includes()
{
  for (size_t index(0); index < includes_.count(); index++)
  {
    if (includes_[index])
    {
      delete includes_[index];
      includes_[index] = NULL;
    }
  }
  includes_.clear();
}

Include* Includes::getInclude(size_t index) const
{
  return index < includes_.count() ? includes_[index] : NULL;
}

Include* Includes::addInclude()
{
  Include* include = new Include(this);
  includes_.append(include);
  connect(include, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(include, SIGNAL(fileChanged(const QString)), this,
          SLOT(includeFileChanged(const QString&)));
  connect(include, SIGNAL(destroyed()), this, SLOT(includeDestroyed()));
  emit added(includes_.count() - 1);
  emit changed();
  return include;
}

void Includes::removeInclude(Include* include)
{
  removeInclude(includes_.indexOf(include));
}

void Includes::removeInclude(size_t index)
{
  if (index >= 0 && index < includes_.count())
  {
    QString file(includes_[index]->getFile());
    includes_.remove(index);
    emit removed(file);
    emit changed();
  }
}

void Includes::clearIncludes()
{
  if (!includes_.isEmpty())
  {
    for (size_t index(0); index < includes_.count(); index++)
    {
      if (includes_[index])
      {
        delete includes_[index];
        includes_[index] = NULL;
      }
    }
    includes_.clear();
    emit cleared();
    emit changed();
  }
}

bool Includes::contains(const QString& file) const
{
  for (size_t index(0); index < includes_.count(); index++)
  {
    if (includes_[index]->getFile() == file)
    {
      return true;
    }
  }
  return false;
}

size_t Includes::count() const { return includes_.count(); }

bool Includes::isEmpty() const { return includes_.isEmpty(); }

QString Includes::validate() const
{
  QString validation;
  for (size_t index(0); index < includes_.count(); index++)
  {
    validation = includes_[index]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

QString Includes::toLaunch(const QString &prefix) const
{
  QString launch;
  for (size_t index(0); index < includes_.count(); index++)
  {
    launch += includes_[index]->toLaunch(prefix + "\t");
  }
  return launch;
}

void Includes::save(QSettings& settings) const
{
  settings.beginGroup("includes");
  for (size_t index(0); index < includes_.count(); index++)
  {
    settings.beginGroup("include_" + QString::number(index));
    includes_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Includes::load(QSettings& settings)
{
  settings.beginGroup("includes");
  QStringList groups(settings.childGroups());
  size_t index(0);
  for (QStringList::iterator it(groups.begin()); it != groups.end(); it++)
  {
    Include* include =
        index < includes_.count() ? include = includes_[index] : addInclude();
    settings.beginGroup(*it);
    include->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < includes_.count())
  {
    removeInclude(index);
  }
}

void Includes::reset() { clearIncludes(); }

void Includes::write(QDataStream& stream) const
{
  for (size_t index(0); index < includes_.count(); index++)
  {
    includes_[index]->write(stream);
  }
}

void Includes::read(QDataStream& stream)
{
  for (size_t index(0); index < includes_.count(); index++)
  {
    includes_[index]->read(stream);
  }
}

Includes& Includes::operator=(const Includes& config)
{
  while (includes_.count() < config.includes_.count())
  {
    addInclude();
  }
  while (includes_.count() > config.includes_.count())
  {
    removeInclude(includes_.count() - 1);
  }
  for (size_t index(0); index < includes_.count(); ++index)
  {
    *includes_[index] = *config.includes_[index];
  }
  return *this;
}

void Includes::includeFileChanged(const QString& file)
{
  int index(includes_.indexOf(static_cast<Include*>(sender())));
  if (index != -1)
  {
    emit includeFileChanged(index, file);
    emit changed();
  }
}

void Includes::includeDestroyed()
{
  Include* include = static_cast<Include*>(sender());
  int index(includes_.indexOf(include));
  if (index >= 0)
  {
    QString file(include->getFile());
    includes_.remove(index);
    emit removed(file);
    emit changed();
  }
}
}
}
