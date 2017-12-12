#include <QStringList>
#include "rqt_mrta/config/args.h"
#include "rqt_mrta/config/arg.h"

namespace rqt_mrta
{
namespace config
{
Args::Args(QObject* parent) : AbstractConfig(parent) {}

Args::~Args()
{
  for (size_t index(0); index < args_.count(); index++)
  {
    if (args_[index])
    {
      delete args_[index];
      args_[index] = NULL;
    }
  }
  args_.clear();
}

Arg* Args::getArg(size_t index) const
{
  return index < args_.count() ? args_[index] : NULL;
}

Arg* Args::addArg()
{
  Arg* arg = new Arg(this);
  args_.append(arg);
  connect(arg, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(arg, SIGNAL(nameChanged(const QString&)), this,
          SLOT(argNameChanged(const QString&)));
  connect(arg, SIGNAL(valueChanged(const QString&)), this,
          SLOT(argValueChanged(const QString&)));
  connect(arg, SIGNAL(defaultValueChanged(const QString&)), this,
          SLOT(argDefaultValueChanged(const QString&)));
  connect(arg, SIGNAL(destroyed()), this, SLOT(argDestroyed()));
  emit added(args_.count() - 1);
  emit changed();
  return arg;
}

void Args::removeArg(Arg* arg) { removeArg(args_.indexOf(arg)); }

void Args::removeArg(size_t index)
{
  if (index >= 0 && index < args_.count())
  {
    QString name(args_[index]->getName());
    args_.remove(index);
    emit removed(name);
    emit changed();
  }
}

void Args::clearArgs()
{
  if (!args_.isEmpty())
  {
    for (size_t index(0); index < args_.count(); index++)
    {
      if (args_[index])
      {
        delete args_[index];
        args_[index] = NULL;
      }
    }
    args_.clear();
    emit cleared();
    emit changed();
  }
}

bool Args::contains(const QString& name) const
{
  for (size_t index(0); index < args_.count(); index++)
  {
    if (args_[index]->getName() == name)
    {
      return true;
    }
  }
  return false;
}

size_t Args::count() const { return args_.count(); }

bool Args::isEmpty() const { return args_.isEmpty(); }

QString Args::validate() const
{
  QString validation;
  for (size_t index(0); index < args_.count(); index++)
  {
    validation = args_[index]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

void Args::save(QSettings& settings) const
{
  settings.beginGroup("args");
  for (size_t index(0); index < args_.count(); index++)
  {
    settings.beginGroup("arg_" + QString::number(index));
    args_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Args::load(QSettings& settings)
{
  settings.beginGroup("args");
  QStringList groups(settings.childGroups());
  size_t index(0);
  for (QStringList::iterator it(groups.begin()); it != groups.end(); it++)
  {
    Arg* arg = index < args_.count() ? arg = args_[index] : addArg();
    settings.beginGroup(*it);
    arg->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
  while (index < args_.count())
  {
    removeArg(index);
  }
}

void Args::reset() { clearArgs(); }

void Args::write(QDataStream& stream) const
{
  for (size_t index(0); index < args_.count(); index++)
  {
    args_[index]->write(stream);
  }
}

void Args::read(QDataStream& stream)
{
  for (size_t index(0); index < args_.count(); index++)
  {
    args_[index]->read(stream);
  }
}

Args& Args::operator=(const Args& config)
{
  while (args_.count() < config.args_.count())
  {
    addArg();
  }
  while (args_.count() > config.args_.count())
  {
    removeArg(args_.count() - 1);
  }
  for (size_t index(0); index < args_.count(); ++index)
  {
    *args_[index] = *config.args_[index];
  }
  return *this;
}

QString Args::toLaunch(const QString &prefix) const
{
  QString launch;
  for (size_t index(0); index < args_.count(); index++)
  {
    launch += args_[index]->toLaunch(prefix + "\t");
  }
  return launch;
}

void Args::argNameChanged(const QString& name)
{
  int index(args_.indexOf(static_cast<Arg*>(sender())));
  if (index != -1)
  {
    emit argNameChanged(index, name);
    emit changed();
  }
}

void Args::argValueChanged(const QString& value)
{
  int index(args_.indexOf(static_cast<Arg*>(sender())));
  if (index != -1)
  {
    emit argValueChanged(index, value);
    emit changed();
  }
}

void Args::argDefaultValueChanged(const QString& value)
{
  int index(args_.indexOf(static_cast<Arg*>(sender())));
  if (index != -1)
  {
    emit argDefaultValueChanged(index, value);
    emit changed();
  }
}

void Args::argDestroyed()
{
  Arg* arg = static_cast<Arg*>(sender());
  int index(args_.indexOf(arg));
  if (index >= 0)
  {
    QString name(arg->getName());
    args_.remove(index);
    emit removed(name);
    emit changed();
  }
}
}
}
