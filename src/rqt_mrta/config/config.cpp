#include <QFile>
#include <QStringList>
#include "rqt_mrta/config/config.h"
#include "rqt_mrta/config/param_factory.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
namespace config
{
Config::Config(QObject* parent) : AbstractConfig(parent) {}

Config::~Config()
{
  clearArrays();
  /*for (size_t index(0); index < params_.count(); index++)
  {
    if (params_[index])
    {
      delete params_[index];
      params_[index] = NULL;
    }
  }*/
  params_.clear();
}

QString Config::getId() const { return id_; }

void Config::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

QVector<ParamInterface*> Config::getChildren() const { return params_; }

ParamInterface* Config::getChild(size_t index) const { return params_[index]; }

ParamInterface* Config::getParam(const QString& relative_name) const
{
  QStringList names(relative_name.split("/"));
  QString root_name(names[0]);
  if (root_name.isEmpty())
  {
    return NULL;
  }
  names.removeFirst();
  for (size_t i(0); i < params_.count(); i++)
  {
    if (params_[i]->getName() == root_name)
    {
      return !names.isEmpty() ? params_[i]->getParam(names.join("/"))
                              : params_[i];
    }
  }
  return NULL;
}

void Config::addParam(ParamInterface* param)
{
  params_.append(param);
  connect(param, SIGNAL(changed()), this, SIGNAL(changed()));
  connect(param, SIGNAL(nameChanged(const QString&, const QString&)), this,
          SIGNAL(nameChanged(const QString&, const QString&)));
  connect(param, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)),
          this, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)));
  connect(param, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
          SIGNAL(valueChanged(const QString&, const QVariant&)));
  connect(param, SIGNAL(defaultValueChanged(const QString&, const QVariant&)),
          this, SIGNAL(defaultValueChanged(const QString&, const QVariant&)));
  connect(param, SIGNAL(toolTipChanged(const QString&, const QString&)), this,
          SIGNAL(toolTipChanged(const QString&, const QString&)));
  connect(param, SIGNAL(added(const QString&)), this,
          SIGNAL(added(const QString&)));
  connect(param, SIGNAL(removed(const QString&)), this,
          SIGNAL(removed(const QString&)));
  connect(param, SIGNAL(cleared(const QString&)), this,
          SIGNAL(cleared(const QString&)));
  connect(param, SIGNAL(destroyed()), this, SLOT(paramDestroyed()));
  emit added(param->getFullName());
  emit changed();
}

void Config::removeParam(const QString& full_name)
{
  ParamInterface* param = getParam(full_name);
  if (param)
  {
    ParamInterface* parent = param->getParentParam();
    if (parent)
    {
      parent->removeParam(param->getName());
    }
    else
    {
      size_t index(params_.indexOf(param));
      if (index == -1)
      {
        return;
      }
      /*if (params_[index])
      {
        delete params_[index];
        params_[index] = NULL;
      }*/
      params_.remove(index);
      emit removed(full_name);
      emit changed();
    }
  }
}

void Config::clearParams()
{
  if (!params_.isEmpty())
  {
    for (size_t index(0); index < params_.count(); index++)
    {
      /*if (params_[index])
      {
        delete params_[index];
        params_[index] = NULL;
      }*/
    }
    params_.clear();
    emit cleared("");
    emit changed();
  }
}

void Config::clearParams(const QString& full_name)
{
  if (!full_name.isEmpty())
  {
    ParamInterface* param = getParam(full_name);
    param->clearParams();
  }
  else if (!params_.isEmpty())
  {
    clearParams();
  }
}

bool Config::contains(const QString& full_name) const
{
  QStringList names(full_name.split("/"));
  for (size_t index(0); index < params_.count(); index++)
  {
    if (params_[index]->getName() == names[0])
    {
      names.removeFirst();
      return names.isEmpty() || params_[index]->contains(names.join("/"));
    }
  }
  return false;
}

size_t Config::count() const { return params_.count(); }

size_t Config::count(const QString& full_name) const
{
  if (full_name.isEmpty())
  {
    return params_.count();
  }
  ParamInterface* param = getParam(full_name);
  return param->count();
}

bool Config::isEmpty() const { return params_.isEmpty(); }

bool Config::isEmpty(const QString& full_name) const
{
  if (full_name.isEmpty())
  {
    return params_.isEmpty();
  }
  ParamInterface* param = getParam(full_name);
  return param->isEmpty();
}

void Config::save(QSettings& settings) const
{
  settings.setValue("id", id_);
  for (size_t index(0); index < params_.count(); ++index)
  {
    settings.beginGroup(params_[index]->getGroupName() + "_" +
                        QString::number(index));
    params_[index]->save(settings);
    settings.endGroup();
  }
}

void Config::load(QSettings& settings)
{
  setId(settings.value("id").toString());
  QStringList groups(Params::sortGroups(settings.childGroups()));
  clearParams();
  for (size_t index(0); index < groups.count(); index++)
  {
    ParamInterface* param = ParamFactory::newInstance(groups[index]);
    addParam(param);
    settings.beginGroup(param->getGroupName() + "_" + QString::number(index));
    param->load(settings);
    settings.endGroup();
  }
}

void Config::reset()
{
  for (size_t index(0); index < params_.count(); index++)
  {
    params_[index]->reset();
  }
}

void Config::write(QDataStream& stream) const
{
  stream << id_;
  stream << params_.count();
  for (size_t index(0); index < params_.count(); ++index)
  {
    params_[index]->write(stream);
  }
}

void Config::read(QDataStream& stream)
{
  quint64 count;
  QString id;
  stream >> id;
  setId(id);
  stream >> count;
  clearParams();
  for (size_t index(0); index < count; ++index)
  {
    params_[index]->read(stream);
  }
}

Config& Config::operator=(const Config& config)
{
  setId(config.id_);
  clearParams();
  for (size_t index(0); index < config.params_.count(); index++)
  {
    addParam(config.params_[index]->clone());
  }
  return *this;
}

QString Config::validate() const
{
  if (id_.isEmpty())
  {
    return "The config name must not be empty.";
  }
  if (id_.contains(' '))
  {
    return "The config name must not contain <space>.";
  }
  if (params_.isEmpty())
  {
    return "Enter the config parameters.";
  }
  QString validation;
  for (size_t i(0); i < params_.count(); i++)
  {
    validation = params_[i]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

void Config::saveAsYaml(const QString &url) const
{
  QFile file(url + ".yaml");
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR_STREAM("Unable to open the " << url.toStdString() << ".yaml file.");
    return;
  }
  file.write(toYaml().toStdString().c_str());
  file.close();
  ROS_INFO_STREAM("Created the " << url.toStdString() << ".yaml file.");
}

QString Config::toYaml() const
{
  QString yaml;
  for (size_t i(0); i < params_.count(); i++)
  {
    yaml += params_[i]->toYaml() + "\n";
  }
  return yaml;
}

void Config::hideArrays()
{
  for (size_t index(0); index < params_.count(); index++)
  {
    if (params_[index]->isArray())
    {
      throw utilities::Exception("ParamArrays must have a Params parent.");
    }
    else if (params_[index]->isParams())
    {
      findArrays(static_cast<Params*>(params_[index]));
    }
  }
}

void Config::paramDestroyed()
{
  ParamInterface* param = static_cast<ParamInterface*>(sender());
  int index(params_.indexOf(param));
  if (index != -1)
  {
    QString full_name(param->getFullName());
    params_.remove(index);
    emit removed(full_name);
    emit changed();
  }
}

void Config::findArrays(Params* parent)
{
  for (size_t index(0); index < parent->count(); index++)
  {
    if (parent->getChild(index)->isArray())
    {
      ParamInterface* size = parent->getParam("size");
      if (!size || !size->isParam())
      {
        throw utilities::Exception("The ParamsArray's parent must have a Param named size.");
      }
      ParamsArray* array = static_cast<ParamsArray*>(parent->getChild(index));
      parent->removeParam(array->getName());
      connect(size, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
              SLOT(arraySizeChanged(const QString&, const QVariant&)));
      arrays_.insert(static_cast<Param*>(size), array);
    }
    else if (parent->getChild(index)->isParams())
    {
      findArrays(static_cast<Params*>(parent->getChild(index)));
    }
  }
}

void Config::clearArrays()
{
  for (iterator it(arrays_.begin()); it != arrays_.end(); it++)
  {
    arrays_[it.key()]  = NULL;
    arrays_.remove(it.key());
  }
  arrays_.clear();
}

void Config::arraySizeChanged(const QString& full_name, const QVariant& value)
{
  Param* size = static_cast<Param*>(sender());
  ParamsArray* array = arrays_[size];
  array->createParams(value.toInt());
  findArrays(array->getParentParam());
}
}
}
