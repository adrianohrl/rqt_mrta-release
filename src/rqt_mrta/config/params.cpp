#include <QStringList>
#include "rqt_mrta/config/params.h"
#include "rqt_mrta/config/param_factory.h"

namespace rqt_mrta
{
namespace config
{
Params::Params(Params* parent) : ParamInterface("params", parent) {}

Params::Params(const QString& group_name, Params* parent)
    : ParamInterface(group_name, parent)
{
}

Params::~Params()
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
}

QVector<ParamInterface*> Params::getChildren() const { return params_; }

ParamInterface* Params::getChild(size_t index) const { return params_[index]; }

ParamInterface* Params::getParam(const QString& relative_name) const
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

void Params::addParam(ParamInterface* param)
{
  param->setParent(this);
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

void Params::removeParam(const QString& relative_name)
{
  ParamInterface* param = getParam(relative_name);
  if (param)
  {
    ParamInterface* parent = param->getParentParam();
    if (parent && parent != this)
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
      QString full_name(params_[index]->getFullName());
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

void Params::clearParams()
{
  if (!params_.isEmpty())
  {
    for (size_t i(0); i < params_.count(); i++)
    {
      if (params_[i])
      {
        delete params_[i];
        params_[i] = NULL;
      }
    }
    params_.clear();
    emit cleared(getFullName());
    emit changed();
  }
}

bool Params::contains(const QString& full_name) const
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

size_t Params::count() const { return params_.count(); }

bool Params::isEmpty() const { return params_.isEmpty(); }

void Params::save(QSettings& settings) const
{
  ParamInterface::save(settings);
  for (size_t index(0); index < params_.count(); ++index)
  {
    settings.beginGroup(params_[index]->getGroupName() + "_" +
                        QString::number(index));
    params_[index]->save(settings);
    settings.endGroup();
  }
}

void Params::load(QSettings& settings)
{
  ParamInterface::load(settings);
  QStringList groups(Params::sortGroups(settings.childGroups()));
  clearParams();
  for (size_t index(0); index < groups.count(); index++)
  {
    ParamInterface* param = ParamFactory::newInstance(groups[index], this);
    addParam(param);
    settings.beginGroup(param->getGroupName() + "_" + QString::number(index));
    param->load(settings);
    settings.endGroup();
  }
}

void Params::reset()
{
  for (size_t index(0); index < params_.count(); index++)
  {
    params_[index]->reset();
  }
}

void Params::write(QDataStream& stream) const
{
  ParamInterface::write(stream);
  stream << params_.count();
  for (size_t index(0); index < params_.count(); ++index)
  {
    params_[index]->write(stream);
  }
}

void Params::read(QDataStream& stream)
{
  quint64 count;
  ParamInterface::read(stream);
  stream >> count;
  clearParams();
  for (size_t index(0); index < count; ++index)
  {
    params_[index]->read(stream);
  }
}

Params& Params::operator=(const Params& config)
{
  ParamInterface::operator=(config);
  clearParams();
  for (size_t index(0); index < config.params_.count(); index++)
  {
    addParam(config.params_[index]->clone());
  }
  return *this;
}

ParamInterface* Params::clone() const
{
  Params* params = new Params();
  *params = *this;
  return params;
}

QString Params::validate() const
{
  if (params_.isEmpty())
  {
    return "Enter the params parameters.";
  }
  QString validation(ParamInterface::validate());
  if (!validation.isEmpty())
  {
    return validation;
  }
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

bool Params::isParams() const { return true; }

void Params::paramDestroyed()
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

QStringList Params::sortGroups(const QStringList& groups)
{
  QStringList sorted_groups;
  for (size_t index(0); index < groups.count(); index++)
  {
    QString group(groups.filter("_" + QString::number(index)).first());
    if (group.isEmpty())
    {
      ROS_ERROR("Invalid param index.");
      return sorted_groups;
    }
    sorted_groups.append(group);
  }
  return sorted_groups;
}

QString Params::toYaml(const QString &prefix) const
{
  QString yaml(ParamInterface::toYaml(prefix));
  for (size_t index(0); index < params_.count(); index++)
  {
    yaml += "\n" + params_[index]->toYaml(prefix + "\t");
  }
  return yaml;
}
}
}
