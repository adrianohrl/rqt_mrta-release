#include "rqt_mrta/config/param_interface.h"
#include "rqt_mrta/config/params.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
namespace config
{
ParamInterface::ParamInterface(const QString& group_name, Params* parent)
    : AbstractConfig(parent), group_name_(group_name)
{
}

ParamInterface::~ParamInterface()
{
  setParent(NULL);
}

QString ParamInterface::getGroupName() const { return group_name_; }

QString ParamInterface::getName() const { return name_; }

QString ParamInterface::getFullName() const
{
  ParamInterface* parent = getParentParam();
  return (parent ? parent->getFullName() + "/" : "") + name_;
}

ParamInterface* ParamInterface::getParentParam() const
{
  return parent() ? static_cast<ParamInterface*>(parent()) : NULL;
}

void ParamInterface::setName(const QString& name)
{
  if (name != name_)
  {
    QString full_name(getFullName());
    name_ = name;
    emit nameChanged(full_name, name);
    emit changed();
  }
}

ParamInterface* ParamInterface::getParam(const QString& full_name) const
{
  return NULL;
}

void ParamInterface::addParam(ParamInterface* param)
{
  throw utilities::Exception("param tags cannot have another param type tags.");
}

void ParamInterface::removeParam(const QString& param)
{
  throw utilities::Exception("param tags do not have another param type tags.");
}

void ParamInterface::clearParams() {}

bool ParamInterface::contains(const QString& name) const { return false; }

size_t ParamInterface::count() const { return 0; }

bool ParamInterface::isEmpty() const { return true; }

QString ParamInterface::validate() const
{
  if (name_.isEmpty())
  {
    return "The ParamInterface name must not be empty.";
  }
  if (name_.contains(' '))
  {
    return "The ParamInterface name must not contain <space>.";
  }
  return "";
}

bool ParamInterface::isParam() const { return false; }

bool ParamInterface::isParams() const { return false; }

bool ParamInterface::isArray() const { return false; }

void ParamInterface::save(QSettings& settings) const
{
  settings.setValue("name", name_);
}

void ParamInterface::load(QSettings& settings)
{
  setName(settings.value("name").toString());
}

void ParamInterface::reset() {}

void ParamInterface::write(QDataStream& stream) const { stream << name_; }

void ParamInterface::read(QDataStream& stream)
{
  QString name;
  stream >> name;
  setName(name);
}

ParamInterface& ParamInterface::operator=(const ParamInterface& config)
{
  setName(config.name_);
  return *this;
}

QString ParamInterface::toYaml(const QString& prefix) const
{
  return prefix + name_ + ": ";
}

void ParamInterface::paramTypeChanged(const QString& full_name,
                                      const QMetaType::Type& type)
{
  emit typeChanged(name_ + "/" + full_name, type);
  emit changed();
}

void ParamInterface::paramValueChanged(const QString& full_name,
                                       const QVariant& value)
{
  emit valueChanged(name_ + "/" + full_name, value);
  emit changed();
}

void ParamInterface::paramDefaultValueChanged(const QString& full_name,
                                              const QVariant& default_value)
{
  emit defaultValueChanged(name_ + "/" + full_name, default_value);
  emit changed();
}

void ParamInterface::paramToolTipChanged(const QString& full_name,
                                         const QString& tool_tip)
{
  emit toolTipChanged(name_ + "/" + full_name, tool_tip);
  emit changed();
}
}
}
