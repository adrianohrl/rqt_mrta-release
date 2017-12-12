#include "rqt_mrta/config/arg.h"

namespace rqt_mrta
{
namespace config
{
Arg::Arg(QObject* parent) : AbstractConfig(parent) {}

Arg::~Arg() {}

QString Arg::getName() const { return name_; }

QString Arg::getValue() const { return value_; }

QString Arg::getDefaultValue() const { return default_value_; }

bool Arg::isMandatory() const { return default_value_.isEmpty(); }

void Arg::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
  }
}

void Arg::setValue(const QString& value)
{
  if (value != value_)
  {
    value_ = value;
    emit valueChanged(value);
    emit changed();
  }
}

void Arg::setDefaultValue(const QString& value)
{
  if (value != value_)
  {
    default_value_ = value;
    emit defaultValueChanged(value);
    emit changed();
  }
}

QString Arg::validate() const
{
  if (isMandatory() && value_.isEmpty())
  {
    return "Enter the argument value, it is mandatory.";
  }
  return "";
}

void Arg::save(QSettings& settings) const
{
  settings.setValue("name", name_);
  settings.setValue("value", value_);
  settings.setValue("default", default_value_);
}

void Arg::load(QSettings& settings)
{
  setName(settings.value("name").toString());
  setValue(settings.value("value").toString());
  setDefaultValue(settings.value("default").toString());
}

void Arg::reset()
{
  setName("");
  setValue("");
  setDefaultValue("");
}

void Arg::write(QDataStream& stream) const
{
  stream << name_;
  stream << value_;
  stream << default_value_;
}

void Arg::read(QDataStream& stream)
{
  QString name;
  QString value;
  QString default_value;
  stream >> name;
  stream >> value;
  stream >> default_value;
  setName(name);
  setValue(value);
  setDefaultValue(default_value);
}

Arg& Arg::operator=(const Arg& config)
{
  setName(config.name_);
  setValue(config.value_);
  setDefaultValue(config.default_value_);
  return *this;
}

QString Arg::toLaunch(const QString &prefix) const
{
  QString launch(prefix + "<arg ");
  launch += "name=\"" + name_ + "\" ";
  launch += "value=\"" + (!value_.isEmpty() ? value_ : default_value_) + "\"";
  launch += " />\n";
  return launch;
}
}
}
