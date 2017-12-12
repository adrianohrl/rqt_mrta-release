#include "rqt_mrta/config/param.h"
#include "rqt_mrta/config/params.h"

namespace rqt_mrta
{
namespace config
{
Param::Param(Params* parent) : ParamInterface("param", parent) {}

Param::~Param() {}

QMetaType::Type Param::getType() const { return type_; }

QString Param::getToolTip() const { return tool_tip_; }

QVariant Param::getValue() const { return value_; }

QVariant Param::getDefaultValue() const { return default_value_; }

bool Param::isMandatory() const { return default_value_.isNull(); }

void Param::setType(const QString& type)
{
  if (type == "bool")
  {
    setType(QMetaType::Bool);
  }
  else if (type == "int")
  {
    setType(QMetaType::Int);
  }
  else if (type == "double")
  {
    setType(QMetaType::Double);
  }
  else
  {
    setType(QMetaType::QString);
  }
}

void Param::setType(const QMetaType::Type& type)
{
  if (type != type_)
  {
    type_ = type;
    emit typeChanged(getFullName(), type);
    emit changed();
  }
}

void Param::setToolTip(const QString& tool_tip)
{
  if (tool_tip != tool_tip_)
  {
    tool_tip_ = tool_tip;
    emit toolTipChanged(getFullName(), tool_tip);
    emit changed();
  }
}

void Param::setValue(const QString& value)
{
  if (type_ == QMetaType::Bool)
  {
    setValue(QVariant::fromValue<bool>(value.toLower() == "true"));
  }
  else if (type_ == QMetaType::Int)
  {
    int v(value.toInt());
    setValue(v != 0 ? QVariant::fromValue<int>(v) : QVariant());
  }
  else if (type_ == QMetaType::Double)
  {
    double v(value.toDouble());
    setValue(v != 0.0 ? QVariant::fromValue<double>(v) : QVariant());
  }
  else
  {
    setValue(!value.isEmpty() ? QVariant::fromValue<QString>(value) : QVariant());
  }
}

void Param::setValue(const QVariant& value)
{
  if (value != value_)
  {
    value_ = value;
    emit valueChanged(getFullName(), value);
    emit changed();
  }
}

void Param::setDefaultValue(const QString& value)
{
  if (type_ == QMetaType::Bool)
  {
    setDefaultValue(QVariant::fromValue<bool>(value.toLower() == "true"));
  }
  else if (type_ == QMetaType::Int)
  {
    int v(value.toInt());
    setDefaultValue(v != 0 ? QVariant::fromValue<int>(v) : QVariant());
  }
  else if (type_ == QMetaType::Double)
  {
    double v(value.toDouble());
    setDefaultValue(v != 0.0 ? QVariant::fromValue<double>(v) : QVariant());
  }
  else
  {
    setDefaultValue(!value.isEmpty() ? QVariant::fromValue<QString>(value) : QVariant());
  }
}

void Param::setDefaultValue(const QVariant& default_value)
{
  if (default_value != default_value_)
  {
    default_value_ = default_value;
    emit defaultValueChanged(getFullName(), default_value);
    emit changed();
  }
}

QString Param::validate() const
{
  if (isMandatory() && value_.isNull())
  {
    return "The " + name_ + " parameter value must be given, because it is mandatory.";
  }
  if (tool_tip_.isEmpty())
  {
    return "The param tool tip must must be given. It helps the architecture "
           "users.";
  }
  return ParamInterface::validate();
}

bool Param::isParam() const { return true; }

void Param::save(QSettings& settings) const
{
  ParamInterface::save(settings);
  settings.setValue("type", type_);
  settings.setValue("value", value_);
  settings.setValue("default", default_value_);
  settings.setValue("tool_tip", tool_tip_);
}

void Param::load(QSettings& settings)
{
  ParamInterface::load(settings);
  setType(settings.value("type").toString());
  setValue(settings.value("value").toString());
  setDefaultValue(settings.value("default").toString());
  setToolTip(settings.value("tool_tip").toString());
}

void Param::reset() { setValue(QString()); }

void Param::write(QDataStream& stream) const
{
  ParamInterface::write(stream);
  stream << (int)type_;
  stream << value_;
  stream << default_value_;
  stream << tool_tip_;
}

void Param::read(QDataStream& stream)
{
  ParamInterface::read(stream);
  int type;
  QVariant value, default_value;
  QString tool_tip;
  stream >> type;
  setType(static_cast<QMetaType::Type>(type));
  stream >> value;
  setValue(value);
  stream >> default_value;
  setDefaultValue(default_value);
  stream >> tool_tip;
  setToolTip(tool_tip);
}

Param& Param::operator=(const Param& config)
{
  ParamInterface::operator=(config);
  setType(config.type_);
  setValue(config.value_);
  setDefaultValue(config.default_value_);
  setToolTip(config.tool_tip_);
  return *this;
}

Param* Param::clone() const
{
  Param* param = new Param();
  *param = *this;
  return param;
}

QString Param::toYaml(const QString &prefix) const
{
  return ParamInterface::toYaml(prefix) + value_.toString();
}
}
}
