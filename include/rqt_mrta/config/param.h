#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_H_

#include "rqt_mrta/config/param_interface.h"

namespace rqt_mrta
{
namespace config
{
class Param : public ParamInterface
{
  Q_OBJECT
public:
  Param(Params* parent = NULL);
  virtual ~Param();
  QMetaType::Type getType() const;
  QString getToolTip() const;
  QVariant getValue() const;
  QVariant getDefaultValue() const;
  bool isMandatory() const;
  void setType(const QString& type);
  void setType(const QMetaType::Type& type);
  void setToolTip(const QString& tool_tip);
  void setValue(const QString& value);
  void setValue(const QVariant& value);
  void setDefaultValue(const QString& value);
  void setDefaultValue(const QVariant& default_value);
  QString validate() const;
  bool isParam() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Param& operator=(const Param& config);
  Param* clone() const;
  QString toYaml(const QString &prefix) const;

signals:
  void typeChanged(const QString& full_name, const QMetaType::Type& type);
  void toolTipChanged(const QString& full_name, const QString& tool_tip);
  void valueChanged(const QString& full_name, const QVariant& value);
  void defaultValueChanged(const QString& full_name,
                           const QVariant& default_value);

private:
  QMetaType::Type type_;
  QString tool_tip_;
  QVariant value_;
  QVariant default_value_;
};
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_H_
