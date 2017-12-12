#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_INTERFACE_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_INTERFACE_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Params;

class ParamInterface : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  virtual ~ParamInterface();
  QString getGroupName() const;
  QString getName() const;
  QString getFullName() const;
  virtual ParamInterface* getParentParam() const;
  virtual void setName(const QString& name);
  virtual ParamInterface* getParam(const QString& full_name) const;
  virtual void addParam(ParamInterface* param);
  virtual void removeParam(const QString& full_name);
  virtual void clearParams();
  virtual bool contains(const QString& name) const;
  virtual size_t count() const;
  virtual bool isEmpty() const;
  virtual QString validate() const;
  virtual bool isParam() const;
  virtual bool isParams() const;
  virtual bool isArray() const;
  virtual void save(QSettings& settings) const;
  virtual void load(QSettings& settings);
  virtual void reset();
  virtual void write(QDataStream& stream) const;
  virtual void read(QDataStream& stream);
  virtual ParamInterface& operator=(const ParamInterface& config);
  virtual ParamInterface* clone() const = 0;
  virtual QString toYaml(const QString& prefix = "") const;

signals:
  void nameChanged(const QString &previous_full_name, const QString &name);
  void typeChanged(const QString &full_name, const QMetaType::Type& type);
  void valueChanged(const QString& full_name, const QVariant& value);
  void defaultValueChanged(const QString &full_name, const QVariant& type);
  void toolTipChanged(const QString &full_name, const QString& type);
  void added(const QString& full_name);
  void removed(const QString& full_name);
  void cleared(const QString& full_name);

protected:
  ParamInterface(const QString &group_name, Params* parent);
  QString name_;
  const QString group_name_;

protected slots:
  void paramTypeChanged(const QString &full_name, const QMetaType::Type& type);
  void paramValueChanged(const QString& full_name, const QVariant& value);
  void paramDefaultValueChanged(const QString &full_name, const QVariant& default_value);
  void paramToolTipChanged(const QString &full_name, const QString& tool_tip);
};
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_INTERFACE_H_
