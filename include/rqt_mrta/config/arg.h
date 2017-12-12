#ifndef _RQT_MRTA_CONFIG_ARG_H_
#define _RQT_MRTA_CONFIG_ARG_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Arg : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Arg(QObject* parent = NULL);
  virtual ~Arg();
  QString getName() const;
  QString getValue() const;
  QString getDefaultValue() const;
  bool isMandatory() const;
  void setName(const QString& name);
  void setValue(const QString& value);
  void setDefaultValue(const QString& value);
  QString validate() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Arg& operator=(const Arg& config);
  QString toLaunch(const QString& prefix) const;

signals:
  void nameChanged(const QString& name);
  void valueChanged(const QString& value);
  void defaultValueChanged(const QString& value);

private:
  QString name_;
  QString value_;
  QString default_value_;
};
}
}

#endif // _RQT_MRTA_CONFIG_ARG_H_
