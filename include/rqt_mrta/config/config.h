#ifndef _RQT_MRTA_CONFIG_CONFIG_H_
#define _RQT_MRTA_CONFIG_CONFIG_H_

#include <QMap>
#include <QVector>
#include "rqt_mrta/config/param_interface.h"

namespace rqt_mrta
{
namespace config
{
class Param;
class ParamInterface;
class Params;
class ParamsArray;

class Config : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Config(QObject* parent = NULL);
  virtual ~Config();
  QString getId() const;
  void setId(const QString& id);
  QVector<ParamInterface*> getChildren() const;
  ParamInterface* getChild(size_t index) const;
  ParamInterface* getParam(const QString& relative_name) const;
  void addParam(ParamInterface* param);
  void removeParam(const QString& full_name);
  void clearParams();
  void clearParams(const QString& full_name);
  bool contains(const QString& full_name) const;
  size_t count() const;
  size_t count(const QString& full_name) const;
  bool isEmpty() const;
  bool isEmpty(const QString& full_name) const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Config& operator=(const Config& config);
  QString validate() const;
  void saveAsYaml(const QString& url) const;
  void hideArrays();

signals:
  void idChanged(const QString& id);
  void added(const QString& full_name);
  void removed(const QString& full_name);
  void cleared(const QString& full_name);
  void nameChanged(const QString& previous_name, const QString& name);
  void typeChanged(const QString& name, const QMetaType::Type& type);
  void valueChanged(const QString& name, const QVariant& value);
  void defaultValueChanged(const QString& name, const QVariant& default_value);
  void toolTipChanged(const QString& name, const QString& tool_tip);

private:
  typedef QMap<Param*, ParamsArray*> ArrayMap;
  typedef ArrayMap::iterator iterator;
  typedef ArrayMap::const_iterator const_iterator;
  QString id_;
  QVector<ParamInterface*> params_;
  ArrayMap arrays_;
  void findArrays(Params *parent);
  void clearArrays();
  QString toYaml() const;

private slots:
  void paramDestroyed();
  void arraySizeChanged(const QString& full_name, const QVariant& value);
};
}
}

#endif // _RQT_MRTA_CONFIG_CONFIG_H_
