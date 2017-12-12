#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_

#include <QVector>
#include "rqt_mrta/config/param_interface.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;
}
class Params : public ParamInterface
{
  Q_OBJECT
public:
  Params(Params* parent = NULL);
  virtual ~Params();
  QVector<ParamInterface*> getChildren() const;
  ParamInterface* getChild(size_t index) const;
  ParamInterface* getParam(const QString& relative_name) const;
  void addParam(ParamInterface* param);
  void removeParam(const QString& relative_name);
  void clearParams();
  bool contains(const QString& full_name) const;
  size_t count() const;
  bool isEmpty() const;
  QString validate() const;
  virtual bool isParams() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Params& operator=(const Params& config);
  ParamInterface* clone() const;
  static QStringList sortGroups(const QStringList& groups);
  QString toYaml(const QString &prefix) const;

protected:
  QVector<ParamInterface*> params_;
  Params(const QString& group_name, Params* parent = NULL);

private slots:
  void paramDestroyed();
};
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_H_
