#ifndef _RQT_MRTA_CONFIG_LAUNCH_H_
#define _RQT_MRTA_CONFIG_LAUNCH_H_

#include <QMap>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Includes;
class Launch : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Launch(QObject* parent = NULL);
  virtual ~Launch();
  QString getId() const;
  Includes* getIncludes() const;
  QMap<QString, QString> getMap() const;
  void setId(const QString& id);
  void add(const QString& key, const QString& value);
  QString validate() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Launch& operator=(const Launch& config);
  void saveAsLaunch(const QString& url) const;

signals:
  void idChanged(const QString &id);

private:
  typedef QMap<QString, QString> Map;
  typedef Map::iterator iterator;
  typedef Map::const_iterator const_iterator;
  QString id_;
  Includes* includes_;
  Map map_;
  QString toLaunch() const;
};
}
}

#endif // _RQT_MRTA_CONFIG_LAUNCH_H_
