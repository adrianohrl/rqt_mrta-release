#ifndef _RQT_MRTA_CONFIG_LAUNCHES_H_
#define _RQT_MRTA_CONFIG_LAUNCHES_H_

#include <QVector>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robots;
}
class Launch;
class Launches : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Launches(QObject* parent = NULL);
  virtual ~Launches();
  Launch* getLaunch(size_t index) const;
  Launch* getLaunch(const QString& id) const;
  Launch* addLaunch();
  void removeLaunch(Launch* launch);
  void removeLaunch(size_t index);
  void clearLaunches();
  bool contains(const QString& id) const;
  size_t count() const;
  bool isEmpty() const;
  QString validate() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Launches& operator=(const Launches& config);
  void setLaunches(const Launches& launches, const application::Robots& robots,
                   const QString& robots_launch_id);
  QStringList willBeGenerated() const;
  void saveAsLaunch(const QString& package_url) const;

signals:
  void added(size_t index);
  void removed(const QString& id);
  void cleared();
  void launchIdChanged(size_t index, const QString& id);

private:
  QVector<Launch*> launches_;

private slots:
  void launchIdChanged(const QString& id);
  void launchDestroyed();
};
}
}

#endif // _RQT_MRTA_CONFIG_LAUNCHES_H_
