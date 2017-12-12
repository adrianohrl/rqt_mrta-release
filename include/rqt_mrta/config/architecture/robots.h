#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOTS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOTS_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/busy_robots.h"
#include "rqt_mrta/config/architecture/idle_robots.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Robots : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Robots(QObject* parent = NULL);
  virtual ~Robots();
  QString getType() const;
  QString getConfigId() const;
  QString getLaunchId() const;
  BusyRobots* getBusyRobots() const;
  IdleRobots* getIdleRobots() const;
  void setType(const QString& type);
  void setConfigId(const QString& config_id);
  void setLaunchId(const QString& launch_id);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Robots& operator=(const Robots& config);

signals:
  void typeChanged(const QString& type);
  void configIdChanged(const QString& config_id);
  void launchIdChanged(const QString& launch_id);

private:
  QString type_;
  QString config_id_;
  QString launch_id_;
  BusyRobots* busy_robots_;
  IdleRobots* idle_robots_;
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ROBOTS_H_
