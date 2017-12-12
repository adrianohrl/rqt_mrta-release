#ifndef _RQT_MRTA_APPLICATION_CONFIG_APPLICATION_H_
#define _RQT_MRTA_APPLICATION_CONFIG_APPLICATION_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/robots.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Application : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Application(QObject* parent = NULL);
  virtual ~Application();
  QString getName() const;
  QString getArchitecturePackage() const;
  Robots* getRobots() const;
  void setName(const QString& name);
  void setArchitecturePackage(const QString& package);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Application& operator=(const Application& config);

signals:
  void nameChanged(const QString& name);
  void architecturePackageChanged(const QString& package);

private:
  QString name_;
  QString architecture_;
  Robots* robots_;
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_APPLICATION_H_
