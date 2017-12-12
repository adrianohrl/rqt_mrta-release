#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/architecture.h"
#include "rqt_mrta/config/architecture/widgets.h"
#include "rqt_mrta/config/configs.h"
#include "rqt_mrta/config/launches.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class RqtMrtaArchitecture : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RqtMrtaArchitecture(QObject* parent = NULL);
  virtual ~RqtMrtaArchitecture();
  QString getArchitecturePackage() const;
  QString getArchitecturePackageUrl() const;
  Architecture* getArchitecture() const;
  Configs* getConfigs() const;
  Launches* getLaunches() const;
  Widgets* getWidgets() const;
  void setArchitecturePackage(const QString& package);
  void setArchitecturePackageUrl(const QString &url);
  void save(const QString& url) const;
  void load(const QString& url);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RqtMrtaArchitecture& operator=(const RqtMrtaArchitecture& config);

signals:
  void architecturePackageChanged(const QString &package);
  void architecturePackageUrlChanged(const QString& url);

private:
  QString package_;
  QString url_;
  Architecture* architecture_;
  Configs* configs_;
  Launches* launches_;
  Widgets* widgets_;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_H_
