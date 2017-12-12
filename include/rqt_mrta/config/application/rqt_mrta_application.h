#ifndef _RQT_MRTA_APPLICATION_CONFIG_H_
#define _RQT_MRTA_APPLICATION_CONFIG_H_

#include "rqt_mrta/config/application/application.h"
#include "rqt_mrta/config/configs.h"
#include "rqt_mrta/config/launches.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RqtMrtaApplication(QObject* parent = NULL);
  virtual ~RqtMrtaApplication();
  QString getApplicationPackage() const;
  QString getApplicationPackageUrl() const;
  Application* getApplication() const;
  Configs* getConfigs() const;
  Launches* getLaunches() const;
  void setApplicationPackage(const QString& package);
  void setApplicationPackageUrl(const QString &url);
  void save() const;
  void save(const QString& filename) const;
  void load(const QString& filename);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RqtMrtaApplication& operator=(const RqtMrtaApplication& config);

signals:
  void applicationPackageChanged(const QString &package);
  void applicationPackageUrlChanged(const QString& url);

private:
  QString package_;
  QString url_;
  Application* application_;
  Configs* configs_;
  Launches* launches_;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_H_
