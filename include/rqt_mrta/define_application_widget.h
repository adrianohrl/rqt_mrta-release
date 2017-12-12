#ifndef _RQT_MRTA_DEFINE_APPLICATION_WIDGET_H_
#define _RQT_MRTA_DEFINE_APPLICATION_WIDGET_H_

#include <QWidget>
#include <rospack/rospack.h>

namespace Ui
{
class DefineApplicationWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication;
class RqtMrtaApplicationPackage;
}
}

typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::application::RqtMrtaApplicationPackage
    RqtMrtaApplicationPackageConfig;

class DefineApplicationWidget : public QWidget
{
  friend class DefineApplicationWizardPage;
  Q_OBJECT
public:
  DefineApplicationWidget(QWidget* parent,
                          RqtMrtaApplicationConfig* application_config, RqtMrtaApplicationPackageConfig *metapackage_config);
  virtual ~DefineApplicationWidget();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaApplicationPackageConfig* getMetapackageConfig() const;
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  void setMetapackageConfig(RqtMrtaApplicationPackageConfig* config);
  void createMetapackage();

signals:
  void changed();

private:
  Ui::DefineApplicationWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaApplicationPackageConfig* package_config_;
  rospack::Rospack rp_;

private slots:
  void configNameChanged(const QString& name);
  void configPackageChanged(const QString& package);
  void configWorkspaceUrlChanged(const QString& url);
  void configVersionChanged(const QString& version);
  void configDescriptionChanged(const QString& description);
  void configMaintainerChanged(const QString& name);
  void configMaintainerEmailChanged(const QString& email);
  void configLicenseChanged(const QString& license);
  void configRunDependsChanged(const QStringList& depends);
  void nameChanged(const QString& name);
  void packageChanged(const QString& package);
  void workspaceBrowserButtonClicked();
  void workspaceUrlChanged(const QString& url);
  void versionChanged(const QString& version);
  void descriptionChanged();
  void maintainerChanged(const QString& name);
  void maintainerEmailChanged(const QString& email);
  void licenseChanged(const QString& license);
  void runDependsChanged();
};
}

#endif // _RQT_MRTA_DEFINE_APPLICATION_WIDGET_H_
