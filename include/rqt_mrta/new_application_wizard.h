#ifndef _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
#define _RQT_MRTA_NEW_APPLICATION_DIALOG_H_

#include <QLabel>
#include <QWizard>
#include "rqt_mrta/config/application/rqt_mrta_application.h"
#include "rqt_mrta/config/application/rqt_mrta_package.h"
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"

namespace rqt_mrta
{
typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::application::RqtMrtaApplicationPackage RqtMrtaApplicationPackageConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;
class NewApplicationWizardPage;
class NewApplicationWizard : public QWizard
{
  Q_OBJECT
public:
  enum Page
  {
    DefineApplication,
    DefineArchitecture,
    DefineRobots,
    DefineParameters,
    Summary
  };
  NewApplicationWizard(QWidget* parent,
                       RqtMrtaApplicationConfig* application_config,
                       Qt::WindowFlags flags = 0);
  virtual ~NewApplicationWizard();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaApplicationPackageConfig* getPackageConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;

private:
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaApplicationPackageConfig* package_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
  QLabel* summary_label_;
  NewApplicationWizardPage *createSummaryPage();

private slots:
  void summary();
  void generate();
  void resetConfig();
};
}

#endif // _RQT_MRTA_NEW_APPLICATION_DIALOG_H_
