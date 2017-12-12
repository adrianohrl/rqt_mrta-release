#ifndef _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIDGET_H_
#define _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIDGET_H_

#include <QWidget>

namespace Ui
{
class DefineParametersWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robot;
class Robots;
class RqtMrtaApplication;
}

namespace architecture
{
class RqtMrtaArchitecture;
}
}

typedef config::application::Robot RobotConfig;
typedef config::application::Robots RobotsConfig;
typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class DefineParametersWidget : public QWidget
{
  friend class DefineParametersWizardPage;
  Q_OBJECT
public:
  DefineParametersWidget(
      QWidget* parent, RqtMrtaApplicationConfig* application_config = NULL,
      RqtMrtaArchitectureConfig* architecture_config = NULL);
  virtual ~DefineParametersWidget();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  void setArchitectureConfig(RqtMrtaArchitectureConfig* config);
  QString validate() const;
  void loadTabs();

signals:
  void changed();

private:
  Ui::DefineParametersWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIDGET_H_
