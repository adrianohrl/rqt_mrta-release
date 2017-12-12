#ifndef _RQT_MRTA_DEFINE_ARCHITECTURE_WIDGET_H_
#define _RQT_MRTA_DEFINE_ARCHITECTURE_WIDGET_H_

#include <QWidget>

namespace Ui
{
class DefineArchitectureWidget;
}

namespace mrta
{
class Architecture;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication;
}

namespace architecture
{
class RqtMrtaArchitecture;
}
}

typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;
typedef config::architecture::RqtMrtaArchitecture RqtMrtaArchitectureConfig;

class DefineArchitectureWidget : public QWidget
{
  friend class DefineArchitectureWizardPage;
  Q_OBJECT
public:
  DefineArchitectureWidget(
      QWidget* parent, RqtMrtaApplicationConfig* application_config = NULL,
      RqtMrtaArchitectureConfig* architecture_config = NULL);
  virtual ~DefineArchitectureWidget();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  RqtMrtaArchitectureConfig* getArchitectureConfig() const;
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  void setArchitectureConfig(RqtMrtaArchitectureConfig* config);

signals:
  void changed();

private:
  Ui::DefineArchitectureWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;

private slots:
  void architectureChanged();
  void setFilterAllocationType();
  void setFilterRobotType();
  void setFilterTaskType();
  void unknownAchitecture();
  void currentArchitectureChanged(mrta::Architecture* architecture);
};
}

#endif // _RQT_MRTA_DEFINE_ARCHITECTURE_WIDGET_H_
