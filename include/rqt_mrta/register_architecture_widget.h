#ifndef _RQT_MRTA_REGISTER_ARCHITECTURE_WIDGET_H_
#define _RQT_MRTA_REGISTER_ARCHITECTURE_WIDGET_H_

#include <QWidget>

namespace Ui
{
class RegisterArchitectureWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Allocations;
class Architecture;
class Robots;
class RqtMrtaArchitecture;
class Tasks;
}
}
typedef config::architecture::RqtMrtaArchitecture Config;
typedef config::architecture::Architecture ArchitectureConfig;
typedef config::architecture::Robots RobotsConfig;
typedef config::architecture::Tasks TasksConfig;
typedef config::architecture::Allocations AllocationsConfig;
class RegisterArchitectureWidget : public QWidget
{
  friend class RegisterArchitectureWizardPage;
  Q_OBJECT
public:
  RegisterArchitectureWidget(QWidget* parent, Config* config);
  virtual ~RegisterArchitectureWidget();
  Config* getConfig() const;
  void setConfig(Config* config);
  QString validate() const;

signals:
  void changed();

private:
  Ui::RegisterArchitectureWidget* ui_;
  Config* config_;

private slots:
  void packageChanged(const QString& package);
  void robotTypeChanged();
  void taskTypeChanged();
  void allocationTypeChanged();
  void configPackageChanged(const QString& package);
  void configRobotsTypeChanged(const QString& type);
  void configTasksTypeChanged(const QString& type);
  void configAllocationsTypeChanged(const QString& type);
};
}

#endif // _RQT_MRTA_REGISTER_ARCHITECTURE_WIDGET_H_
