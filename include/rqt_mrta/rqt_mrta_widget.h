#ifndef _RQT_MRTA_MRTA_WIDGET_H_
#define _RQT_MRTA_MRTA_WIDGET_H_

#include <QMessageBox>
#include <QVector>
#include <QWidget>
#include <pluginlib/class_loader.h>
#include <rqt_gui_cpp/plugin.h>

namespace mrta
{
class System;
}

namespace utilities
{
class MessageSubscriberRegistry;
}

namespace Ui
{
class RqtMrtaWidget;
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

class RqtMrtaWidget : public QWidget
{
  Q_OBJECT
public:
  RqtMrtaWidget(QWidget* parent, const qt_gui_cpp::PluginContext& context);
  virtual ~RqtMrtaWidget();

private:
  typedef boost::shared_ptr<rqt_gui_cpp::Plugin> PluginPtr;
  typedef QVector<PluginPtr> VectorPluginPtr;
  Ui::RqtMrtaWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  RqtMrtaArchitectureConfig* architecture_config_;
  utilities::MessageSubscriberRegistry* registry_;
  qt_gui_cpp::PluginContext context_;
  VectorPluginPtr external_plugins_;
  pluginlib::ClassLoader<rqt_gui_cpp::Plugin> loader_;
  mrta::System* system_;

private:
  void clear();
  void loadSystem(RqtMrtaApplicationConfig* application_config = NULL);
  void loadApplication(const QString& url = "");
  void loadArchitecture(const QString& url = "");
  void loadRobots();
  QMap<QString, QString> findPlugins(const QString& attribute) const;
  QString askItem(const char* title, const char* label,
                  const QStringList& items);
  void showMessage(const QString& title, const QString& message = "",
                   QMessageBox::Icon icon = QMessageBox::Critical) const;

private slots:
  void newApplicationPushButtonClicked();
  void openApplicationPushButtonClicked();
  void newArchitecturePushButtonClicked();
  void openArchitecturePushButtonClicked();
};
}

#endif // _RQT_MRTA_MRTA_WIDGET_H_
