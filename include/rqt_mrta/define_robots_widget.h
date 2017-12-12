#ifndef _RQT_MRTA_DEFINE_ROBOTS_WIDGET_H_
#define _RQT_MRTA_DEFINE_ROBOTS_WIDGET_H_

#include <QWidget>
#include <QStringListModel>

namespace Ui
{
class DefineRobotsWidget;
}

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplication;
}
}

typedef config::application::RqtMrtaApplication RqtMrtaApplicationConfig;

class DefineRobotsWidget : public QWidget
{
  friend class DefineRobotsWizardPage;
  Q_OBJECT
public:
  DefineRobotsWidget(
      QWidget* parent, RqtMrtaApplicationConfig* application_config = NULL);
  virtual ~DefineRobotsWidget();
  RqtMrtaApplicationConfig* getApplicationConfig() const;
  void setApplicationConfig(RqtMrtaApplicationConfig* config);
  QString validate() const;

signals:
  void changed();

private:
  Ui::DefineRobotsWidget* ui_;
  RqtMrtaApplicationConfig* application_config_;
  QStringListModel* tasks_model_;

private slots:
  void idChanged(const QString& id);
  void newRobotButtonClicked();
  void newTaskButtonClicked();
  void robotSelected(const QString& robot_id);
  void taskSelected(const QString& task_id);
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_WIDGET_H_
