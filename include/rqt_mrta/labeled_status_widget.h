#ifndef _RQT_MRTA_LABELED_STATUS_WIDGET_H_
#define _RQT_MRTA_LABELED_STATUS_WIDGET_H_

#include <QWidget>

namespace Ui
{
class LabeledStatusWidget;
}

namespace mrta
{
class Robot;
class Task;
}

namespace rqt_mrta
{
class LabeledStatusWidget : public QWidget
{
  Q_OBJECT
public:
  LabeledStatusWidget(QWidget* parent, mrta::Robot* robot);
  LabeledStatusWidget(QWidget* parent, mrta::Task* task);
  virtual ~LabeledStatusWidget();

private:
  Ui::LabeledStatusWidget* ui_;

private slots:
  void labelChanged(const QString& label);
  void setGreen();
  void setYellow();
  void setRed();
  void setBlue();
  void setGray();
  void setLabel(const QString& label);
  void objectDestroyed();
};
}

#endif // _RQT_MRTA_LABELED_STATUS_WIDGET_H_
