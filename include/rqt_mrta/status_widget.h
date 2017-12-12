#ifndef _RQT_MRTA_STATUS_WIDGET_H_
#define _RQT_MRTA_STATUS_WIDGET_H_

#include <QGridLayout>
#include <QImage>
#include <QLabel>
#include <QList>
#include <QMap>
#include <QPixmap>
#include <QTimer>
#include <QWidget>

namespace rqt_mrta
{
class StatusWidget : public QWidget
{
  Q_OBJECT
public:
  enum Role
  {
    None,
    Okay,
    Warn,
    Error,
    Busy,
    Green,
    Yellow,
    Red,
    Blue,
    Gray
  };
  StatusWidget(QWidget* parent = NULL, Role role = None);
  virtual ~StatusWidget();
  void setIcon(Role role, const QPixmap& icon);
  const QPixmap& getIcon(Role role) const;
  void setFrames(Role role, const QPixmap& frames, size_t num_frames,
                 double frame_fate = 10.0);
  void setFrames(Role role, const QList<QPixmap>& frame_list,
                 double frame_rate = 10.0);
  const QList<QPixmap>& getFrames(Role role) const;
  void setFrameRate(Role role, double frame_rate);
  double getFrameRate(Role role) const;
  void setCurrentRole(Role role, const QString& tool_tip = QString());
  Role getCurrentRole() const;
  void pushCurrentRole();
  bool popCurrentRole();

signals:
  void currentRoleChanged(Role role);

private:
  QGridLayout* layout_;
  QLabel* label_icon_;
  QTimer* timer_;
  QMap<Role, QList<QPixmap>> frames_;
  QMap<Role, double> frame_rates_;
  QList<Role> role_stack_;
  QList<QString> tool_tip_stack_;
  Role current_role_;
  size_t current_frame_;
  void start();
  void step();
  void stop();

private slots:
  void timerTimeout();
};
}

#endif //_RQT_MRTA_STATUS_WIDGET_H_
