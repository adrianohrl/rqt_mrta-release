#include "rqt_mrta/status_widget.h"
#include "ros/package.h"

namespace rqt_mrta
{
StatusWidget::StatusWidget(QWidget* parent, Role role)
    : QWidget(parent), layout_(new QGridLayout(this)),
      label_icon_(new QLabel(this)), timer_(new QTimer(this)),
      current_role_(role), current_frame_(0)
{
  setLayout(layout_);
  layout_->setContentsMargins(0, 0, 0, 0);
  layout_->addWidget(label_icon_, 0, 0);
  frames_[None] = QList<QPixmap>();
  frames_[Okay] = QList<QPixmap>();
  frames_[Warn] = QList<QPixmap>();
  frames_[Error] = QList<QPixmap>();
  frames_[Busy] = QList<QPixmap>();
  frames_[Green] = QList<QPixmap>();
  frames_[Yellow] = QList<QPixmap>();
  frames_[Red] = QList<QPixmap>();
  frames_[Blue] = QList<QPixmap>();
  frames_[Gray] = QList<QPixmap>();
  frame_rates_[None] = 0.0;
  frame_rates_[Okay] = 0.0;
  frame_rates_[Warn] = 0.0;
  frame_rates_[Error] = 0.0;
  frame_rates_[Busy] = 0.0;
  frame_rates_[Green] = 0.0;
  frame_rates_[Yellow] = 0.0;
  frame_rates_[Red] = 0.0;
  frame_rates_[Blue] = 0.0;
  frame_rates_[Gray] = 0.0;
  QPixmap pixmap_none(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/none.png")));
  QPixmap pixmap_okay(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/okay.png")));
  QPixmap pixmap_warn(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/warn.png")));
  QPixmap pixmap_error(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/error.png")));
  QPixmap pixmap_busy(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/busy.png")));
  QPixmap pixmap_green(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/green.png")));
  QPixmap pixmap_yellow(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/yellow.png")));
  QPixmap pixmap_red(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/red.png")));
  QPixmap pixmap_blue(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/blue.png")));
  QPixmap pixmap_gray(QString::fromStdString(
      ros::package::getPath("rqt_mrta").append("/resource/22x22/gray.png")));
  setIcon(None, pixmap_none);
  setIcon(Okay, pixmap_okay);
  setIcon(Warn, pixmap_warn);
  setIcon(Error, pixmap_error);
  setFrames(Busy, pixmap_busy, 8);
  setIcon(Green, pixmap_green);
  setIcon(Yellow, pixmap_yellow);
  setIcon(Red, pixmap_red);
  setIcon(Blue, pixmap_blue);
  setIcon(Gray, pixmap_gray);
  connect(timer_, SIGNAL(timeout()), this, SLOT(timerTimeout()));
}

StatusWidget::~StatusWidget() {}

void StatusWidget::setIcon(Role role, const QPixmap& icon)
{
  setFrames(role, icon, 1, 0.0);
}

const QPixmap& StatusWidget::getIcon(Role role) const
{
  if (frames_[role].isEmpty())
  {
    static QPixmap icon;
    return icon;
  }
  return frames_[role].front();
}

void StatusWidget::setFrames(Role role, const QPixmap& frames,
                             size_t num_frames, double frame_rate)
{
  QList<QPixmap> frame_list;
  size_t frame_height = frames.height() / num_frames;
  for (size_t i = 0; i < num_frames; ++i)
  {
    QPixmap frame =
        frames.copy(0, i * frame_height, frames.width(), frame_height);
    frame_list.append(frame);
  }
  setFrames(role, frame_list, frame_rate);
}

void StatusWidget::setFrames(Role role, const QList<QPixmap>& frame_list,
                             double frame_rate)
{
  bool wasStarted(false);
  if (role == current_role_)
  {
    wasStarted = true;
    stop();
  }
  frames_[role] = frame_list;
  frame_rates_[role] = frame_rate;
  if (wasStarted)
  {
    start();
  }
}

const QList<QPixmap>& StatusWidget::getFrames(Role role) const
{
  QMap<Role, QList<QPixmap>>::const_iterator it(frames_.find(role));
  if (it == frames_.end())
  {
    static QList<QPixmap> frames;
    return frames;
  }
  return it.value();
}

void StatusWidget::setFrameRate(Role role, double frame_rate)
{
  if (frame_rate != frame_rates_[role])
  {
    frame_rates_[role] = frame_rate;
    if ((role == current_role_) && timer_->isActive())
    {
      if (frame_rate > 0.0)
      {
        timer_->setInterval(1.0 / frame_rate * 1e3);
      }
      else
      {
        timer_->stop();
      }
    }
  }
}

double StatusWidget::getFrameRate(Role role) const
{
  return frame_rates_[role];
}

void StatusWidget::setCurrentRole(Role role, const QString& tool_tip)
{
  if (role != current_role_)
  {
    stop();
    current_role_ = role;
    setToolTip(tool_tip);
    start();
    emit currentRoleChanged(role);
  }
  else
  {
    setToolTip(tool_tip);
  }
}

StatusWidget::Role StatusWidget::getCurrentRole() const
{
  return current_role_;
}

void StatusWidget::pushCurrentRole()
{
  role_stack_.append(current_role_);
  tool_tip_stack_.append(toolTip());
}

bool StatusWidget::popCurrentRole()
{
  if (role_stack_.isEmpty())
  {
    return false;
  }
  setCurrentRole(role_stack_.last(), tool_tip_stack_.last());
  role_stack_.removeLast();
  tool_tip_stack_.removeLast();
  return true;
}

void StatusWidget::start()
{
  if (timer_->isActive())
  {
    timer_->stop();
  }
  current_frame_ = 0;
  if (!frames_[current_role_].isEmpty())
  {
    label_icon_->setPixmap(frames_[current_role_].front());
    if (frame_rates_[current_role_] > 0.0)
    {
      timer_->start(1.0 / frame_rates_[current_role_] * 1e3);
    }
  }
}

void StatusWidget::step()
{
  ++current_frame_;
  if (current_frame_ >= frames_[current_role_].length())
  {
    current_frame_ = 0;
  }
  if (!frames_[current_role_].isEmpty())
  {
    label_icon_->setPixmap(frames_[current_role_].at(current_frame_));
  }
}

void StatusWidget::stop()
{
  if (timer_->isActive())
  {
    timer_->stop();
  }
}

void StatusWidget::timerTimeout() { step(); }
}
