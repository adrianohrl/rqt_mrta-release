#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_TOPIC_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_TOPIC_H_

#include <ros/duration.h>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Topic : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Topic(QObject* parent = NULL);
  virtual ~Topic();
  QString getName() const;
  QString getType() const;
  size_t getQueueSize() const;
  QString getField() const;
  ros::Duration getTimeout() const;
  ros::Duration getHorizon() const;
  void setName(const QString& name);
  void setType(const QString& type);
  void setQueueSize(const size_t& queue_size);
  void setField(const QString& field);
  void setTimeout(const ros::Duration& timeout);
  void setHorizon(const ros::Duration& horizon);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Topic& operator=(const Topic& config);

signals:
  void nameChanged(const QString& name);
  void typeChanged(const QString& type);
  void queueSizeChanged(const size_t& queue_size);
  void fieldChanged(const QString& field);
  void timeoutChanged(const ros::Duration& timeout);
  void horizonChanged(const ros::Duration& horizon);

private:
  QString name_;
  QString type_;
  size_t queue_size_;
  QString field_;
  ros::Duration timeout_;
  ros::Duration horizon_;
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_TOPIC_H_
