#ifndef _MRTA_MONITOR_H_
#define _MRTA_MONITOR_H_

#include <QMap>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include "utilities/message_field_subscriber.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Topic;
}
}
}

namespace mrta
{
class StateMonitor;
class System;
class Monitor : public QObject
{
  Q_OBJECT
public:
  typedef rqt_mrta::config::architecture::Topic Config;
  virtual ~Monitor();
  Config* getConfig() const;
  QString getName() const;
  QString getType() const;
  size_t getQueueSize() const;
  QString getField() const;
  ros::Duration getTimeout() const;
  ros::Duration getHorizon() const;
  void setConfig(Config* config);
  void setRegistry(utilities::MessageSubscriberRegistry* registry);

public slots:
  void setName(const QString& name);
  void setType(const QString& type);
  void setQueueSize(const size_t& queue_size);
  void setField(const QString& field);
  void setTimeout(const ros::Duration& timeout);
  void setHorizon(const ros::Duration& horizon);

protected:
  Monitor(System* system, utilities::MessageSubscriberRegistry* registry,
          Config* config);
  void addStateMonitor(const QString& id, StateMonitor* monitor);

protected slots:
  virtual void add(const QString& id) = 0;

signals:
  void changed();
  void received(const QString& id);

protected:
  virtual void update(const QString& id) = 0;
  void update(const QString& id, int state);
  System* getSystem() const;

private:
  typedef QMap<QString, StateMonitor*> StateMonitorMap;
  QString name_;
  QString type_;
  size_t queue_size_;
  QString field_;
  ros::Duration timeout_;
  ros::Duration horizon_;
  Config* config_;
  utilities::MessageSubscriberRegistry* registry_;
  utilities::MessageFieldSubscriber* subscriber_;
  StateMonitorMap monitors_;
  bool subscribe();
  bool unsubscribe();

private slots:
  void updateSubscriber();
  void subscriberReceived(variant_topic_tools::BuiltinVariant field_value);
};
}

#endif // _MRTA_MONITOR_H_
