#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_INCOMING_TASKS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_INCOMING_TASKS_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/topic.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class IncomingTasks : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  IncomingTasks(QObject* parent = NULL);
  virtual ~IncomingTasks();
  Topic* getTopic() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  IncomingTasks& operator=(const IncomingTasks& config);

private:
  Topic* topic_;

private slots:
  void topicChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_INCOMING_TASKS_H_
