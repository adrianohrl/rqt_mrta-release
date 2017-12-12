#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_IDLE_ROBOTS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_IDLE_ROBOTS_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/topic.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class IdleRobots : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  IdleRobots(QObject* parent = NULL);
  virtual ~IdleRobots();
  Topic* getTopic() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  IdleRobots& operator=(const IdleRobots& config);

private:
  Topic* topic_;

private slots:
  void topicChanged();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_IDLE_ROBOTS_H_
