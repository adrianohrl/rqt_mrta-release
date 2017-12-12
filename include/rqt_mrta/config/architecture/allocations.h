#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_ALLOCATIONS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_ALLOCATIONS_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/allocated_tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Allocations : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Allocations(QObject* parent = NULL);
  virtual ~Allocations();
  QString getType() const;
  AllocatedTasks* getAllocatedTasks() const;
  void setType(const QString& type);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Allocations& operator=(const Allocations& config);

signals:
  void changed();
  void typeChanged(const QString& type);

private:
  QString type_;
  AllocatedTasks* allocated_tasks_;
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_ALLOCATIONS_H_
