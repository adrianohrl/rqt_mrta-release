#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_TASKS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_TASKS_H_

#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/incoming_tasks.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Tasks : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Tasks(QObject* parent = NULL);
  virtual ~Tasks();
  QString getType() const;
  IncomingTasks* getIncomingTasks() const;
  void setType(const QString& type);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Tasks& operator=(const Tasks& config);

signals:
  void changed();
  void typeChanged(const QString& type);

private:
  QString type_;
  IncomingTasks* incoming_tasks_;
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_TASKS_H_
