#ifndef _RQT_MRTA_APPLICATION_CONFIG_TASK_H_
#define _RQT_MRTA_APPLICATION_CONFIG_TASK_H_

#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Task : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Task(QObject* parent = NULL);
  virtual ~Task();
  QString getId() const;
  void setId(const QString& id);
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Task& operator=(const Task& config);
  QString validate() const;

signals:
  void idChanged(const QString &id);

private:
  QString id_;
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_TASK_H_
