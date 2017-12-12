#ifndef _MRTA_TASK_H_
#define _MRTA_TASK_H_

#include <QObject>

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Task;
}
}
}

namespace mrta
{
class Task : public QObject
{
  Q_OBJECT
public:
  typedef rqt_mrta::config::application::Task Config;
  Task(QObject *parent = NULL);
  Task(QObject *parent, Config* config);
  Task(const Task& task);
  virtual ~Task();
  Config *getConfig() const;
  void setConfig(Config* config);
  QString getId() const;
  Task& operator=(const Task& task);

public slots:
  void setId(const QString& id);

signals:
  void changed();
  void idChanged(const QString& id);

private:
  QString id_;
  Config* config_;

private slots:
  void configDestroyed();
};
}

#endif // _MRTA_TASK_H_
