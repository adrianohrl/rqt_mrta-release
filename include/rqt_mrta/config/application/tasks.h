#ifndef _RQT_MRTA_APPLICATION_CONFIG_TASKS_H_
#define _RQT_MRTA_APPLICATION_CONFIG_TASKS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/application/task.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Tasks : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Tasks(QObject* parent = NULL);
  virtual ~Tasks();
  Task* getTask(size_t index) const;
  Task* addTask();
  void removeTask(Task* task);
  void removeTask(size_t index);
  void clearTasks();
  bool contains(const QString &id) const;
  size_t count() const;
  bool isEmpty() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Tasks& operator=(const Tasks& config);
  QString validate() const;

signals:
  void added(size_t index);
  void removed(const QString& task_id);
  void cleared();
  void taskChanged(size_t index);
  void taskIdChanged(size_t task_index, const QString& task_id);

private:
  QVector<Task*> tasks_;

private slots:
  void taskChanged();
  void taskIdChanged(const QString& task_id);
  void taskDestroyed();
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_CONFIG_TASKS_H_
