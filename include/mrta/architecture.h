#ifndef _MRTA_ARCHITECTURE_H_
#define _MRTA_ARCHITECTURE_H_

#include <QObject>
#include "mrta/taxonomy.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class RqtMrtaArchitecture;
}
}
}

namespace mrta
{
typedef rqt_mrta::config::architecture::RqtMrtaArchitecture Config;
class Architecture : public QObject
{
  Q_OBJECT
public:
  Architecture(QObject* parent, Config* config);
  Architecture(QObject* parent, const QString& package,
               const QString& config_file_path);
  virtual ~Architecture();
  QString getPackage() const;
  QString getConfigFilePath() const;
  QString getName() const;
  Taxonomy::AllocationType getAllocationType() const;
  Taxonomy::RobotType getRobotType() const;
  Taxonomy::TaskType getTaskType() const;
  void setName(const QString& name);
  void setAllocationType(const Taxonomy::AllocationType& type);
  void setRobotType(const Taxonomy::RobotType& type);
  void setTaskType(const Taxonomy::TaskType& type);
  bool belongs(const Taxonomy::AllocationType& allocation_type,
               const Taxonomy::RobotType& robot_type,
               const Taxonomy::TaskType& task_type) const;
  QString toString() const;
  bool operator==(const QString& package) const;
  bool operator==(const Architecture& architecture) const;
  bool isValid(Taxonomy::AllocationType type) const;
  bool isValid(Taxonomy::RobotType type) const;
  bool isValid(Taxonomy::TaskType type) const;

private:
  QString package_;
  QString config_file_path_;
  QString name_;
  Taxonomy::AllocationType allocation_type_;
  Taxonomy::RobotType robot_type_;
  Taxonomy::TaskType task_type_;
};
}

#endif // _MRTA_ARCHITECTURE_H_
