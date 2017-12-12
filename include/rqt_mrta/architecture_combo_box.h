#ifndef _RQT_MRTA_ARCHITECTURE_COMBO_BOX_H_
#define _RQT_MRTA_ARCHITECTURE_COMBO_BOX_H_

#include <QComboBox>
#include <QList>
#include "mrta/taxonomy.h"

namespace rqt_mrta
{
class ArchitectureComboBox : public QComboBox
{
  Q_OBJECT
public:
  ArchitectureComboBox(QWidget* parent);
  virtual ~ArchitectureComboBox();
  mrta::Architecture* getCurrentArchitecture() const;
  void setFilterAllocationType(const mrta::Taxonomy::AllocationType& allocation_type);
  void setFilterRobotType(const mrta::Taxonomy::RobotType& robot_type);
  void setFilterTaskType(const mrta::Taxonomy::TaskType& task_type);

signals:
  void changed();
  void unknownAchitecture();
  void currentArchitectureChanged(mrta::Architecture* architecture);
  void filterChanged();
  void filtered();
  void loaded();

private:
  typedef QList<mrta::Architecture*>::iterator iterator;
  typedef QList<mrta::Architecture*>::const_iterator const_iterator;
  QList<mrta::Architecture*> architectures_;
  QList<int> indexes_;
  mrta::Architecture* current_architecture_;
  mrta::Taxonomy::AllocationType allocation_type_;
  mrta::Taxonomy::RobotType robot_type_;
  mrta::Taxonomy::TaskType task_type_;
  void filter();
  void load();

private slots:
  void currentArchitectureChanged(int index);
};
}

#endif // _RQT_MRTA_ARCHITECTURE_COMBO_BOX_H_
