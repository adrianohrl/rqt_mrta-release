#include "mrta/architecture.h"
#include <rospack/rospack.h>
#include "rqt_mrta/architecture_combo_box.h"

namespace rqt_mrta
{
ArchitectureComboBox::ArchitectureComboBox(QWidget* parent)
    : QComboBox(parent), current_architecture_(NULL),
      allocation_type_(mrta::Taxonomy::UnknownAllocationType),
      robot_type_(mrta::Taxonomy::UnknownRobotType),
      task_type_(mrta::Taxonomy::UnknownTaskType)
{
  load();
  filter();
  connect(this, SIGNAL(currentIndexChanged(int)), this,
          SLOT(currentArchitectureChanged(int)));
}

ArchitectureComboBox::~ArchitectureComboBox()
{
  current_architecture_ = NULL;
}

mrta::Architecture* ArchitectureComboBox::getCurrentArchitecture() const
{
  return current_architecture_;
}

void ArchitectureComboBox::setFilterAllocationType(
    const mrta::Taxonomy::AllocationType& allocation_type)
{
  if (allocation_type != allocation_type_)
  {
    allocation_type_ = allocation_type;
    filter();
    emit filterChanged();
    emit changed();
  }
}

void ArchitectureComboBox::setFilterRobotType(
    const mrta::Taxonomy::RobotType& robot_type)
{
  if (robot_type != robot_type_)
  {
    robot_type_ = robot_type;
    filter();
    emit filterChanged();
    emit changed();
  }
}

void ArchitectureComboBox::setFilterTaskType(
    const mrta::Taxonomy::TaskType& task_type)
{
  if (task_type != task_type_)
  {
    task_type_ = task_type;
    filter();
    emit filterChanged();
    emit changed();
  }
}

void ArchitectureComboBox::load()
{
  std::vector<std::string> architectures;
  rospack::Rospack rp;
  rp.setQuiet(true);
  std::vector<std::string> search_path;
  rp.getSearchPathFromEnv(search_path);
  rp.crawl(search_path, true);
  indexes_.clear();
  indexes_.append(-1);
  architectures_.clear();
  if (rp.plugins("rqt_mrta", "architecture", "", architectures))
  {
    for (size_t i(0); i < architectures.size(); i++)
    {
      size_t index(architectures[i].find(' '));
      QString package(
          QString::fromStdString(architectures[i].substr(0, index)));
      QString architecture_config_path(
          QString::fromStdString(architectures[i].substr(index + 1)));
      indexes_.append(i);
      architectures_.append(
          new mrta::Architecture(NULL, package, architecture_config_path));
    }
  }
}

void ArchitectureComboBox::filter()
{
  setEnabled(false);
  blockSignals(true);
  clear();
  addItem("");
  int counter(1), index(-1);
  for (size_t i(0); i < architectures_.count(); i++)
  {
    indexes_[counter] = -1;
    if (architectures_[i]->belongs(allocation_type_, robot_type_, task_type_))
    {
      indexes_[counter] = i;
      addItem(architectures_[i]->getPackage());
      if (current_architecture_ && *architectures_[i] == *current_architecture_)
      {
        index = counter;
      }
      counter++;
    }
  }
  if (count() > 1)
  {
    setEnabled(true);
  }
  blockSignals(false);
  if (index != -1)
  {
    setCurrentIndex(index);
  }
  else
  {
    current_architecture_ = NULL;
    emit unknownAchitecture();
  }
  emit filtered();
  emit changed();
}

void ArchitectureComboBox::currentArchitectureChanged(int index)
{
  current_architecture_ = index != -1 && indexes_[index] != -1
                              ? architectures_[indexes_[index]]
                              : NULL;
  if (!current_architecture_)
  {
    emit unknownAchitecture();
  }
  emit currentArchitectureChanged(current_architecture_);
  emit changed();
}
}
