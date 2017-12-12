#include "rqt_mrta/config/config.h"
#include "rqt_mrta/config/param.h"
#include "rqt_mrta/config/params.h"
#include "rqt_mrta/config/param_interface.h"
#include "rqt_mrta/param_tree_widget.h"

Q_DECLARE_METATYPE(rqt_mrta::config::ParamInterface*)

namespace rqt_mrta
{
ParamTreeWidget::ParamTreeWidget(QWidget* parent)
    : QTreeWidget(parent), config_(NULL)
{
  setColumnCount(2);
  headerItem()->setText(0, "Name");
  headerItem()->setText(1, "Value");
  connect(this, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
          this, SLOT(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)));
  connect(this, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)), this,
          SLOT(itemDoubleClicked(QTreeWidgetItem*, int)));
}

ParamTreeWidget::~ParamTreeWidget() {}

config::Config* ParamTreeWidget::getConfig() const { return config_; }

void ParamTreeWidget::setConfig(config::Config* config)
{
  if (config != config_)
  {
    if (config_)
    {
      disconnect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      disconnect(config_, SIGNAL(idChanged(const QString&)), this,
                 SLOT(configIdChanged(const QString&)));
      disconnect(config_, SIGNAL(added(const QString&)), this,
                 SLOT(configAdded(const QString&)));
      disconnect(config_, SIGNAL(removed(const QString&)), this,
                 SLOT(configRemoved(const QString&)));
      disconnect(config_, SIGNAL(cleared(const QString&)), this,
                 SLOT(configCleared(const QString&)));
      disconnect(config_, SIGNAL(nameChanged(const QString&, const QString&)),
                 this, SLOT(configNameChanged(const QString&, const QString&)));
      disconnect(config_, SIGNAL(valueChanged(const QString&, const QVariant&)),
                 this,
                 SLOT(configValueChanged(const QString&, const QVariant&)));
      disconnect(config_,
                 SIGNAL(toolTipChanged(const QString&, const QString&)), this,
                 SLOT(configToolTipChanged(const QString&, const QString&)));
      // disconnect(config_, SIGNAL(destroyed()), this,
      // SLOT(configDestroyed()));
    }
    config_ = config;
    if (config_)
    {
      connect(config_, SIGNAL(changed()), this, SIGNAL(changed()));
      connect(config_, SIGNAL(idChanged(const QString&)), this,
              SLOT(configIdChanged(const QString&)));
      connect(config_, SIGNAL(added(const QString&)), this,
              SLOT(configAdded(const QString&)));
      connect(config_, SIGNAL(removed(const QString&)), this,
              SLOT(configRemoved(const QString&)));
      connect(config_, SIGNAL(cleared(const QString&)), this,
              SLOT(configCleared(const QString&)));
      connect(config_, SIGNAL(nameChanged(const QString&, const QString&)),
              this, SLOT(configNameChanged(const QString&, const QString&)));
      connect(config_, SIGNAL(valueChanged(const QString&, const QVariant&)),
              this, SLOT(configValueChanged(const QString&, const QVariant&)));
      connect(config_, SIGNAL(toolTipChanged(const QString&, const QString&)),
              this, SLOT(configToolTipChanged(const QString&, const QString&)));
      // connect(config_, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
      for (size_t index(0); index < config_->count(); index++)
      {
        config::ParamInterface* param = config_->getChild(index);
        connect(param, SIGNAL(destroyed()), this, SLOT(paramDestroyed()));
        if (param->isParam())
        {
          addParam(static_cast<ParamConfig*>(param), invisibleRootItem());
        }
        else
        {
          addParam(static_cast<ParamsConfig*>(param), invisibleRootItem());
        }
      }
      setCurrentItem(invisibleRootItem());
    }
  }
}

QString ParamTreeWidget::validate(QTreeWidgetItem* parent) const
{
  if (!parent)
  {
    parent = invisibleRootItem();
  }
  QString validation;
  for (size_t index(0); index < parent->childCount(); index++)
  {
    QTreeWidgetItem* item = parent->child(index);
    config::ParamInterface* param =
        item->data(0, Qt::UserRole).value<config::ParamInterface*>();
    validation = param->validate();
    if (!validation.isEmpty())
    {
      return validation;
    }
  }
  return validation;
}

void ParamTreeWidget::addParam(ParamConfig* param, QTreeWidgetItem* parent)
{
  QTreeWidgetItem* item = new QTreeWidgetItem(parent);
  item->setText(0, param->getName() + (param->isMandatory() ? " *" : ""));
  item->setData(0, Qt::UserRole,
                QVariant::fromValue<config::ParamInterface*>(param));
  item->setText(1, param->getDefaultValue().toString());
  item->setToolTip(0, param->getToolTip());
  item->setToolTip(1, param->getToolTip());
  parent->addChild(item);
  emit paramAdded(param->getFullName());
  setCurrentItem(item);
}

void ParamTreeWidget::addParam(ParamsConfig* params, QTreeWidgetItem* parent)
{
  QTreeWidgetItem* item = new QTreeWidgetItem(parent);
  item->setExpanded(true);
  item->setText(0, params->getName());
  item->setData(0, Qt::UserRole,
                QVariant::fromValue<config::ParamInterface*>(params));
  parent->addChild(item);
  emit paramAdded(params->getFullName());
  for (size_t index(0); index < params->count(); index++)
  {
    config::ParamInterface* param = params->getChild(index);
    if (param->isParam())
    {
      addParam(static_cast<ParamConfig*>(param), item);
    }
    else
    {
      addParam(static_cast<ParamsConfig*>(param), item);
    }
  }
  setCurrentItem(item);
}

QTreeWidgetItem* ParamTreeWidget::getItem(const QString& full_name,
                                          QTreeWidgetItem* parent) const
{
  if (!parent)
  {
    parent = invisibleRootItem();
  }
  QStringList names(full_name.split("/"));
  QString name(names.first());
  names.removeFirst();
  for (size_t index(0); index < parent->childCount(); index++)
  {
    QTreeWidgetItem* item = parent->child(index);
    if (item->text(0) == name)
    {
      return names.isEmpty() ? item : getItem(names.join("/"), item);
    }
  }
  return NULL;
}

void ParamTreeWidget::currentItemChanged(QTreeWidgetItem* current,
                                         QTreeWidgetItem* previous)
{
  if (previous && current && previous != current && previous->childCount() == 0)
  {
    config::ParamInterface* param =
        previous->data(0, Qt::UserRole).value<config::ParamInterface*>();
    if (param->isParam())
    {
      ParamConfig* p = static_cast<ParamConfig*>(param);
      p->setValue(previous->text(1));
    }
  }
}

void ParamTreeWidget::itemDoubleClicked(QTreeWidgetItem* item, int column)
{
  if (column == 0)
  {
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
  }
  if (column == 1 && item->childCount() == 0)
  {
    item->setFlags(item->flags() | Qt::ItemIsEditable);
  }
}

void ParamTreeWidget::configIdChanged(const QString& config_id) {}

void ParamTreeWidget::configAdded(const QString& full_name)
{
  config::ParamInterface* param = config_->getParam(full_name);
  QStringList parent_name(full_name.split("/"));
  parent_name.removeLast();
  QTreeWidgetItem* parent = getItem(parent_name.join("/"));
  if (param->isParam())
  {
    addParam(static_cast<ParamConfig*>(param), parent);
  }
  else
  {
    addParam(static_cast<ParamsConfig*>(param), parent);
  }
}

void ParamTreeWidget::configRemoved(const QString& full_name)
{
  QTreeWidgetItem* item = getItem(full_name);
  if (item)
  {
    delete item;
  }
}

void ParamTreeWidget::configCleared(const QString& full_name) {}

void ParamTreeWidget::configNameChanged(const QString& previous_name,
                                        const QString& name)
{
  QTreeWidgetItem* item = getItem(previous_name);
  if (item)
  {
    item->setText(0, name);
  }
}

void ParamTreeWidget::configValueChanged(const QString& name,
                                         const QVariant& value)
{
  QTreeWidgetItem* item = getItem(name);
  if (item)
  {
    item->setText(1, value.toString());
  }
}

void ParamTreeWidget::configToolTipChanged(const QString& name,
                                           const QString& tool_tip)
{
  QTreeWidgetItem* item = getItem(name);
  if (item)
  {
    item->setToolTip(0, tool_tip);
    item->setToolTip(1, tool_tip);
  }
}

void ParamTreeWidget::paramDestroyed()
{
  ROS_INFO_STREAM("[ParamTreeWidget::paramDestroyed]");
}
}
