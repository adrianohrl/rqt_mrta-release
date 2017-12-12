#include "rqt_mrta/config/param.h"
#include "rqt_mrta/config/params.h"
#include "rqt_mrta/param_item.h"
#include "rqt_mrta/param_item_model.h"
#include "utilities/exception.h"

Q_DECLARE_METATYPE(rqt_mrta::config::Param*)
Q_DECLARE_METATYPE(rqt_mrta::config::Params*)

namespace rqt_mrta
{

ParamItemModel::ParamItemModel(QObject* parent)
    : QAbstractItemModel(parent), root_(NULL)
{
}

ParamItemModel::~ParamItemModel()
{
  if (root_)
  {
    delete root_;
    root_ = NULL;
  }
}

void ParamItemModel::setConfig(config::Config* config) {}

int ParamItemModel::rowCount(const QModelIndex& parent) const
{
  if (parent.column() <= 0)
  {
    ParamItem* item = !parent.isValid()
                          ? root_
                          : static_cast<ParamItem*>(parent.internalPointer());
    if (item)
    {
      return item->getNumChildren();
    }
  }
  return 0;
}

int ParamItemModel::columnCount(const QModelIndex& parent) const
{
  if (parent.isValid())
  {
    ParamItem* item = static_cast<ParamItem*>(parent.internalPointer());
    if (item)
    {
      return item->getNumColumns();
    }
  }
  else if (root_)
  {
    return root_->getNumColumns();
  }
  return 0;
}

QVariant ParamItemModel::data(const QModelIndex& index, int role) const
{
  if (index.isValid())
  {
    if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
      ParamItem* item = static_cast<ParamItem*>(index.internalPointer());
      if (item)
      {
        if (item->isLeaf())
        {
          config::Param* param = static_cast<config::Param*>(item->getParam());
          return QVariant::fromValue<config::Param*>(param);
        }
        config::Params* params = static_cast<config::Params*>(item->getParam());
        return QVariant::fromValue<config::Params*>(params);
      }
    }
  }
  return QVariant();
}

QModelIndex ParamItemModel::index(int row, int column,
                                  const QModelIndex& parent) const
{
  if (hasIndex(row, column, parent))
  {
    ParamItem* item = !parent.isValid() ? root_ : static_cast<ParamItem*>(parent.internalPointer());
    if (item)
    {
      item = item->getChild(row);
      if (item)
      {
        return createIndex(row, column, item);
      }
    }
  }
  return QModelIndex();
}

QModelIndex ParamItemModel::parent(const QModelIndex& index) const
{
  if (index.isValid())
  {
    ParamItem* item = static_cast<ParamItem*>(index.internalPointer());
    if (item)
    {
      item = item->getParent();
      if (item != root_)
      {
        return createIndex(item->getRow(), 0, item);
      }
    }
  }
  return QModelIndex();
}
}
