#include "rqt_mrta/config/param.h"
#include "rqt_mrta/config/params.h"
#include "rqt_mrta/param_item.h"

namespace rqt_mrta
{

ParamItem::ParamItem(config::Param* param, ParamItem* parent)
    : parent_(parent), param_(param)
{
}

ParamItem::ParamItem(config::Params* params, ParamItem* parent)
    : parent_(parent), param_(params)
{
  for (size_t index(0); index < params->count(); index++)
  {
    config::ParamInterface* param = params->getChild(index);
    ParamItem* item =
        param->isEmpty()
            ? new ParamItem(static_cast<config::Param*>(param), this)
            : new ParamItem(static_cast<config::Params*>(param), this);
    appendChild(item);
  }
}

ParamItem::~ParamItem()
{
  param_ = NULL;
  parent_ = NULL;
  for (QList<ParamItem*>::iterator it(children_.begin()); it != children_.end();
       ++it)
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
  }
  children_.clear();
}

config::ParamInterface* ParamItem::getParam() const { return param_; }

ParamItem* ParamItem::getParent() const { return parent_; }

size_t ParamItem::getNumChildren() const { return children_.count(); }

ParamItem* ParamItem::getChild(size_t row) const
{
  return children_.value(row);
}

ParamItem* ParamItem::getChild(const QString& full_name) const
{
  for (QList<ParamItem*>::const_iterator it(children_.begin());
       it != children_.end(); ++it)
  {
    ParamItem* item = *it;
    if (item->getFullName() == full_name)
    {
      return item;
    }
  }
  return NULL;
}

bool ParamItem::isLeaf() const { return children_.isEmpty(); }

int ParamItem::getRow() const
{
  return parent_ ? parent_->children_.indexOf(const_cast<ParamItem*>(this))
                 : -1;
}

size_t ParamItem::getNumColumns() const { return 2; }

QString ParamItem::getFullName() const { return param_->getFullName(); }

void ParamItem::appendChild(ParamItem* child) { children_.append(child); }
}
