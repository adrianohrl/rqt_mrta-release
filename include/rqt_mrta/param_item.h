#ifndef _RQT_MRTA_PARAM_ITEM_H_
#define _RQT_MRTA_PARAM_ITEM_H_

#include <QList>
#include <QString>

namespace rqt_mrta
{
namespace config
{
class ParamInterface;
class Param;
class Params;
}
class ParamItem
{
public:
  ParamItem(config::Param* param, ParamItem* parent = NULL);
  ParamItem(config::Params* params, ParamItem* parent = NULL);
  ~ParamItem();
  config::ParamInterface* getParam() const;
  ParamItem* getParent() const;
  size_t getNumChildren() const;
  ParamItem* getChild(size_t row) const;
  ParamItem* getChild(const QString& full_name) const;
  bool isLeaf() const;
  int getRow() const;
  size_t getNumColumns() const;
  QString getFullName() const;
  void appendChild(ParamItem* child);

private:
  ParamItem* parent_;
  QList<ParamItem*> children_;
  config::ParamInterface* param_;
};
}

#endif // _RQT_MRTA_PARAM_ITEM_H_
