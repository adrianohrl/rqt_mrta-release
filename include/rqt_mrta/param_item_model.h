#ifndef _RQT_MRTA_PARAM_ITEM_MODEL_H_
#define _RQT_MRTA_PARAM_ITEM_MODEL_H_

#include <QAbstractItemModel>

namespace rqt_mrta
{
namespace config
{
class Config;
class ParamInterface;
}
class ParamItem;
class ParamItemModel : public QAbstractItemModel
{
  Q_OBJECT
public:
  ParamItemModel(QObject* parent = NULL);
  virtual ~ParamItemModel();
  void setConfig(config::Config* config);
  int rowCount(const QModelIndex& parent) const;
  int columnCount(const QModelIndex& parent) const;
  QVariant data(const QModelIndex& index, int role) const;
  QModelIndex index(int row, int column, const QModelIndex& parent) const;
  QModelIndex parent(const QModelIndex& index) const;

private:
  ParamItem* root_;
};
}

#endif // _RQT_MRTA_PARAM_ITEM_MODEL_H_
