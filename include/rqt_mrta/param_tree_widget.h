#ifndef _RQT_MRTA_PARAMETER_TREE_WIDGET_H_
#define _RQT_MRTA_PARAMETER_TREE_WIDGET_H_

#include <QTreeWidget>

namespace rqt_mrta
{
namespace config
{
class Config;
class Param;
class Params;
}
typedef config::Param ParamConfig;
typedef config::Params ParamsConfig;
class ParamTreeWidget : public QTreeWidget
{
  Q_OBJECT
public:
  ParamTreeWidget(QWidget* parent = NULL);
  virtual ~ParamTreeWidget();
  config::Config* getConfig() const;
  void setConfig(config::Config* config);
  QString validate(QTreeWidgetItem* parent = NULL) const;

signals:
  void changed();
  void paramAdded(const QString& full_name);

private:
  config::Config* config_;
  void addParam(ParamConfig* param, QTreeWidgetItem *parent);
  void addParam(ParamsConfig *params, QTreeWidgetItem* parent);
  QTreeWidgetItem* getItem(const QString& full_name, QTreeWidgetItem* parent = NULL) const;

private slots:
  void currentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem*
    previous);
  void itemDoubleClicked(QTreeWidgetItem* item, int column);
  void configIdChanged(const QString& config_id);
  void configAdded(const QString& full_name);
  void configRemoved(const QString& full_name);
  void configCleared(const QString& full_name);
  void configNameChanged(const QString& previous_name, const QString& name);
  void configValueChanged(const QString& name, const QVariant& value);
  void configToolTipChanged(const QString& name, const QString& tool_tip);
  void paramDestroyed();

};
}

#endif // _RQT_MRTA_PARAMETER_TREE_WIDGET_H_
