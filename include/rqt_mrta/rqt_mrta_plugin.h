#ifndef _RQT_MRTA_PLUGIN_H_
#define _RQT_MRTA_PLUGIN_H_

#include <QWidget>
#include <rqt_gui_cpp/plugin.h>

namespace rqt_mrta
{
class RqtMrtaWidget;

class RqtMrtaPlugin
  : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
  RqtMrtaPlugin();
  virtual ~RqtMrtaPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

private:
  QWidget* mrta_widget_;
};
}

#endif  // _RQT_MRTA_PLUGIN_H_
