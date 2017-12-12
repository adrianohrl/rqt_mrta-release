#include <ros/package.h>
#include <rospack/macros.h>
#include <rospack/rospack.h>
#include "rqt_mrta/rqt_mrta_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <rqt_mrta/rqt_mrta_widget.h>

PLUGINLIB_DECLARE_CLASS(rqt_mrta, RqtMrtaPlugin, rqt_mrta::RqtMrtaPlugin,
                        rqt_gui_cpp::Plugin)

namespace rqt_mrta
{
RqtMrtaPlugin::RqtMrtaPlugin()
    : rqt_gui_cpp::Plugin(), mrta_widget_(NULL)
{
  setObjectName("RqtMrtaPlugin");
}

RqtMrtaPlugin::~RqtMrtaPlugin()
{
  if (mrta_widget_)
  {
    delete mrta_widget_;
    mrta_widget_ = NULL;
  }
}

void RqtMrtaPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  mrta_widget_ = new RqtMrtaWidget(NULL, context);
  context.addWidget(mrta_widget_);
}
}
