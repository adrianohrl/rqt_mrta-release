#include "rqt_mrta/config/architecture/widget.h"

#include <ros/console.h>

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Widget::Widget(QObject* parent) : AbstractConfig(parent) {}

Widget::~Widget() {}

QString Widget::getPluginName() const { return plugin_name_; }

void Widget::setPluginName(const QString& plugin_name)
{
  if (plugin_name != plugin_name_)
  {
    plugin_name_ = plugin_name;
    emit pluginNameChanged(plugin_name);
    emit changed();
  }
}

void Widget::save(QSettings& settings) const
{
  settings.setValue("plugin_name", plugin_name_);
}

void Widget::load(QSettings& settings)
{
  setPluginName(settings.value("plugin_name").toString());
}

void Widget::reset() { setPluginName(""); }

void Widget::write(QDataStream& stream) const { stream << plugin_name_; }

void Widget::read(QDataStream& stream)
{
  QString plugin_name;
  stream >> plugin_name;
  setPluginName(plugin_name);
}

Widget& Widget::operator=(const Widget& config)
{
  setPluginName(config.plugin_name_);
  return *this;
}
}
}
}
