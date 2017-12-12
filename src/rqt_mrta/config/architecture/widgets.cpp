#include <QStringList>
#include "rqt_mrta/config/architecture/widgets.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
Widgets::Widgets(QObject* parent) : AbstractConfig(parent) {}

Widgets::~Widgets()
{
  for (size_t index(0); index < widgets_.count(); index++)
  {
    if (widgets_[index])
    {
      delete widgets_[index];
      widgets_[index] = NULL;
    }
  }
  widgets_.clear();
}

size_t Widgets::count() const { return widgets_.count(); }

Widget* Widgets::getWidget(size_t index) const
{
  return index < widgets_.count() ? widgets_[index] : NULL;
}

Widget* Widgets::addWidget()
{
  Widget* widget = new Widget(this);
  widgets_.append(widget);
  connect(widget, SIGNAL(changed()), this, SLOT(widgetChanged()));
  connect(widget, SIGNAL(pluginNameChanged(const QString&)), this,
          SLOT(widgetPluginNameChanged(const QString&)));
  connect(widget, SIGNAL(destroyed()), this, SLOT(widgetDestroyed()));
  emit added(widgets_.count() - 1);
  emit changed();
  return widget;
}

void Widgets::clearWidgets()
{
  if (!widgets_.isEmpty())
  {
    for (size_t i(0); i < widgets_.count(); ++i)
    {
      if (widgets_[i])
      {
        delete widgets_[i];
        widgets_[i] = NULL;
      }
    }
    widgets_.clear();
    emit cleared();
    emit changed();
  }
}

void Widgets::save(QSettings& settings) const
{
  settings.beginGroup("widgets");
  for (size_t index(0); index < widgets_.count(); ++index)
  {
    settings.beginGroup("widget_" + QString::number(index));
    widgets_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Widgets::load(QSettings& settings)
{
  settings.beginGroup("widgets");
  QStringList groups(settings.childGroups());
  size_t index(0);
  clearWidgets();
  for (QStringList::iterator it = groups.begin(); it != groups.end(); ++it)
  {
    Widget* widget = addWidget();
    settings.beginGroup(*it);
    widget->load(settings);
    settings.endGroup();
    ++index;
  }
  settings.endGroup();
}

void Widgets::reset() { clearWidgets(); }

void Widgets::write(QDataStream& stream) const
{
  for (size_t index(0); index < widgets_.count(); ++index)
  {
    widgets_[index]->write(stream);
  }
}

void Widgets::read(QDataStream& stream)
{
  for (size_t index(0); index < widgets_.count(); ++index)
  {
    widgets_[index]->read(stream);
  }
}

Widgets& Widgets::operator=(const Widgets& config)
{
  while (widgets_.count() < config.widgets_.count())
  {
    addWidget();
  }
  while (widgets_.count() > config.widgets_.count())
  {
    removeWidget(widgets_.count() - 1);
  }
  for (size_t index(0); index < widgets_.count(); ++index)
  {
    *widgets_[index] = *config.widgets_[index];
  }
  return *this;
}

void Widgets::widgetChanged()
{
  for (size_t index(0); index < widgets_.count(); ++index)
  {
    if (widgets_[index] == sender())
    {
      emit widgetChanged(index);
      break;
    }
  }
  emit changed();
}

void Widgets::widgetPluginNameChanged(const QString& plugin_name)
{
  Widget* widget = static_cast<Widget*>(sender());
  int index(widgets_.indexOf(widget));
  if (index != -1)
  {
    emit widgetPluginNameChanged(index, plugin_name);
    emit changed();
  }
}

void Widgets::widgetDestroyed()
{
  int index(widgets_.indexOf(static_cast<Widget*>(sender())));
  if (index >= 0)
  {
    widgets_.remove(index);
    emit removed(index);
    emit changed();
  }
}
}
}
}
