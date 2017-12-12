#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGETS_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGETS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/architecture/widget.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class Widgets : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Widgets(QObject* parent = NULL);
  virtual ~Widgets();
  size_t count() const;
  Widget* getWidget(size_t index) const;
  Widget* addWidget();
  void removeWidget(Widget* task);
  void removeWidget(size_t index);
  void clearWidgets();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Widgets& operator=(const Widgets& config);

signals:
  void added(size_t index);
  void removed(size_t index);
  void cleared();
  void widgetChanged(size_t index);
  void widgetPluginNameChanged(size_t index, const QString& plugin_name);

private:
  QVector<Widget*> widgets_;

private slots:
  void widgetChanged();
  void widgetPluginNameChanged(const QString& plugin_name);
  void widgetDestroyed();
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_WIDGETS_H_
