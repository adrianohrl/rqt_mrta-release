#ifndef _RQT_MRTA_CONFIG_CONFIGS_H_
#define _RQT_MRTA_CONFIG_CONFIGS_H_

#include <QVector>
#include "utilities/abstract_config.h"
#include "rqt_mrta/config/config.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class Robots;
}
class Configs : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Configs(QObject* parent = NULL);
  virtual ~Configs();
  Config* getConfig(size_t index) const;
  Config* getConfig(const QString& id) const;
  Config* addConfig();
  void removeConfig(Config* config);
  void removeConfig(size_t index);
  void clearConfigs();
  bool contains(const QString& id) const;
  size_t count() const;
  bool isEmpty() const;
  QString validate() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Configs& operator=(const Configs& config);
  void setConfigs(const Configs& configs, const application::Robots& robots,
                  const QString& robots_config_id);
  QStringList willBeGenerated() const;
  void saveAsYaml(const QString& package_url) const;

signals:
  void added(size_t index);
  void removed(const QString& config_id);
  void cleared();
  void configChanged(size_t index);
  void configIdChanged(size_t index, const QString& config_id);
  void configAdded(size_t index, const QString& full_name);
  void configRemoved(size_t index, const QString& full_name);
  void configCleared(size_t index, const QString& full_name);
  void configNameChanged(size_t index, const QString& previous_name,
                         const QString& name);
  void configTypeChanged(size_t index, const QString& name,
                         const QMetaType::Type& type);
  void configValueChanged(size_t index, const QString& name,
                          const QVariant& value);
  void configDefaultValueChanged(size_t index, const QString& name,
                                 const QVariant& default_value);
  void configToolTipChanged(size_t index, const QString& name,
                            const QString& tool_tip);

private:
  QVector<Config*> configs_;

private slots:
  void configChanged();
  void configIdChanged(const QString& config_id);
  void configAdded(const QString& full_name);
  void configRemoved(const QString& full_name);
  void configCleared(const QString& full_name);
  void configNameChanged(const QString& previous_name, const QString& name);
  void configTypeChanged(const QString& name, const QMetaType::Type& type);
  void configValueChanged(const QString& name, const QVariant& value);
  void configDefaultValueChanged(const QString& name,
                                 const QVariant& default_value);
  void configToolTipChanged(const QString& name, const QString& tool_tip);
  void configDestroyed();
};
}
}

#endif // _RQT_MRTA_CONFIG_CONFIGS_H_
