#include <QDir>
#include <QStringList>
#include "rqt_mrta/config/configs.h"
#include "rqt_mrta/config/application/robots.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
namespace config
{
Configs::Configs(QObject* parent) : AbstractConfig(parent) {}

Configs::~Configs()
{
  for (size_t index(0); index < configs_.count(); index++)
  {
    if (configs_[index])
    {
      delete configs_[index];
      configs_[index] = NULL;
    }
  }
  configs_.clear();
}

Config* Configs::getConfig(size_t index) const
{
  return index < configs_.count() ? configs_[index] : NULL;
}

Config* Configs::getConfig(const QString& id) const
{
  for (size_t index(0); index < configs_.count(); index)
  {
    if (configs_[index]->getId() == id)
    {
      return configs_[index];
    }
  }
  return NULL;
}

Config* Configs::addConfig()
{
  Config* config = new Config(this);
  configs_.append(config);
  connect(config, SIGNAL(changed()), this, SLOT(configChanged()));
  connect(config, SIGNAL(idChanged(const QString&)), this,
          SLOT(configIdChanged(const QString&)));
  connect(config, SIGNAL(added(const QString&)), this,
          SLOT(configAdded(const QString&)));
  connect(config, SIGNAL(removed(const QString&)), this,
          SLOT(configRemoved(const QString&)));
  connect(config, SIGNAL(cleared(const QString&)), this,
          SLOT(configCleared(const QString&)));
  connect(config, SIGNAL(nameChanged(const QString&, const QString&)), this,
          SLOT(configNameChanged(const QString&, const QString&)));
  connect(config, SIGNAL(typeChanged(const QString&, const QMetaType::Type&)),
          this,
          SLOT(configTypeChanged(const QString&, const QMetaType::Type&)));
  connect(config, SIGNAL(valueChanged(const QString&, const QVariant&)), this,
          SLOT(configValueChanged(const QString&, const QVariant&)));
  connect(config, SIGNAL(defaultValueChanged(const QString&, const QVariant&)),
          this,
          SLOT(configDefaultValueChanged(const QString&, const QVariant&)));
  connect(config, SIGNAL(toolTipChanged(const QString&, const QString&)), this,
          SLOT(configToolTipChanged(const QString&, const QString&)));
  connect(config, SIGNAL(destroyed()), this, SLOT(configDestroyed()));
  emit added(configs_.count() - 1);
  emit changed();
  return config;
}

void Configs::removeConfig(Config* config)
{
  removeConfig(configs_.indexOf(config));
}

void Configs::removeConfig(size_t index)
{
  if (index >= 0 && index < configs_.count())
  {
    QString config_id(configs_[index]->getId());
    configs_.remove(index);
    emit removed(config_id);
    emit changed();
  }
}

void Configs::clearConfigs()
{
  if (!configs_.isEmpty())
  {
    for (size_t i(0); i < configs_.count(); ++i)
    {
      if (configs_[i])
      {
        delete configs_[i];
        configs_[i] = NULL;
      }
    }
    configs_.clear();
    emit cleared();
    emit changed();
  }
}

bool Configs::contains(const QString& id) const
{
  for (size_t i(0); i < configs_.count(); i++)
  {
    if (configs_[i]->getId() == id)
    {
      return true;
    }
  }
  return false;
}

size_t Configs::count() const { return configs_.count(); }

bool Configs::isEmpty() const { return configs_.isEmpty(); }

void Configs::save(QSettings& settings) const
{
  settings.beginGroup("configs");
  for (size_t index(0); index < configs_.count(); ++index)
  {
    settings.beginGroup("config_" + QString::number(index));
    configs_[index]->save(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Configs::load(QSettings& settings)
{
  settings.beginGroup("configs");
  clearConfigs();
  QStringList groups(settings.childGroups());
  for (size_t index(0); index < groups.count(); index++)
  {
    Config* config = addConfig();
    settings.beginGroup("config_" + QString::number(index));
    config->load(settings);
    settings.endGroup();
  }
  settings.endGroup();
}

void Configs::reset() { clearConfigs(); }

void Configs::write(QDataStream& stream) const
{
  stream << configs_.count();
  for (size_t index(0); index < configs_.count(); ++index)
  {
    configs_[index]->write(stream);
  }
}

void Configs::read(QDataStream& stream)
{
  quint64 count;
  stream >> count;
  clearConfigs();
  for (size_t index(0); index < count; ++index)
  {
    configs_[index]->read(stream);
  }
}

Configs& Configs::operator=(const Configs& config)
{
  while (configs_.count() < config.configs_.count())
  {
    addConfig();
  }
  while (configs_.count() > config.configs_.count())
  {
    removeConfig(configs_.count() - 1);
  }
  for (size_t index(0); index < configs_.count(); ++index)
  {
    *configs_[index] = *config.configs_[index];
  }
  return *this;
}

void Configs::setConfigs(const Configs& configs,
                         const application::Robots& robots,
                         const QString& robots_config_id)
{
  clearConfigs();
  Config* robots_config = configs.getConfig(robots_config_id);
  for (size_t i(0); i < configs.count(); i++)
  {
    Config* template_config = configs.getConfig(i);
    bool is_robot_template = robots_config && template_config == robots_config;
    size_t count = is_robot_template ? robots.count() : 1;
    for (size_t j(0); j < count; j++)
    {
      Config* config = addConfig();
      *config = *template_config;
      config->hideArrays();
      if (is_robot_template)
      {
        config->setId(robots.getRobot(j)->getId() + "_" + config->getId());
      }
    }
  }
}

QString Configs::validate() const
{
  if (configs_.isEmpty())
  {
    return "Enter the system robots.";
  }
  QString validation;
  for (size_t i(0); i < configs_.count(); i++)
  {
    validation = configs_[i]->validate();
    if (!validation.isEmpty())
    {
      break;
    }
  }
  return validation;
}

QStringList Configs::willBeGenerated() const
{
  QStringList list;
  for (size_t index(0); index < configs_.count(); index++)
  {
    list.append("config/" + configs_[index]->getId() + ".yaml");
  }
  return list;
}

void Configs::saveAsYaml(const QString& package_url) const
{
  QDir package(package_url);
  if (!package.exists())
  {
    throw utilities::Exception("Inexistent package.");
  }
  if (package.cd("config"))
  {
    package.cd("..");
  }
  else if (!package.mkdir("config"))
  {
    throw utilities::Exception("Unable to create the config folder.");
  }
  for (size_t index(0); index < configs_.count(); index++)
  {
    QString validation(configs_[index]->validate());
    if (!validation.isEmpty())
    {
      ROS_ERROR("%s", validation.toStdString().c_str());
      continue;
    }
    configs_[index]->saveAsYaml(package.path() + "/config/" +
                                configs_[index]->getId());
  }
}

void Configs::configChanged() { emit changed(); }

void Configs::configIdChanged(const QString& config_id)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configIdChanged(index, config_id);
    emit changed();
  }
}

void Configs::configAdded(const QString& full_name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configAdded(index, full_name);
    emit changed();
  }
}

void Configs::configRemoved(const QString& full_name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configRemoved(index, full_name);
    emit changed();
  }
}

void Configs::configCleared(const QString& full_name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configCleared(index, full_name);
    emit changed();
  }
}

void Configs::configNameChanged(const QString& previous_name,
                                const QString& name)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configNameChanged(index, previous_name, name);
    emit changed();
  }
}

void Configs::configTypeChanged(const QString& name,
                                const QMetaType::Type& type)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configTypeChanged(index, name, type);
    emit changed();
  }
}

void Configs::configValueChanged(const QString& name, const QVariant& value)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configValueChanged(index, name, value);
    emit changed();
  }
}

void Configs::configDefaultValueChanged(const QString& name,
                                        const QVariant& default_value)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configDefaultValueChanged(index, name, default_value);
    emit changed();
  }
}

void Configs::configToolTipChanged(const QString& name, const QString& tool_tip)
{
  int index(configs_.indexOf(static_cast<Config*>(sender())));
  if (index != -1)
  {
    emit configToolTipChanged(index, name, tool_tip);
    emit changed();
  }
}

void Configs::configDestroyed()
{
  Config* config = static_cast<Config*>(sender());
  int index(configs_.indexOf(config));
  if (index != -1)
  {
    QString config_id(config->getId());
    configs_.remove(index);
    emit removed(config_id);
    emit changed();
  }
}
}
}
