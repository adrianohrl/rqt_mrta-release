#include <QFileInfo>
#include "rqt_mrta/config/launch.h"
#include "rqt_mrta/config/includes.h"
#include "utilities/exception.h"

namespace rqt_mrta
{
namespace config
{
Launch::Launch(QObject* parent)
    : AbstractConfig(parent), includes_(new Includes(this))
{
  reset();
  connect(includes_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Launch::~Launch()
{
  if (includes_)
  {
    delete includes_;
    includes_ = NULL;
  }
}

QString Launch::getId() const { return id_; }

Includes* Launch::getIncludes() const { return includes_; }

QMap<QString, QString> Launch::getMap() const { return map_; }

void Launch::setId(const QString& id)
{
  if (id != id_)
  {
    id_ = id;
    emit idChanged(id);
    emit changed();
  }
}

void Launch::add(const QString& key, const QString& value)
{
  map_[(!key.startsWith('@') ? "@" : "") + key + (!key.endsWith('@') ? "@" : "")] = value;
}

QString Launch::validate() const
{
  if (id_.contains(' '))
  {
    return "The launch file id must not have <space>.";
  }
  return includes_->validate();
}

void Launch::save(QSettings& settings) const
{
  settings.setValue("id", id_);
  includes_->save(settings);
}

void Launch::load(QSettings& settings)
{
  setId(settings.value("id").toString());
  includes_->load(settings);
}

void Launch::reset()
{
  setId("");
  includes_->reset();
}

void Launch::write(QDataStream& stream) const
{
  stream << id_;
  includes_->write(stream);
}

void Launch::read(QDataStream& stream)
{
  QString id;
  stream >> id;
  setId(id);
  includes_->read(stream);
}

Launch& Launch::operator=(const Launch& config)
{
  setId(config.id_);
  *includes_ = *config.includes_;
  return *this;
}

void Launch::saveAsLaunch(const QString& url) const
{
  QFile file(url + ".launch");
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR_STREAM("Unable to open the " << url.toStdString()
                                           << ".launch file.");
    return;
  }
  file.write(toLaunch().toStdString().c_str());
  file.close();
  ROS_INFO_STREAM("Created the " << url.toStdString() << ".launch file.");
}

QString Launch::toLaunch() const
{
  QString launch;
  launch += "<?xml version=\"1.0\"?>\n";
  launch += "<launch>\n";
  launch += includes_->toLaunch();
  launch += "</launch>\n";
  for (const_iterator it(map_.constBegin()); it != map_.constEnd(); it++)
  {
    launch.replace(it.key(), it.value());
  }
  return launch;
}
}
}
