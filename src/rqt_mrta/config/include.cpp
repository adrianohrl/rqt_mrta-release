#include "rqt_mrta/config/include.h"
#include "rqt_mrta/config/args.h"

namespace rqt_mrta
{
namespace config
{
Include::Include(QObject* parent)
    : AbstractConfig(parent), args_(new Args(this))
{
  connect(args_, SIGNAL(changed()), this, SIGNAL(changed()));
}

Include::~Include()
{
  if (args_)
  {
    delete args_;
    args_ = NULL;
  }
}

QString Include::getFile() const { return file_; }

Args* Include::getArgs() const { return args_; }

void Include::setFile(const QString& file)
{
  if (file != file_)
  {
    file_ = file;
    emit fileChanged(file);
    emit changed();
  }
}

QString Include::validate() const
{
  if (file_.isEmpty())
  {
    return "Enter the file location to be included.";
  }
  return args_->validate();
}

void Include::save(QSettings& settings) const
{
  settings.setValue("file", file_);
  args_->save(settings);
}

void Include::load(QSettings& settings)
{
  setFile(settings.value("file").toString());
  args_->load(settings);
}

void Include::reset()
{
  setFile("");
  args_->reset();
}

void Include::write(QDataStream& stream) const
{
  stream << file_;
  args_->write(stream);
}

void Include::read(QDataStream& stream)
{
  QString file;
  quint64 count;
  QString config;
  stream >> file;
  setFile(file);
  stream >> count;
  args_->read(stream);
}

Include& Include::operator=(const Include& config)
{
  setFile(config.file_);
  *args_ = *config.args_;
  return *this;
}

QString Include::toLaunch(const QString &prefix) const
{
  QString launch;
  launch += prefix + "<include file=\"" + file_ + "\">\n";
  launch += args_->toLaunch(prefix);
  launch += prefix + "</include>\n";
  return launch;
}
}
}
