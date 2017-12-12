#ifndef _RQT_MRTA_CONFIG_ARGS_H_
#define _RQT_MRTA_CONFIG_ARGS_H_

#include <QVector>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Arg;
class Args : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Args(QObject* parent = NULL);
  virtual ~Args();
  Arg* getArg(size_t index) const;
  Arg* addArg();
  void removeArg(Arg* arg);
  void removeArg(size_t index);
  void clearArgs();
  bool contains(const QString& name) const;
  size_t count() const;
  bool isEmpty() const;
  QString validate() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Args& operator=(const Args& config);
  QString toLaunch(const QString& prefix) const;

signals:
  void added(size_t index);
  void removed(const QString& id);
  void cleared();
  void argChanged(size_t index);
  void argNameChanged(size_t index, const QString& name);
  void argValueChanged(size_t index, const QString& value);
  void argDefaultValueChanged(size_t index, const QString& value);

private:
  QVector<Arg*> args_;

private slots:
  void argNameChanged(const QString& name);
  void argValueChanged(const QString& value);
  void argDefaultValueChanged(const QString& value);
  void argDestroyed();
};
}
}

#endif // _RQT_MRTA_CONFIG_ARGS_H_
