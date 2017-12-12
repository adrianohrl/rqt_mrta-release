#ifndef _UTILITIES_ABSTRACT_CONFIG_H_
#define _UTILITIES_ABSTRACT_CONFIG_H_

#include <QDataStream>
#include <QObject>
#include <QSettings>
#include <ros/console.h>

namespace utilities
{
class AbstractConfig : public QObject
{
  Q_OBJECT
public:
  AbstractConfig(QObject* parent = NULL);
  virtual ~AbstractConfig();
  virtual void save(QSettings& settings) const = 0;
  virtual void load(QSettings& settings) = 0;
  virtual void reset() = 0;
  virtual void write(QDataStream& stream) const = 0;
  virtual void read(QDataStream& stream) = 0;

signals:
  void changed();
};

QDataStream& operator<<(QDataStream& stream, const AbstractConfig& config);
QDataStream& operator>>(QDataStream& stream, AbstractConfig& config);
}

#endif // _UTILITIES_ABSTRACT_CONFIG_H_
