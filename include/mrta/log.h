#ifndef _MRTA_LOG_H_
#define _MRTA_LOG_H_

#include <QObject>
#include <ros/time.h>

namespace mrta
{
class History;
class Log : public QObject
{
  Q_OBJECT
public:
  enum Type
  {
    Robot,
    Task,
    Allocation
  };
  enum Severity
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };
  Log(History* history, const ros::Time& timestamp, Type type,
      Severity severity, const QString& id, int state);
  Log(const Log& log);
  virtual ~Log();
  ros::Time getTimestamp() const;
  Type getType() const;
  Severity getSeverity() const;
  QString getId() const;
  int getState() const;
  QString toString() const;
  const char* toCString() const;

private:
  ros::Time timestamp_;
  Type type_;
  Severity severity_;
  QString id_;
  int state_;
};
}

#endif // _MRTA_LOG_H_
