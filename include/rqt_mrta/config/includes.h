#ifndef _RQT_MRTA_CONFIG_INCLUDES_H_
#define _RQT_MRTA_CONFIG_INCLUDES_H_

#include <QVector>
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Include;
class Includes : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  Includes(QObject* parent = NULL);
  virtual ~Includes();
  Include* getInclude(size_t index) const;
  Include* addInclude();
  void removeInclude(Include* include);
  void removeInclude(size_t index);
  void clearIncludes();
  bool contains(const QString &file) const;
  size_t count() const;
  bool isEmpty() const;
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Includes& operator=(const Includes& config);
  QString validate() const;
  QString toLaunch(const QString &prefix = "") const;

signals:
  void added(size_t index);
  void removed(const QString& id);
  void cleared();
  void includeFileChanged(size_t index, const QString& file);

private:
  QVector<Include*> includes_;

private slots:
  void includeFileChanged(const QString& file);
  void includeDestroyed();
};
}
}

#endif // _RQT_MRTA_CONFIG_INCLUDES_H_
