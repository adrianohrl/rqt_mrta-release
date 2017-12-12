#ifndef _MRTA_ARCHITECTURE_CONFIG_H_
#define _MRTA_ARCHITECTURE_CONFIG_H_

#include "utilities/abstract_config.h"
#include "mrta/taxonomy.h"

namespace mrta
{
class Architecture;

class ArchitectureConfig : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  ArchitectureConfig(QObject* parent = NULL);
  virtual ~ArchitectureConfig() {}
  Architecture* getArchitecture() const;
  void setArchitecture(Architecture* architecture);
  void save(QSettings& settings) const {}
  void load(QSettings& settings);
  void reset() {}
  void write(QDataStream& stream) const {}
  void read(QDataStream& stream) {}

private:
  Architecture* architecture_;
};
}

#endif // _MRTA_ARCHITECTURE_CONFIG_H_
