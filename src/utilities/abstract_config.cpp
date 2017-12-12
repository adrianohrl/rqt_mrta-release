#include "utilities/abstract_config.h"

namespace utilities
{
AbstractConfig::AbstractConfig(QObject* parent) : QObject(parent)
{
}

AbstractConfig::~AbstractConfig() {}

QDataStream& operator<<(QDataStream& stream, const AbstractConfig& config)
{
  config.write(stream);
  return stream;
}

QDataStream& operator>>(QDataStream& stream, AbstractConfig& config)
{
  config.read(stream);
  return stream;
}
}
