#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_ARRAY_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_ARRAY_H_

#include <QStringList>
#include "rqt_mrta/config/params.h"
#include "utilities/abstract_config.h"

namespace rqt_mrta
{
namespace config
{
class Param;
class ParamsArray : public Params
{
  Q_OBJECT
public:
  ParamsArray(Params* parent = NULL);
  virtual ~ParamsArray();
  Params* getParentParam() const;
  bool isArray() const;
  void createParams(size_t size);
  ParamInterface* clone() const;

private:
  QStringList names_;
};
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAMS_ARRAY_H_
