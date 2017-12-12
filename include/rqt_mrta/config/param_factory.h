#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_FACTORY_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_FACTORY_H_

#include <ros/console.h>
#include "rqt_mrta/config/param.h"
#include "rqt_mrta/config/params.h"
#include "rqt_mrta/config/params_array.h"

namespace rqt_mrta
{
namespace config
{
class ParamFactory
{
public:
  static ParamInterface* newInstance(const QString& group_name, Params* parent = NULL)
  {
    ParamInterface* param = NULL;
    if (group_name.startsWith("param_"))
    {
      param = new Param(parent);
    }
    else if (group_name.startsWith("params_"))
    {
      param = new Params(parent);
    }
    else if (group_name.startsWith("array_"))
    {
      param = new ParamsArray(parent);
    }
    return param;
  }
};
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_FACTORY_H_
