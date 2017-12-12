#ifndef _RQT_MRTA_APPLICATION_PACKAGE_CONFIG_H_
#define _RQT_MRTA_APPLICATION_PACKAGE_CONFIG_H_

#include "utilities/ros_package.h"

namespace rqt_mrta
{
namespace config
{
namespace application
{
class RqtMrtaApplicationPackage : public utilities::RosPackage
{
  Q_OBJECT
public:
  RqtMrtaApplicationPackage(QObject* parent);
  virtual ~RqtMrtaApplicationPackage();
  virtual bool createPackage();
  QStringList willBeGenerated() const;
};
}
}
}

#endif // _RQT_MRTA_APPLICATION_PACKAGE_CONFIG_H_
