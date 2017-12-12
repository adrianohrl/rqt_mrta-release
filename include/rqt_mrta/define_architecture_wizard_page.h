#ifndef _RQT_MRTA_DEFINE_ARCHITECTURE_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_ARCHITECTURE_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
class DefineArchitectureWidget;

class DefineArchitectureWizardPage : public NewApplicationWizardPage
{
  Q_OBJECT
public:
  DefineArchitectureWizardPage(NewApplicationWizard* parent);
  virtual ~DefineArchitectureWizardPage();
  bool validatePage();
  bool isComplete() const;
};
}

#endif // _RQT_MRTA_DEFINE_ARCHITECTURE_WIZARD_PAGE_H_
