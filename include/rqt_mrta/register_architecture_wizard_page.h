#ifndef _RQT_MRTA_REGISTER_ARCHITECTURE_WIZARD_PAGE_H_
#define _RQT_MRTA_REGISTER_ARCHITECTURE_WIZARD_PAGE_H_

#include "rqt_mrta/new_architecture_wizard_page.h"

namespace rqt_mrta
{
class RegisterArchitectureWidget;
class RegisterArchitectureWizardPage : public NewArchitectureWizardPage
{
  Q_OBJECT
public:
  RegisterArchitectureWizardPage(NewArchitectureWizard* parent);
  virtual ~RegisterArchitectureWizardPage();
  bool isComplete() const;
};
}

#endif // _RQT_MRTA_REGISTER_ARCHITECTURE_WIZARD_PAGE_H_
