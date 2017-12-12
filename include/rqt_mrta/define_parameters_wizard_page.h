#ifndef _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
class DefineParametersWidget;

class DefineParametersWizardPage : public NewApplicationWizardPage
{
  Q_OBJECT
public:
  DefineParametersWizardPage(NewApplicationWizard* parent);
  virtual ~DefineParametersWizardPage();
  void initializePage();
  bool isComplete() const;
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_PARAMETERS_WIZARD_PAGE_H_

