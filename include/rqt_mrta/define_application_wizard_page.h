#ifndef _RQT_MRTA_DEFINE_APPLICATION_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_APPLICATION_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
class DefineApplicationWidget;

class DefineApplicationWizardPage : public NewApplicationWizardPage
{
  Q_OBJECT
public:
  DefineApplicationWizardPage(NewApplicationWizard* parent);
  virtual ~DefineApplicationWizardPage();
  void initializePage();
  bool isComplete() const;
};
}

#endif // _RQT_MRTA_DEFINE_APPLICATION_WIZARD_PAGE_H_
