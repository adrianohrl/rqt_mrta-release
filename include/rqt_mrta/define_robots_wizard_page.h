#ifndef _RQT_MRTA_DEFINE_ROBOTS_WIZARD_PAGE_H_
#define _RQT_MRTA_DEFINE_ROBOTS_WIZARD_PAGE_H_

#include "rqt_mrta/new_application_wizard_page.h"

namespace rqt_mrta
{
class DefineRobotsWidget;

class DefineRobotsWizardPage : public NewApplicationWizardPage
{
  Q_OBJECT
public:
  DefineRobotsWizardPage(NewApplicationWizard* parent);
  virtual ~DefineRobotsWizardPage();
  void cleanupPage();
  bool isComplete() const;
};
}

#endif // _RQT_MRTA_DEFINE_ROBOTS_WIZARD_PAGE_H_

