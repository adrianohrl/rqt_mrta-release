#include <QVBoxLayout>
#include "rqt_mrta/define_robots_widget.h"
#include "rqt_mrta/define_robots_wizard_page.h"
#include "rqt_mrta/ui_define_robots_widget.h"

namespace rqt_mrta
{
DefineRobotsWizardPage::DefineRobotsWizardPage(NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent,
                               "Define the Application Robots and Tasks")
{
  DefineRobotsWidget* widget =
      new DefineRobotsWidget(this, application_config_);
  connect(widget, SIGNAL(changed()), this, SIGNAL(completeChanged()));
  setWidget(widget);
}

DefineRobotsWizardPage::~DefineRobotsWizardPage() {}

void DefineRobotsWizardPage::cleanupPage() { architecture_config_->reset(); }

bool DefineRobotsWizardPage::isComplete() const
{
  return static_cast<DefineRobotsWidget*>(widget_)->validate().isEmpty();
}
}
