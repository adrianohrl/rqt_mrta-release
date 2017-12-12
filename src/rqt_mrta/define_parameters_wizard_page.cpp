#include <QVBoxLayout>
#include "rqt_mrta/define_parameters_widget.h"
#include "rqt_mrta/define_parameters_wizard_page.h"
#include "rqt_mrta/ui_define_robots_parameters_widget.h"

namespace rqt_mrta
{
DefineParametersWizardPage::DefineParametersWizardPage(
    NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent,
                               "Define the Application Robots and Tasks")
{
  DefineParametersWidget* widget = new DefineParametersWidget(
      this, application_config_, architecture_config_);
  connect(application_config_, SIGNAL(changed()), this, SIGNAL(completeChanged()));
  setWidget(widget);
}

DefineParametersWizardPage::~DefineParametersWizardPage() {}

void DefineParametersWizardPage::initializePage()
{
  static_cast<DefineParametersWidget*>(widget_)->loadTabs();
}

bool DefineParametersWizardPage::isComplete() const
{
  return static_cast<DefineParametersWidget*>(widget_)->validate().isEmpty();
}
}
