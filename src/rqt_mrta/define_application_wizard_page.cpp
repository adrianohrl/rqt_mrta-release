#include <QVBoxLayout>
#include <rqt_mrta/config/application/rqt_mrta_package.h>
#include "rqt_mrta/define_application_widget.h"
#include "rqt_mrta/define_application_wizard_page.h"
#include "rqt_mrta/ui_define_application_widget.h"

namespace rqt_mrta
{
DefineApplicationWizardPage::DefineApplicationWizardPage(
    NewApplicationWizard* parent)
    : NewApplicationWizardPage(parent, "Define the Application")
{
  DefineApplicationWidget* widget = new DefineApplicationWidget(
      this, parent->getApplicationConfig(), parent->getPackageConfig());
  registerField("name*", widget->ui_->name_line_edit);
  registerField("package*", widget->ui_->package_line_edit);
  registerField("workspace_url*", widget->ui_->workspace_package_line_edit);
  registerField("version*", widget->ui_->version_line_edit);
  registerField("description*", widget->ui_->description_plain_text_edit);
  registerField("maintainer*", widget->ui_->maintainer_line_edit);
  registerField("maintainer_email*", widget->ui_->maintainer_email_line_edit);
  registerField("license*", widget->ui_->license_line_edit);
  registerField("run_depends", widget->ui_->run_depends_plain_text_edit);
  connect(package_config_, SIGNAL(changed()), this, SIGNAL(completeChanged()));
  setWidget(widget);
}

DefineApplicationWizardPage::~DefineApplicationWizardPage() {}

void DefineApplicationWizardPage::initializePage()
{
  application_config_->reset();
  package_config_->reset();
  architecture_config_->reset();
}

bool DefineApplicationWizardPage::isComplete() const
{
  return package_config_->validate().isEmpty();
}
}
