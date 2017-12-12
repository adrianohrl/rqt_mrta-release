#include "rqt_mrta/register_architecture_widget.h"
#include "rqt_mrta/register_architecture_wizard_page.h"
#include "rqt_mrta/ui_register_architecture_widget.h"

namespace rqt_mrta
{
RegisterArchitectureWizardPage::RegisterArchitectureWizardPage(
    NewArchitectureWizard* parent)
    : NewArchitectureWizardPage(parent, "Register the Architecture")
{
  RegisterArchitectureWidget* widget =
      new RegisterArchitectureWidget(this, parent->getConfig());
  registerField("package*", widget->ui_->architecture_line_edit);
  registerField("robots_type*", widget->ui_->robots_type_combo_box);
  registerField("tasks_type*", widget->ui_->tasks_type_combo_box);
  registerField("allocations_type*", widget->ui_->allocations_type_combo_box);
  connect(widget, SIGNAL(changed()), this, SLOT(updateComplete()));
}

RegisterArchitectureWizardPage::~RegisterArchitectureWizardPage() {}

bool RegisterArchitectureWizardPage::isComplete() const
{
  return widget_ && static_cast<RegisterArchitectureWidget*>(widget_)->validate().isEmpty();
}
}
