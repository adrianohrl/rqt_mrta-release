#include <QVBoxLayout>
#include "rqt_mrta/new_architecture_wizard_page.h"

namespace rqt_mrta
{
NewArchitectureWizardPage::NewArchitectureWizardPage(
    NewArchitectureWizard* parent, const QString& title)
    : QWizardPage(parent), config_(parent->getConfig())
{
  setTitle(title);
}

NewArchitectureWizardPage::~NewArchitectureWizardPage() { config_ = NULL; }

void NewArchitectureWizardPage::setWidget(QWidget* widget)
{
  if (!setted_)
  {
    widget_ = widget;
    widget_->setParent(this);
    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(widget_);
    setLayout(layout);
    setted_ = true;
  }
}
}
