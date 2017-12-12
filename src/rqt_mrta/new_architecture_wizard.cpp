#include "rqt_mrta/new_architecture_wizard.h"
#include "rqt_mrta/register_architecture_wizard_page.h"
#include "utilities/exception.h"

namespace rqt_mrta
{

NewArchitectureWizard::NewArchitectureWizard(QWidget* parent, Config* config,
                                             Qt::WindowFlags flags)
    : QWizard(parent, flags), config_(config)
{
  if (!config_)
  {
    throw utilities::Exception(
        "The architecture configuration must not be null.");
  }
  setPage(RegisterArchitecture, new RegisterArchitectureWizardPage(this));
  setWindowTitle("New architecture ...");
  connect(this, SIGNAL(accepted()), this, SLOT(generate()));
  connect(this, SIGNAL(rejected()), this, SLOT(resetConfig()));
}

NewArchitectureWizard::~NewArchitectureWizard() { config_ = NULL; }

Config* NewArchitectureWizard::getConfig() const { return config_; }

void NewArchitectureWizard::generate() {}

void NewArchitectureWizard::resetConfig() { config_->reset(); }
}
