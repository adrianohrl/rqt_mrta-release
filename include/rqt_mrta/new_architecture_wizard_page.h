#ifndef _RQT_MRTA_NEW_ARCHITECTURE_WIZARD_PAGE_H_
#define _RQT_MRTA_NEW_ARCHITECTURE_WIZARD_PAGE_H_

#include "rqt_mrta/new_architecture_wizard.h"

namespace rqt_mrta
{
class NewArchitectureWizardPage : public QWizardPage
{
  Q_OBJECT
public:
  NewArchitectureWizardPage(NewArchitectureWizard* parent, const QString &title);
  virtual ~NewArchitectureWizardPage();

signals:
  void completeChanged();

protected:
  Config* config_;
  QWidget* widget_;
  void setWidget(QWidget* widget);

private:
  bool setted_;

};
}

#endif // _RQT_MRTA_NEW_ARCHITECTURE_WIZARD_PAGE_H_
