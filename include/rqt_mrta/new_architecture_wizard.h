#ifndef _RQT_MRTA_NEW_ARCHITECTURE_WIZARD_H_
#define _RQT_MRTA_NEW_ARCHITECTURE_WIZARD_H_

#include <QWizard>
#include "rqt_mrta/config/architecture/rqt_mrta_architecture.h"

namespace rqt_mrta
{
typedef config::architecture::RqtMrtaArchitecture Config;

class NewArchitectureWizard : public QWizard
{
  Q_OBJECT
public:
  enum Page
  {
    RegisterArchitecture
  };
  NewArchitectureWizard(QWidget* parent, Config* config,
                       Qt::WindowFlags flags = 0);
  virtual ~NewArchitectureWizard();
  Config* getConfig() const;

private:
  Config* config_;

private slots:
  void generate();
  void resetConfig();
};
}

#endif // _RQT_MRTA_NEW_ARCHITECTURE_WIZARD_H_
