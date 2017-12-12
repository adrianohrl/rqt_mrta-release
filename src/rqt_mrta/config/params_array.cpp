#include "rqt_mrta/config/params_array.h"

namespace rqt_mrta
{
namespace config
{
ParamsArray::ParamsArray(Params* parent) : Params("array", parent) {}

ParamsArray::~ParamsArray() {}

Params* ParamsArray::getParentParam() const
{
  return parent() ? static_cast<Params*>(parent()) : NULL;
}

ParamInterface* ParamsArray::clone() const
{
  ParamsArray* array = new ParamsArray();
  *array = *this;
  return array;
}

bool ParamsArray::isArray() const { return true; }

void ParamsArray::createParams(size_t size)
{
  Params* parent = getParentParam();
  for (size_t index(0); index < names_.count(); index++)
  {
    parent->removeParam(names_[index]);
  }
  names_.clear();
  for (size_t i(0); i < size; i++)
  {
    QString name(name_);
    Params* params = new Params(parent);
    params->setName(name.replace("@index@", QString::number(i)));
    names_.append(name);
    for (size_t j(0); j < params_.count(); j++)
    {
      params->addParam(params_[j]->clone());
    }
    parent->addParam(params);
  }
}
}
}
