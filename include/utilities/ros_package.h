#ifndef _UTILITIES_ROS_PACKAGE_CONFIG_H_
#define _UTILITIES_ROS_PACKAGE_CONFIG_H_

#include <QStringList>
#include <rospack/rospack.h>
#include "utilities/abstract_config.h"

namespace utilities
{
class Export;

class RosPackage : public utilities::AbstractConfig
{
  Q_OBJECT
public:
  RosPackage(QObject* parent = NULL);
  virtual ~RosPackage();
  QString getWorkspaceUrl() const;
  QString getName() const;
  QString getUrl() const;
  QString getVersion() const;
  QString getDescription() const;
  QString getAuthor() const;
  QString getAuthorEmail() const;
  QString getMaintainer() const;
  QString getMaintainerEmail() const;
  QString getLicense() const;
  QString getBuildtoolDepend() const;
  QStringList getBuildDepends() const;
  QStringList getRunDepends() const;
  Export* getExport() const;
  void setWorkspaceUrl(const QString& url);
  void setName(const QString& name);
  void setUrl(const QString& url);
  void setVersion(const QString& version);
  void setDescription(const QString& description);
  void setAuthor(const QString& name);
  void setAuthorEmail(const QString& email);
  void setMaintainer(const QString& name);
  void setMaintainerEmail(const QString& email);
  void setLicense(const QString& license);
  void setBuildtoolDepend(const QString& depend);
  void setBuildDepends(const QString& depends);
  void setBuildDepends(const QStringList& depends);
  void setRunDepends(const QString& depends);
  void setRunDepends(const QStringList& depends);
  size_t countBuildDepends() const;
  size_t countRunDepends() const;
  QString getBuildDepend(size_t index) const;
  QString getRunDepend(size_t index) const;
  void addBuildDepend(const QString& depend);
  void addRunDepend(const QString& depend);
  void removeBuildDepend(const QString& depend);
  void removeRunDepend(const QString& depend);
  void clearBuildDepends();
  void clearRunDepends();
  QString validate() const;
  bool isValidPackageName() const;
  virtual bool createPackage();
  bool createManifest();
  virtual bool createCMakeLists();
  bool updateManifest();
  QString getManifestUrl() const;
  QString getCMakeListsUrl() const;
  bool catkinMake() const;
  bool workspaceExists() const;
  bool catkinInitWorkspace() const;
  void save() const;
  void load(const QString& url);
  virtual void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  RosPackage& operator=(const RosPackage& config);

signals:
  void workspaceUrlChanged(const QString& url);
  void nameChanged(const QString& name);
  void urlChanged(const QString& url);
  void versionChanged(const QString& version);
  void descriptionChanged(const QString& description);
  void authorChanged(const QString& name);
  void authorEmailChanged(const QString& email);
  void maintainerChanged(const QString& name);
  void maintainerEmailChanged(const QString& email);
  void licenseChanged(const QString& license);
  void buildtoolDependChanged(const QString& depend);
  void buildDependsChanged(const QStringList& depends);
  void buildDependAdded(const QString& depend);
  void buildDependRemoved(const QString& depend);
  void buildDependsCleared();
  void runDependsChanged(const QStringList& depends);
  void runDependAdded(const QString& depend);
  void runDependRemoved(const QString& depend);
  void runDependsCleared();

protected:
  rospack::Rospack rp_;
  void save(QSettings& settings) const;
  void load(QSettings& settings);

protected:
  QString workspace_url_;
  QString name_;
  QString url_;
  QString version_;
  QString description_;
  QString author_;
  QString author_email_;
  QString maintainer_;
  QString maintainer_email_;
  QString license_;
  QString buildtool_depend_;
  QStringList build_depends_;
  QStringList run_depends_;
  Export* export_;
  void updateRp();

private slots:
  void setUrl();
};

class Export : public AbstractConfig
{
  Q_OBJECT
public:
  Export(RosPackage* parent);
  virtual ~Export();
  QString getName(size_t index) const;
  QString getValue(const QString &name) const;
  QStringList getAllNames() const;
  QStringList getAllValues() const;
  void setAll(const QStringList& all_names, const QStringList& all_values);
  size_t count() const;
  void add(const QString& name, const QString& value);
  void remove(const QString& name);
  void clear();
  void save(QSettings& settings) const;
  void load(QSettings& settings);
  virtual void reset();
  void write(QDataStream& stream) const;
  void read(QDataStream& stream);
  Export& operator=(const Export& config);

signals:
  void allChanged(const QStringList& all_names,
                  const QStringList& all_values);
  void added(const QString& name, const QString& value);
  void removed(const QString& name, const QString& value);
  void cleared();

private:
  QStringList all_names_;
  QStringList all_values_;
};

class RosMetapackage : public RosPackage
{
  Q_OBJECT
public:
  RosMetapackage(QObject* parent = NULL);
  virtual ~RosMetapackage();
  virtual void reset();
  bool createCMakeLists();
};
}

#endif // _UTILITIES_ROS_PACKAGE_CONFIG_H_
