#include <QDir>
#include <QFileInfo>
#include <QTextStream>
#include <ros/console.h>
#include "utilities/ros_package.h"
#include "utilities/exception.h"
#include "utilities/xml_settings.h"

namespace utilities
{
RosPackage::RosPackage(QObject* parent)
    : AbstractConfig(parent), export_(new Export(this))
{
  reset();
  updateRp();
  connect(export_, SIGNAL(changed()), this, SIGNAL(changed()));
}

RosPackage::~RosPackage() {}

QString RosPackage::getWorkspaceUrl() const { return workspace_url_; }

QString RosPackage::getName() const { return name_; }

QString RosPackage::getUrl() const { return url_; }

QString RosPackage::getVersion() const { return version_; }

QString RosPackage::getDescription() const { return description_; }

QString RosPackage::getAuthor() const { return author_; }

QString RosPackage::getAuthorEmail() const { return author_email_; }

QString RosPackage::getMaintainer() const { return maintainer_; }

QString RosPackage::getMaintainerEmail() const { return maintainer_email_; }

QString RosPackage::getLicense() const { return license_; }

QString RosPackage::getBuildtoolDepend() const { return buildtool_depend_; }

QStringList RosPackage::getBuildDepends() const { return build_depends_; }

QStringList RosPackage::getRunDepends() const { return run_depends_; }

Export* RosPackage::getExport() const { return export_; }

void RosPackage::setWorkspaceUrl(const QString& url)
{
  if (url != workspace_url_)
  {
    workspace_url_ = url;
    emit workspaceUrlChanged(url);
    emit changed();
    setUrl();
  }
}

void RosPackage::setName(const QString& name)
{
  if (name != name_)
  {
    name_ = name;
    emit nameChanged(name);
    emit changed();
    setUrl();
  }
}

void RosPackage::setUrl()
{
  std::string url;
  rp_.find(name_.toStdString(), url);
  if (url.empty() && !workspace_url_.isEmpty() && !name_.isEmpty())
  {
    setUrl(workspace_url_ + "/src/" + name_);
  }
  else
  {
    setUrl(QString::fromStdString(url));
  }
}

void RosPackage::setUrl(const QString& url)
{
  if (url != url_)
  {
    url_ = url;
    emit urlChanged(url);
    emit changed();
  }
}

void RosPackage::setVersion(const QString& version)
{
  if (version != version_)
  {
    version_ = version;
    emit versionChanged(version);
    emit changed();
  }
}

void RosPackage::setDescription(const QString& description)
{
  if (description != description_)
  {
    description_ = description;
    emit descriptionChanged(description);
    emit changed();
  }
}

void RosPackage::setAuthor(const QString& name)
{
  if (name != author_)
  {
    author_ = name;
    emit authorChanged(name);
    emit changed();
  }
}

void RosPackage::setAuthorEmail(const QString& email)
{
  if (email != author_email_)
  {
    author_email_ = email;
    emit authorEmailChanged(email);
    emit changed();
  }
}

void RosPackage::setMaintainer(const QString& name)
{
  if (name != maintainer_)
  {
    maintainer_ = name;
    emit maintainerChanged(name);
    emit changed();
  }
}

void RosPackage::setMaintainerEmail(const QString& email)
{
  if (email != maintainer_email_)
  {
    maintainer_email_ = email;
    emit maintainerEmailChanged(email);
    emit changed();
  }
}

void RosPackage::setLicense(const QString& license)
{
  if (license != license_)
  {
    license_ = license;
    emit licenseChanged(license);
    emit changed();
  }
}

void RosPackage::setBuildtoolDepend(const QString& depend)
{
  if (depend != buildtool_depend_)
  {
    buildtool_depend_ = depend;
    emit buildtoolDependChanged(depend);
    emit changed();
  }
}

void RosPackage::setBuildDepends(const QString& depends)
{
  int count(depends.count());
  QStringRef depends_ref(&depends, depends[0] == '[' ? 1 : 0,
                         count - (depends[count - 1] == ']' ? 2 : 0));
  setBuildDepends(depends_ref.toString().split(','));
}

void RosPackage::setBuildDepends(const QStringList& depends)
{
  if (depends != build_depends_)
  {
    build_depends_ = depends;
    emit buildDependsChanged(depends);
    emit changed();
  }
}

void RosPackage::setRunDepends(const QString& depends)
{
  int count(depends.count());
  QStringRef depends_ref(&depends, depends[0] == '[' ? 1 : 0,
                         count - (depends[count - 1] == ']' ? 2 : 0));
  setBuildDepends(depends_ref.toString().split(','));
}

void RosPackage::setRunDepends(const QStringList& depends)
{
  if (depends != run_depends_)
  {
    run_depends_ = depends;
    emit runDependsChanged(depends);
    emit changed();
  }
}

size_t RosPackage::countBuildDepends() const { return build_depends_.count(); }

size_t RosPackage::countRunDepends() const { return run_depends_.count(); }

QString RosPackage::getBuildDepend(size_t index) const
{
  return build_depends_[index];
}

QString RosPackage::getRunDepend(size_t index) const
{
  return run_depends_[index];
}

void RosPackage::addBuildDepend(const QString& depend)
{
  std::string package_path;
  rp_.find(depend.toStdString(), package_path);
  if (!build_depends_.contains(depend) && !package_path.empty())
    ;
  {
    build_depends_.append(depend);
    emit buildDependAdded(depend);
    emit changed();
  }
}

void RosPackage::addRunDepend(const QString& depend)
{
  std::string package_path;
  rp_.find(depend.toStdString(), package_path);
  if (!run_depends_.contains(depend) && !package_path.empty())
  {
    run_depends_.append(depend);
    emit runDependAdded(depend);
    emit changed();
  }
}

void RosPackage::removeBuildDepend(const QString& depend)
{
  if (build_depends_.contains(depend))
  {
    build_depends_.removeAll(depend);
    emit buildDependRemoved(depend);
    emit changed();
  }
}

void RosPackage::removeRunDepend(const QString& depend)
{
  if (run_depends_.contains(depend))
  {
    run_depends_.removeAll(depend);
    emit runDependRemoved(depend);
    emit changed();
  }
}

void RosPackage::clearBuildDepends()
{
  if (!build_depends_.isEmpty())
  {
    build_depends_.clear();
    emit buildDependsCleared();
    emit changed();
  }
}

void RosPackage::clearRunDepends()
{
  if (!run_depends_.isEmpty())
  {
    run_depends_.clear();
    emit runDependsCleared();
    emit changed();
  }
}

QString RosPackage::validate() const
{
  if (name_.isEmpty())
  {
    return "The package name must not be empty.";
  }
  if (name_.contains(' '))
  {
    return "The package name must not contain space characters.";
  }
  if (version_.isEmpty())
  {
    return "The package version must be defined.";
  }
  if (description_.isEmpty())
  {
    return "The package description must be given.";
  }
  if (maintainer_.isEmpty())
  {
    return "The package maintainer must be given.";
  }
  if (maintainer_email_.isEmpty() || !maintainer_email_.contains('@'))
  {
    return "The e-mail of the package maintainer must be given.";
  }
  if (license_.isEmpty())
  {
    return "The package license must be defined.";
  }
  return "";
}

bool RosPackage::isValidPackageName() const
{
  return !name_.isEmpty() && !name_.contains(' ');
}

bool RosPackage::createPackage()
{
  QDir dir(url_);
  if (dir.exists())
  {
    ROS_WARN_STREAM("The given package [" << name_.toStdString()
                                          << "] already exists.");
    return false;
  }
  if (workspaceExists() || catkinInitWorkspace())
  {
    QDir package_dir(workspace_url_ + "/src");
    package_dir.mkdir(name_);
    if (createManifest() && createCMakeLists() && catkinMake())
    {
      updateRp();
      setUrl(workspace_url_ + "/src/" + name_);
    }
  }
  return url_ == workspace_url_ + "/src/" + name_;
}

bool RosPackage::createManifest()
{
  QString error_message(validate());
  if (!error_message.isEmpty())
  {
    ROS_ERROR("%s", error_message.toStdString().c_str());
    return false;
  }
  if (!workspaceExists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is not a ROS workspace.");
    return false;
  }
  QString package_url(workspace_url_ + "/src/" + name_);
  QDir package_dir(package_url);
  if (!package_dir.exists())
  {
    ROS_ERROR_STREAM("The given package location [" << package_url.toStdString()
                                                    << "] does not exist.");
    return false;
  }
  QFileInfo file_info(getManifestUrl());
  if (file_info.exists())
  {
    ROS_ERROR_STREAM("The given package ["
                     << name_.toStdString()
                     << "] already has its package.xml file.");
    return false;
  }
  QSettings settings(package_url + "/package.xml",
                     utilities::XmlSettings::format);
  if (settings.isWritable())
  {
    settings.clear();
    save(settings);
    settings.sync();
    if (settings.status() == QSettings::NoError)
    {
      ROS_INFO_STREAM("Created the " << name_.toStdString()
                                     << " manifest file.");
      return true;
    }
  }
  ROS_ERROR_STREAM("Unable to create the " << name_.toStdString()
                                           << " manifest file.");
  return false;
}

bool RosPackage::createCMakeLists()
{
  if (!isValidPackageName())
  {
    ROS_ERROR_STREAM("The given package name [" << name_.toStdString()
                                                << "] is not valid.");
    return false;
  }
  if (!workspaceExists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is not a ROS workspace.");
    return false;
  }
  QString package_url(workspace_url_ + "/src/" + name_);
  QDir package_dir(package_url);
  if (!package_dir.exists())
  {
    ROS_ERROR_STREAM("The given package location [" << package_url.toStdString()
                                                    << "] does not exist.");
    return false;
  }
  QFile file(package_url + "/CMakeLists.txt");
  if (file.exists())
  {
    ROS_ERROR_STREAM("The given package ["
                     << name_.toStdString()
                     << "] already has its CMakeLists.txt file.");
    return false;
  }
  QString cmake_lists_text;
  cmake_lists_text += "cmake_minimum_required(VERSION 2.8.3)\n";
  cmake_lists_text += "project(" + name_ + ")\n";
  cmake_lists_text += "find_package(catkin REQUIRED)\n";
  cmake_lists_text += "catkin_package()";
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR_STREAM("Unable to open the " << package_url.toStdString()
                                           << "/CMakeLists.txt file.");
    return false;
  }
  file.write(cmake_lists_text.toStdString().c_str());
  file.close();
  ROS_INFO_STREAM("Created the " << name_.toStdString()
                                 << " CMakeLists.txt file.");
  return true;
}

bool RosPackage::updateManifest()
{
  ROS_FATAL("The RosPackage::updateManifest() has not been implemented yet.");
  return false;
}

QString RosPackage::getManifestUrl() const
{
  return !getUrl().isEmpty() ? getUrl() + "/package.xml" : "";
}

QString RosPackage::getCMakeListsUrl() const
{
  return !getUrl().isEmpty() ? getUrl() + "/CMakeLists.txt" : "";
}

bool RosPackage::catkinMake() const
{
  if (workspace_url_.isEmpty())
  {
    ROS_ERROR("The workspace url is not defined yet.");
    return false;
  }
  QDir workspace_dir(workspace_url_);
  if (!workspace_dir.exists())
  {
    ROS_ERROR_STREAM("The given workspace url [" << workspace_url_.toStdString()
                                                 << "] does not exist.");
    return false;
  }
  QFileInfo file_info(workspace_url_ + "/src/CMakeLists.txt");
  if (!file_info.exists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is not a ROS workspace.");
    return false;
  }
  ROS_INFO_STREAM("Running the catkin_make command at the given workspace url ["
                  << workspace_url_.toStdString() << "]");
  QString cmd("cd " + workspace_url_ + " && catkin_make");
  system(cmd.toStdString().c_str());
  return true;
}

bool RosPackage::workspaceExists() const
{
  if (workspace_url_.isEmpty())
  {
    return false;
  }
  QFileInfo file_info(workspace_url_ + "/src/CMakeLists.txt");
  return file_info.exists();
}

bool RosPackage::catkinInitWorkspace() const
{
  if (workspaceExists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is already a ROS workspace.");
    return false;
  }
  QDir workspace_dir(workspace_url_);
  if (!workspace_dir.exists())
  {
    ROS_ERROR_STREAM("The given url ["
                     << workspace_url_.toStdString()
                     << "] ror the ROS workspace does not exist.");
    return false;
  }
  QString workspace_src_url(workspace_url_ + "/src");
  QDir workspace_src_dir(workspace_src_url);
  if (!workspace_src_dir.exists())
  {
    workspace_dir.mkdir("src");
    if (!workspace_src_dir.exists())
    {
      ROS_ERROR_STREAM(
          "Unable to make directory: " << workspace_src_url.toStdString());
      return false;
    }
    ROS_INFO_STREAM("Made directory: " << workspace_src_url.toStdString());
  }
  QString cmd("cd " + workspace_src_url + " && catkin_init_workspace");
  system(cmd.toStdString().c_str());
  if (!workspaceExists())
  {
    ROS_ERROR_STREAM(
        "Unable to create ROS workspace at: " << workspace_url_.toStdString());
    return false;
  }
  ROS_INFO_STREAM(
      "Created ROS workspace at: " << workspace_src_url.toStdString());
  catkinMake();
  QString bashrc_url(QDir::homePath() + "/.bashrc");
  QFileInfo bashrc_file_info(bashrc_url);
  if (!bashrc_file_info.exists())
  {
    ROS_ERROR_STREAM("Unable to configure ROS workspace at the ~/bashrc file.");
    return false;
  }
  QFile bashrc_file(bashrc_url);
  if (!bashrc_file.open(QIODevice::ReadWrite | QIODevice::Append))
  {
    ROS_ERROR_STREAM("Unable to open the ~/bashrc file.");
    return false;
  }
  cmd = "\nsource " + workspace_url_ + "/devel/setup.bash";
  QTextStream stream(&bashrc_file);
  QString line(stream.readLine());
  while (!line.isNull())
    ;
  {
    if (line.contains(cmd))
    {
      bashrc_file.close();
      return true;
    }
    line = stream.readLine();
  }
  ROS_INFO_STREAM("Appending the 'source "
                  << workspace_url_.toStdString()
                  << "/devel/setup.bash' command to ~/.bashrc ");
  bashrc_file.write(cmd.toStdString().c_str());
  bashrc_file.close();
  return true;
}

void RosPackage::save() const
{
  QString url(getManifestUrl());
  if (url.isEmpty())
  {
    throw Exception(
        "The package url must be setted before saving its manifest file.");
  }
  QSettings settings(url, utilities::XmlSettings::format);
  if (!settings.isWritable())
  {
    throw Exception("The package manifest url must be writable.");
  }
  settings.clear();
  save(settings);
  settings.sync();
  if (settings.status() == QSettings::NoError)
  {
    ROS_INFO_STREAM("Saved application configuration file ["
                    << url.toStdString() << "].");
  }
}

void RosPackage::save(QSettings& settings) const
{
  QString validation(validate());
  if (!validation.isEmpty())
  {
    throw Exception("All mandatory parameters of the package manifest must be "
                    "given before saving it: " +
                    validation.toStdString());
  }
  settings.beginGroup("package");
  settings.setValue("name", name_);
  settings.setValue("version", version_);
  settings.setValue("description", description_);
  settings.setValue("author", author_);
  settings.setValue("author/@email", author_email_);
  settings.setValue("maintainer", maintainer_);
  settings.setValue("maintainer/@email", maintainer_email_);
  settings.setValue("license", license_);
  settings.setValue("buildtool_depend", buildtool_depend_);
  if (!build_depends_.isEmpty())
  {
    QString depends(build_depends_[0]);
    for (size_t i(1); i < build_depends_.count(); i++)
    {
      depends += "," + build_depends_[i];
    }
    settings.setValue("build_depend", "[" + depends + "]");
  }
  if (!run_depends_.isEmpty())
  {
    QString depends(run_depends_[0]);
    for (size_t i(1); i < run_depends_.count(); i++)
    {
      depends += "," + run_depends_[i];
    }
    settings.setValue("run_depend", "[" + depends + "]");
  }
  export_->save(settings);
  settings.endGroup();
}

void RosPackage::load(const QString& url)
{
  QFileInfo file_info(url);
  ROS_ERROR_STREAM("[RosPackage::load] url: " << url.toStdString());
  if (!file_info.isReadable())
  {
    throw Exception("The given package manifest url must be readeable.");
  }
  QSettings settings(url, utilities::XmlSettings::format);
  if (settings.status() == QSettings::NoError)
  {
    load(settings);
    ROS_INFO_STREAM("Loaded application configuration file ["
                    << url.toStdString() << "].");
  }
}

void RosPackage::load(QSettings& settings)
{
  if (settings.value("package@format").toString() == "2")
  {
    throw Exception("The given manifest format (2) is not treated yet.");
  }
  settings.beginGroup("package");
  setName(settings.value("name").toString());
  std::string url;
  rp_.find(name_.toStdString(), url);
  workspace_url_ = QString::fromStdString(url);
  setWorkspaceUrl(workspace_url_.left(workspace_url_.lastIndexOf("/src/")));
  setVersion(settings.value("version").toString());
  setDescription(settings.value("description").toString());
  setAuthor(settings.value("author").toString());
  setAuthorEmail(settings.value("author/@email").toString());
  setMaintainer(settings.value("maintainer").toString());
  setMaintainerEmail(settings.value("maintainer/@email").toString());
  setLicense(settings.value("license").toString());
  setBuildtoolDepend(settings.value("buildtool_depend").toString());
  setBuildDepends(settings.value("build_depend").toString());
  setRunDepends(settings.value("run_depend").toString());
  export_->load(settings);
  settings.endGroup();
}

void RosPackage::updateRp()
{
  rp_ = rospack::Rospack();
  rp_.setQuiet(true);
  std::vector<std::string> search_path;
  rp_.getSearchPathFromEnv(search_path);
  rp_.crawl(search_path, true);
}

void RosPackage::reset()
{
  setWorkspaceUrl("");
  setName("");
  setVersion("1.0.0");
  setDescription("");
  setAuthor("Adriano Henrique Rossette Leite");
  setAuthorEmail("adrianohrl@gmail.com");
  setMaintainer("");
  setMaintainerEmail("");
  setLicense("BSD");
  setBuildtoolDepend("catkin");
  clearBuildDepends();
  clearRunDepends();
  export_->reset();
}

void RosPackage::write(QDataStream& stream) const
{
  stream << workspace_url_;
  stream << name_;
  stream << version_;
  stream << description_;
  stream << author_;
  stream << author_email_;
  stream << maintainer_;
  stream << maintainer_email_;
  stream << license_;
  stream << buildtool_depend_;
  stream << build_depends_.count();
  for (size_t i(0); i < build_depends_.count(); i++)
  {
    stream << build_depends_[i];
  }
  stream << run_depends_.count();
  for (size_t i(0); i < run_depends_.count(); i++)
  {
    stream << run_depends_[i];
  }
  export_->write(stream);
}

void RosPackage::read(QDataStream& stream)
{
  QString workspace_url;
  QString name;
  QString version;
  QString description;
  QString author;
  QString author_email;
  QString maintainer;
  QString maintainer_email;
  QString license;
  QString buildtool_depend;
  quint64 count;
  QString depend;
  QStringList build_depends;
  QStringList run_depends;
  stream >> workspace_url;
  setWorkspaceUrl(workspace_url);
  stream >> name;
  setName(name);
  stream >> version;
  setVersion(version);
  stream >> description;
  setDescription(description);
  stream >> author;
  setAuthor(author);
  stream >> author_email;
  setAuthorEmail(author_email);
  stream >> maintainer;
  setMaintainer(maintainer);
  stream >> maintainer_email;
  setMaintainerEmail(maintainer_email);
  stream >> license;
  setLicense(license);
  stream >> buildtool_depend;
  setBuildtoolDepend(buildtool_depend);
  stream >> count;
  for (size_t i(0); i < count; i++)
  {
    stream >> depend;
    build_depends.append(depend);
  }
  setBuildDepends(build_depends);
  stream >> count;
  for (size_t i(0); i < count; i++)
  {
    stream >> depend;
    run_depends.append(depend);
  }
  setBuildDepends(run_depends);
  export_->read(stream);
}

RosPackage& RosPackage::operator=(const RosPackage& config)
{
  setWorkspaceUrl(config.workspace_url_);
  setName(config.name_);
  setVersion(config.version_);
  setDescription(config.description_);
  setAuthor(config.author_);
  setAuthorEmail(config.author_email_);
  setMaintainer(config.maintainer_);
  setMaintainerEmail(config.maintainer_email_);
  setLicense(config.license_);
  setBuildtoolDepend(config.buildtool_depend_);
  setBuildDepends(config.build_depends_);
  setRunDepends(config.run_depends_);
  *export_ = *config.export_;
  return *this;
}

RosMetapackage::RosMetapackage(QObject* parent) : RosPackage(parent) {}

RosMetapackage::~RosMetapackage() {}

void RosMetapackage::reset()
{
  RosPackage::reset();
  export_->add("metapackage", "");
}

bool RosMetapackage::createCMakeLists()
{
  if (!isValidPackageName())
  {
    ROS_ERROR_STREAM("The given package name [" << name_.toStdString()
                                                << "] is not valid.");
    return false;
  }
  if (!workspaceExists())
  {
    ROS_ERROR_STREAM("The given url [" << workspace_url_.toStdString()
                                       << "] is not a ROS workspace.");
    return false;
  }
  QString package_url(workspace_url_ + "/src/" + name_);
  QDir package_dir(package_url);
  if (!package_dir.exists())
  {
    ROS_ERROR_STREAM("The given package location [" << package_url.toStdString()
                                                    << "] does not exist.");
    return false;
  }
  QFile file(package_url + "/CMakeLists.txt");
  if (file.exists())
  {
    ROS_ERROR_STREAM("The given package ["
                     << name_.toStdString()
                     << "] already has its CMakeLists.txt file.");
    return false;
  }
  QString cmake_lists_text;
  cmake_lists_text += "cmake_minimum_required(VERSION 2.8.3)\n";
  cmake_lists_text += "project(" + name_ + ")\n";
  cmake_lists_text += "find_package(catkin REQUIRED)\n";
  cmake_lists_text += "catkin_metapackage()";
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
  {
    ROS_ERROR_STREAM("Unable to open the " << package_url.toStdString()
                                           << "/CMakeLists.txt file.");
    return false;
  }
  file.write(cmake_lists_text.toStdString().c_str());
  file.close();
  ROS_INFO_STREAM("Created the " << name_.toStdString()
                                 << " CMakeLists.txt file.");
  return true;
}

Export::Export(RosPackage* parent) : AbstractConfig(parent) {}

Export::~Export() {}

QString Export::getName(size_t index) const { return all_names_[index]; }

QString Export::getValue(const QString& name) const
{
  return all_values_[all_names_.indexOf(name)];
}

QStringList Export::getAllNames() const { return all_names_; }

QStringList Export::getAllValues() const { return all_values_; }

void Export::setAll(const QStringList& all_names, const QStringList& all_values)
{
  if (all_names != all_names_ && all_values != all_values_)
  {
    all_names_ = all_names;
    all_values_ = all_values;
    emit allChanged(all_names, all_values);
    emit changed();
  }
}

size_t Export::count() const { return all_names_.count(); }

void Export::add(const QString& name, const QString& value)
{
  int index(all_names_.indexOf(name));
  if (index == -1)
  {
    all_names_.append(name);
    all_values_.append(value);
    emit added(name, value);
    emit changed();
  }
}

void Export::remove(const QString& name)
{
  int index(all_names_.indexOf(name));
  if (index != -1)
  {
    all_names_.removeAt(index);
    QString value(all_values_[index]);
    all_values_.removeAt(index);
    emit removed(name, value);
    emit changed();
  }
}

void Export::clear()
{
  all_names_.clear();
  all_values_.clear();
}

void Export::save(QSettings& settings) const
{
  settings.beginGroup("export");
  for (size_t i(0); i < count(); i++)
  {
    settings.setValue(all_names_[i], all_values_[i]);
  }
  settings.endGroup();
}

void Export::load(QSettings& settings)
{
  settings.beginGroup("export");
  QStringList all_names(settings.allKeys());
  QStringList all_values;
  for (size_t i(0); i < all_names.count(); i++)
  {
    all_values.append(settings.value(all_names[i]).toString());
  }
  setAll(all_names, all_values);
  settings.endGroup();
}

void Export::reset()
{
  all_names_.clear();
  all_values_.clear();
}

void Export::write(QDataStream& stream) const
{
  stream << (quint64)count();
  for (size_t i(0); i < count(); i++)
  {
    stream << all_names_[i];
    stream << all_values_[i];
  }
}

void Export::read(QDataStream& stream)
{
  quint64 count;
  QString name;
  QString value;
  stream >> count;
  QStringList all_names;
  QStringList all_values;
  for (size_t i(0); i < count; i++)
  {
    stream >> name;
    stream >> value;
    all_names.append(name);
    all_values.append(value);
  }
  setAll(all_names, all_values);
}

Export& Export::operator=(const Export& config)
{
  setAll(config.all_names_, config.all_values_);
  return *this;
}
}
