#include "lrs_halmstad_gui_plugins/UavSpawner.hh"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QString>
#include <QStringList>

#include <gz/gui/qt.h>
#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

namespace lrs_halmstad_gui_plugins
{

namespace
{

constexpr double kEpsilon = 1e-6;
constexpr auto kGuiDetachedSpawnDelay = std::chrono::milliseconds(250);

bool ParseEnvDouble(const char *_name, double &_value)
{
  const char *raw = std::getenv(_name);
  if (raw == nullptr || *raw == '\0')
  {
    return false;
  }

  errno = 0;
  char *end = nullptr;
  const double parsed = std::strtod(raw, &end);
  if (end == raw || (end != nullptr && *end != '\0') || errno != 0)
  {
    return false;
  }

  _value = parsed;
  return true;
}

bool UseUgvSpawnAnchor(const std::string &_relativeTo)
{
  return _relativeTo == "a201_0000/robot" ||
         _relativeTo == "a201_0000/robot::base_link" ||
         _relativeTo == "ugv_spawn";
}

bool ReadUgvSpawnPose(gz::math::Pose3d &_pose)
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  const bool hasX = ParseEnvDouble("LRS_UGV_SPAWN_X", x);
  const bool hasY = ParseEnvDouble("LRS_UGV_SPAWN_Y", y);
  const bool hasZ = ParseEnvDouble("LRS_UGV_SPAWN_Z", z);
  const bool hasYaw = ParseEnvDouble("LRS_UGV_SPAWN_YAW", yaw);
  if (!(hasX && hasY))
  {
    return false;
  }

  _pose.Set(
      x,
      y,
      hasZ ? z : 0.0,
      0.0,
      0.0,
      hasYaw ? yaw : 0.0);
  return true;
}

std::string ReadText(const tinyxml2::XMLElement *_parent, const char *_name)
{
  if (_parent == nullptr)
  {
    return "";
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr || elem->GetText() == nullptr)
  {
    return "";
  }

  return elem->GetText();
}

bool ReadBool(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    bool _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  bool value = _fallback;
  if (elem->QueryBoolText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

double ReadDouble(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    double _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  double value = _fallback;
  if (elem->QueryDoubleText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

int ReadInt(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    int _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  int value = _fallback;
  if (elem->QueryIntText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

unsigned int ReadUnsignedInt(
    const tinyxml2::XMLElement *_parent,
    const char *_name,
    unsigned int _fallback)
{
  if (_parent == nullptr)
  {
    return _fallback;
  }

  const auto *elem = _parent->FirstChildElement(_name);
  if (elem == nullptr)
  {
    return _fallback;
  }

  unsigned int value = _fallback;
  if (elem->QueryUnsignedText(&value) == tinyxml2::XML_SUCCESS)
  {
    return value;
  }

  return _fallback;
}

std::string TrimmedSummary(const QString &_text)
{
  const auto simplified = _text.simplified();
  constexpr int kLimit = 180;

  if (simplified.size() <= kLimit)
  {
    return simplified.toStdString();
  }

  return simplified.left(kLimit).toStdString() + "...";
}

bool NearlyEqual(double _lhs, double _rhs)
{
  return std::abs(_lhs - _rhs) <= kEpsilon;
}

double EffectiveSpacing(double _configured, double _fallback)
{
  if (std::abs(_configured) > kEpsilon)
  {
    return _configured;
  }

  if (std::abs(_fallback) > kEpsilon)
  {
    return _fallback;
  }

  return 2.0;
}

}  // namespace

UavSpawner::UavSpawner()
{
  this->title = "UAV spawner";
}

UavSpawner::~UavSpawner()
{
  this->StopBridgeProcess(this->setPoseBridgeProcess_.get());
  for (const auto &cameraBridge : this->cameraBridgeProcesses_)
  {
    this->StopBridgeProcess(cameraBridge.process.get());
  }
}

QString UavSpawner::NextName() const
{
  return QString("%1%2").arg(this->namePrefix_).arg(this->nextIndex_);
}

QString UavSpawner::NextSpawnPoint() const
{
  const auto nextPose = this->SpawnPoseForIndex(this->nextIndex_);
  return QString("[%1, %2, %3]")
      .arg(nextPose.Pos().X(), 0, 'f', 1)
      .arg(nextPose.Pos().Y(), 0, 'f', 1)
      .arg(nextPose.Pos().Z(), 0, 'f', 1);
}

bool UavSpawner::CanRemoveLast() const
{
  return !this->spawnedUavNames_.empty();
}

QString UavSpawner::StatusText() const
{
  return this->statusText_;
}

bool UavSpawner::Busy() const
{
  return this->busy_;
}

bool UavSpawner::UseGridSpacing() const
{
  return this->useGridSpacing_;
}

double UavSpawner::GridSpacingX() const
{
  return this->gridSpacing_.X();
}

double UavSpawner::GridSpacingY() const
{
  return this->gridSpacing_.Y();
}

double UavSpawner::GridSpacingZ() const
{
  return this->gridSpacing_.Z();
}

double UavSpawner::CustomSpawnX() const
{
  return this->customSpawnPose_.Pos().X();
}

double UavSpawner::CustomSpawnY() const
{
  return this->customSpawnPose_.Pos().Y();
}

double UavSpawner::CustomSpawnZ() const
{
  return this->customSpawnPose_.Pos().Z();
}

void UavSpawner::SetUseGridSpacing(bool _useGridSpacing)
{
  if (this->useGridSpacing_ == _useGridSpacing)
  {
    return;
  }

  this->useGridSpacing_ = _useGridSpacing;
  this->NotifyPlacementChanged();
}

void UavSpawner::SetGridSpacingX(double _gridSpacingX)
{
  if (NearlyEqual(this->gridSpacing_.X(), _gridSpacingX))
  {
    return;
  }

  this->gridSpacing_.Set(
      _gridSpacingX,
      this->gridSpacing_.Y(),
      this->gridSpacing_.Z());
  this->NotifyPlacementChanged();
}

void UavSpawner::SetGridSpacingY(double _gridSpacingY)
{
  if (NearlyEqual(this->gridSpacing_.Y(), _gridSpacingY))
  {
    return;
  }

  this->gridSpacing_.Set(
      this->gridSpacing_.X(),
      _gridSpacingY,
      this->gridSpacing_.Z());
  this->NotifyPlacementChanged();
}

void UavSpawner::SetGridSpacingZ(double _gridSpacingZ)
{
  if (NearlyEqual(this->gridSpacing_.Z(), _gridSpacingZ))
  {
    return;
  }

  this->gridSpacing_.Set(
      this->gridSpacing_.X(),
      this->gridSpacing_.Y(),
      _gridSpacingZ);
  this->NotifyPlacementChanged();
}

void UavSpawner::SetCustomSpawnX(double _customSpawnX)
{
  if (NearlyEqual(this->customSpawnPose_.Pos().X(), _customSpawnX))
  {
    return;
  }

  this->customSpawnPose_.Set(
      _customSpawnX,
      this->customSpawnPose_.Pos().Y(),
      this->customSpawnPose_.Pos().Z(),
      this->customSpawnPose_.Rot().Euler().X(),
      this->customSpawnPose_.Rot().Euler().Y(),
      this->customSpawnPose_.Rot().Euler().Z());
  this->NotifyPlacementChanged();
}

void UavSpawner::SetCustomSpawnY(double _customSpawnY)
{
  if (NearlyEqual(this->customSpawnPose_.Pos().Y(), _customSpawnY))
  {
    return;
  }

  this->customSpawnPose_.Set(
      this->customSpawnPose_.Pos().X(),
      _customSpawnY,
      this->customSpawnPose_.Pos().Z(),
      this->customSpawnPose_.Rot().Euler().X(),
      this->customSpawnPose_.Rot().Euler().Y(),
      this->customSpawnPose_.Rot().Euler().Z());
  this->NotifyPlacementChanged();
}

void UavSpawner::SetCustomSpawnZ(double _customSpawnZ)
{
  if (NearlyEqual(this->customSpawnPose_.Pos().Z(), _customSpawnZ))
  {
    return;
  }

  this->customSpawnPose_.Set(
      this->customSpawnPose_.Pos().X(),
      this->customSpawnPose_.Pos().Y(),
      _customSpawnZ,
      this->customSpawnPose_.Rot().Euler().X(),
      this->customSpawnPose_.Rot().Euler().Y(),
      this->customSpawnPose_.Rot().Euler().Z());
  this->NotifyPlacementChanged();
}

void UavSpawner::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (_pluginElem == nullptr)
  {
    return;
  }

  const auto namePrefix = ReadText(_pluginElem, "name_prefix");
  if (!namePrefix.empty())
  {
    this->namePrefix_ = QString::fromStdString(namePrefix);
  }

  this->nextIndex_ = ReadInt(_pluginElem, "start_index", this->nextIndex_);

  const auto robotType = ReadText(_pluginElem, "type");
  if (!robotType.empty())
  {
    this->robotType_ = robotType;
  }

  const auto cameraName = ReadText(_pluginElem, "camera_name");
  if (!cameraName.empty())
  {
    this->cameraName_ = cameraName;
  }

  const auto relativeTo = ReadText(_pluginElem, "relative_to");
  if (!relativeTo.empty())
  {
    this->relativeTo_ = relativeTo;
  }

  this->configuredWorldName_ = ReadText(_pluginElem, "world_name");
  this->withCamera_ = ReadBool(_pluginElem, "with_camera", this->withCamera_);
  this->detachedCamera_ =
      ReadBool(_pluginElem, "detached_camera", this->detachedCamera_);
  this->modelStatic_ = ReadBool(_pluginElem, "model_static", this->modelStatic_);
  this->allowRenaming_ = ReadBool(_pluginElem, "allow_renaming", this->allowRenaming_);
  this->bridgeCamera_ = ReadBool(_pluginElem, "bridge_camera", this->bridgeCamera_);
  this->bridgeSetPose_ = ReadBool(_pluginElem, "bridge_set_pose", this->bridgeSetPose_);
  this->requestTimeoutMs_ =
      ReadUnsignedInt(_pluginElem, "request_timeout_ms", this->requestTimeoutMs_);
  this->useGridSpacing_ =
      ReadBool(_pluginElem, "use_grid_spacing", this->useGridSpacing_);

  this->spawnPose_.Set(
      ReadDouble(_pluginElem, "spawn_x", this->spawnPose_.Pos().X()),
      ReadDouble(_pluginElem, "spawn_y", this->spawnPose_.Pos().Y()),
      ReadDouble(_pluginElem, "spawn_z", this->spawnPose_.Pos().Z()),
      ReadDouble(_pluginElem, "spawn_roll", this->spawnPose_.Rot().Euler().X()),
      ReadDouble(_pluginElem, "spawn_pitch", this->spawnPose_.Rot().Euler().Y()),
      ReadDouble(_pluginElem, "spawn_yaw", this->spawnPose_.Rot().Euler().Z()));
  const double legacyGridSpacingX =
      ReadDouble(_pluginElem, "spawn_step_x", this->gridSpacing_.X());
  const double legacyGridSpacingY =
      ReadDouble(_pluginElem, "spawn_step_y", this->gridSpacing_.Y());
  const double legacyGridSpacingZ =
      ReadDouble(_pluginElem, "spawn_step_z", this->gridSpacing_.Z());
  this->gridSpacing_.Set(
      ReadDouble(_pluginElem, "grid_spacing_x", legacyGridSpacingX),
      ReadDouble(_pluginElem, "grid_spacing_y", legacyGridSpacingY),
      ReadDouble(_pluginElem, "grid_spacing_z", legacyGridSpacingZ));
  this->customSpawnPose_.Set(
      ReadDouble(_pluginElem, "custom_spawn_x", this->spawnPose_.Pos().X()),
      ReadDouble(_pluginElem, "custom_spawn_y", this->spawnPose_.Pos().Y()),
      ReadDouble(_pluginElem, "custom_spawn_z", this->spawnPose_.Pos().Z()),
      ReadDouble(_pluginElem, "custom_spawn_roll", this->spawnPose_.Rot().Euler().X()),
      ReadDouble(_pluginElem, "custom_spawn_pitch", this->spawnPose_.Rot().Euler().Y()),
      ReadDouble(_pluginElem, "custom_spawn_yaw", this->spawnPose_.Rot().Euler().Z()));

  this->SetStatus(this->NextSpawnSummary());
}

void UavSpawner::Spawn()
{
  if (this->busy_)
  {
    return;
  }

  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    this->SetStatus("World name is unset. Launch Gazebo through managed_clearpath_sim.");
    return;
  }

  const auto spawnName = this->NextName();
  const auto cameraModelName =
      QString::fromStdString(this->CameraModelName(spawnName.toStdString()));
  const auto spawnPose = this->SpawnPoseForIndex(this->nextIndex_);
  QString uavSdf;
  QString cameraSdf;
  QString error;

  this->SetBusy(true);
  this->SetStatus(QString("Generating %1...").arg(spawnName));

  if (!this->GenerateRobotSdf(spawnName, uavSdf, error))
  {
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  if (!this->RequestSpawn(uavSdf, spawnName.toStdString(), spawnPose, error))
  {
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  // The launch-based spawn path has process boundaries between model spawns.
  // Add a short delay here so detached camera sensors do not race Ogre resource
  // initialization from the GUI thread.
  std::this_thread::sleep_for(kGuiDetachedSpawnDelay);

  if (this->withCamera_ && this->detachedCamera_)
  {
    if (!this->GenerateDetachedCameraSdf(spawnName, cameraSdf, error))
    {
      QString cleanupError;
      this->RequestRemove(spawnName.toStdString(), cleanupError);
      this->SetStatus(error);
      this->SetBusy(false);
      return;
    }

    if (!this->RequestSpawn(cameraSdf, cameraModelName.toStdString(), spawnPose, error))
    {
      QString cleanupError;
      this->RequestRemove(spawnName.toStdString(), cleanupError);
      this->SetStatus(error);
      this->SetBusy(false);
      return;
    }

    std::this_thread::sleep_for(kGuiDetachedSpawnDelay);
  }

  this->spawnedUavNames_.push_back(spawnName.toStdString());
  this->spawnedCameraNames_.push_back(
      (this->withCamera_ && this->detachedCamera_) ? cameraModelName.toStdString() : "");
  ++this->nextIndex_;
  emit this->nextNameChanged();
  emit this->placementChanged();
  emit this->spawnedUavsChanged();

  if (!this->EnsureSetPoseBridge(error))
  {
    this->SetStatus(
        QString("Spawned %1 in %2, but %3")
            .arg(spawnName)
            .arg(QString::fromStdString(worldName))
            .arg(error));
    this->SetBusy(false);
    return;
  }

  if (!this->StartCameraBridge(spawnName.toStdString(), error))
  {
    this->SetStatus(
        QString("Spawned %1 in %2, but %3")
            .arg(spawnName)
            .arg(QString::fromStdString(worldName))
            .arg(error));
    this->SetBusy(false);
    return;
  }

  this->SetStatus(
      QString("Spawned %1 in %2 and bridged it to ROS 2. Next: %3")
          .arg(spawnName)
          .arg(QString::fromStdString(worldName))
          .arg(this->NextSpawnSummary()));
  this->SetBusy(false);
}

void UavSpawner::RemoveLast()
{
  if (this->busy_)
  {
    return;
  }

  if (this->spawnedUavNames_.empty())
  {
    this->SetStatus("No UAV spawned from this panel yet.");
    return;
  }

  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    this->SetStatus("World name is unset. Launch Gazebo through managed_clearpath_sim.");
    return;
  }

  const auto name = this->spawnedUavNames_.back();
  const auto cameraName =
      this->spawnedCameraNames_.empty() ? std::string{} : this->spawnedCameraNames_.back();
  QString error;
  QString cameraError;

  this->SetBusy(true);
  this->SetStatus(QString("Removing %1...").arg(QString::fromStdString(name)));

  if (!cameraName.empty())
  {
    if (!this->RequestRemove(cameraName, cameraError))
    {
      cameraError = QString("Could not remove detached camera %1: %2")
                        .arg(QString::fromStdString(cameraName))
                        .arg(cameraError);
    }
  }

  if (!this->RequestRemove(name, error))
  {
    if (!cameraError.isEmpty())
    {
      error = QString("%1. %2").arg(error, cameraError);
    }
    this->SetStatus(error);
    this->SetBusy(false);
    return;
  }

  this->StopCameraBridge(name);
  this->spawnedUavNames_.pop_back();
  if (!this->spawnedCameraNames_.empty())
  {
    this->spawnedCameraNames_.pop_back();
  }
  if (this->nextIndex_ > 0)
  {
    --this->nextIndex_;
  }

  emit this->nextNameChanged();
  emit this->placementChanged();
  emit this->spawnedUavsChanged();
  QString status =
      QString("Removed %1 from %2. Next: %3")
          .arg(QString::fromStdString(name))
          .arg(QString::fromStdString(worldName))
          .arg(this->NextSpawnSummary());
  if (!cameraError.isEmpty())
  {
    status += QString(" (%1)").arg(cameraError);
  }
  this->SetStatus(status);
  this->SetBusy(false);
}

bool UavSpawner::GenerateRobotSdf(
    const QString &_name,
    QString &_sdf,
    QString &_error) const
{
  const bool attachIntegratedCamera = this->withCamera_ && !this->detachedCamera_;
  return this->GenerateSdf(
      _name,
      _name,
      attachIntegratedCamera,
      false,
      _sdf,
      _error);
}

bool UavSpawner::GenerateDetachedCameraSdf(
    const QString &_uavName,
    QString &_sdf,
    QString &_error) const
{
  return this->GenerateSdf(
      QString::fromStdString(this->CameraModelName(_uavName.toStdString())),
      _uavName,
      false,
      true,
      _sdf,
      _error);
}

bool UavSpawner::GenerateSdf(
    const QString &_name,
    const QString &_robotName,
    bool _withCamera,
    bool _gimbal,
    QString &_sdf,
    QString &_error) const
{
  std::string packagePrefix;
  try
  {
    packagePrefix = ament_index_cpp::get_package_prefix("lrs_halmstad");
  }
  catch (const std::exception &ex)
  {
    _error = QString("Failed to locate lrs_halmstad: %1").arg(ex.what());
    return false;
  }

  const QString program =
      QString::fromStdString(packagePrefix + "/lib/lrs_halmstad/generate_sdf");

  if (!QFileInfo::exists(program))
  {
    _error = QString("Missing generate_sdf executable at %1").arg(program);
    return false;
  }

  QProcess process;
  QStringList arguments{
      "--ros-args",
      "-p", QString("type:=%1").arg(QString::fromStdString(this->robotType_)),
      "-p", QString("name:=%1").arg(_name),
      "-p", QString("robot:=%1").arg(_gimbal ? "False" : "True"),
      "-p", QString("with_camera:=%1").arg(_withCamera ? "true" : "false"),
      "-p", QString("model_static:=%1").arg(this->modelStatic_ ? "true" : "false"),
      "-p", QString("camera_name:=%1").arg(QString::fromStdString(this->cameraName_)),
  };
  if (_gimbal)
  {
    arguments << "-p" << "gimbal:=True"
              << "-p" << QString("robot_name:=%1").arg(_robotName);
  }
  else
  {
    arguments << "-p" << QString("robot_name:=%1").arg(_robotName);
  }

  process.start(program, arguments);
  if (!process.waitForStarted(3000))
  {
    _error = QString("Failed to start %1").arg(program);
    return false;
  }

  if (!process.waitForFinished(10000))
  {
    process.kill();
    process.waitForFinished(1000);
    _error = "Timed out while generating UAV SDF.";
    return false;
  }

  const QString stdOut = QString::fromUtf8(process.readAllStandardOutput());
  const QString stdErr = QString::fromUtf8(process.readAllStandardError());

  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
  {
    QString summary = stdErr.trimmed();
    if (summary.isEmpty())
    {
      summary = stdOut.trimmed();
    }
    _error = QString("generate_sdf failed: %1")
                 .arg(QString::fromStdString(TrimmedSummary(summary)));
    return false;
  }

  _sdf = stdOut.trimmed();
  if (_sdf.isEmpty())
  {
    _error = "generate_sdf returned an empty SDF string.";
    return false;
  }

  return true;
}

bool UavSpawner::RequestRemove(const std::string &_name, QString &_error) const
{
  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    _error = "World name is unset. Cannot call /world/<name>/remove.";
    return false;
  }

  gz::msgs::Entity request;
  request.set_name(_name);
  request.set_type(gz::msgs::Entity::MODEL);

  gz::msgs::Boolean reply;
  bool result = false;
  gz::transport::Node node;
  const std::string service = "/world/" + worldName + "/remove";

  const bool executed =
      node.Request(service, request, this->requestTimeoutMs_, reply, result);

  if (!executed)
  {
    _error = QString("Timed out calling %1").arg(QString::fromStdString(service));
    return false;
  }

  if (!result)
  {
    _error = QString("%1 rejected the remove request")
                 .arg(QString::fromStdString(service));
    return false;
  }

  if (!reply.data())
  {
    _error = QString("Gazebo did not remove %1").arg(QString::fromStdString(_name));
    return false;
  }

  return true;
}

bool UavSpawner::RequestSpawn(
    const QString &_sdf,
    const std::string &_name,
    const gz::math::Pose3d &_pose,
    QString &_error) const
{
  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    _error = "World name is unset. Cannot spawn model.";
    return false;
  }

  QString executableError;
  const auto executable =
      this->PackageExecutable("ros_gz_sim", "create", executableError);
  if (executable.empty())
  {
    _error = executableError;
    return false;
  }

  QProcess process;
  process.setProgram(QString::fromStdString(executable));
  process.setArguments(QStringList{
      "-world", QString::fromStdString(worldName),
      "-name", QString::fromStdString(_name),
      "-string", _sdf,
      "-x", QString::number(_pose.Pos().X(), 'f', 6),
      "-y", QString::number(_pose.Pos().Y(), 'f', 6),
      "-z", QString::number(_pose.Pos().Z(), 'f', 6),
      "-R", QString::number(_pose.Rot().Euler().X(), 'f', 6),
      "-P", QString::number(_pose.Rot().Euler().Y(), 'f', 6),
      "-Y", QString::number(_pose.Rot().Euler().Z(), 'f', 6),
      "--ros-args", "--log-level", "warn",
  });
  process.setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  process.setProcessChannelMode(QProcess::MergedChannels);
  process.start();
  if (!process.waitForStarted(3000))
  {
    _error = QString("Failed to start %1").arg(QString::fromStdString(executable));
    return false;
  }

  if (!process.waitForFinished(static_cast<int>(this->requestTimeoutMs_) + 5000))
  {
    process.kill();
    process.waitForFinished(1000);
    _error = QString("Timed out waiting for ros_gz_sim create to spawn %1")
                 .arg(QString::fromStdString(_name));
    return false;
  }

  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
  {
    const QString output = QString::fromUtf8(process.readAllStandardOutput()).trimmed();
    _error = QString("ros_gz_sim create failed for %1: %2")
                 .arg(QString::fromStdString(_name))
                 .arg(QString::fromStdString(TrimmedSummary(output)));
    return false;
  }

  return true;
}

bool UavSpawner::EnsureSetPoseBridge(QString &_error)
{
  if (!this->bridgeSetPose_)
  {
    return true;
  }

  if (this->setPoseBridgeProcess_ != nullptr &&
      this->setPoseBridgeProcess_->state() != QProcess::NotRunning)
  {
    return true;
  }

  const auto worldName = this->WorldName();
  if (worldName.empty())
  {
    _error = "World name is unset. Cannot bridge /world/<name>/set_pose.";
    return false;
  }

  QString executableError;
  const auto executable =
      this->PackageExecutable("ros_gz_bridge", "parameter_bridge", executableError);
  if (executable.empty())
  {
    _error = executableError;
    return false;
  }

  auto process = std::make_unique<QProcess>();
  process->setProgram(QString::fromStdString(executable));
  process->setArguments(QStringList{
      QString("/world/%1/set_pose@ros_gz_interfaces/srv/SetEntityPose")
          .arg(QString::fromStdString(worldName)),
  });
  process->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  process->setProcessChannelMode(QProcess::MergedChannels);
  process->start();
  if (!process->waitForStarted(3000))
  {
    _error = "Failed to start ROS set_pose bridge.";
    return false;
  }

  this->setPoseBridgeProcess_ = std::move(process);
  return true;
}

bool UavSpawner::StartCameraBridge(const std::string &_name, QString &_error)
{
  if (!this->withCamera_ || !this->bridgeCamera_)
  {
    return true;
  }

  QString executableError;
  const auto executable =
      this->PackageExecutable("ros_gz_bridge", "parameter_bridge", executableError);
  if (executable.empty())
  {
    _error = executableError;
    return false;
  }

  const auto qName = QString::fromStdString(_name);
  const auto qCameraName = QString::fromStdString(this->cameraName_);

  auto process = std::make_unique<QProcess>();
  process->setProgram(QString::fromStdString(executable));
  process->setArguments(QStringList{
      QString("/%1/%2/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image")
          .arg(qName, qCameraName),
      QString("/%1/%2/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo")
          .arg(qName, qCameraName),
  });
  process->setProcessEnvironment(QProcessEnvironment::systemEnvironment());
  process->setProcessChannelMode(QProcess::MergedChannels);
  process->start();
  if (!process->waitForStarted(3000))
  {
    _error = QString("Failed to start ROS camera bridge for %1").arg(qName);
    return false;
  }

  this->cameraBridgeProcesses_.push_back(
      CameraBridgeProcess{_name, std::move(process)});
  return true;
}

std::string UavSpawner::PackageExecutable(
    const std::string &_packageName,
    const std::string &_executableName,
    QString &_error) const
{
  std::string packagePrefix;
  try
  {
    packagePrefix = ament_index_cpp::get_package_prefix(_packageName);
  }
  catch (const std::exception &ex)
  {
    _error = QString("Failed to locate %1: %2")
                 .arg(QString::fromStdString(_packageName))
                 .arg(ex.what());
    return "";
  }

  const auto executablePath =
      packagePrefix + "/lib/" + _packageName + "/" + _executableName;

  if (!QFileInfo::exists(QString::fromStdString(executablePath)))
  {
    _error = QString("Missing executable %1")
                 .arg(QString::fromStdString(executablePath));
    return "";
  }

  return executablePath;
}

std::string UavSpawner::CameraModelName(const std::string &_uavName) const
{
  return _uavName + "_" + this->cameraName_;
}

std::tuple<int, int, int> UavSpawner::GridCellForIndex(int _index) const
{
  if (_index <= 0)
  {
    return {0, 0, 0};
  }

  const bool useVerticalSpacing = std::abs(this->gridSpacing_.Z()) > kEpsilon;
  const std::array<std::tuple<int, int, int>, 26> neighborOrder{{
      {0, 1, 0},
      {0, -1, 0},
      {-1, 0, 0},
      {1, 0, 0},
      {0, 0, -1},
      {0, 0, 1},
      {-1, 1, 0},
      {-1, -1, 0},
      {1, 1, 0},
      {1, -1, 0},
      {-1, 0, -1},
      {-1, 0, 1},
      {1, 0, -1},
      {1, 0, 1},
      {0, 1, -1},
      {0, 1, 1},
      {0, -1, -1},
      {0, -1, 1},
      {-1, 1, -1},
      {-1, 1, 1},
      {-1, -1, -1},
      {-1, -1, 1},
      {1, 1, -1},
      {1, 1, 1},
      {1, -1, -1},
      {1, -1, 1},
  }};

  std::queue<std::tuple<int, int, int>> pending;
  std::set<std::tuple<int, int, int>> visited;
  pending.push({0, 0, 0});
  visited.insert({0, 0, 0});

  int currentIndex = 0;
  while (!pending.empty())
  {
    const auto cell = pending.front();
    pending.pop();

    if (currentIndex == _index)
    {
      return cell;
    }

    ++currentIndex;
    for (const auto &direction : neighborOrder)
    {
      if (!useVerticalSpacing && std::get<2>(direction) != 0)
      {
        continue;
      }

      const std::tuple<int, int, int> nextCell{
          std::get<0>(cell) + std::get<0>(direction),
          std::get<1>(cell) + std::get<1>(direction),
          std::get<2>(cell) + std::get<2>(direction),
      };
      if (visited.insert(nextCell).second)
      {
        pending.push(nextCell);
      }
    }
  }

  return {0, 0, 0};
}

gz::math::Pose3d UavSpawner::SpawnPoseForIndex(int _index) const
{
  gz::math::Pose3d localPose;
  if (!this->useGridSpacing_)
  {
    localPose = this->customSpawnPose_;
  }
  else
  {
    const auto cell = this->GridCellForIndex(_index);
    const double forwardSpacing =
        EffectiveSpacing(this->gridSpacing_.X(), this->gridSpacing_.Y());
    const double lateralSpacing =
        EffectiveSpacing(this->gridSpacing_.Y(), this->gridSpacing_.X());
    const double verticalSpacing = this->gridSpacing_.Z();

    const gz::math::Vector3d localOffset{
        static_cast<double>(std::get<0>(cell)) * forwardSpacing,
        static_cast<double>(std::get<1>(cell)) * lateralSpacing,
        static_cast<double>(std::get<2>(cell)) * verticalSpacing,
    };
    const auto rotatedOffset = this->spawnPose_.Rot().RotateVector(localOffset);
    localPose = gz::math::Pose3d{
        this->spawnPose_.Pos() + rotatedOffset,
        this->spawnPose_.Rot(),
    };
  }

  if (!UseUgvSpawnAnchor(this->relativeTo_))
  {
    return localPose;
  }

  gz::math::Pose3d ugvSpawnPose;
  if (!ReadUgvSpawnPose(ugvSpawnPose))
  {
    return localPose;
  }

  return gz::math::Pose3d{
      ugvSpawnPose.Pos() + ugvSpawnPose.Rot().RotateVector(localPose.Pos()),
      ugvSpawnPose.Rot() * localPose.Rot(),
  };
}

void UavSpawner::NotifyPlacementChanged()
{
  emit this->placementChanged();
  this->SetStatus(this->NextSpawnSummary());
}

void UavSpawner::StopCameraBridge(const std::string &_name)
{
  const auto it = std::find_if(
      this->cameraBridgeProcesses_.begin(),
      this->cameraBridgeProcesses_.end(),
      [&_name](const CameraBridgeProcess &_cameraBridge)
      {
        return _cameraBridge.name == _name;
      });

  if (it == this->cameraBridgeProcesses_.end())
  {
    return;
  }

  this->StopBridgeProcess(it->process.get());
  this->cameraBridgeProcesses_.erase(it);
}

std::string UavSpawner::WorldName() const
{
  if (!this->configuredWorldName_.empty())
  {
    return this->configuredWorldName_;
  }

  const char *worldName = std::getenv("LRS_GAZEBO_WORLD");
  if (worldName == nullptr)
  {
    return "";
  }

  return worldName;
}

QString UavSpawner::NextSpawnSummary() const
{
  const auto nextPose = this->SpawnPoseForIndex(this->nextIndex_);
  return QString("%1 at (%2, %3, %4)")
      .arg(this->NextName())
      .arg(nextPose.Pos().X(), 0, 'f', 1)
      .arg(nextPose.Pos().Y(), 0, 'f', 1)
      .arg(nextPose.Pos().Z(), 0, 'f', 1);
}

void UavSpawner::StopBridgeProcess(QProcess *_process)
{
  if (_process == nullptr || _process->state() == QProcess::NotRunning)
  {
    return;
  }

  _process->terminate();
  if (_process->waitForFinished(1000))
  {
    return;
  }

  _process->kill();
  _process->waitForFinished(1000);
}

void UavSpawner::SetBusy(bool _busy)
{
  if (this->busy_ == _busy)
  {
    return;
  }

  this->busy_ = _busy;
  emit this->busyChanged();
}

void UavSpawner::SetStatus(const QString &_status)
{
  if (this->statusText_ == _status)
  {
    return;
  }

  this->statusText_ = _status;
  emit this->statusTextChanged();
}

}  // namespace lrs_halmstad_gui_plugins

GZ_ADD_PLUGIN(
    lrs_halmstad_gui_plugins::UavSpawner,
    gz::gui::Plugin)

GZ_ADD_PLUGIN_ALIAS(
    lrs_halmstad_gui_plugins::UavSpawner,
    "UavSpawner",
    "UAV spawner")
