#ifndef LRS_HALMSTAD_GUI_PLUGINS_UAV_SPAWNER_HH_
#define LRS_HALMSTAD_GUI_PLUGINS_UAV_SPAWNER_HH_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <QString>
#include <tinyxml2.h>

#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

class QProcess;

namespace lrs_halmstad_gui_plugins
{

class UavSpawner : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString nextName READ NextName NOTIFY nextNameChanged)
  Q_PROPERTY(QString nextSpawnPoint READ NextSpawnPoint NOTIFY placementChanged)
  Q_PROPERTY(bool canRemoveLast READ CanRemoveLast NOTIFY spawnedUavsChanged)
  Q_PROPERTY(QString statusText READ StatusText NOTIFY statusTextChanged)
  Q_PROPERTY(bool busy READ Busy NOTIFY busyChanged)
  Q_PROPERTY(bool useGridSpacing READ UseGridSpacing WRITE SetUseGridSpacing
      NOTIFY placementChanged)
  Q_PROPERTY(double gridSpacingX READ GridSpacingX WRITE SetGridSpacingX
      NOTIFY placementChanged)
  Q_PROPERTY(double gridSpacingY READ GridSpacingY WRITE SetGridSpacingY
      NOTIFY placementChanged)
  Q_PROPERTY(double gridSpacingZ READ GridSpacingZ WRITE SetGridSpacingZ
      NOTIFY placementChanged)
  Q_PROPERTY(double customSpawnX READ CustomSpawnX WRITE SetCustomSpawnX
      NOTIFY placementChanged)
  Q_PROPERTY(double customSpawnY READ CustomSpawnY WRITE SetCustomSpawnY
      NOTIFY placementChanged)
  Q_PROPERTY(double customSpawnZ READ CustomSpawnZ WRITE SetCustomSpawnZ
      NOTIFY placementChanged)

  public: UavSpawner();
  public: ~UavSpawner() override;

  public: QString NextName() const;
  public: QString NextSpawnPoint() const;
  public: bool CanRemoveLast() const;
  public: QString StatusText() const;
  public: bool Busy() const;
  public: bool UseGridSpacing() const;
  public: double GridSpacingX() const;
  public: double GridSpacingY() const;
  public: double GridSpacingZ() const;
  public: double CustomSpawnX() const;
  public: double CustomSpawnY() const;
  public: double CustomSpawnZ() const;

  public: void SetUseGridSpacing(bool _useGridSpacing);
  public: void SetGridSpacingX(double _gridSpacingX);
  public: void SetGridSpacingY(double _gridSpacingY);
  public: void SetGridSpacingZ(double _gridSpacingZ);
  public: void SetCustomSpawnX(double _customSpawnX);
  public: void SetCustomSpawnY(double _customSpawnY);
  public: void SetCustomSpawnZ(double _customSpawnZ);

  public: Q_INVOKABLE void Spawn();
  public: Q_INVOKABLE void RemoveLast();

  signals: void nextNameChanged();
  signals: void statusTextChanged();
  signals: void busyChanged();
  signals: void placementChanged();
  signals: void spawnedUavsChanged();

  protected: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  private: bool GenerateRobotSdf(
      const QString &_name,
      QString &_sdf,
      QString &_error) const;
  private: bool GenerateDetachedCameraSdf(
      const QString &_uavName,
      QString &_sdf,
      QString &_error) const;
  private: bool GenerateSdf(
      const QString &_name,
      const QString &_robotName,
      bool _withCamera,
      bool _gimbal,
      QString &_sdf,
      QString &_error) const;
  private: bool RequestRemove(const std::string &_name, QString &_error) const;
  private: bool RequestSpawn(
      const QString &_sdf,
      const std::string &_name,
      const gz::math::Pose3d &_pose,
      QString &_error) const;
  private: bool EnsureSetPoseBridge(QString &_error);
  private: bool StartCameraBridge(const std::string &_name, QString &_error);
  private: std::string PackageExecutable(
      const std::string &_packageName,
      const std::string &_executableName,
      QString &_error) const;
  private: std::string CameraModelName(const std::string &_uavName) const;
  private: std::tuple<int, int, int> GridCellForIndex(int _index) const;
  private: gz::math::Pose3d SpawnPoseForIndex(int _index) const;
  private: void NotifyPlacementChanged();
  private: void StopCameraBridge(const std::string &_name);
  private: void StopBridgeProcess(QProcess *_process);
  private: std::string WorldName() const;
  private: QString NextSpawnSummary() const;
  private: void SetBusy(bool _busy);
  private: void SetStatus(const QString &_status);

  private: struct CameraBridgeProcess
  {
    std::string name;
    std::unique_ptr<QProcess> process;
  };

  private: QString namePrefix_{"dji"};
  private: int nextIndex_{0};
  private: std::string robotType_{"m100"};
  private: bool withCamera_{true};
  private: bool detachedCamera_{false};
  private: bool modelStatic_{true};
  private: bool allowRenaming_{false};
  private: bool bridgeCamera_{true};
  private: bool bridgeSetPose_{true};
  private: std::string cameraName_{"camera0"};
  private: std::string relativeTo_{"world"};
  private: std::string configuredWorldName_;
  private: unsigned int requestTimeoutMs_{5000};
  private: bool useGridSpacing_{true};
  private: gz::math::Pose3d spawnPose_{0.0, 0.0, 5.0, 0.0, 0.0, 0.0};
  private: gz::math::Pose3d customSpawnPose_{0.0, 0.0, 5.0, 0.0, 0.0, 0.0};
  private: gz::math::Vector3d gridSpacing_{0.0, 2.0, 0.0};
  private: QString statusText_{"Ready to spawn a UAV."};
  private: bool busy_{false};
  private: std::unique_ptr<QProcess> setPoseBridgeProcess_;
  private: std::vector<std::string> spawnedUavNames_;
  private: std::vector<std::string> spawnedCameraNames_;
  private: std::vector<CameraBridgeProcess> cameraBridgeProcesses_;
};

}  // namespace lrs_halmstad_gui_plugins

#endif  // LRS_HALMSTAD_GUI_PLUGINS_UAV_SPAWNER_HH_
