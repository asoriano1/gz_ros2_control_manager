#pragma once

#include <atomic>
#include <memory>

#include <QFutureSynchronizer>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QVariantList>

#include <gz/gui/Plugin.hh>

#include "gz_ros2_control_manager/ControllerManagerClient.hh"
#include "gz_ros2_control_manager/Ros2ControlDiscovery.hh"

namespace gz_ros2_control_manager
{

/// Gazebo Harmonic GUI plugin: ROS 2 Control Manager.
///
/// Discovers controller_manager instances by scanning ROS 2 services only
/// (no ECM, no SDF, no URDF, no XACRO, no topic inspection, no importer
/// metadata).  For each discovered controller_manager, queries
/// list_controllers + (optionally) list_hardware_interfaces +
/// list_hardware_components and lets the user activate/deactivate
/// already-loaded controllers via STRICT switch_controller calls.
class Ros2ControlManagerGui : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString      statusText        READ statusText        NOTIFY statusTextChanged)
  Q_PROPERTY(bool         busy              READ busy              NOTIFY busyChanged)
  Q_PROPERTY(bool         autoRefresh       READ autoRefresh       NOTIFY autoRefreshChanged)
  Q_PROPERTY(bool         debugMode         READ debugMode         NOTIFY debugModeChanged)
  Q_PROPERTY(QString      lastRefreshTime   READ lastRefreshTime   NOTIFY lastRefreshTimeChanged)
  Q_PROPERTY(QVariantList controllerManagers
             READ controllerManagers
             NOTIFY controllerManagersChanged)

public:
  Ros2ControlManagerGui();
  ~Ros2ControlManagerGui() override;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  // ---- Property reads ----
  QString      statusText()        const { return statusText_; }
  bool         busy()              const { return busy_; }
  bool         autoRefresh()       const { return autoRefresh_; }
  bool         debugMode()         const { return debugMode_; }
  QString      lastRefreshTime()   const { return lastRefreshTime_; }
  QVariantList controllerManagers() const { return managerCards_; }

  // ---- Invokables ----
  Q_INVOKABLE void refresh();
  Q_INVOKABLE void setAutoRefresh(bool enabled);
  Q_INVOKABLE void setDebugMode(bool enabled);
  Q_INVOKABLE void activateController(const QString &managerPath,
                                      const QString &controllerName);
  Q_INVOKABLE void deactivateController(const QString &managerPath,
                                        const QString &controllerName);
  /// Call configure_controller on an already-loaded but unconfigured controller.
  Q_INVOKABLE void configureController(const QString &managerPath,
                                       const QString &controllerName);
  /// Load a configured-but-not-loaded controller and immediately configure it
  /// to the inactive state (load_controller → configure_controller sequence).
  Q_INVOKABLE void loadControllerInactive(const QString &managerPath,
                                          const QString &controllerName);

signals:
  void statusTextChanged();
  void busyChanged();
  void autoRefreshChanged();
  void debugModeChanged();
  void lastRefreshTimeChanged();
  void controllerManagersChanged();

private slots:
  void onAutoRefreshTick();

private:
  /// One refresh worth of data gathered for a single controller_manager.
  /// Built off the main thread; published to QML as one card.
  struct ManagerSnapshot
  {
    DiscoveredManager                  manager;
    ListControllersResult              controllers;
    ListHardwareInterfacesResult       hwInterfaces;
    ListHardwareComponentsResult       hwComponents;
    ListConfiguredControllersResult    configuredControllers;
  };

  void setStatus(const QString &text);
  void setBusy(bool value);
  void publishSnapshots(const std::vector<ManagerSnapshot> &snaps);

  void runRefreshAsync();
  void runSwitchAsync(QString managerPath, QString controllerName, bool activate);
  void runConfigureAsync(QString managerPath, QString controllerName);
  void runLoadInactiveAsync(QString managerPath, QString controllerName);

  /// Try to claim the single in-flight slot. Returns true on success.
  /// On failure, leaves the current operation alone and updates status.
  bool tryBeginOperation(const QString &description);
  void endOperation();

  std::shared_ptr<Ros2ControlDiscovery>     discovery_;
  std::shared_ptr<ControllerManagerClient>  client_;

  // Main-thread Qt state.
  QVariantList managerCards_;
  QString      statusText_{"Not yet refreshed"};
  QString      lastRefreshTime_;
  bool         busy_{false};
  bool         autoRefresh_{false};
  bool         debugMode_{false};

  QTimer       autoRefreshTimer_;

  // ---- Concurrency / lifetime ----
  // Single guard used by both refresh and switch — disables UI actions and
  // prevents two service-pipeline workers from running at once.
  std::atomic<bool> operationInFlight_{false};

  // Set in the destructor before stopping things; workers should stop
  // posting back to Qt when this is true.
  std::atomic<bool> shuttingDown_{false};

  // Tracks every QtConcurrent::run we launch so the destructor can wait for
  // them before tearing down discovery_/client_.
  QFutureSynchronizer<void> futures_;
};

}  // namespace gz_ros2_control_manager
