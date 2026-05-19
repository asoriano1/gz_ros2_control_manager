#include "gz_ros2_control_manager/Ros2ControlManagerGui.hh"

#include <QDateTime>
#include <QVariantMap>
#include <QtConcurrent/QtConcurrentRun>

#include <unordered_set>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/plugin/Register.hh>

// Q_INIT_RESOURCE must live outside any namespace.
static void initControlManagerResources()
{
  Q_INIT_RESOURCE(Ros2ControlManagerGui);
}

namespace gz_ros2_control_manager
{

namespace
{

constexpr int kAutoRefreshIntervalMs = 3000;

QVariantList controllersToVariantList(
    const std::vector<ControllerInfo> &controllers)
{
  QVariantList out;
  out.reserve(static_cast<int>(controllers.size()));
  for (const auto &c : controllers)
  {
    QVariantMap m;
    m[QStringLiteral("name")]  = QString::fromStdString(c.name);
    m[QStringLiteral("type")]  = QString::fromStdString(c.type);
    m[QStringLiteral("state")] = QString::fromStdString(c.state);

    // Single source of truth for the state→action mapping.  Tests cover this
    // in test_controller_action.cpp.
    const auto action = actionFor(c.state);
    m[QStringLiteral("isActionable")]    = action.isActionable;
    m[QStringLiteral("actionLabel")]     = QString::fromStdString(action.label);
    m[QStringLiteral("actionActivates")] = action.activates;

    QStringList claimed;
    claimed.reserve(static_cast<int>(c.claimedInterfaces.size()));
    for (const auto &iface : c.claimedInterfaces)
      claimed << QString::fromStdString(iface);
    m[QStringLiteral("claimedInterfaces")] = claimed;

    out.append(m);
  }
  return out;
}

QVariantList configuredControllersToVariantList(
    const std::vector<ConfiguredController> &declared,
    const std::vector<ControllerInfo>       &loaded)
{
  // We surface only the entries that are NOT loaded, so the user sees the
  // gap rather than the full configured set echoed.
  QVariantList out;
  std::unordered_set<std::string> loadedNames;
  loadedNames.reserve(loaded.size());
  for (const auto &c : loaded)
    loadedNames.insert(c.name);
  for (const auto &c : declared)
  {
    if (loadedNames.count(c.name) > 0)
      continue;
    QVariantMap m;
    m[QStringLiteral("name")]      = QString::fromStdString(c.name);
    m[QStringLiteral("type")]      = QString::fromStdString(c.type);
    m[QStringLiteral("isLoadable")] = true;
    m[QStringLiteral("loadLabel")]  = QStringLiteral("Load inactive");
    out.append(m);
  }
  return out;
}

QVariantList hwInterfacesToVariantList(
    const std::vector<HardwareInterfaceInfo> &interfaces)
{
  QVariantList out;
  out.reserve(static_cast<int>(interfaces.size()));
  for (const auto &iface : interfaces)
  {
    QVariantMap m;
    m[QStringLiteral("name")]        = QString::fromStdString(iface.name);
    m[QStringLiteral("isAvailable")] = iface.isAvailable;
    m[QStringLiteral("isClaimed")]   = iface.isClaimed;
    out.append(m);
  }
  return out;
}

QVariantList hwComponentsToVariantList(
    const std::vector<HardwareComponentInfo> &components)
{
  QVariantList out;
  out.reserve(static_cast<int>(components.size()));
  for (const auto &c : components)
  {
    QVariantMap m;
    m[QStringLiteral("name")]       = QString::fromStdString(c.name);
    m[QStringLiteral("type")]       = QString::fromStdString(c.type);
    m[QStringLiteral("pluginName")] = QString::fromStdString(c.pluginName);
    m[QStringLiteral("state")]      = QString::fromStdString(c.state);
    out.append(m);
  }
  return out;
}

}  // namespace

// ============================================================================
// Construction / lifecycle
// ============================================================================

Ros2ControlManagerGui::Ros2ControlManagerGui()
: gz::gui::Plugin()
{
  initControlManagerResources();

  if (auto *engine = gz::gui::App()->Engine())
  {
    engine->rootContext()->setContextProperty(
        QStringLiteral("controlManager"), this);
  }

  discovery_ = std::make_shared<Ros2ControlDiscovery>();
  client_    = std::make_shared<ControllerManagerClient>(discovery_->node());

  autoRefreshTimer_.setInterval(kAutoRefreshIntervalMs);
  connect(&autoRefreshTimer_, &QTimer::timeout,
          this, &Ros2ControlManagerGui::onAutoRefreshTick);

  gzdbg << "[gz_ros2_control_manager] plugin constructed; ROS 2 node="
        << discovery_->node()->get_fully_qualified_name() << '\n';
}

Ros2ControlManagerGui::~Ros2ControlManagerGui()
{
  gzdbg << "[gz_ros2_control_manager] plugin destruction starting\n";

  // 1) Block new auto-refresh ticks and stop any in-flight worker from
  //    posting fresh state back into a half-destroyed object.
  shuttingDown_ = true;
  autoRefreshTimer_.stop();

  // 2) Wait for outstanding QtConcurrent workers.  Each worker has bounded
  //    timeouts on every service call, so the worst case here is the sum of
  //    the per-service timeouts (≈ a few seconds).
  gzdbg << "[gz_ros2_control_manager] waiting for worker tasks\n";
  futures_.waitForFinished();
  gzdbg << "[gz_ros2_control_manager] worker tasks complete\n";

  // 3) Drop the client first so it releases its node SharedPtr, then the
  //    discovery — its destructor cancels the executor, joins the spin
  //    thread, and finally drops the last reference to the node.
  client_.reset();
  if (discovery_)
  {
    gzdbg << "[gz_ros2_control_manager] stopping ROS executor\n";
    discovery_.reset();
  }
  gzdbg << "[gz_ros2_control_manager] plugin destruction complete\n";
}

void Ros2ControlManagerGui::LoadConfig(const tinyxml2::XMLElement * /*_elem*/)
{
  if (this->title.empty())
    this->title = "ROS 2 Control Manager";
}

// ============================================================================
// Property setters / signals
// ============================================================================

void Ros2ControlManagerGui::setStatus(const QString &text)
{
  if (statusText_ == text) return;
  statusText_ = text;
  emit statusTextChanged();
}

void Ros2ControlManagerGui::setBusy(bool value)
{
  if (busy_ == value) return;
  busy_ = value;
  emit busyChanged();
}

void Ros2ControlManagerGui::setAutoRefresh(bool enabled)
{
  if (autoRefresh_ == enabled) return;
  autoRefresh_ = enabled;
  if (autoRefresh_)
    autoRefreshTimer_.start();
  else
    autoRefreshTimer_.stop();
  emit autoRefreshChanged();
}

void Ros2ControlManagerGui::setDebugMode(bool enabled)
{
  if (debugMode_ == enabled) return;
  debugMode_ = enabled;
  emit debugModeChanged();
}

void Ros2ControlManagerGui::onAutoRefreshTick()
{
  if (operationInFlight_.load() || shuttingDown_.load())
    return;
  refresh();
}

// ============================================================================
// Operation guard — single-slot serialization for refresh & switch
// ============================================================================

bool Ros2ControlManagerGui::tryBeginOperation(const QString &description)
{
  if (shuttingDown_.load())
    return false;
  if (operationInFlight_.exchange(true))
  {
    setStatus(QStringLiteral("Busy: ") + description + QStringLiteral(" rejected (operation already in progress)"));
    return false;
  }
  setBusy(true);
  setStatus(description);
  return true;
}

void Ros2ControlManagerGui::endOperation()
{
  operationInFlight_.store(false);
  setBusy(false);
}

// ============================================================================
// Refresh — discovery + per-manager queries on a worker thread
// ============================================================================

void Ros2ControlManagerGui::refresh()
{
  if (!tryBeginOperation(QStringLiteral("Scanning ROS 2 services…")))
    return;
  runRefreshAsync();
}

void Ros2ControlManagerGui::runRefreshAsync()
{
  Ros2ControlManagerGui *self = this;
  auto discovery = discovery_;
  auto client    = client_;

  auto future = QtConcurrent::run([self, discovery, client]()
  {
    std::vector<ManagerSnapshot> snaps;
    if (discovery && client && !self->shuttingDown_.load())
    {
      const auto managers = discovery->discover();
      snaps.reserve(managers.size());
      for (const auto &mgr : managers)
      {
        if (self->shuttingDown_.load())
          break;
        ManagerSnapshot s;
        s.manager      = mgr;
        s.controllers  = client->listControllers(mgr.managerPath);
        if (self->shuttingDown_.load()) break;
        s.hwInterfaces = client->listHardwareInterfaces(mgr.managerPath);
        if (self->shuttingDown_.load()) break;
        s.hwComponents = client->listHardwareComponents(mgr.managerPath);
        if (self->shuttingDown_.load()) break;
        s.configuredControllers =
            client->listConfiguredControllers(mgr.managerPath);
        snaps.push_back(std::move(s));
      }
    }

    QMetaObject::invokeMethod(self,
      [self, snaps]()
      {
        if (self->shuttingDown_.load())
        {
          self->endOperation();
          return;
        }
        self->publishSnapshots(snaps);
        self->lastRefreshTime_ =
            QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss"));
        emit self->lastRefreshTimeChanged();
        self->endOperation();
      }, Qt::QueuedConnection);
  });
  futures_.addFuture(future);
}

void Ros2ControlManagerGui::publishSnapshots(
    const std::vector<ManagerSnapshot> &snaps)
{
  managerCards_.clear();

  if (snaps.empty())
  {
    gzdbg << "[gz_ros2_control_manager] refresh: no controller_manager instances found\n";
  }

  for (const auto &s : snaps)
  {
    gzdbg << "[gz_ros2_control_manager] manager=" << s.manager.managerPath
          << "  list_controllers={"
          << "avail=" << s.controllers.serviceAvailable
          << " ok=" << s.controllers.callSucceeded
          << " count=" << s.controllers.controllers.size() << "}"
          << "  list_parameters={"
          << "avail=" << s.configuredControllers.serviceAvailable
          << " ok=" << s.configuredControllers.callSucceeded
          << " count=" << s.configuredControllers.controllers.size()
          << (s.configuredControllers.errorMessage.empty()
                ? ""
                : " err=\"" + s.configuredControllers.errorMessage + "\"")
          << "}\n";
    QVariantMap card;
    card[QStringLiteral("managerPath")]    = QString::fromStdString(s.manager.managerPath);
    // Display label for empty (root) namespace.  We keep two fields so QML can
    // show a friendly label without losing the literal value.
    const QString rawNs = s.manager.modelNamespace.empty()
        ? QString()
        : QString::fromStdString(s.manager.modelNamespace);
    card[QStringLiteral("modelNamespace")]      = rawNs;
    card[QStringLiteral("modelNamespaceLabel")] =
        rawNs.isEmpty() ? QStringLiteral("<root>") : rawNs;

    int active = 0, inactive = 0, other = 0;
    for (const auto &c : s.controllers.controllers)
    {
      if (c.state == "active")        ++active;
      else if (c.state == "inactive") ++inactive;
      else                            ++other;
    }
    card[QStringLiteral("loadedCount")]   = static_cast<int>(s.controllers.controllers.size());
    card[QStringLiteral("activeCount")]   = active;
    card[QStringLiteral("inactiveCount")] = inactive;
    card[QStringLiteral("otherCount")]    = other;

    card[QStringLiteral("controllers")] =
        controllersToVariantList(s.controllers.controllers);
    card[QStringLiteral("controllersAvailable")] = s.controllers.serviceAvailable;
    card[QStringLiteral("controllersOk")]        = s.controllers.callSucceeded;
    card[QStringLiteral("controllersError")]     =
        QString::fromStdString(s.controllers.errorMessage);

    card[QStringLiteral("hwInterfacesAvailable")] = s.hwInterfaces.serviceAvailable;
    card[QStringLiteral("hwInterfacesOk")]        = s.hwInterfaces.callSucceeded;
    card[QStringLiteral("hwCommandInterfaces")] =
        hwInterfacesToVariantList(s.hwInterfaces.commandInterfaces);
    card[QStringLiteral("hwStateInterfaces")] =
        hwInterfacesToVariantList(s.hwInterfaces.stateInterfaces);

    card[QStringLiteral("hwComponentsAvailable")] = s.hwComponents.serviceAvailable;
    card[QStringLiteral("hwComponentsOk")]        = s.hwComponents.callSucceeded;
    card[QStringLiteral("hwComponents")] =
        hwComponentsToVariantList(s.hwComponents.components);

    // Configured-but-not-loaded controllers (parameter-service discovery).
    card[QStringLiteral("configuredAvailable")] = s.configuredControllers.serviceAvailable;
    card[QStringLiteral("configuredOk")]        = s.configuredControllers.callSucceeded;
    card[QStringLiteral("configuredError")]     =
        QString::fromStdString(s.configuredControllers.errorMessage);
    card[QStringLiteral("configuredNotLoadedControllers")] =
        configuredControllersToVariantList(
            s.configuredControllers.controllers,
            s.controllers.controllers);
    // Convenience for QML: a load_controller example using the manager path.
    card[QStringLiteral("loadControllerHint")] =
        QString("ros2 control load_controller --set-state inactive "
                "<controller_name> -c %1")
            .arg(QString::fromStdString(s.manager.managerPath));

    QString hwSummary;
    if (s.hwComponents.serviceAvailable && s.hwComponents.callSucceeded)
    {
      hwSummary = QString("%1 hardware component(s)")
                      .arg(static_cast<int>(s.hwComponents.components.size()));
    }
    if (s.hwInterfaces.serviceAvailable && s.hwInterfaces.callSucceeded)
    {
      const int nCmd = static_cast<int>(s.hwInterfaces.commandInterfaces.size());
      const int nSt  = static_cast<int>(s.hwInterfaces.stateInterfaces.size());
      const QString prefix = hwSummary.isEmpty() ? QString() : QStringLiteral("  •  ");
      hwSummary += QStringLiteral("%1%2 cmd / %3 state interfaces")
                       .arg(prefix).arg(nCmd).arg(nSt);
    }
    card[QStringLiteral("hardwareSummary")] = hwSummary;

    managerCards_.append(card);
  }

  if (snaps.empty())
  {
    setStatus(QStringLiteral(
        "No ROS 2 controller_manager instances were found."));
  }
  else
  {
    int unreachable = 0;
    for (const auto &s : snaps)
      if (!s.controllers.serviceAvailable || !s.controllers.callSucceeded)
        ++unreachable;
    if (unreachable == 0)
    {
      setStatus(QString("Discovered %1 controller_manager instance(s).")
                    .arg(static_cast<int>(snaps.size())));
    }
    else
    {
      setStatus(QString("Discovered %1 controller_manager(s); %2 with unreachable list_controllers.")
                    .arg(static_cast<int>(snaps.size()))
                    .arg(unreachable));
    }
  }

  emit controllerManagersChanged();
}

// ============================================================================
// Activate / deactivate
// ============================================================================

void Ros2ControlManagerGui::activateController(
    const QString &managerPath, const QString &controllerName)
{
  runSwitchAsync(managerPath, controllerName, /*activate=*/true);
}

void Ros2ControlManagerGui::deactivateController(
    const QString &managerPath, const QString &controllerName)
{
  runSwitchAsync(managerPath, controllerName, /*activate=*/false);
}

void Ros2ControlManagerGui::runSwitchAsync(
    QString managerPath, QString controllerName, bool activate)
{
  const QString verb = activate ? QStringLiteral("Activating")
                                : QStringLiteral("Deactivating");
  if (!tryBeginOperation(QString("%1 '%2'…").arg(verb, controllerName)))
    return;

  Ros2ControlManagerGui *self = this;
  auto client = client_;
  const std::string mgr  = managerPath.toStdString();
  const std::string ctrl = controllerName.toStdString();

  const auto serviceTimeout = ControllerManagerClient::kSwitchServiceTimeout;
  const auto clientTimeout  = ControllerManagerClient::kSwitchClientTimeout;

  gzdbg << "[gz_ros2_control_manager] "
        << (activate ? "activate" : "deactivate") << " '" << ctrl << "' on "
        << mgr << " (STRICT, server_timeout=" << serviceTimeout.count() << "s, "
        << "client_wait=" << clientTimeout.count() << "ms)\n";

  auto future = QtConcurrent::run(
      [self, client, mgr, ctrl, activate, serviceTimeout, clientTimeout]()
  {
    SwitchControllerResult result;
    if (client && !self->shuttingDown_.load())
    {
      result = activate
          ? client->activateController(mgr, ctrl, serviceTimeout, clientTimeout)
          : client->deactivateController(mgr, ctrl, serviceTimeout, clientTimeout);
    }

    const QString verb = activate ? QStringLiteral("Activate")
                                  : QStringLiteral("Deactivate");
    QString status;
    if (!result.serviceAvailable)
    {
      status = QString("%1 failed: switch_controller service unavailable on %2")
                   .arg(verb, QString::fromStdString(mgr));
    }
    else if (result.clientTimedOut)
    {
      status = QString("%1 of '%2' timed out client-side after %3 ms; the "
                       "controller_manager may still finish the transition. "
                       "Refreshing.")
                   .arg(verb, QString::fromStdString(ctrl))
                   .arg(static_cast<qlonglong>(clientTimeout.count()));
    }
    else if (!result.callSucceeded)
    {
      status = QString("%1 failed: %2")
                   .arg(verb, QString::fromStdString(result.errorMessage));
    }
    else if (!result.ok)
    {
      QString msg = QString::fromStdString(result.errorMessage);
      if (msg.isEmpty()) msg = QStringLiteral("controller_manager returned ok=false");
      status = QString("%1 of '%2' refused: %3")
                   .arg(verb, QString::fromStdString(ctrl), msg);
    }
    else
    {
      status = QString("%1 of '%2' succeeded")
                   .arg(verb, QString::fromStdString(ctrl));
    }

    QMetaObject::invokeMethod(self,
      [self, status]()
      {
        if (self->shuttingDown_.load())
        {
          self->endOperation();
          return;
        }
        self->setStatus(status);
        // Release the operation slot before triggering the post-switch
        // refresh so refresh() can claim it.  We refresh on every terminal
        // path (success / refused / failed / timed out) so the UI reflects
        // the controller_manager's final view of the world even if our
        // client gave up early.
        self->endOperation();
        self->refresh();
      }, Qt::QueuedConnection);
  });
  futures_.addFuture(future);
}

// ============================================================================
// Configure (already-loaded, unconfigured controller)
// ============================================================================

void Ros2ControlManagerGui::configureController(
    const QString &managerPath, const QString &controllerName)
{
  if (controllerName.trimmed().isEmpty())
  {
    setStatus(QStringLiteral("Configure rejected: controller name is empty"));
    return;
  }
  runConfigureAsync(managerPath, controllerName);
}

void Ros2ControlManagerGui::runConfigureAsync(
    QString managerPath, QString controllerName)
{
  if (!tryBeginOperation(
          QString("Configuring '%1'…").arg(controllerName)))
    return;

  Ros2ControlManagerGui *self = this;
  auto client = client_;
  const std::string mgr  = managerPath.toStdString();
  const std::string ctrl = controllerName.toStdString();

  gzdbg << "[gz_ros2_control_manager] configure '" << ctrl
        << "' on " << mgr << '\n';

  auto future = QtConcurrent::run(
      [self, client, mgr, ctrl]()
  {
    ConfigureControllerResult result;
    if (client && !self->shuttingDown_.load())
      result = client->configureController(mgr, ctrl);

    QString status;
    if (!result.serviceAvailable)
    {
      status = QString("Configure of '%1' failed: configure_controller "
                       "service unavailable on %2")
                   .arg(QString::fromStdString(ctrl),
                        QString::fromStdString(mgr));
    }
    else if (result.clientTimedOut)
    {
      status = QString("Configure of '%1' timed out (client-side).")
                   .arg(QString::fromStdString(ctrl));
    }
    else if (!result.callSucceeded)
    {
      status = QString("Configure of '%1' failed: %2")
                   .arg(QString::fromStdString(ctrl),
                        QString::fromStdString(result.errorMessage));
    }
    else if (!result.ok)
    {
      status = QString("Configure of '%1' refused by controller_manager.")
                   .arg(QString::fromStdString(ctrl));
    }
    else
    {
      status = QString("Configure of '%1' succeeded → inactive.")
                   .arg(QString::fromStdString(ctrl));
    }

    QMetaObject::invokeMethod(self,
      [self, status]()
      {
        if (self->shuttingDown_.load()) { self->endOperation(); return; }
        self->setStatus(status);
        self->endOperation();
        self->refresh();
      }, Qt::QueuedConnection);
  });
  futures_.addFuture(future);
}

// ============================================================================
// Load inactive (configured-but-not-loaded: load_controller → configure_controller)
// ============================================================================

void Ros2ControlManagerGui::loadControllerInactive(
    const QString &managerPath, const QString &controllerName)
{
  if (controllerName.trimmed().isEmpty())
  {
    setStatus(QStringLiteral("Load rejected: controller name is empty"));
    return;
  }
  runLoadInactiveAsync(managerPath, controllerName);
}

void Ros2ControlManagerGui::runLoadInactiveAsync(
    QString managerPath, QString controllerName)
{
  if (!tryBeginOperation(
          QString("Loading '%1'…").arg(controllerName)))
    return;

  Ros2ControlManagerGui *self = this;
  auto client = client_;
  const std::string mgr  = managerPath.toStdString();
  const std::string ctrl = controllerName.toStdString();

  gzdbg << "[gz_ros2_control_manager] load-inactive '" << ctrl
        << "' on " << mgr << '\n';

  auto future = QtConcurrent::run(
      [self, client, mgr, ctrl]()
  {
    QString status;

    // -- Phase 1: load_controller --
    LoadControllerResult loadResult;
    if (client && !self->shuttingDown_.load())
      loadResult = client->loadController(mgr, ctrl);

    if (!loadResult.serviceAvailable)
    {
      status = QString("Load of '%1' failed: load_controller service "
                       "unavailable on %2")
                   .arg(QString::fromStdString(ctrl),
                        QString::fromStdString(mgr));
      QMetaObject::invokeMethod(self,
        [self, status]()
        {
          if (self->shuttingDown_.load()) { self->endOperation(); return; }
          self->setStatus(status);
          self->endOperation();
          self->refresh();
        }, Qt::QueuedConnection);
      return;
    }
    if (loadResult.clientTimedOut)
    {
      status = QString("Load of '%1' timed out (client-side).")
                   .arg(QString::fromStdString(ctrl));
      QMetaObject::invokeMethod(self,
        [self, status]()
        {
          if (self->shuttingDown_.load()) { self->endOperation(); return; }
          self->setStatus(status);
          self->endOperation();
          self->refresh();
        }, Qt::QueuedConnection);
      return;
    }
    if (!loadResult.callSucceeded || !loadResult.ok)
    {
      status = QString("Failed to load '%1': %2")
                   .arg(QString::fromStdString(ctrl),
                        QString::fromStdString(loadResult.errorMessage));
      QMetaObject::invokeMethod(self,
        [self, status]()
        {
          if (self->shuttingDown_.load()) { self->endOperation(); return; }
          self->setStatus(status);
          self->endOperation();
          self->refresh();
        }, Qt::QueuedConnection);
      return;
    }

    // -- Phase 2: configure_controller --
    if (self->shuttingDown_.load())
    {
      self->endOperation();
      return;
    }

    // Update status to "Configuring..." while we do the second call.
    QMetaObject::invokeMethod(self,
      [self, ctrl]()
      {
        if (!self->shuttingDown_.load())
          self->setStatus(QString("Configuring '%1'…")
                              .arg(QString::fromStdString(ctrl)));
      }, Qt::QueuedConnection);

    ConfigureControllerResult cfgResult;
    if (client && !self->shuttingDown_.load())
      cfgResult = client->configureController(mgr, ctrl);

    if (!cfgResult.serviceAvailable)
    {
      status = QString("'%1' loaded but configure_controller service "
                       "unavailable on %2")
                   .arg(QString::fromStdString(ctrl),
                        QString::fromStdString(mgr));
    }
    else if (cfgResult.clientTimedOut)
    {
      status = QString("'%1' loaded; configure timed out (client-side).")
                   .arg(QString::fromStdString(ctrl));
    }
    else if (!cfgResult.callSucceeded || !cfgResult.ok)
    {
      status = QString("'%1' loaded; configure failed: %2")
                   .arg(QString::fromStdString(ctrl),
                        QString::fromStdString(cfgResult.errorMessage));
    }
    else
    {
      status = QString("'%1' loaded as inactive.")
                   .arg(QString::fromStdString(ctrl));
    }

    QMetaObject::invokeMethod(self,
      [self, status]()
      {
        if (self->shuttingDown_.load()) { self->endOperation(); return; }
        self->setStatus(status);
        self->endOperation();
        self->refresh();
      }, Qt::QueuedConnection);
  });
  futures_.addFuture(future);
}

}  // namespace gz_ros2_control_manager

GZ_ADD_PLUGIN(gz_ros2_control_manager::Ros2ControlManagerGui,
              gz::gui::Plugin)
