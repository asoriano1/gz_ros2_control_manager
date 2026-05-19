import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// Context property: controlManager → Ros2ControlManagerGui (C++)
Rectangle {
  id: root
  color: "transparent"
  anchors.fill: parent
  implicitWidth: 380
  implicitHeight: 600
  Layout.minimumWidth: 320
  Layout.minimumHeight: 280
  Layout.preferredHeight: 600

  // Per-card collapsed state (default: expanded).  Stored outside the
  // Repeater delegates so it survives controllerManagers rebuilds triggered
  // by refresh().
  property var collapsedManagers: ({})

  function setManagerCollapsed(path, val) {
    var m = {}
    for (var k in collapsedManagers) m[k] = collapsedManagers[k]
    if (val) m[path] = true
    else     delete m[path]
    collapsedManagers = m
  }

  // Short display label for the state badge — keeps the badge narrow.
  function stateLabel(state) {
    if (state === "unconfigured") return "unconfig"
    return state.length > 0 ? state : "?"
  }

  function controllerStateColor(state) {
    if (state === "active")        return "#1b5e20"
    if (state === "inactive")      return "#1565c0"
    if (state === "unconfigured")  return "#6a1b9a"
    if (state === "finalized")     return "#424242"
    return "#b71c1c"
  }

  function controllerStateBg(state) {
    if (state === "active")        return "#e8f5e9"
    if (state === "inactive")      return "#e3f2fd"
    if (state === "unconfigured")  return "#f3e5f5"
    if (state === "finalized")     return "#eceff1"
    return "#ffebee"
  }

  ScrollView {
    id: mainScroll
    anchors.fill: parent
    contentWidth: availableWidth
    clip: true
    ScrollBar.vertical.policy: ScrollBar.AsNeeded

    ColumnLayout {
      id: mainCol
      width: mainScroll.availableWidth
      spacing: 6

      Item { implicitHeight: 2 }

      // ── 1. Header ─────────────────────────────────────────────────
      RowLayout {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true

        CheckBox {
          text: "Debug"
          font.pixelSize: 10; padding: 4
          checked: controlManager.debugMode
          onToggled: controlManager.setDebugMode(checked)
          ToolTip.visible: hovered
          ToolTip.text: "Show hardware interfaces / components details"
          ToolTip.delay: 400
        }

        CheckBox {
          text: "Auto"
          font.pixelSize: 10; padding: 4
          checked: controlManager.autoRefresh
          enabled: !controlManager.busy
          onToggled: controlManager.setAutoRefresh(checked)
          ToolTip.visible: hovered
          ToolTip.text: "Auto-refresh every ~3 s"
          ToolTip.delay: 400
        }

        Button {
          text: controlManager.busy ? "…" : "Refresh"
          enabled: !controlManager.busy
          implicitWidth: 70; font.pixelSize: 11
          onClicked: controlManager.refresh()
        }
      }

      // ── 2. Status bar ─────────────────────────────────────────────
      Rectangle {
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        implicitHeight: statusRow.implicitHeight + 14
        color: controlManager.controllerManagers.length > 0 ? "#e8f5e9" : "#fff3e0"
        radius: 4

        RowLayout {
          id: statusRow
          anchors {
            left: parent.left; right: parent.right
            verticalCenter: parent.verticalCenter
            leftMargin: 8; rightMargin: 8
          }
          spacing: 8

          BusyIndicator {
            running: controlManager.busy; visible: controlManager.busy
            width: 16; height: 16
          }

          Label {
            text: controlManager.statusText; font.pixelSize: 11
            color: controlManager.controllerManagers.length > 0 ? "#1b5e20" : "#e65100"
            elide: Text.ElideRight; Layout.fillWidth: true
            wrapMode: Text.Wrap
          }

          Label {
            visible: controlManager.lastRefreshTime.length > 0
            text: "@ " + controlManager.lastRefreshTime
            font.pixelSize: 10; color: "#616161"
          }
        }
      }

      // ── 3. Empty state ────────────────────────────────────────────
      Rectangle {
        visible: controlManager.controllerManagers.length === 0
        Layout.leftMargin: 10; Layout.rightMargin: 10
        Layout.fillWidth: true
        color: "#fff8e1"
        radius: 4
        border.color: "#f57f17"; border.width: 1
        implicitHeight: emptyCol.implicitHeight + 16

        ColumnLayout {
          id: emptyCol
          anchors {
            left: parent.left; right: parent.right
            verticalCenter: parent.verticalCenter
            leftMargin: 10; rightMargin: 10
          }
          spacing: 4

          Label {
            text: "No ROS 2 controller_manager instances were found."
            font.pixelSize: 11; font.bold: true; color: "#e65100"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }

          Label {
            text: "• Check that the model was spawned with gz_ros2_control."
            font.pixelSize: 10; color: "#5d4037"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }
          Label {
            text: "• Check that the model namespace matches the expected controller_manager namespace."
            font.pixelSize: 10; color: "#5d4037"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }
          Label {
            text: "• Check ROS_DOMAIN_ID."
            font.pixelSize: 10; color: "#5d4037"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }
          Label {
            text: "• Try: ros2 control list_controllers -c /<model_namespace>/controller_manager"
            font.pixelSize: 10; color: "#5d4037"; font.family: "monospace"
            wrapMode: Text.Wrap; Layout.fillWidth: true
          }
        }
      }

      // ── 4. Manager cards ──────────────────────────────────────────
      Repeater {
        model: controlManager.controllerManagers

        delegate: Rectangle {
          id: card
          Layout.leftMargin: 10; Layout.rightMargin: 10
          Layout.fillWidth: true
          color: "#ffffff"
          border.color: "#bdbdbd"; border.width: 1
          radius: 5
          implicitHeight: cardCol.implicitHeight + 14

          property var entry: modelData
          // Default: expanded.  User can collapse, and the choice is
          // remembered through refreshes via root.collapsedManagers.
          property bool expanded:
              root.collapsedManagers[entry.managerPath] !== true

          ColumnLayout {
            id: cardCol
            anchors {
              left: parent.left; right: parent.right
              top: parent.top
              leftMargin: 10; rightMargin: 10; topMargin: 7
            }
            spacing: 4

            // ── Card header ─────────────────────────────────────
            RowLayout {
              Layout.fillWidth: true
              spacing: 8

              ToolButton {
                text: card.expanded ? "▼" : "▶"
                font.pixelSize: 11
                implicitWidth: 24; padding: 0
                onClicked: root.setManagerCollapsed(
                    card.entry.managerPath, card.expanded)
              }

              ColumnLayout {
                Layout.fillWidth: true; spacing: 1

                Label {
                  text: card.entry.managerPath
                  font.pixelSize: 11; font.bold: true; font.family: "monospace"
                  color: "#0d47a1"
                  elide: Text.ElideMiddle; Layout.fillWidth: true
                }
                Label {
                  text: "namespace: " + card.entry.modelNamespaceLabel
                  font.pixelSize: 9; color: "#616161"
                  elide: Text.ElideMiddle; Layout.fillWidth: true
                }
              }

              // Counts pills
              Rectangle {
                color: "#e3f2fd"; radius: 3
                implicitHeight: 18; implicitWidth: countLabel.implicitWidth + 12
                Label {
                  id: countLabel
                  anchors.centerIn: parent
                  text: card.entry.loadedCount + " loaded"
                  font.pixelSize: 9; color: "#0d47a1"
                }
              }
              Rectangle {
                color: "#e8f5e9"; radius: 3
                visible: card.entry.activeCount > 0
                implicitHeight: 18; implicitWidth: activeLabel.implicitWidth + 12
                Label {
                  id: activeLabel
                  anchors.centerIn: parent
                  text: card.entry.activeCount + " active"
                  font.pixelSize: 9; color: "#1b5e20"
                }
              }
              Rectangle {
                color: "#fff3e0"; radius: 3
                visible: card.entry.inactiveCount > 0
                implicitHeight: 18; implicitWidth: inactiveLabel.implicitWidth + 12
                Label {
                  id: inactiveLabel
                  anchors.centerIn: parent
                  text: card.entry.inactiveCount + " inactive"
                  font.pixelSize: 9; color: "#e65100"
                }
              }
            }

            // ── Hardware summary ────────────────────────────────
            Label {
              visible: card.entry.hardwareSummary.length > 0
              text: card.entry.hardwareSummary
              font.pixelSize: 9; color: "#5d4037"
              Layout.fillWidth: true
              wrapMode: Text.Wrap
            }

            // ── Service-availability error banners ─────────────
            Label {
              visible: !card.entry.controllersAvailable
              text: "list_controllers service is not available on this controller_manager."
              font.pixelSize: 10; color: "#b71c1c"
              wrapMode: Text.Wrap; Layout.fillWidth: true
            }
            Label {
              visible: card.entry.controllersAvailable
                       && !card.entry.controllersOk
              text: "list_controllers call failed"
                    + (card.entry.controllersError.length > 0
                         ? ": " + card.entry.controllersError
                         : ".")
              font.pixelSize: 10; color: "#b71c1c"
              wrapMode: Text.Wrap; Layout.fillWidth: true
            }

            // ── Expanded body ────────────────────────────────────
            ColumnLayout {
              visible: card.expanded
              Layout.fillWidth: true
              spacing: 4

              Repeater {
                model: card.entry.controllers

                delegate: Rectangle {
                  Layout.fillWidth: true
                  implicitHeight: ctrlInner.implicitHeight + 8
                  color: index % 2 === 0 ? "#ffffff" : "#fafafa"

                  property var ctrl: modelData

                  ColumnLayout {
                    id: ctrlInner
                    anchors {
                      left: parent.left; right: parent.right
                      top: parent.top
                      leftMargin: 6; rightMargin: 6; topMargin: 4
                    }
                    spacing: 2

                    // ── Fila 1: nombre ──────────────────────────────
                    Label {
                      text: ctrl.name
                      font.pixelSize: 10; font.family: "monospace"
                      wrapMode: Text.Wrap
                      Layout.fillWidth: true
                      ToolTip.visible: nameHover.containsMouse
                      ToolTip.text: ctrl.name; ToolTip.delay: 400
                      MouseArea {
                        id: nameHover; anchors.fill: parent
                        hoverEnabled: true; acceptedButtons: Qt.NoButton
                      }
                    }

                    // ── Fila 2: tipo ────────────────────────────────
                    Label {
                      text: ctrl.type || "?"
                      font.pixelSize: 9; color: "#5d4037"
                      wrapMode: Text.Wrap
                      Layout.fillWidth: true
                      ToolTip.visible: typeHover.containsMouse
                      ToolTip.text: ctrl.type; ToolTip.delay: 400
                      MouseArea {
                        id: typeHover; anchors.fill: parent
                        hoverEnabled: true; acceptedButtons: Qt.NoButton
                      }
                    }

                    // ── Fila 3: badge de estado  •  botón de acción ─
                    RowLayout {
                      Layout.fillWidth: true
                      spacing: 6

                      Rectangle {
                        implicitWidth: stateLabel.implicitWidth + 12
                        implicitHeight: 20
                        radius: 3
                        color: root.controllerStateBg(ctrl.state)
                        Label {
                          id: stateLabel
                          anchors.centerIn: parent
                          text: root.stateLabel(ctrl.state)
                          font.pixelSize: 10; font.bold: true
                          color: root.controllerStateColor(ctrl.state)
                        }
                      }

                      Item { Layout.fillWidth: true }

                      Button {
                        visible: ctrl.isActionable
                        text: ctrl.actionLabel
                        enabled: !controlManager.busy && ctrl.isActionable
                        font.pixelSize: 10
                        padding: 4
                        implicitHeight: 28
                        ToolTip.visible: hovered
                        ToolTip.text: ctrl.actionLabel === "Configure"
                            ? ("configure_controller → " + ctrl.name)
                            : ctrl.actionActivates
                                ? ("switch_controller (STRICT) → activate "
                                   + ctrl.name)
                                : ("switch_controller (STRICT) → deactivate "
                                   + ctrl.name)
                        ToolTip.delay: 400
                        onClicked: {
                          if (ctrl.actionLabel === "Configure")
                            controlManager.configureController(
                                card.entry.managerPath, ctrl.name)
                          else if (ctrl.actionActivates)
                            controlManager.activateController(
                                card.entry.managerPath, ctrl.name)
                          else
                            controlManager.deactivateController(
                                card.entry.managerPath, ctrl.name)
                        }
                      }
                      Label {
                        visible: !ctrl.isActionable
                        text: "—"
                        font.pixelSize: 11; color: "#9e9e9e"
                        ToolTip.visible: noActionHover.containsMouse
                        ToolTip.text: "No safe action for state '" + ctrl.state + "'."
                        ToolTip.delay: 400
                        MouseArea {
                          id: noActionHover; anchors.fill: parent
                          hoverEnabled: true; acceptedButtons: Qt.NoButton
                        }
                      }
                    }
                  }
                }
              }

              // ── Empty controllers state — compact, collapsed by default ──
              Rectangle {
                id: noCtrlBlock
                visible: card.entry.controllersAvailable
                         && card.entry.controllersOk
                         && card.entry.controllers.length === 0
                Layout.fillWidth: true
                color: "#fff8e1"
                radius: 4
                border.color: "#f9a825"; border.width: 1
                implicitHeight: noCtrlContent.implicitHeight + 10

                property bool detailsExpanded: false

                ColumnLayout {
                  id: noCtrlContent
                  anchors {
                    left: parent.left; right: parent.right
                    top: parent.top
                    leftMargin: 8; rightMargin: 8; topMargin: 5
                  }
                  spacing: 3

                  // Always-visible summary row
                  RowLayout {
                    Layout.fillWidth: true
                    spacing: 6

                    Label {
                      text: "No controllers are currently loaded."
                      font.pixelSize: 10; font.bold: true; color: "#e65100"
                      elide: Text.ElideRight; Layout.fillWidth: true
                    }

                    Button {
                      text: noCtrlBlock.detailsExpanded ? "▴ hide" : "details ▸"
                      font.pixelSize: 9
                      implicitWidth: 68; implicitHeight: 20
                      padding: 2
                      onClicked: noCtrlBlock.detailsExpanded = !noCtrlBlock.detailsExpanded
                      ToolTip.visible: hovered
                      ToolTip.text: noCtrlBlock.detailsExpanded
                          ? "Hide help"
                          : "Show how to load a controller"
                      ToolTip.delay: 400
                    }
                  }

                  // Collapsible details
                  ColumnLayout {
                    visible: noCtrlBlock.detailsExpanded
                    Layout.fillWidth: true
                    spacing: 3

                    Label {
                      text: "• Activate / Deactivate buttons appear only after "
                          + "a controller is loaded."
                      font.pixelSize: 10; color: "#5d4037"
                      wrapMode: Text.Wrap; Layout.fillWidth: true
                    }
                    Label {
                      text: "• Use the “Configured (not loaded)” section "
                          + "below to load a configured controller."
                      font.pixelSize: 10; color: "#5d4037"
                      wrapMode: Text.Wrap; Layout.fillWidth: true
                    }
                    Label {
                      text: "• Or load via CLI:"
                      font.pixelSize: 10; color: "#5d4037"
                    }
                    Rectangle {
                      Layout.fillWidth: true
                      color: "#fffde7"; radius: 3
                      border.color: "#fbc02d"; border.width: 1
                      implicitHeight: hintLabel.implicitHeight + 8

                      Label {
                        id: hintLabel
                        anchors {
                          fill: parent
                          leftMargin: 6; rightMargin: 6
                          topMargin: 4; bottomMargin: 4
                        }
                        text: card.entry.loadControllerHint
                        font.pixelSize: 9; font.family: "monospace"
                        color: "#3e2723"; wrapMode: Text.Wrap
                      }
                    }
                  }
                }
              }

              // ── Configured-but-not-loaded controllers (parameter-svc) ──

              // Parameter service not reachable on this controller_manager.
              Label {
                visible: !card.entry.configuredAvailable
                text: "⚠ Parameter service not available on "
                      + card.entry.managerPath
                      + ".\nCheck the node is fully started and re-Refresh."
                font.pixelSize: 10; color: "#b71c1c"
                wrapMode: Text.Wrap; Layout.fillWidth: true
                Layout.topMargin: 4
                Layout.leftMargin: 4
              }

              // Parameter service available but the call itself failed.
              Label {
                visible: card.entry.configuredAvailable
                         && !card.entry.configuredOk
                text: "⚠ list_parameters failed"
                      + (card.entry.configuredError.length > 0
                           ? ": " + card.entry.configuredError
                           : ".")
                font.pixelSize: 10; color: "#b71c1c"
                wrapMode: Text.Wrap; Layout.fillWidth: true
                Layout.topMargin: 4
                Layout.leftMargin: 4
              }

              // Service OK but no configured controllers found (YAML not loaded?).
              Label {
                visible: card.entry.configuredAvailable
                         && card.entry.configuredOk
                         && card.entry.configuredNotLoadedControllers.length === 0
                         && card.entry.controllers.length === 0
                text: "ℹ No controllers are declared on this controller_manager.\n"
                      + "Check that the robot URDF references a valid <parameters> YAML."
                font.pixelSize: 10; color: "#5d4037"
                wrapMode: Text.Wrap; Layout.fillWidth: true
                Layout.topMargin: 4
                Layout.leftMargin: 4
              }

              ColumnLayout {
                visible: card.entry.configuredAvailable
                         && card.entry.configuredOk
                         && card.entry.configuredNotLoadedControllers.length > 0
                Layout.fillWidth: true; spacing: 2

                Label {
                  text: "Configured (not loaded)"
                  font.pixelSize: 9; font.bold: true; color: "#37474f"
                  Layout.topMargin: 6
                  ToolTip.visible: cfgHover.containsMouse
                  ToolTip.text: "Controllers declared on the controller_manager "
                              + "node parameters but not yet loaded. "
                              + "Use 'Load inactive' to load and configure."
                  ToolTip.delay: 400
                  MouseArea {
                    id: cfgHover; anchors.fill: parent
                    hoverEnabled: true; acceptedButtons: Qt.NoButton
                  }
                }
                Repeater {
                  model: card.entry.configuredNotLoadedControllers
                  delegate: Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: cfgInner.implicitHeight + 8
                    color: "#f3f4f6"

                    property var cfg: modelData

                    ColumnLayout {
                      id: cfgInner
                      anchors {
                        left: parent.left; right: parent.right
                        top: parent.top
                        leftMargin: 6; rightMargin: 6; topMargin: 4
                      }
                      spacing: 2

                      // ── Fila 1: nombre ──────────────────────────────
                      Label {
                        text: cfg.name
                        font.pixelSize: 10; font.family: "monospace"
                        color: "#37474f"
                        elide: Text.ElideMiddle
                        Layout.fillWidth: true
                        ToolTip.visible: cfgNameHover.containsMouse
                        ToolTip.text: cfg.name; ToolTip.delay: 400
                        MouseArea {
                          id: cfgNameHover; anchors.fill: parent
                          hoverEnabled: true; acceptedButtons: Qt.NoButton
                        }
                      }

                      // ── Fila 2: tipo ────────────────────────────────
                      Label {
                        text: cfg.type
                        font.pixelSize: 9; color: "#5d4037"
                        wrapMode: Text.Wrap
                        Layout.fillWidth: true
                        ToolTip.visible: cfgTypeHover.containsMouse
                        ToolTip.text: cfg.type; ToolTip.delay: 400
                        MouseArea {
                          id: cfgTypeHover; anchors.fill: parent
                          hoverEnabled: true; acceptedButtons: Qt.NoButton
                        }
                      }

                      // ── Fila 3: botón Load ───────────────────────────
                      Button {
                        visible: cfg.isLoadable
                        text: "Load inactive"
                        enabled: !controlManager.busy && cfg.isLoadable
                        font.pixelSize: 10
                        padding: 4
                        Layout.alignment: Qt.AlignRight
                        implicitHeight: 28
                        ToolTip.visible: hovered
                        ToolTip.text: "Load controller as inactive: " + cfg.name
                        ToolTip.delay: 400
                        onClicked: controlManager.loadControllerInactive(
                            card.entry.managerPath, cfg.name)
                      }
                    }
                  }
                }
              }

              // ── Debug section: hardware components / interfaces ──
              ColumnLayout {
                visible: controlManager.debugMode
                Layout.fillWidth: true; spacing: 2

                Label {
                  visible: !card.entry.hwComponentsAvailable
                  text: "list_hardware_components service not available."
                  font.pixelSize: 9; color: "#9e9e9e"; font.italic: true
                  Layout.topMargin: 4; wrapMode: Text.Wrap; Layout.fillWidth: true
                }
                Label {
                  visible: card.entry.hwComponentsAvailable
                  text: "Hardware components"
                  font.pixelSize: 9; font.bold: true; color: "#37474f"
                  Layout.topMargin: 6
                }
                Repeater {
                  model: controlManager.debugMode
                         ? card.entry.hwComponents : []
                  delegate: Label {
                    text: "• " + modelData.name
                          + "  [" + modelData.state + "]"
                          + (modelData.type.length > 0 ? "  type=" + modelData.type : "")
                    font.pixelSize: 9; font.family: "monospace"
                    color: "#37474f"
                    Layout.fillWidth: true; wrapMode: Text.Wrap
                  }
                }

                Label {
                  visible: !card.entry.hwInterfacesAvailable
                  text: "list_hardware_interfaces service not available."
                  font.pixelSize: 9; color: "#9e9e9e"; font.italic: true
                  Layout.topMargin: 4; wrapMode: Text.Wrap; Layout.fillWidth: true
                }
                Label {
                  visible: card.entry.hwInterfacesAvailable
                  text: "Command interfaces ("
                        + card.entry.hwCommandInterfaces.length + ")"
                  font.pixelSize: 9; font.bold: true; color: "#37474f"
                  Layout.topMargin: 6
                }
                Repeater {
                  model: controlManager.debugMode
                         ? card.entry.hwCommandInterfaces : []
                  delegate: Label {
                    text: "• " + modelData.name
                          + (modelData.isClaimed ? "  [claimed]" : "")
                          + (modelData.isAvailable ? "" : "  [unavailable]")
                    font.pixelSize: 9; font.family: "monospace"
                    color: modelData.isClaimed ? "#0d47a1" : "#37474f"
                    Layout.fillWidth: true; wrapMode: Text.Wrap
                  }
                }

                Label {
                  visible: card.entry.hwInterfacesAvailable
                  text: "State interfaces ("
                        + card.entry.hwStateInterfaces.length + ")"
                  font.pixelSize: 9; font.bold: true; color: "#37474f"
                  Layout.topMargin: 6
                }
                Repeater {
                  model: controlManager.debugMode
                         ? card.entry.hwStateInterfaces : []
                  delegate: Label {
                    text: "• " + modelData.name
                          + (modelData.isAvailable ? "" : "  [unavailable]")
                    font.pixelSize: 9; font.family: "monospace"
                    color: "#37474f"
                    Layout.fillWidth: true; wrapMode: Text.Wrap
                  }
                }
              }
            }
          }
        }
      }

      Item { Layout.fillHeight: true; implicitHeight: 6 }
    }
  }
}
