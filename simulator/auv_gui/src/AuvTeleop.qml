import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: panel
  color: "#16242e"
  anchors.fill: parent
  Layout.minimumWidth: 300
  Layout.minimumHeight: 380

  property var st: ({ enabled: false, engaged: false, mode: "normal", gear: 1.0,
                      surge_cmd: 0, sway_cmd: 0, yaw_cmd: 0, speed: 0,
                      depth: 0, holding: false, source: "none",
                      cruise_speed: 0.6, cruise_yaw: 1.0 })

  function refresh() {
    try {
      var parsed = JSON.parse(AuvTeleop.status)
      if (parsed.enabled !== undefined) panel.st = parsed
    } catch (e) {
      // keep the last good status rather than blanking the panel
    }
  }

  Connections {
    target: AuvTeleop
    function onStatusChanged() { panel.refresh() }
  }
  Component.onCompleted: panel.refresh()

  function modeColor(m) {
    if (m === "boost") return "#ff8a65"
    if (m === "precision") return "#64b5f6"
    return "#d6e5ee"
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 7

    RowLayout {
      Layout.fillWidth: true
      Text {
        text: "KEYBOARD TELEOP"
        color: "#7d9cad"; font.pixelSize: 11; font.letterSpacing: 2
        Layout.fillWidth: true
      }
      Rectangle {
        width: 10; height: 10; radius: 5
        color: panel.st.enabled ? (panel.st.engaged ? "#79d17a" : "#ffd54f") : "#5b6b76"
      }
    }

    RowLayout {
      Layout.fillWidth: true
      Switch {
        id: enableSwitch
        text: panel.st.enabled ? "Enabled" : "Disabled"
        checked: panel.st.enabled
        onToggled: AuvTeleop.Send(JSON.stringify({ enabled: checked }))
        contentItem: Text {
          text: enableSwitch.text
          color: "#d6e5ee"; font.pixelSize: 13
          leftPadding: enableSwitch.indicator.width + 6
          verticalAlignment: Text.AlignVCenter
        }
      }
      Item { Layout.fillWidth: true }
      Switch {
        id: captureSwitch
        text: "Keys"
        checked: AuvTeleop.capturing
        onToggled: AuvTeleop.capturing = checked
        contentItem: Text {
          text: captureSwitch.text
          color: AuvTeleop.capturing ? "#79d17a" : "#7d9cad"; font.pixelSize: 13
          leftPadding: captureSwitch.indicator.width + 6
          verticalAlignment: Text.AlignVCenter
        }
      }
    }

    // Live picture of what is held down right now - the whole point of this
    // panel is that release is visible, so show it.
    Rectangle {
      Layout.fillWidth: true
      height: 30
      radius: 3
      color: AuvTeleop.heldKeys.length > 0 ? "#1d3a2a" : "#12202a"
      border.width: 1
      border.color: AuvTeleop.heldKeys.length > 0 ? "#3e7d55" : "#2b4351"
      Text {
        anchors.centerIn: parent
        text: AuvTeleop.heldKeys.length > 0 ? AuvTeleop.heldKeys : "— frozen —"
        color: AuvTeleop.heldKeys.length > 0 ? "#8ce29a" : "#6b8494"
        font.pixelSize: 13; font.letterSpacing: 2; font.bold: true
      }
    }

    GridLayout {
      columns: 2
      Layout.fillWidth: true
      columnSpacing: 10
      rowSpacing: 2

      Text { text: "mode"; color: "#7d9cad"; font.pixelSize: 12 }
      Text {
        text: panel.st.mode + "  ×" + panel.st.gear.toFixed(2)
        color: panel.modeColor(panel.st.mode); font.pixelSize: 12; font.bold: true
      }
      Text { text: "speed"; color: "#7d9cad"; font.pixelSize: 12 }
      Text {
        text: panel.st.speed.toFixed(2) + " m/s   (cmd "
              + panel.st.surge_cmd.toFixed(2) + " / " + panel.st.sway_cmd.toFixed(2) + ")"
        color: "#d6e5ee"; font.pixelSize: 12
      }
      Text { text: "yaw"; color: "#7d9cad"; font.pixelSize: 12 }
      Text { text: panel.st.yaw_cmd.toFixed(2) + " rad/s"; color: "#d6e5ee"; font.pixelSize: 12 }
      Text { text: "depth"; color: "#7d9cad"; font.pixelSize: 12 }
      Text { text: panel.st.depth.toFixed(2) + " m"; color: "#d6e5ee"; font.pixelSize: 12 }
      Text { text: "station"; color: "#7d9cad"; font.pixelSize: 12 }
      Text {
        text: panel.st.holding ? "frozen" : "flying"
        color: panel.st.holding ? "#79d17a" : "#ffd54f"; font.pixelSize: 12
      }
      Text { text: "input"; color: "#7d9cad"; font.pixelSize: 12 }
      Text {
        text: panel.st.source
        color: panel.st.source === "none" ? "#e57373" : "#d6e5ee"; font.pixelSize: 12
      }
    }

    Rectangle { Layout.fillWidth: true; height: 1; color: "#2b4351" }

    Text {
      text: "cruise  " + panel.st.cruise_speed.toFixed(2) + " m/s"
      color: "#7d9cad"; font.pixelSize: 12
    }
    Slider {
      Layout.fillWidth: true
      from: 0.05; to: 1.60; value: panel.st.cruise_speed
      onMoved: AuvTeleop.Send(JSON.stringify({ cruise_speed: value }))
    }

    Text {
      text: "cruise yaw  " + panel.st.cruise_yaw.toFixed(2) + " rad/s"
      color: "#7d9cad"; font.pixelSize: 12
    }
    Slider {
      Layout.fillWidth: true
      from: 0.10; to: 2.50; value: panel.st.cruise_yaw
      onMoved: AuvTeleop.Send(JSON.stringify({ cruise_yaw: value }))
    }

    RowLayout {
      Layout.fillWidth: true
      spacing: 8
      Button {
        Layout.fillWidth: true
        text: "FREEZE"
        onClicked: AuvTeleop.Send(JSON.stringify({ stop: true }))
      }
      Button {
        Layout.fillWidth: true
        text: "CUT THRUST"
        onClicked: AuvTeleop.Send(JSON.stringify({ cut: true }))
      }
    }

    Text {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      color: "#6b8494"; font.pixelSize: 11
      text: "Hold to move, release to freeze.\n" +
            "W/S surge · A/D sway · Q/E yaw · R/F depth\n" +
            "SHIFT boost · CTRL precision · Z/C gear\n" +
            "SPACE re-anchor · X cut · T toggle"
    }

    Item { Layout.fillHeight: true }
  }
}
