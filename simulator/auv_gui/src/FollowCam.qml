import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

// Chase-camera controls. Four hit targets and a readout, sized for a mouse at a
// noisy stand rather than for a developer who knows where everything is.
Rectangle {
  id: panel
  color: "#16242e"
  anchors.fill: parent
  Layout.minimumWidth: 260
  Layout.minimumHeight: 96

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10
    spacing: 8

    RowLayout {
      Layout.fillWidth: true
      spacing: 8

      Text {
        text: FollowCam.following ? "FOLLOWING" : "FREE"
        color: FollowCam.following ? "#79d17a" : "#ffd54f"
        font.pixelSize: 13
        font.bold: true
        font.letterSpacing: 1
      }

      Item { Layout.fillWidth: true }

      Text {
        text: FollowCam.distance + " m"
        color: "#7d9cad"
        font.pixelSize: 13
        font.family: "monospace"
      }
    }

    RowLayout {
      Layout.fillWidth: true
      spacing: 6

      // Zoom OUT is the bigger number: the offset is scaled by this, so >1
      // moves the camera further back.
      Button {
        text: "−"
        Layout.preferredWidth: 44
        Layout.preferredHeight: 32
        ToolTip.visible: hovered
        ToolTip.text: "Zoom out - pull the camera back"
        onClicked: FollowCam.Send("zoom_out")
      }

      Button {
        text: "+"
        Layout.preferredWidth: 44
        Layout.preferredHeight: 32
        ToolTip.visible: hovered
        ToolTip.text: "Zoom in - move the camera closer"
        onClicked: FollowCam.Send("zoom_in")
      }

      Item { Layout.fillWidth: true }

      // One button that means "put it back": whatever the mouse has done to
      // the view, this returns to third person behind the vehicle.
      Button {
        text: FollowCam.following ? "Free look" : "Third person"
        Layout.preferredHeight: 32
        onClicked: FollowCam.Send(FollowCam.following ? "free" : "follow")
      }
    }
  }
}
