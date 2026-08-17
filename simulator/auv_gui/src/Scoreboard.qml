import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: panel
  color: "#16242e"
  anchors.fill: parent
  Layout.minimumWidth: 260
  Layout.minimumHeight: 240

  // Parsed form of Scoreboard.payload. Named `board` rather than `model` so it
  // does not shadow ListView's own `model` property below.
  property var board: ({ total: 0, entries: [] })

  function refresh() {
    try {
      var parsed = JSON.parse(Scoreboard.payload)
      panel.board = {
        total: parsed.total !== undefined ? parsed.total : 0,
        entries: parsed.entries !== undefined ? parsed.entries : []
      }
    } catch (e) {
      // A malformed payload should leave the last good score on screen rather
      // than blanking the panel mid-run.
    }
  }

  Connections {
    target: Scoreboard
    function onPayloadChanged() { panel.refresh() }
  }

  Component.onCompleted: panel.refresh()

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 6

    Text {
      text: "COMPETITION SCORE"
      color: "#7d9cad"
      font.pixelSize: 11
      font.letterSpacing: 2
    }

    Text {
      id: totalText
      text: panel.board.total
      color: panel.board.total < 0 ? "#ff7b72" : "#ffd54f"
      font.pixelSize: 46
      font.bold: true
    }

    Rectangle { Layout.fillWidth: true; height: 1; color: "#2b4351" }

    Text {
      visible: panel.board.entries.length === 0
      text: "No events yet."
      color: "#6b8494"
      font.pixelSize: 12
      font.italic: true
    }

    ListView {
      Layout.fillWidth: true
      Layout.fillHeight: true
      clip: true
      spacing: 4
      model: panel.board.entries

      delegate: RowLayout {
        width: ListView.view.width
        spacing: 6

        Text {
          text: modelData.label
          color: "#d6e5ee"
          font.pixelSize: 13
          Layout.fillWidth: true
          elide: Text.ElideRight
        }
        Text {
          text: modelData.count > 1 ? "x" + modelData.count : ""
          color: "#6b8494"
          font.pixelSize: 12
        }
        Text {
          text: (modelData.points > 0 ? "+" : "") + modelData.points
          color: modelData.points >= 0 ? "#79d17a" : "#ff7b72"
          font.pixelSize: 13
          font.bold: true
        }
      }
    }
  }
}
