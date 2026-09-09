import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: panel
  color: "#16242e"
  anchors.fill: parent
  Layout.minimumWidth: 280
  Layout.minimumHeight: 340

  // Parsed form of Scoreboard.payload. Named `board` rather than `model` so it
  // does not shadow ListView's own `model` property below.
  //
  // `run` stays null when nothing is publishing a run state - the simulator
  // launched with lifecycle:=false - and the clock row hides itself.
  //
  // `speedMode` is the same story for the demo gamepad's gear: null unless that
  // node is on the graph, and the row draws nothing when it is. A competition
  // entry has no gears, so this stays empty for a scored run.
  property var board: ({ total: 0, entries: [], run: null, speedMode: null })

  function refresh() {
    try {
      var parsed = JSON.parse(Scoreboard.payload)
      panel.board = {
        total: parsed.total !== undefined ? parsed.total : 0,
        entries: parsed.entries !== undefined ? parsed.entries : [],
        run: parsed.run !== undefined ? parsed.run : null,
        speedMode: parsed.speed_mode !== undefined ? parsed.speed_mode : null
      }
    } catch (e) {
      // A malformed payload should leave the last good score on screen rather
      // than blanking the panel mid-run.
    }
  }

  // Seconds as mm:ss. The run limit is minutes long, so hours never appear.
  // Guarded against a missing field so an older scorer that does not publish
  // `elapsed` shows 00:00 rather than "NaN:NaN".
  function clock(seconds) {
    var whole = Math.max(0, Math.round(seconds || 0))
    var minutes = Math.floor(whole / 60)
    var rest = whole % 60
    return (minutes < 10 ? "0" : "") + minutes + ":" + (rest < 10 ? "0" : "") + rest
  }

  function running() {
    return panel.board.run !== null && panel.board.run.state === "RUNNING"
  }

  // Cold to hot, so the pill reads from across a room before the word does.
  // Anything unrecognised falls through to the neutral grey rather than
  // colouring a gear this panel has not been taught about.
  function modeColor(mode) {
    if (mode === "SLOW")   return "#5aa9d6"
    if (mode === "NORMAL") return "#79d17a"
    if (mode === "FAST")   return "#ffd54f"
    if (mode === "TURBO")  return "#ff8a4c"
    return "#7d9cad"
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

    // TIME and SCORE side by side, both large. The challenge is most points in
    // least time, so the two numbers that decide it get the top of the panel
    // and a size that reads from across a table.
    RowLayout {
      Layout.fillWidth: true
      spacing: 14
      visible: panel.board.run !== null

      ColumnLayout {
        spacing: 0
        Text {
          text: "TIME"
          color: "#7d9cad"
          font.pixelSize: 11
          font.letterSpacing: 2
        }
        Text {
          // Counts UP. `remaining` is still what the run manager enforces, but
          // a stopwatch is what the challenge is actually about, and a number
          // going up is what a ten-year-old reads as "your time".
          text: panel.board.run ? panel.clock(panel.board.run.elapsed) : "00:00"
          color: panel.running() ? "#79d17a"
                 : (panel.board.run && panel.board.run.state === "FINISHED"
                    ? "#ffd54f" : "#7d9cad")
          font.pixelSize: 38
          font.bold: true
          // Tabular-ish: without this the whole row twitches every time the
          // digits change width.
          font.family: "monospace"
        }
      }

      Item { Layout.fillWidth: true }

      ColumnLayout {
        spacing: 0
        Text {
          text: "SCORE"
          color: "#7d9cad"
          font.pixelSize: 11
          font.letterSpacing: 2
          Layout.alignment: Qt.AlignRight
        }
        Text {
          id: totalText
          text: panel.board.total
          color: panel.board.total < 0 ? "#ff7b72" : "#ffd54f"
          font.pixelSize: 38
          font.bold: true
          Layout.alignment: Qt.AlignRight
        }
      }
    }

    // State, and the run manager's own countdown kept as a small second line -
    // it is still the thing that ends the run, so hiding it entirely would be
    // a surprise when the run stops on its own.
    RowLayout {
      Layout.fillWidth: true
      visible: panel.board.run !== null

      Text {
        text: panel.board.run ? panel.board.run.state : ""
        color: panel.running() ? "#79d17a" : "#7d9cad"
        font.pixelSize: 12
        font.bold: true
      }

      Item { Layout.fillWidth: true }

      Text {
        text: panel.board.run ? panel.clock(panel.board.run.remaining) + " left" : ""
        // Red under a minute: the run manager is about to end the run whether
        // or not the vehicle is finished.
        color: panel.board.run && panel.board.run.remaining <= 60 ? "#ff7b72" : "#6b8494"
        font.pixelSize: 12
      }
    }

    // --- speed mode -------------------------------------------------------
    // The gamepad's gear, for the crowd as much as the driver: at a stand the
    // person on the pad has been told they are in TURBO, and everyone watching
    // can see why the vehicle is suddenly quick. Filled pill rather than plain
    // text because it is a state, not a reading - it should look like a switch
    // position from the back of the room.
    Rectangle {
      visible: panel.board.speedMode !== null
      Layout.fillWidth: true
      Layout.preferredHeight: 30
      radius: 4
      color: "#1d3340"
      border.width: 1
      border.color: panel.modeColor(panel.board.speedMode)

      RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 8
        anchors.rightMargin: 8
        spacing: 8

        Text {
          text: "SPEED"
          color: "#7d9cad"
          font.pixelSize: 11
          font.letterSpacing: 2
        }

        Item { Layout.fillWidth: true }

        Text {
          text: panel.board.speedMode ? panel.board.speedMode : ""
          color: panel.modeColor(panel.board.speedMode)
          font.pixelSize: 17
          font.bold: true
          font.letterSpacing: 1
        }
      }
    }

    // Score with no lifecycle running: no timer to show, so the old big number
    // stands on its own exactly as it did before.
    Text {
      visible: panel.board.run === null
      text: panel.board.total
      color: panel.board.total < 0 ? "#ff7b72" : "#ffd54f"
      font.pixelSize: 46
      font.bold: true
    }

    // --- reset ------------------------------------------------------------
    // Teleports the vehicle back to the start, clears the score and puts the
    // clock back to zero. That is run_manager's own reset, reached through the
    // same /simulator/run_control path a competitor uses - the panel is not
    // allowed to do anything a competitor could not.
    Button {
      id: resetButton
      Layout.fillWidth: true
      Layout.preferredHeight: 40
      text: resetButton.confirming ? "TAP AGAIN TO RESET" : "RESET"

      // Two taps, because at a stand this button is under a child's thumb and
      // a stray press mid-run would wipe a good score with no way back. The arm
      // lapses after three seconds so it cannot sit armed unnoticed.
      property bool confirming: false

      // Everything below addresses the button by id rather than through
      // `parent`. background and contentItem happen to have the control as
      // their parent, but Timer is not a visual item and has no parent at all -
      // `parent.confirming` there is undefined, and the arm would never lapse.
      background: Rectangle {
        color: resetButton.confirming ? "#8a3a34"
               : (resetButton.hovered ? "#2b4351" : "#22323d")
        border.color: resetButton.confirming ? "#ff7b72" : "#2b4351"
        border.width: 1
        radius: 4
      }
      contentItem: Text {
        text: resetButton.text
        color: resetButton.confirming ? "#ffd7d4" : "#d6e5ee"
        font.pixelSize: 14
        font.bold: true
        font.letterSpacing: 1
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
      }

      Timer {
        id: armTimer
        interval: 3000
        onTriggered: resetButton.confirming = false
      }

      onClicked: {
        if (resetButton.confirming) {
          resetButton.confirming = false
          armTimer.stop()
          Scoreboard.SendRunControl("reset")
        } else {
          resetButton.confirming = true
          armTimer.restart()
        }
      }
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
