#ifndef AUV_GUI__AUVTELEOP_HH_
#define AUV_GUI__AUVTELEOP_HH_

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <QString>

#include <chrono>
#include <map>
#include <mutex>
#include <set>

namespace auv_gui
{

/// \brief Teleop panel: hold-to-move keyboard flying for the AUV.
///
/// The reason this plugin exists rather than reusing Gazebo's KeyPublisher is
/// key RELEASE. KeyPublisher only ever publishes presses, so a teleop built on
/// it cannot tell "still holding W" from "let go of W half a second ago", which
/// forces a setpoint-stepping design. Here we install an event filter on the
/// main window and see both edges, so we can publish the set of keys that are
/// held RIGHT NOW - and the vehicle can stop the instant you let go.
///
/// Two event paths have to be covered. When the 3D viewport has focus, keys
/// arrive as gz::gui::events::KeyPressOnScene / KeyReleaseOnScene; when any
/// other widget has focus they arrive as plain QKeyEvent. Both are handled.
///
/// The held set is published as JSON at a fixed rate, not just on change, so the
/// consumer can freeze the vehicle if the stream ever dies (panel closed,
/// Gazebo hung) instead of running away on a stale command.
///
/// Same shape as Scoreboard - QML owns the layout, C++ only moves strings on
/// and off Gazebo Transport.
class AuvTeleop : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Latest status JSON published by the teleop node.
  Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)

  /// \brief Space-separated names of the keys currently held, for display.
  Q_PROPERTY(QString heldKeys READ HeldKeys NOTIFY HeldKeysChanged)

  /// \brief Whether keyboard capture is armed.
  Q_PROPERTY(bool capturing READ Capturing WRITE SetCapturing NOTIFY CapturingChanged)

public:
  AuvTeleop();
  ~AuvTeleop() override = default;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  bool eventFilter(QObject * _obj, QEvent * _event) override;

  QString Status() const;
  QString HeldKeys() const;
  bool Capturing() const;
  void SetCapturing(bool _capturing);

  /// \brief Send a JSON command to the teleop node. Called from QML.
  Q_INVOKABLE void Send(const QString & _json);

signals:
  void StatusChanged();
  void HeldKeysChanged();
  void CapturingChanged();

private slots:
  /// \brief Publish the held-key set. Runs on a timer so the consumer gets a
  /// heartbeat it can time out on, and so releases can be de-bounced.
  void OnTick();

private:
  void OnStatus(const gz::msgs::StringMsg & _msg);

  /// \brief Record a key edge. Releases are held back by a short grace period
  /// because the scene event path carries no auto-repeat flag, and X11
  /// auto-repeat can present itself as release-then-press.
  void KeyEdge(int _key, bool _pressed, bool _shift, bool _control, bool _alt);

  /// \brief True when a text field has focus, so typing does not fly the AUV.
  static bool TypingSomewhere();

  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  gz::transport::Node::Publisher keysPub;

  mutable std::mutex mutex;
  QString status{"{}"};
  QString heldKeysText;

  std::set<int> held;
  /// \brief Keys whose release is pending, with the deadline it takes effect.
  std::map<int, std::chrono::steady_clock::time_point> releasing;
  bool shift{false};
  bool control{false};
  bool alt{false};
  bool capturing{true};
  uint64_t seq{0};

  /// \brief Grace period before a release is believed, in milliseconds.
  int releaseGraceMs{45};
};

}  // namespace auv_gui

#endif  // AUV_GUI__AUVTELEOP_HH_
