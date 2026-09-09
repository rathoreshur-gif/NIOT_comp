#ifndef AUV_GUI__SCOREBOARD_HH_
#define AUV_GUI__SCOREBOARD_HH_

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <QString>

#include <mutex>

namespace auv_gui
{

/// \brief Competition scoreboard, drawn as a docked panel in the Gazebo window.
///
/// The panel is deliberately dumb in one direction: the scorer publishes a JSON
/// document on a Gazebo Transport topic and this plugin hands the raw string to
/// QML, which parses and lays it out. Adding a FIELD to the scoreboard therefore
/// means editing the scorer and the QML, never this class.
///
/// The reset button is the one thing that goes the other way. A button has to
/// send something, so this class also advertises a command topic. It stays a
/// thin pipe: QML decides when to send and what word to send, and the string is
/// forwarded verbatim. run_manager on the ROS side is what actually re-poses
/// the vehicle and clears the score.
class Scoreboard : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Raw JSON from the scorer. QML parses this.
  Q_PROPERTY(QString payload READ Payload NOTIFY PayloadChanged)

public:
  Scoreboard();
  ~Scoreboard() override = default;

  /// \brief Read the <topic> setting out of the world's <gui> block.
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  /// \brief Latest JSON document received, or "{}" before the first message.
  QString Payload() const;

  /// \brief Send a run-lifecycle command: "start", "end" or "reset".
  ///
  /// Q_INVOKABLE so the QML button can call it directly. The string crosses to
  /// ROS through the parameter_bridge entry for this topic and lands on
  /// /simulator/run_control, which run_manager already handles - the same path
  /// a competitor's own reset takes, so the button cannot do anything a
  /// competitor could not.
  Q_INVOKABLE void SendRunControl(const QString & _command);

signals:
  void PayloadChanged();

private:
  /// \brief Transport callback. Runs on a transport thread, not the GUI thread.
  void OnScore(const gz::msgs::StringMsg & _msg);

  gz::transport::Node node;

  /// \brief Outbound run-lifecycle commands. Default-constructed until
  /// LoadConfig advertises it, so it is checked before every publish.
  gz::transport::Node::Publisher controlPub;

  mutable std::mutex mutex;
  QString payload{"{}"};
};

}  // namespace auv_gui

#endif  // AUV_GUI__SCOREBOARD_HH_
