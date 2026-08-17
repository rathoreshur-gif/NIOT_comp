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
/// The panel is deliberately dumb: the scorer publishes a JSON document on a
/// Gazebo Transport topic and this plugin hands the raw string to QML, which
/// parses and lays it out. Adding a field to the scoreboard therefore means
/// editing the scorer and the QML, never this class - which is what makes it a
/// reasonable base for the other GUI panels to come.
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

signals:
  void PayloadChanged();

private:
  /// \brief Transport callback. Runs on a transport thread, not the GUI thread.
  void OnScore(const gz::msgs::StringMsg & _msg);

  gz::transport::Node node;
  mutable std::mutex mutex;
  QString payload{"{}"};
};

}  // namespace auv_gui

#endif  // AUV_GUI__SCOREBOARD_HH_
