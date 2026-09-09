#ifndef AUV_GUI__FOLLOWCAM_HH_
#define AUV_GUI__FOLLOWCAM_HH_

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <QString>

#include <mutex>
#include <string>

namespace auv_gui
{

/// \brief Chase-camera controls: lock the view behind the vehicle, and zoom.
///
/// Gazebo can already follow a model - the GUI advertises /gui/follow and
/// /gui/follow/offset - but it never starts on its own, so every session opened
/// on the world pose in robosub.sdf with the vehicle a speck at the far end of
/// the pool. At an outreach stand that is the difference between a game and a
/// puzzle.
///
/// WHY THIS PANEL DOES NOT CALL THOSE SERVICES ITSELF. It cannot. They are
/// advertised by the scene plugin inside this same process, and gz-transport
/// does not route a request from a node back to a service advertised by the
/// process it lives in: the request is simply never answered. Measured, not
/// assumed - with the GUI up, `gz service -s /gui/follow` from a shell answers
/// `data: true` immediately, while the identical request from here times out,
/// blocking or asynchronous, on the Qt thread or on a worker thread.
///
/// So this panel is a remote control, not a driver. It publishes a word on
/// `<command_topic>` and camera_director (auv_worlds/scripts) - a different
/// process, which therefore CAN reach the services - does the work and reports
/// back on `<status_topic>`. The same shape as the Scoreboard's reset button,
/// which publishes to a topic rather than calling run_manager directly.
class FollowCam : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Whether the director says the camera is locked on.
  Q_PROPERTY(bool following READ Following NOTIFY StateChanged)

  /// \brief Distance from the target, in metres, for the QML readout.
  Q_PROPERTY(QString distance READ Distance NOTIFY StateChanged)

public:
  FollowCam();
  ~FollowCam() override = default;

  /// \brief Read the two topic names out of the world's <gui> block.
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  bool Following() const;
  QString Distance() const;

  /// \brief Send one word to the director: "follow", "free", "zoom_in" or
  /// "zoom_out". Q_INVOKABLE so the QML buttons can call it directly.
  Q_INVOKABLE void Send(const QString & _command);

signals:
  void StateChanged();

private:
  /// \brief Status from the director: "<following|free> <distance>".
  void OnStatus(const gz::msgs::StringMsg & _msg);

  gz::transport::Node node;
  gz::transport::Node::Publisher commandPub;

  bool following{false};
  QString distance{"-"};
  mutable std::mutex mutex;
};

}  // namespace auv_gui

#endif  // AUV_GUI__FOLLOWCAM_HH_
