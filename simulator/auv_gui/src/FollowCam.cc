#include "FollowCam.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <string>

namespace auv_gui
{

FollowCam::FollowCam()
: gz::gui::Plugin()
{
}

void FollowCam::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Camera";
  }

  std::string commandTopic{"/gui/camera_control"};
  std::string statusTopic{"/gui/camera_status"};
  if (_pluginElem != nullptr) {
    auto * commandElem = _pluginElem->FirstChildElement("command_topic");
    if (commandElem != nullptr && commandElem->GetText() != nullptr) {
      commandTopic = commandElem->GetText();
    }
    auto * statusElem = _pluginElem->FirstChildElement("status_topic");
    if (statusElem != nullptr && statusElem->GetText() != nullptr) {
      statusTopic = statusElem->GetText();
    }
  }

  // Advertised even with no director listening: without one the buttons should
  // do nothing quietly, not stop the panel from being built.
  this->commandPub = this->node.Advertise<gz::msgs::StringMsg>(commandTopic);
  if (!this->commandPub) {
    gzerr << "FollowCam could not advertise [" << commandTopic
          << "]; the camera buttons will do nothing." << std::endl;
  }

  if (!this->node.Subscribe(statusTopic, &FollowCam::OnStatus, this)) {
    gzerr << "FollowCam could not subscribe to [" << statusTopic
          << "]; the readout will stay blank." << std::endl;
  }

  gzmsg << "FollowCam sending on [" << commandTopic << "], listening on ["
        << statusTopic << "]" << std::endl;
}

bool FollowCam::Following() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->following;
}

QString FollowCam::Distance() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->distance;
}

void FollowCam::Send(const QString & _command)
{
  if (!this->commandPub) {
    return;
  }
  gz::msgs::StringMsg msg;
  msg.set_data(_command.toStdString());
  this->commandPub.Publish(msg);
}

void FollowCam::OnStatus(const gz::msgs::StringMsg & _msg)
{
  // "<following|free> <distance>", space separated - the whole protocol. A
  // JSON blob would need a parser on this side for two fields.
  const std::string & data = _msg.data();
  const auto space = data.find(' ');
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->following = data.compare(0, space, "following") == 0;
    this->distance = (space == std::string::npos)
      ? QString("-")
      : QString::fromStdString(data.substr(space + 1));
  }
  // Queued across to the GUI thread by Qt: this runs on a transport thread.
  emit this->StateChanged();
}

}  // namespace auv_gui

GZ_ADD_PLUGIN(auv_gui::FollowCam, gz::gui::Plugin)
