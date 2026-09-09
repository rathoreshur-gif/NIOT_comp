#include "Scoreboard.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <string>

namespace auv_gui
{

Scoreboard::Scoreboard()
: gz::gui::Plugin()
{
}

void Scoreboard::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Scoreboard";
  }

  std::string topic{"/scoring/display"};
  std::string controlTopic{"/gui/run_control"};
  if (_pluginElem != nullptr) {
    auto topicElem = _pluginElem->FirstChildElement("topic");
    if (topicElem != nullptr && topicElem->GetText() != nullptr) {
      topic = topicElem->GetText();
    }
    auto controlElem = _pluginElem->FirstChildElement("control_topic");
    if (controlElem != nullptr && controlElem->GetText() != nullptr) {
      controlTopic = controlElem->GetText();
    }
  }

  // Advertised even when nothing is listening: with lifecycle:=false there is
  // no run_manager, and the button then publishes into the void rather than
  // failing to construct the panel.
  this->controlPub = this->node.Advertise<gz::msgs::StringMsg>(controlTopic);
  if (!this->controlPub) {
    gzerr << "Scoreboard could not advertise [" << controlTopic
          << "]; the reset button will do nothing." << std::endl;
  }

  if (!this->node.Subscribe(topic, &Scoreboard::OnScore, this)) {
    gzerr << "Scoreboard could not subscribe to [" << topic << "]" << std::endl;
    return;
  }
  gzmsg << "Scoreboard listening on [" << topic << "]" << std::endl;
}

QString Scoreboard::Payload() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->payload;
}

void Scoreboard::SendRunControl(const QString & _command)
{
  if (!this->controlPub) {
    gzerr << "Scoreboard has no run-control publisher; ignoring ["
          << _command.toStdString() << "]." << std::endl;
    return;
  }

  gz::msgs::StringMsg msg;
  msg.set_data(_command.toStdString());
  this->controlPub.Publish(msg);
  gzmsg << "Scoreboard sent run_control [" << _command.toStdString() << "]"
        << std::endl;
}

void Scoreboard::OnScore(const gz::msgs::StringMsg & _msg)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->payload = QString::fromStdString(_msg.data());
  }
  // Queued across to the GUI thread by Qt, since this runs on a transport thread.
  emit this->PayloadChanged();
}

}  // namespace auv_gui

GZ_ADD_PLUGIN(auv_gui::Scoreboard, gz::gui::Plugin)
