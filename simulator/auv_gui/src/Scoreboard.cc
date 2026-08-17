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
  if (_pluginElem != nullptr) {
    auto topicElem = _pluginElem->FirstChildElement("topic");
    if (topicElem != nullptr && topicElem->GetText() != nullptr) {
      topic = topicElem->GetText();
    }
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
