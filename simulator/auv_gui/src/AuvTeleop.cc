#include "AuvTeleop.hh"

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

#include <QGuiApplication>
#include <QKeyEvent>
#include <QObject>
#include <QTimer>

#include <sstream>
#include <string>
#include <vector>

namespace auv_gui
{

namespace
{

/// \brief Human-readable name for the keys we care about, for the panel.
std::string KeyName(int _key)
{
  switch (_key) {
    case Qt::Key_W: return "W";
    case Qt::Key_A: return "A";
    case Qt::Key_S: return "S";
    case Qt::Key_D: return "D";
    case Qt::Key_Q: return "Q";
    case Qt::Key_E: return "E";
    case Qt::Key_R: return "R";
    case Qt::Key_F: return "F";
    case Qt::Key_Space: return "SPACE";
    default: return "";
  }
}

}  // namespace

AuvTeleop::AuvTeleop()
: gz::gui::Plugin()
{
}

void AuvTeleop::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Teleop";
  }

  std::string statusTopic{"/teleop/status"};
  std::string commandTopic{"/teleop/command"};
  std::string keysTopic{"/teleop/keys"};
  if (_pluginElem != nullptr) {
    auto elem = _pluginElem->FirstChildElement("status_topic");
    if (elem != nullptr && elem->GetText() != nullptr) {
      statusTopic = elem->GetText();
    }
    elem = _pluginElem->FirstChildElement("command_topic");
    if (elem != nullptr && elem->GetText() != nullptr) {
      commandTopic = elem->GetText();
    }
    elem = _pluginElem->FirstChildElement("keys_topic");
    if (elem != nullptr && elem->GetText() != nullptr) {
      keysTopic = elem->GetText();
    }
    elem = _pluginElem->FirstChildElement("release_grace_ms");
    if (elem != nullptr) {
      elem->QueryIntText(&this->releaseGraceMs);
    }
  }

  this->pub = this->node.Advertise<gz::msgs::StringMsg>(commandTopic);
  this->keysPub = this->node.Advertise<gz::msgs::StringMsg>(keysTopic);
  if (!this->node.Subscribe(statusTopic, &AuvTeleop::OnStatus, this)) {
    gzerr << "Teleop panel could not subscribe to [" << statusTopic << "]" << std::endl;
  }

  // Key events reach the main window whichever widget has focus, including the
  // 3D viewport (which re-broadcasts them as scene events).
  if (gz::gui::App() != nullptr) {
    auto * window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
    if (window != nullptr) {
      window->installEventFilter(this);
    } else {
      gzerr << "Teleop panel found no main window; keyboard capture is off."
            << std::endl;
    }
  }

  auto * timer = new QTimer(this);
  this->connect(timer, &QTimer::timeout, this, &AuvTeleop::OnTick);
  timer->start(20);   // 50 Hz: fast enough that a release is felt immediately

  gzmsg << "Teleop panel on [" << statusTopic << "] -> [" << commandTopic
        << "], keys on [" << keysTopic << "]" << std::endl;
}

// -- keyboard ---------------------------------------------------------------

bool AuvTeleop::TypingSomewhere()
{
  auto * focus = QGuiApplication::focusObject();
  if (focus == nullptr) {
    return false;
  }
  // Covers QML TextField / TextArea / SpinBox editors and their widget
  // equivalents, which is everywhere in Gazebo a user might type a number.
  return focus->inherits("QQuickTextInput") || focus->inherits("QQuickTextEdit") ||
         focus->inherits("QLineEdit") || focus->inherits("QTextEdit");
}

bool AuvTeleop::eventFilter(QObject * _obj, QEvent * _event)
{
  if (this->capturing && !TypingSomewhere()) {
    if (_event->type() == QEvent::KeyPress || _event->type() == QEvent::KeyRelease) {
      auto * key = static_cast<QKeyEvent *>(_event);
      // Auto-repeat presses carry no new information: we already know the key
      // is down, and believing an auto-repeat release would stutter the drive.
      if (!key->isAutoRepeat()) {
        this->KeyEdge(
          key->key(), _event->type() == QEvent::KeyPress,
          (key->modifiers() & Qt::ShiftModifier) != 0,
          (key->modifiers() & Qt::ControlModifier) != 0,
          (key->modifiers() & Qt::AltModifier) != 0);
      }
    } else if (_event->type() == gz::gui::events::KeyPressOnScene::kType) {
      auto key = static_cast<gz::gui::events::KeyPressOnScene *>(_event)->Key();
      this->KeyEdge(key.Key(), true, key.Shift(), key.Control(), key.Alt());
    } else if (_event->type() == gz::gui::events::KeyReleaseOnScene::kType) {
      auto key = static_cast<gz::gui::events::KeyReleaseOnScene *>(_event)->Key();
      this->KeyEdge(key.Key(), false, key.Shift(), key.Control(), key.Alt());
    }
  }
  // Never consume: Gazebo's own shortcuts and text entry must keep working.
  return QObject::eventFilter(_obj, _event);
}

void AuvTeleop::KeyEdge(int _key, bool _pressed, bool _shift, bool _control, bool _alt)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->shift = _shift;
  this->control = _control;
  this->alt = _alt;

  if (_pressed) {
    this->releasing.erase(_key);
    this->held.insert(_key);
  } else {
    this->releasing[_key] = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(this->releaseGraceMs);
  }
}

void AuvTeleop::OnTick()
{
  std::ostringstream json;
  std::ostringstream names;
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    auto now = std::chrono::steady_clock::now();
    for (auto it = this->releasing.begin(); it != this->releasing.end(); ) {
      if (now >= it->second) {
        this->held.erase(it->first);
        it = this->releasing.erase(it);
      } else {
        ++it;
      }
    }

    json << "{\"keys\":[";
    bool first = true;
    for (int key : this->held) {
      json << (first ? "" : ",") << key;
      first = false;
      auto name = KeyName(key);
      if (!name.empty()) {
        names << (names.tellp() > 0 ? " " : "") << name;
      }
    }
    json << "],\"shift\":" << (this->shift ? "true" : "false")
         << ",\"ctrl\":" << (this->control ? "true" : "false")
         << ",\"alt\":" << (this->alt ? "true" : "false")
         << ",\"capture\":" << (this->capturing ? "true" : "false")
         << ",\"seq\":" << ++this->seq << "}";
  }

  gz::msgs::StringMsg msg;
  msg.set_data(json.str());
  this->keysPub.Publish(msg);

  auto text = QString::fromStdString(names.str());
  if (text != this->heldKeysText) {
    this->heldKeysText = text;
    emit this->HeldKeysChanged();
  }
}

// -- properties -------------------------------------------------------------

QString AuvTeleop::Status() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->status;
}

QString AuvTeleop::HeldKeys() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->heldKeysText;
}

bool AuvTeleop::Capturing() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->capturing;
}

void AuvTeleop::SetCapturing(bool _capturing)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->capturing == _capturing) {
      return;
    }
    this->capturing = _capturing;
    if (!_capturing) {
      this->held.clear();
      this->releasing.clear();
    }
  }
  emit this->CapturingChanged();
}

void AuvTeleop::Send(const QString & _json)
{
  gz::msgs::StringMsg msg;
  msg.set_data(_json.toStdString());
  this->pub.Publish(msg);
}

void AuvTeleop::OnStatus(const gz::msgs::StringMsg & _msg)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->status = QString::fromStdString(_msg.data());
  }
  emit this->StatusChanged();
}

}  // namespace auv_gui

GZ_ADD_PLUGIN(auv_gui::AuvTeleop, gz::gui::Plugin)
