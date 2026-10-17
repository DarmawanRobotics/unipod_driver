#include "unipod_driver/unipod_driver_node.hpp"

#include <functional>
#include <stdexcept>

namespace unipod_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using Clock = std::chrono::steady_clock;

namespace
{
std::string streamName(uint8_t s)
{
  static const char * names[] = {"zoom", "wide", "thermal", "zoom+thermal", "wide+thermal",
    "zoom+wide", "none"};
  return s <= kStreamNone ? names[s] : "unknown(" + std::to_string(s) + ")";
}
}  // namespace

UnipodDriverNode::UnipodDriverNode(const rclcpp::NodeOptions & options)
: Node("unipod_driver", options)
{
  declareParameters();

  const auto type = get_parameter("transport").as_string();
  auto transport = makeTransport(
    type, get_parameter("ip").as_string(),
    static_cast<uint16_t>(get_parameter("port").as_int()),
    get_parameter("serial_device").as_string(),
    static_cast<uint32_t>(get_parameter("serial_baudrate").as_int()));
  if (!transport) {
    throw std::invalid_argument("transport must be udp, tcp or serial, not '" + type + "'");
  }

  UnipodClient::Options opts;
  opts.timeout = std::chrono::milliseconds(get_parameter("request_timeout_ms").as_int());
  client_ = std::make_unique<UnipodClient>(std::move(transport), opts);

  attitude_pub_ = create_publisher<msg::GimbalAttitude>("attitude", 10);
  status_pub_ = create_publisher<msg::GimbalStatus>("status", 10);
  temperature_pub_ = create_publisher<msg::ThermalData>("thermal/temperature", 10);

  client_->onLog([this](const std::string & m) {RCLCPP_INFO(get_logger(), "%s", m.c_str());});
  client_->onAttitude([this](const Attitude & a) {
      msg::GimbalAttitude m;
      m.header.stamp = now();
      m.header.frame_id = frame_id_;
      m.yaw = a.yaw;
      m.pitch = a.pitch;
      m.roll = a.roll;
      m.yaw_rate = a.yaw_rate;
      m.pitch_rate = a.pitch_rate;
      m.roll_rate = a.roll_rate;
      attitude_pub_->publish(m);
    });
  client_->onFrameTemperature([this](const Temperature & t) {temperature_pub_->publish(toMsg(t));});

  request_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ptz_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = ptz_group_;
  ptz_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "ptz_cmd", 10, std::bind(&UnipodDriverNode::onPtzCmd, this, _1), sub_opts);
  ptz_timer_ = create_wall_timer(50ms, std::bind(&UnipodDriverNode::ptzWatchdog, this), ptz_group_);

  const auto qos = rmw_qos_profile_services_default;
  set_angle_srv_ = create_service<srv::SetGimbalAngle>(
    "set_angle",
    [this](const std::shared_ptr<srv::SetGimbalAngle::Request> req,
    std::shared_ptr<srv::SetGimbalAngle::Response> res) {
      auto a = client_->setAngle(req->yaw, req->pitch);
      res->success = a.has_value();
      if (a) {
        res->yaw = a->yaw;
        res->pitch = a->pitch;
        res->roll = a->roll;
      }
      res->message = a ? "ok" : "no reply from the pod";
    }, qos, request_group_);

  recenter_srv_ = create_service<srv::Recenter>(
    "recenter",
    [this](const std::shared_ptr<srv::Recenter::Request> req,
    std::shared_ptr<srv::Recenter::Response> res) {
      const uint8_t mode = req->mode == 0 ? srv::Recenter::Request::RESET : req->mode;
      if (mode > srv::Recenter::Request::FACE_DOWN) {
        res->success = false;
        res->message = "mode must be 1..4";
        return;
      }
      res->success = client_->recenter(static_cast<CenterMode>(mode));
      res->message = res->success ? "ok" : "rejected or no reply";
    }, qos, request_group_);

  zoom_srv_ = create_service<srv::ZoomControl>(
    "zoom",
    [this](const std::shared_ptr<srv::ZoomControl::Request> req,
    std::shared_ptr<srv::ZoomControl::Response> res) {
      auto z = client_->zoom(req->command);
      res->success = z.has_value();
      res->zoom = z.value_or(0.0f);
      res->message = z ? "ok" : "no reply from the pod";
    }, qos, request_group_);

  set_zoom_srv_ = create_service<srv::SetZoom>(
    "set_zoom",
    [this](const std::shared_ptr<srv::SetZoom::Request> req,
    std::shared_ptr<srv::SetZoom::Response> res) {
      float zoom_max;
      {
        std::lock_guard<std::mutex> lk(state_mtx_);
        zoom_max = status_.zoom_max;
      }
      if (zoom_max > 0.0f && (req->zoom < 1.0f || req->zoom > zoom_max)) {
        res->success = false;
        res->zoom_max = zoom_max;
        res->message = "zoom must be 1.0 .. " + std::to_string(zoom_max);
        return;
      }
      res->success = client_->setZoom(req->zoom);
      res->zoom = client_->getZoom().value_or(0.0f);
      res->zoom_max = zoom_max;
      res->message = res->success ? "ok" : "rejected or no reply";
    }, qos, request_group_);

  video_mode_srv_ = create_service<srv::SetVideoMode>(
    "set_video_mode",
    [this](const std::shared_ptr<srv::SetVideoMode::Request> req,
    std::shared_ptr<srv::SetVideoMode::Response> res) {
      auto m = client_->setVideoMode(req->main_stream, req->sub_stream);
      res->success = m && m->main == req->main_stream && m->sub == req->sub_stream;
      if (m) {
        res->main_stream = m->main;
        res->sub_stream = m->sub;
        res->message = "main " + streamName(m->main) + ", sub " + streamName(m->sub);
      } else {
        res->message = "no reply from the pod";
      }
    }, qos, request_group_);

  temperature_srv_ = create_service<srv::GetTemperature>(
    "get_temperature",
    [this](const std::shared_ptr<srv::GetTemperature::Request> req,
    std::shared_ptr<srv::GetTemperature::Response> res) {
      std::optional<Temperature> t;
      switch (req->mode) {
        case srv::GetTemperature::Request::POINT:
          t = client_->pointTemperature(req->x, req->y);
          break;
        case srv::GetTemperature::Request::REGION:
          t = client_->regionTemperature(req->x, req->y, req->x2, req->y2);
          break;
        case srv::GetTemperature::Request::FRAME:
          t = client_->frameTemperature();
          break;
        default:
          res->success = false;
          res->message = "mode must be POINT (0), REGION (1) or FRAME (2)";
          return;
      }
      res->success = t.has_value();
      if (t) {
        res->data = toMsg(*t);
      }
      res->message = t ? "ok" : "no reply from the pod";
    }, qos, request_group_);

  thermal_srv_ = create_service<srv::ThermalControl>(
    "thermal_control",
    [this](const std::shared_ptr<srv::ThermalControl::Request> req,
    std::shared_ptr<srv::ThermalControl::Response> res) {
      using R = srv::ThermalControl::Request;
      std::optional<uint8_t> v;
      switch (req->command) {
        case R::SET_PALETTE:
          if (req->value > kGloryHot || req->value == 1) {
            res->message = "palette must be 0 or 2..11";
            return;
          }
          v = client_->setPalette(req->value);
          break;
        case R::SET_GAIN:
          v = client_->setThermalGain(req->value);
          break;
        case R::REFRESH_SHUTTER:
          v = client_->refreshShutter() ? std::optional<uint8_t>(1) : std::nullopt;
          break;
        default:
          res->message = "command must be SET_PALETTE (0), SET_GAIN (1) or REFRESH_SHUTTER (2)";
          return;
      }
      res->success = v.has_value();
      res->value = v.value_or(0);
      res->message = v ? "ok" : "rejected or no reply";
    }, qos, request_group_);

  const double status_hz = get_parameter("status_rate_hz").as_double();
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / (status_hz > 0.0 ? status_hz : 1.0)),
    std::bind(&UnipodDriverNode::statusTick, this), request_group_);

  RCLCPP_INFO(get_logger(), "connecting to the UniPod over %s", client_->describe().c_str());
  client_->start();
}

UnipodDriverNode::~UnipodDriverNode()
{
  if (client_) {
    client_->rotate(0, 0);
    client_->stop();
  }
}

void UnipodDriverNode::declareParameters()
{
  declare_parameter("transport", "tcp");
  declare_parameter("ip", "192.168.144.25");
  declare_parameter("port", 37260);
  declare_parameter("serial_device", "/dev/ttyUSB0");
  declare_parameter("serial_baudrate", 115200);
  declare_parameter("request_timeout_ms", 500);
  frame_id_ = declare_parameter("frame_id", "unipod_gimbal");
  thermal_frame_id_ = declare_parameter("thermal_frame_id", "unipod_thermal");
  declare_parameter("status_rate_hz", 1.0);
  attitude_rate_ = nearestStreamRate(declare_parameter("attitude_rate_hz", 10.0));
  ptz_max_speed_ = static_cast<int>(declare_parameter("ptz_max_speed", 100));
  ptz_timeout_sec_ = declare_parameter("ptz_cmd_timeout", 0.5);
  invert_yaw_ = declare_parameter("invert_yaw", false);
  invert_pitch_ = declare_parameter("invert_pitch", false);
  main_stream_ = static_cast<int>(declare_parameter("main_stream", -1));
  sub_stream_ = static_cast<int>(declare_parameter("sub_stream", -1));
  thermal_palette_ = static_cast<int>(declare_parameter("thermal_palette", -1));
  thermal_gain_ = static_cast<int>(declare_parameter("thermal_gain", -1));
  temperature_stream_ = declare_parameter("temperature_stream", true);
}

msg::ThermalData UnipodDriverNode::toMsg(const Temperature & t)
{
  msg::ThermalData m;
  m.header.stamp = now();
  m.header.frame_id = thermal_frame_id_;
  m.mode = static_cast<uint8_t>(t.mode);
  m.x1 = t.x1;
  m.y1 = t.y1;
  m.x2 = t.x2;
  m.y2 = t.y2;
  m.temp_max = t.max_c;
  m.temp_min = t.min_c;
  m.max_x = t.max_x;
  m.max_y = t.max_y;
  m.min_x = t.min_x;
  m.min_y = t.min_y;
  return m;
}

// Runs on every link-up (first connect and after the pod rebooted or the cable came back).
void UnipodDriverNode::onLinkUp()
{
  if (auto fw = client_->getFirmware()) {
    RCLCPP_INFO(get_logger(), "UniPod firmware: camera %s, gimbal %s%s", fw->camera.c_str(),
      fw->gimbal.c_str(),
      fw->camera == "v0.0.0" ? " (camera still booting, up to 30 s)" : "");
    std::lock_guard<std::mutex> lk(state_mtx_);
    status_.camera_firmware = fw->camera;
    status_.gimbal_firmware = fw->gimbal;
  }
  if (auto zmax = client_->getMaxZoom()) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    status_.zoom_max = *zmax;
  }
  if (attitude_rate_ != kRateOff && !client_->setAttitudeStream(attitude_rate_)) {
    RCLCPP_WARN(get_logger(), "attitude stream request not acknowledged");
  }
  if (main_stream_ >= 0 && sub_stream_ >= 0) {
    auto m = client_->setVideoMode(
      static_cast<uint8_t>(main_stream_), static_cast<uint8_t>(sub_stream_));
    if (m && m->main == main_stream_ && m->sub == sub_stream_) {
      RCLCPP_INFO(get_logger(), "video: main %s, sub %s", streamName(m->main).c_str(),
        streamName(m->sub).c_str());
    } else {
      RCLCPP_WARN(get_logger(), "video mode main %d / sub %d not taken (pod reports %s / %s)",
        main_stream_, sub_stream_, m ? streamName(m->main).c_str() : "-",
        m ? streamName(m->sub).c_str() : "-");
    }
  }
  if (thermal_palette_ >= 0) {
    client_->setPalette(static_cast<uint8_t>(thermal_palette_));
  }
  if (thermal_gain_ >= 0) {
    client_->setThermalGain(static_cast<uint8_t>(thermal_gain_));
  }
  client_->setFrameTemperatureStream(temperature_stream_);

  for (uint8_t s : {uint8_t{1}, uint8_t{2}}) {
    if (auto e = client_->getEncoding(s)) {
      RCLCPP_INFO(get_logger(), "rtsp video%u: %s %ux%u %u fps %u kbps", s,
        e->codec == 2 ? "H265" : "H264", e->width, e->height, e->fps, e->bitrate_kbps);
    }
  }
}

void UnipodDriverNode::statusTick()
{
  const bool up = client_->linkUp();
  if (up && !was_up_) {
    onLinkUp();
  } else if (!up && was_up_) {
    RCLCPP_WARN(get_logger(), "lost the UniPod on %s", client_->describe().c_str());
  }
  was_up_ = up;

  msg::GimbalStatus s;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    s = status_;
  }
  if (up) {
    if (auto z = client_->getZoom()) {s.zoom = *z;}
    if (auto m = client_->getVideoMode()) {
      s.main_stream = m->main;
      s.sub_stream = m->sub;
    }
    if (auto p = client_->getPalette()) {s.thermal_palette = *p;}
    if (auto g = client_->getThermalGain()) {s.thermal_gain = *g;}
  }
  s.connected = up;
  s.header.stamp = now();
  s.header.frame_id = frame_id_;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    status_ = s;
  }
  status_pub_->publish(s);
}

void UnipodDriverNode::onPtzCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto yaw = axisToSpeed((invert_yaw_ ? -1.0 : 1.0) * msg->angular.z, ptz_max_speed_);
  const auto pitch = axisToSpeed((invert_pitch_ ? -1.0 : 1.0) * msg->angular.y, ptz_max_speed_);
  const auto zoom = axisToZoom(msg->linear.z);
  {
    std::lock_guard<std::mutex> lk(ptz_mtx_);
    last_ptz_cmd_ = Clock::now();
  }
  sendPtz(yaw, pitch, zoom);
}

void UnipodDriverNode::sendPtz(int8_t yaw, int8_t pitch, int8_t zoom)
{
  std::lock_guard<std::mutex> lk(ptz_mtx_);
  if (yaw != last_yaw_ || pitch != last_pitch_ || yaw != 0 || pitch != 0) {
    client_->rotate(yaw, pitch);
  }
  if (zoom != last_zoom_) {
    client_->send(protocol::cmd::kManualZoom, protocol::Writer().i8(zoom).bytes());
  }
  last_yaw_ = yaw;
  last_pitch_ = pitch;
  last_zoom_ = zoom;
}

void UnipodDriverNode::ptzWatchdog()
{
  bool moving;
  Clock::time_point last;
  {
    std::lock_guard<std::mutex> lk(ptz_mtx_);
    moving = last_yaw_ != 0 || last_pitch_ != 0 || last_zoom_ != 0;
    last = last_ptz_cmd_;
  }
  if (moving && Clock::now() - last > std::chrono::duration<double>(ptz_timeout_sec_)) {
    RCLCPP_INFO(get_logger(), "no ptz_cmd for %.2f s, stopping", ptz_timeout_sec_);
    sendPtz(0, 0, 0);
  }
}

}  // namespace unipod_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<unipod_driver::UnipodDriverNode>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("unipod_driver"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
