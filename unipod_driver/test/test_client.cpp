// UnipodClient against an in-memory pod: request/ACK matching, decoding, streams, timeouts and
// reconnects.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "fake_pod.hpp"
#include "unipod_driver/client.hpp"

using namespace std::chrono_literals;
using unipod_driver::Attitude;
using unipod_driver::Temperature;
using unipod_driver::UnipodClient;
using unipod_driver::protocol::Frame;
using unipod_driver::protocol::Writer;
using unipod_driver::test::FakePodTransport;
using unipod_driver::test::PodState;
using Bytes = std::vector<uint8_t>;

namespace
{

template<typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds limit = 3000ms)
{
  const auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return pred();
}

class ClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pod = std::make_shared<PodState>();
    pod->responder = [this](const Frame & f) {return respond(f);};
    UnipodClient::Options o;
    o.timeout = 100ms;
    o.retries = 1;
    o.reconnect_period = 50ms;
    client = std::make_unique<UnipodClient>(std::make_unique<FakePodTransport>(pod), o);
  }

  void TearDown() override {client->stop();}

  void startAndWaitOpen()
  {
    client->start();
    ASSERT_TRUE(waitFor([&] {std::lock_guard<std::mutex> lk(pod->mtx); return pod->open;}));
  }

  // A pod that answers like the document says.
  std::optional<Bytes> respond(const Frame & f)
  {
    const auto & p = f.payload;
    switch (f.cmd) {
      case 0x01:  // camera 0x89010001 = v1.0.1, gimbal 0x8A00010A = v0.1.10, no zoom fw
        return Bytes{0x01, 0x00, 0x01, 0x89, 0x0A, 0x01, 0x00, 0x8A, 0, 0, 0, 0};
      case 0x05: return Writer().u16(25).bytes();        // 2.5x
      case 0x07: return Bytes{1};
      case 0x08: return Bytes{1};
      case 0x0D: return Writer().i16(123).i16(-450).i16(5).i16(10).i16(-20).i16(0).bytes();
      case 0x0E: return Bytes{p[0], p[1], p[2], p[3], 0x00, 0x00};
      case 0x0F: zoom_int = p[0]; zoom_frac = p[1]; return Bytes{1};
      case 0x10: return Bytes{0, 2};
      case 0x11: return Bytes{p[0], p[1]};
      case 0x12: return Writer().u16(3650).u16(protocol_u16(p, 0)).u16(protocol_u16(p, 2)).bytes();
      case 0x13:
        return Writer().u16(200).u16(200).u16(400).u16(400).u16(4510).u16(2133)
               .u16(300).u16(310).u16(210).u16(390).bytes();
      case 0x14: return Writer().u16(6000).u16(1500).u16(320).u16(256).u16(1).u16(2).bytes();
      case 0x16: return Bytes{30, 0};
      case 0x18: return Bytes{zoom_int, zoom_frac};
      case 0x1A: return Bytes{3};
      case 0x1B: return Bytes{p[0]};
      case 0x20: return Writer().u8(p[0]).u8(2).u16(1920).u16(1080).u16(4000).u8(30).bytes();
      case 0x25: return Bytes{p[0]};
      case 0x37: return Bytes{0};
      case 0x38: return Bytes{p[0]};
      case 0x4F: return Bytes{1};
      default: return std::nullopt;  // heartbeat and unknown: no reply
    }
  }

  static uint16_t protocol_u16(const Bytes & b, size_t off)
  {
    return static_cast<uint16_t>(b[off] | (b[off + 1] << 8));
  }

  std::shared_ptr<PodState> pod;
  std::unique_ptr<UnipodClient> client;
  uint8_t zoom_int{1};
  uint8_t zoom_frac{0};
};

}  // namespace

TEST_F(ClientTest, SetAngleSendsTenthsAndReturnsAck)
{
  startAndWaitOpen();
  auto a = client->setAngle(30.0f, 20.0f);
  ASSERT_TRUE(a);
  EXPECT_FLOAT_EQ(a->yaw, 30.0f);
  EXPECT_FLOAT_EQ(a->pitch, 20.0f);
  EXPECT_EQ(pod->last(0x0E)->payload, (Bytes{0x2C, 0x01, 0xC8, 0x00}));  // document example
}

TEST_F(ClientTest, SetAngleClampsPitch)
{
  startAndWaitOpen();
  auto a = client->setAngle(0.0f, -120.0f);
  ASSERT_TRUE(a);
  EXPECT_FLOAT_EQ(a->pitch, -90.0f);
  EXPECT_EQ(pod->last(0x0E)->payload, (Bytes{0x00, 0x00, 0x7C, 0xFC}));
}

TEST_F(ClientTest, ZoomAbsoluteRoundsToTenthsAndReadsBack)
{
  startAndWaitOpen();
  EXPECT_TRUE(client->setZoom(7.55f));
  EXPECT_EQ(pod->last(0x0F)->payload, (Bytes{7, 6}));
  auto z = client->getZoom();
  ASSERT_TRUE(z);
  EXPECT_NEAR(*z, 7.6f, 1e-4);
  EXPECT_TRUE(client->setZoom(99.0f));  // clamped to the 30x the document allows
  EXPECT_EQ(pod->last(0x0F)->payload, (Bytes{30, 0}));
  auto max = client->getMaxZoom();
  ASSERT_TRUE(max);
  EXPECT_FLOAT_EQ(*max, 30.0f);
}

TEST_F(ClientTest, ManualZoomReturnsLevel)
{
  startAndWaitOpen();
  auto z = client->zoom(1);
  ASSERT_TRUE(z);
  EXPECT_FLOAT_EQ(*z, 2.5f);
  EXPECT_EQ(pod->last(0x05)->payload, (Bytes{0x01}));
  client->zoom(-5);  // clamped to -1
  EXPECT_EQ(pod->last(0x05)->payload, (Bytes{0xFF}));
}

TEST_F(ClientTest, RotateAndRecenter)
{
  startAndWaitOpen();
  EXPECT_TRUE(client->rotate(20, 0));
  EXPECT_TRUE(waitFor([&] {return pod->count(0x07) == 1;}));
  EXPECT_EQ(pod->last(0x07)->payload, (Bytes{0x14, 0x00}));
  EXPECT_TRUE(client->recenter(unipod_driver::kCenterOnly));
  EXPECT_EQ(pod->last(0x08)->payload, (Bytes{0x03}));
}

TEST_F(ClientTest, TemperaturesAreDecodedInCelsius)
{
  startAndWaitOpen();
  auto pt = client->pointTemperature(960, 540);
  ASSERT_TRUE(pt);
  EXPECT_EQ(pt->mode, Temperature::kPoint);
  EXPECT_FLOAT_EQ(pt->max_c, 36.5f);
  EXPECT_EQ(pt->x1, 960);
  EXPECT_EQ(pt->y1, 540);
  EXPECT_EQ(pod->last(0x12)->payload, (Bytes{0xC0, 0x03, 0x1C, 0x02, 0x01}));

  auto rg = client->regionTemperature(200, 200, 400, 400);
  ASSERT_TRUE(rg);
  EXPECT_FLOAT_EQ(rg->max_c, 45.1f);
  EXPECT_FLOAT_EQ(rg->min_c, 21.33f);
  EXPECT_EQ(rg->max_x, 300);
  EXPECT_EQ(rg->min_y, 390);

  auto fr = client->frameTemperature();
  ASSERT_TRUE(fr);
  EXPECT_FLOAT_EQ(fr->max_c, 60.0f);
  EXPECT_FLOAT_EQ(fr->min_c, 15.0f);
  EXPECT_EQ(fr->max_x, 320);
}

TEST_F(ClientTest, ThermalSettingsVideoModeAndInfo)
{
  startAndWaitOpen();
  EXPECT_EQ(client->getPalette().value_or(255), 3);
  EXPECT_EQ(client->setPalette(0).value_or(255), 0);
  EXPECT_EQ(client->setThermalGain(1).value_or(255), 1);
  EXPECT_TRUE(client->refreshShutter());
  auto vm = client->setVideoMode(2, 0);
  ASSERT_TRUE(vm);
  EXPECT_EQ(vm->main, 2);
  EXPECT_EQ(vm->sub, 0);
  auto enc = client->getEncoding(1);
  ASSERT_TRUE(enc);
  EXPECT_EQ(enc->codec, 2);
  EXPECT_EQ(enc->width, 1920);
  auto fw = client->getFirmware();
  ASSERT_TRUE(fw);
  EXPECT_EQ(fw->camera, "v1.0.1");
  EXPECT_EQ(fw->gimbal, "v0.1.10");  // the document's example
}

TEST_F(ClientTest, TimeoutReturnsNulloptAfterRetries)
{
  pod->responder = [](const Frame &) {return std::nullopt;};
  startAndWaitOpen();
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(client->getZoom());
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(dt, 190ms);  // 2 attempts x 100 ms
  EXPECT_LT(dt, 1000ms);
  EXPECT_EQ(pod->count(0x18), 2u);
}

TEST_F(ClientTest, FailsFastWithoutTransport)
{
  pod->refuse_open = true;
  client->start();
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(client->getZoom());
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 50ms);
}

TEST_F(ClientTest, PushedAttitudeAndTemperatureReachCallbacks)
{
  std::atomic<int> attitudes{0};
  std::atomic<int> temps{0};
  Attitude last{};
  std::mutex m;
  client->onAttitude([&](const Attitude & a) {
      std::lock_guard<std::mutex> lk(m);
      last = a;
      ++attitudes;
    });
  client->onFrameTemperature([&](const Temperature &) {++temps;});
  startAndWaitOpen();

  pod->push(0x0D, Writer().i16(123).i16(-450).i16(5).i16(10).i16(-20).i16(0).bytes());
  pod->push(0x14, Writer().u16(6000).u16(1500).u16(1).u16(2).u16(3).u16(4).bytes());
  ASSERT_TRUE(waitFor([&] {return attitudes >= 1 && temps >= 1;}));
  std::lock_guard<std::mutex> lk(m);
  EXPECT_FLOAT_EQ(last.yaw, 12.3f);
  EXPECT_FLOAT_EQ(last.pitch, -45.0f);
  EXPECT_FLOAT_EQ(last.yaw_rate, 1.0f);
  EXPECT_FLOAT_EQ(last.pitch_rate, -2.0f);
  EXPECT_TRUE(client->linkUp());
}

TEST_F(ClientTest, HeartbeatIsSentOnStreamTransports)
{
  startAndWaitOpen();
  ASSERT_TRUE(waitFor([&] {return pod->count(0x00) >= 1;}));
  EXPECT_EQ(pod->last(0x00)->payload, (Bytes{0x00}));
}

TEST_F(ClientTest, StreamsAreResentAfterReconnect)
{
  startAndWaitOpen();
  ASSERT_TRUE(client->setAttitudeStream(unipod_driver::kRate10Hz));
  EXPECT_TRUE(client->setFrameTemperatureStream(true));
  ASSERT_TRUE(waitFor([&] {return client->linkUp();}));
  const size_t before = pod->count(0x25);

  {
    std::lock_guard<std::mutex> lk(pod->mtx);
    pod->drop = true;
  }
  pod->cv.notify_all();
  ASSERT_TRUE(waitFor([&] {std::lock_guard<std::mutex> lk(pod->mtx); return pod->opens >= 2;}));
  // The probe gets an answer, the link comes up again and the streams are requested again.
  ASSERT_TRUE(waitFor([&] {return pod->count(0x25) > before;}));
  EXPECT_EQ(pod->last(0x25)->payload, (Bytes{0x01, 0x04}));
  EXPECT_EQ(pod->last(0x14)->payload, (Bytes{0x02}));
}
