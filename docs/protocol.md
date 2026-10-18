# SDK protocol summary

What the driver uses from *UniPod MT11 External SDK Protocol* V0.1.0
([PDF](UniPod_MT11_External_SDK_Protocol_V0.1.0.pdf)).

## Transport

TCP or UDP port **37260** (TCP needs a heartbeat, the driver sends one every second), or TTL
serial 115200 8N1. Video is RTSP on port 8554: `/video1` (main) and `/video2` (sub).

## Frame

| Bytes | Field | |
|---|---|---|
| 2 | STX | `0x6655` (on the wire `55 66`) |
| 1 | CTRL | bit 0 need ACK, bit 1 this is an ACK |
| 2 | data length | little endian |
| 2 | SEQ | little endian |
| 1 | CMD | below |
| n | payload | little endian fields |
| 2 | CRC16 | CCITT (X¹⁶+X¹²+X⁵+1) over everything before it |

## Commands used

| CMD | | Payload / answer | Driver |
|---|---|---|---|
| `0x00` | heartbeat | `00` | TCP link supervision |
| `0x01` | firmware version | 3 × u32 | `status` |
| `0x05` | manual zoom | i8 `1` in, `0` stop, `-1` out → zoom ×10 (u16) | `zoom` |
| `0x07` | rotate | i8 yaw speed, i8 pitch speed (−100..100) | `ptz_cmd` |
| `0x08` | center | u8 mode: 1 reset, 2 center + face down, 3 center, 4 face down | `recenter` |
| `0x0D` | attitude | 6 × i16, degrees and °/s ×10: yaw, pitch, roll, rates | `attitude` |
| `0x0E` | set angle | i16 yaw, i16 pitch, degrees ×10 → current yaw, pitch, roll | `set_angle` |
| `0x0F` | absolute zoom | u8 integer part, u8 tenths | `set_zoom` |
| `0x10` / `0x11` | get / set video mode | u8 main, u8 sub stream content | `set_video_mode` |
| `0x12` | temperature at a point | u16 x, u16 y, u8 flag → °C ×100 | `get_temperature` mode 0 |
| `0x13` | temperature in a box | 4 × u16, u8 flag → max/min °C ×100 and where | mode 1 |
| `0x14` | full-frame temperature | u8 0 off, 1 once, 2 continuous → max/min °C ×100 and where | mode 2, `thermal/temperature` |
| `0x16` / `0x18` | zoom range / current zoom | integer + tenths | `status` |
| `0x1A` / `0x1B` | get / set thermal palette | u8 | `thermal_control` 0 |
| `0x20` | stream encoding | stream → codec, resolution, bitrate | logged at connect |
| `0x25` | data stream | u8 type (1 attitude), u8 rate code (2..100 Hz) | `attitude_rate_hz` |
| `0x37` / `0x38` | get / set thermal gain | u8 0 low, 1 high | `thermal_control` 1 |
| `0x4F` | thermal shutter (FFC) | — | `thermal_control` 2 |

Every request is retried once after `request_timeout_ms` without an ACK.
