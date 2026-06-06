# AKM Extended Telemetry Protocol

This document records the AKM serial telemetry extension used by the STM32
firmware and the wheeltec14 ROS parser.

## Design

The original 24-byte frame is kept byte-for-byte as the compatibility payload:

```text
0x7B / stop flag / vx / vy / vz / accel xyz / gyro xyz / battery / bcc / 0x7D
```

For the AKM target, UART3 now sends a 72-byte extension frame instead of only
the bare 24-byte legacy frame. The extension embeds the complete legacy frame,
so a new parser can preserve the existing `/odom`, `/imu`, and `/PowerVoltage`
chain while publishing lower-level wheel and steering observations.

CAN telemetry remains the original 24-byte frame split over IDs `0x101`,
`0x102`, and `0x103`.

## Frame

All multi-byte integers are big endian.

```text
byte 0    header              uint8   0x7E
byte 1    frame_type          uint8   0x01, AKM motion extension
byte 2    protocol_version    uint8   0x01
byte 3    payload_len         uint8   66
byte 4..69 payload
byte 70   checksum            uint8   XOR of bytes 0..69
byte 71   tail                uint8   0x7F
```

Payload layout:

```text
offset  size  field
0       2     seq_id                         uint16
2       4     control_tick_us                uint32
6       2     dt_us                          uint16
8       24    legacy_24b_frame               uint8[24]
32      4     left_encoder_delta             int32
36      4     right_encoder_delta            int32
40      2     left_wheel_speed_mps_x1000     int16
42      2     right_wheel_speed_mps_x1000    int16
44      2     steering_feedback_raw          int16
46      2     steering_target_raw            int16
48      2     steering_angle_rad_x10000      int16
50      2     steering_pwm                   int16
52      2     motor_left_pwm                 int16
54      2     motor_right_pwm                int16
56      2     target_vx_mps_x1000            int16
58      2     target_vy_mps_x1000            int16
60      2     target_vz_rps_x1000            int16
62      2     status_flags                   uint16
64      1     control_mode                   uint8
65      1     robot_type                     uint8
```

`control_tick_us` comes from the STM32 TIM6 1 MHz free-running counter used by
the AKM encoder path. It wraps as a 32-bit microsecond counter. `dt_us` is the
wrapped delta between extension frames, truncated to `uint16`; the first
extension frame reports `dt_us = 0`.

## Status Flags

```text
bit 0   FlagStop
bit 1   command timeout observed by STM32
bit 2   low voltage
bit 4   startup self-check error
bit 5   steering_angle_rad_x10000 is calibrated from AKM steering feedback
bit 6   embedded legacy 24-byte frame is valid
```

## wheeltec14 Topics

The replacement parser keeps the old topics:

```text
/odom
/imu
/PowerVoltage
```

It also publishes:

```text
/wheeltec/akm_state
/wheeltec/control_debug
/wheeltec/chassis_diagnostics
```

`/odom` remains the legacy STM32-integrated velocity chain. New estimation
nodes should subscribe to `/wheeltec/akm_state` for encoder, wheel-speed, and
steering observations, and to `/wheeltec/control_debug` or
`/wheeltec/chassis_diagnostics` for weighting, filtering, and experiment
diagnosis.
