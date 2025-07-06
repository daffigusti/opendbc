
#pragma once

#include "opendbc/safety/safety_declarations.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
// CAN msgs we care about
#define CHERY_ACC_CMD 0x3A2
#define CHERY_ACC_STATUS 0x3A5
#define CHERY_LKAS_HUD 0x307
#define CHERY_LKAS_CMD 0x345
#define CHERY_ACC_SETTING 0x387
#define CHERY_HUD_ALERT 0x3FC

#define CHERY_ENGINE 0x3E // Keep this for now, but it's problematic
#define CHERY_BRAKE 0x29A
#define CHERY_BRAKE_SENSOR 0x4ED // Use this for brake pressed
#define CHERY_WHEEL_SENSOR 0x316 // RX for vehicle speed
#define CHERY_ACC_DATA 0x3A5
#define CHERY_STEER_BUTTON 0x360
#define CHERY_STEER_SENSOR 0x1D3

// CAN bus numbers
#define CHERY_MAIN 0
#define CHERY_AUX 1
#define CHERY_CAM 2

static bool chery_longitudinal = false;
static void chery_rx_hook(const CANPacket_t *to_push)
{
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if (bus == CHERY_MAIN)
  {

    if (addr == CHERY_WHEEL_SENSOR) // 0x316
    {
      // Get current speed and standstill
      uint16_t right_front_speed = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
      uint16_t left_front_speed = (GET_BYTE(to_push, 2) << 8) | GET_BYTE(to_push, 3);
      vehicle_moving = (right_front_speed > 0) || (left_front_speed > 0);
      // Correct scale factor to 0.00829 to match DBC, and convert km/h to m/s by dividing by 3.6
      UPDATE_VEHICLE_SPEED((right_front_speed + left_front_speed) / 2.0 * 0.00829 / 3.6);
    }



    // steering angle
    if (addr == CHERY_STEER_SENSOR) {
      // STEER_ANGLE: 7|14@0+ (start bit 7, 14 bits, little-endian)
      // Extract STEER_ANGLE raw value (14 bits big-endian starting at bit 7)
      int steer_raw = (GET_BYTE(to_push, 0) << 6) | (GET_BYTE(to_push, 1) >> 2);
      // Apply scaling factor (0.1) and offset (-780)
      float steer_angle = (steer_raw) - 7800.0f;
      update_sample(&angle_meas,  steer_angle);
    }

    if (addr == CHERY_BRAKE_SENSOR) // 0x4ED
    {
      // BRAKE_PRESS is at bit 37 (byte 4, bit 5) in BRAKE_SENSOR message
      brake_pressed = GET_BIT(to_push, 37U);
    }
  }
  else if (bus == CHERY_CAM) // 2
  {
    if (addr == CHERY_ACC_CMD) // 0x3A2
    {
      // gas_pressed is from byte 5, bit 7
      gas_pressed = (GET_BYTE(to_push, 5) & 0x80U) != 0U;
    }
    if (addr == CHERY_ACC_STATUS) // 0x3A5
    {
      // acc_main_on is from bit 20
      acc_main_on = GET_BIT(to_push, 20U);
      // cruise_engaged is from bit 20
      bool cruise_engaged = GET_BIT(to_push, 20U);
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static safety_config chery_init(uint16_t param)
{
  static const CanMsg CHERY_TX_MSGS[] = {
      {CHERY_LKAS_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_HUD, 0, 8, .check_relay = true},
      {CHERY_ACC_DATA, 2, 8, .check_relay = false},
      // {CHERY_HUD_ALERT, 0, 8, .check_relay = true},
      // {CHERY_ACC_SETTING, 0, 8, .check_relay = true},
      {CHERY_STEER_BUTTON, 0, 6, .check_relay = false},
      {CHERY_STEER_BUTTON, 2, 6, .check_relay = false},
  };
  static const CanMsg CHERY_LONG_TX_MSGS[] = {
      {CHERY_ACC_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_HUD, 0, 8, .check_relay = true},
      {CHERY_ACC_DATA, 2, 8, .check_relay = false},
      // {CHERY_HUD_ALERT, 0, 8, .check_relay = true},
      // {CHERY_ACC_SETTING, 0, 8, .check_relay = true},
      {CHERY_STEER_BUTTON, 0, 6, .check_relay = false},
      {CHERY_STEER_BUTTON, 2, 6, .check_relay = false},
  };

  static RxCheck chery_rx_checks[] = {
      {.msg = {{CHERY_WHEEL_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, {0}, {0}}},
      {.msg = {{CHERY_BRAKE_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 10U}, {0}, {0}}}, // Use BRAKE_SENSOR for brake
      {.msg = {{CHERY_ACC_CMD, CHERY_CAM, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, {0}, {0}}},
      {.msg = {{CHERY_ACC_DATA, CHERY_CAM, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, {0}, {0}}},
      {.msg = {{CHERY_STEER_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, {0}, {0}}},
  };
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa

  const uint16_t CHERY_PARAM_LONGITUDINAL = 1;
  chery_longitudinal = GET_FLAG(param, CHERY_PARAM_LONGITUDINAL);

  safety_config ret;
  ret = chery_longitudinal ? BUILD_SAFETY_CFG(chery_rx_checks, CHERY_LONG_TX_MSGS) : BUILD_SAFETY_CFG(chery_rx_checks, CHERY_TX_MSGS);
  return ret;
}

// Helper function to check if controls are allowed
extern bool controls_allowed;

static bool chery_tx_hook(const CANPacket_t *to_send)
{
  const AngleSteeringLimits CHERY_STEERING_LIMITS = {
      .max_angle = 3600, // 360 deg, EPAS faults above this
      .angle_deg_to_can = 10,
      .frequency = 50U,
  };

  const AngleSteeringParams CHERY_STEERING_PARAMS = {
      .slip_factor = -0.000580374383851451, // calc_slip_factor(VM)
      .steer_ratio = 17.,
      .wheelbase = 2.63,
  };

  const int addr = GET_ADDR(to_send);
  bool tx = true;
  bool violation = false;
  if (addr == CHERY_LKAS_CMD)
  {
    // LKAS_CAM_CMD_345 message ID
    // Extract 13 bits starting at bit 6 (little-endian)
    int raw_cmd = ((GET_BYTE(to_send, 0) & 0x7F) << 6) | ((GET_BYTE(to_send, 1) & 0xFC) >> 2);

    // Proper sign extension for 13 bits:
    if (raw_cmd & 0x1000) {  // bit 12 set means negative
      raw_cmd -= 0x2000;     // subtract 8192 for correct negative representation
    }

    // Convert back to steering angle in degrees
    int desired_angle = (raw_cmd + 392);
    // Extract LKA_ACTIVE (bit 1 of byte 1)
    bool lka_active = (GET_BYTE(to_send, 1) >> 1) & 0x1;

    if (steer_angle_cmd_checks_vm(desired_angle, lka_active, CHERY_STEERING_LIMITS, CHERY_STEERING_PARAMS))
    {
      violation = true;
    }

  }

  if (violation)
  {
    tx = false;
  }
  // When controls are not allowed, only allow main_button (ACC) messages to transmit
  // Block other button messages like set and res
  if (addr == CHERY_STEER_BUTTON)
  {
    // Extract button bits from message data to check which button is pressed
    // Assuming byte 0 contains button bits: ACC, RES_PLUS, RES_MINUS
    uint8_t byte3 = GET_BYTE(to_send, 3);
    uint8_t byte4 = GET_BYTE(to_send, 4);

    bool main_button_pressed = (byte3 & (1 << 0)) != 0; // ACC bit 24
    bool res_pressed = (byte3 & (1 << 6)) != 0;         // RES_PLUS bit 30
    bool set_pressed = (byte4 & (1 << 0)) != 0;         // RES_MINUS bit 32

    int buttons_pressed = main_button_pressed + res_pressed + set_pressed;

    const bool allowed = (buttons_pressed == 1) &&
                         (main_button_pressed ||
                          (set_pressed && controls_allowed) ||
                          (res_pressed && controls_allowed));
    if (!allowed)
    {
      tx = false;
    }
  }

  // Allow other messages that are not steer button
  return tx;
}

const safety_hooks chery_hooks = {
    .init = chery_init,
    .rx = chery_rx_hook,
    .tx = chery_tx_hook,
};
