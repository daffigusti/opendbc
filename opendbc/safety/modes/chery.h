#pragma once

#include "opendbc/safety/safety_declarations.h"

// CAN msgs we care about
#define CHERY_ACC_CMD 0x3A2
#define CHERY_ACC_STATUS 0x3A5
#define CHERY_LKAS_HUD 0x307
#define CHERY_LKAS_CMD 0x345
#define CHERY_ACC_SETTING 0x387
#define CHERY_HUD_ALERT 0x3FC

#define CHERY_ENGINE 0x3E
#define CHERY_BRAKE 0x29A
#define CHERY_BRAKE_SENSOR 0x4ED
#define CHERY_WHEEL_SENSOR 0x316 // RX for vehicle speed
#define CHERY_ACC_DATA 0x3A5
#define CHERY_STEER_BUTTON 0x360

// CAN bus numbers
#define CHERY_MAIN 0
#define CHERY_AUX 1
#define CHERY_CAM 2

bool chery_longitudinal = false;
// GCOV_EXCL_START
// Unreachable by design (doesn't define any rx msgs)
void chery_rx_hook(const CANPacket_t *to_push)
{
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if (bus == CHERY_MAIN)
  {

    if (addr == CHERY_WHEEL_SENSOR)
    {
      // Get current speed and standstill
      uint16_t right_rear = (GET_BYTE(to_push, 0) << 8) | (GET_BYTE(to_push, 1));
      uint16_t left_rear = (GET_BYTE(to_push, 2) << 8) | (GET_BYTE(to_push, 3));
      vehicle_moving = (right_rear | left_rear) != 0U;
      UPDATE_VEHICLE_SPEED((right_rear + left_rear) / 2.0 * 0.00828 / 3.6);
    }

    // if (addr == CHERY_STEER_TORQUE) {
    //   int torque_driver_new = GET_BYTE(to_push, 0) - 127U;
    //   // update array of samples
    //   update_sample(&torque_driver, torque_driver_new);
    // }

    // // enter controls on rising edge of ACC, exit controls on ACC off
    // if (addr == CHERY_CRZ_CTRL) {
    //   acc_main_on = GET_BIT(to_push, 17U);
    //   bool cruise_engaged = GET_BYTE(to_push, 0) & 0x8U;
    //   pcm_cruise_check(cruise_engaged);
    // }

    // if (addr == CHERY_ENGINE_DATA) {
    //   gas_pressed = (GET_BYTE(to_push, 4) || (GET_BYTE(to_push, 5) & 0xF0U));
    // }

    if (addr == CHERY_ENGINE)
    {
      brake_pressed = ((GET_BYTES(to_push, 0, 27) >> 4) & 0x01) != 0U;
    }
  }
  else if (bus == CHERY_CAM)
  {
    if (addr == CHERY_ACC_CMD)
    {
      acc_main_on = ((GET_BYTE(to_push, 1) & 0x03) != 1U);
      // bool stand_still = (GET_BYTE(to_push, 1) >> 2) & 0x01;

      gas_pressed = (GET_BYTE(to_push, 5) & 0x80U) != 0U;
    }
    if (addr == CHERY_ACC_DATA)
    {
      // Signal: ACCStatus
      bool cruise_engaged = GET_BIT(to_push, 20U);
      pcm_cruise_check(cruise_engaged);
    }
  }
  controls_allowed = true;
}

static safety_config chery_init(uint16_t param)
{
  static const CanMsg CHERY_TX_MSGS[] = {
      {CHERY_LKAS_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_HUD, 0, 8, .check_relay = true},
      // {CHERY_HUD_ALERT, 0, 8, .check_relay = true},
      // {CHERY_ACC_SETTING, 0, 8, .check_relay = true},
      // {CHERY_STEER_BUTTON, 0, 6, .check_relay = true},
      {CHERY_STEER_BUTTON, 2, 6, .check_relay = false},
  };
  static const CanMsg CHERY_LONG_TX_MSGS[] = {
      {CHERY_ACC_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_CMD, 0, 8, .check_relay = true},
      {CHERY_LKAS_HUD, 0, 8, .check_relay = true},
      // {CHERY_HUD_ALERT, 0, 8, .check_relay = true},
      // {CHERY_ACC_SETTING, 0, 8, .check_relay = true},
      // {CHERY_STEER_BUTTON, 0, 6, .check_relay = true},
      {CHERY_STEER_BUTTON, 2, 6, .check_relay = false},
  };

  static RxCheck chery_rx_checks[] = {
      // {.msg = {{CHERY_WHEEL_SENSOR, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, {0}, {0}}},
      // {.msg = {{CHERY_WHEEL_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, {0}, {0}}},
      // {.msg = {{CHERY_ENGINE, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, {0}, {0}}},
      // {.msg = {{CHERY_BRAKE, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 50U}, {0}, {0}}},
      // {.msg = {{CHERY_BRAKE_SENSOR, CHERY_MAIN, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, {0}, {0}}},
  };
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa

#ifdef ALLOW_DEBUG
  const uint16_t CHERY_PARAM_LONGITUDINAL = 1;
  chery_longitudinal = GET_FLAG(param, CHERY_PARAM_LONGITUDINAL);
#endif

  safety_config ret;
  ret = chery_longitudinal ? BUILD_SAFETY_CFG(chery_rx_checks, CHERY_LONG_TX_MSGS) : BUILD_SAFETY_CFG(chery_rx_checks, CHERY_TX_MSGS);
  return ret;
}

static bool chery_tx_hook(const CANPacket_t *to_send)
{
  UNUSED(to_send);
  return true;
}

const safety_hooks chery_hooks = {
    .init = chery_init,
    .rx = chery_rx_hook,
    .tx = chery_tx_hook,
};
