#pragma once

#include "opendbc/safety/declarations.h"

static bool chery_acc_available = false;
static bool chery_acc_active = false;
static bool chery_stock_aeb = false;
static bool chery_engine_gas = false;
static bool chery_acc_gas = false;
static bool chery_inhibited = false;
static bool chery_sensor_invalid = false;
static uint8_t chery_rx_seen_mask = 0U;

static bool chery_authorized(void) {
  return (chery_rx_seen_mask == 0x3FU) && !safety_rx_checks_invalid && !chery_sensor_invalid &&
         chery_acc_available && chery_acc_active && !chery_inhibited;
}

static void chery_pcm_cruise_check(void) {
  const bool physical_cruise_engaged = chery_acc_available && chery_acc_active;
  if (!physical_cruise_engaged) {
    // Physical disengagement must be observed even during an RX fault.
    pcm_cruise_check(false);
  } else if (chery_authorized()) {
    // Preserve physical ACC edge while health is invalid.
    pcm_cruise_check(true);
  }
}

static void chery_update_gas(void) {
  gas_pressed = chery_engine_gas || chery_acc_gas;
}

static void chery_apply_inhibitors(void) {
  if (brake_pressed || gas_pressed || chery_stock_aeb) {
    chery_inhibited = true;
    controls_allowed = false;
  }
}

static void chery_rx_hook(const CANPacket_t *msg) {
  const uint8_t seen_bit = (msg->addr == 0x03EU) ? 0U :
                           (msg->addr == 0x1D3U) ? 1U :
                           (msg->addr == 0x316U) ? 2U :
                           (msg->addr == 0x394U) ? 3U :
                           (msg->addr == 0x3A2U) ? 4U : 5U;
  chery_rx_seen_mask |= (uint8_t)(1U << seen_bit);
  if (msg->addr == 0x316U) {
    // DBC order is FR (bytes 0-1), FL (bytes 2-3).
    const int front_right = to_signed((msg->data[0] << 8U) | msg->data[1], 16);
    const int front_left = to_signed((msg->data[2] << 8U) | msg->data[3], 16);
    if ((front_right < 0) || (front_left < 0)) {
      chery_sensor_invalid = true;
      controls_allowed = false;
      mads_exit_controls(MADS_DISENGAGE_REASON_INVALID_RX);
    } else {
      chery_sensor_invalid = false;
      UPDATE_VEHICLE_SPEED(((front_left + front_right) / 2.0) * 0.00829 / 3.6);
      vehicle_moving = (front_left > 0) || (front_right > 0);
    }
  } else if (msg->addr == 0x1D3U) {
    const uint16_t raw = (uint16_t)(((msg->data[0] << 6U) | (msg->data[1] >> 2U)) & 0x3FFFU);
    update_sample(&angle_meas, (raw * 10) - 78000);
  } else if (msg->addr == 0x394U) {
    const uint16_t raw = (uint16_t)(((msg->data[0] << 4U) | (msg->data[1] >> 4U)) & 0x0FFFU);
    update_sample(&torque_driver, to_signed(raw, 12));
  } else if (msg->addr == 0x03EU) {
    brake_pressed = GET_BIT(msg, 220U);
    const uint16_t engine_gas = (uint16_t)((msg->data[22] << 8U) | msg->data[23]);
    chery_engine_gas = engine_gas > 10U;
  } else if (msg->addr == 0x3A2U) {
    const uint8_t state = msg->data[1] & 0x03U;
    chery_acc_available = (state == 2U) || (state == 3U);
    chery_acc_gas = GET_BIT(msg, 47U);
    chery_pcm_cruise_check();
  } else if (msg->addr == 0x3A5U) {
    chery_acc_active = GET_BIT(msg, 20U);
    chery_stock_aeb = GET_BIT(msg, 46U);
    if (!chery_acc_active) {
      chery_inhibited = false;
    }
    chery_pcm_cruise_check();
  }
  chery_update_gas();
  chery_apply_inhibitors();
}

static bool chery_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return false;
}

static bool chery_fwd_hook(int bus_num, int addr) {
  SAFETY_UNUSED(bus_num);
  SAFETY_UNUSED(addr);
  return false;
}

static uint32_t chery_get_checksum(const CANPacket_t *msg) {
  return msg->addr == 0x03EU ? msg->data[24] : msg->data[7];
}

static uint8_t chery_j1850(const CANPacket_t *msg, int start, int end) {
  uint8_t crc = 0xFFU;
  for (int i = start; i <= end; i++) {
    crc ^= msg->data[i];
    for (int j = 0; j < 8; j++) {
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x1DU) : (uint8_t)(crc << 1U);
    }
  }
  return crc ^ 0xFFU;
}

static uint32_t chery_compute_checksum(const CANPacket_t *msg) {
  if (msg->addr == 0x316U) {
    uint8_t checksum = 0U;
    for (int i = 0; i < 7; i++) {
      checksum = (uint8_t)(checksum + msg->data[i]);
    }
    return (uint8_t)(~checksum);
  }

  return (msg->addr == 0x03EU) ? chery_j1850(msg, 25, 31) : chery_j1850(msg, 0, 6);
}

static uint8_t chery_get_counter(const CANPacket_t *msg) {
  return msg->addr == 0x03EU ? (msg->data[25] & 0x0FU) : (msg->data[6] & 0x0FU);
}

static bool chery_get_quality_flag_valid(const CANPacket_t *msg) {
  if (msg->addr != 0x03EU) {
    return true;
  }

  uint8_t counter = msg->data[1] & 0x0FU;
  for (int offset = 0; offset < 40; offset += 8) {
    if ((msg->data[offset + 1] & 0x0FU) != counter ||
        msg->data[offset] != chery_j1850(msg, offset + 1, offset + 7)) {
      return false;
    }
  }
  return true;
}

static safety_config chery_init(uint16_t param) {
  SAFETY_UNUSED(param);
  chery_acc_available = false;
  chery_acc_active = false;
  chery_stock_aeb = false;
  chery_engine_gas = false;
  chery_acc_gas = false;
  chery_inhibited = false;
  chery_sensor_invalid = false;
  chery_rx_seen_mask = 0U;
  // Require initial valid data and an explicit safety tick before authorizing ACC/MADS.
  safety_rx_checks_invalid = true;
  gas_pressed = false;
  brake_pressed = false;
  static RxCheck chery_rx_checks[] = {
    // max_counter=15 is full-route aggregate: 385 observed 0x1D3 wraps support four-bit counter.
    {.msg = {{0x03E, 0, 48, 100U, .max_counter = 15U, .ignore_quality_flag = false}, {0}, {0}}},
    {.msg = {{0x1D3, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x316, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x394, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x3A2, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{0x3A5, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
  };
  static const CanMsg chery_tx_msgs[] = {{0, 0, 0, false, false}};
  safety_config config = {
    .rx_checks = chery_rx_checks,
    .rx_checks_len = sizeof(chery_rx_checks) / sizeof(chery_rx_checks[0]),
    .tx_msgs = chery_tx_msgs,
    .tx_msgs_len = 0,
    .disable_forwarding = true,
  };
  return config;
}

const safety_hooks chery_hooks = {
  .init = chery_init,
  .rx = chery_rx_hook,
  .tx = chery_tx_hook,
  .fwd = chery_fwd_hook,
  .get_checksum = chery_get_checksum,
  .compute_checksum = chery_compute_checksum,
  .get_counter = chery_get_counter,
  .get_quality_flag_valid = chery_get_quality_flag_valid,
};
