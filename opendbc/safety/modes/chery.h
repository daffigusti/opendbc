#pragma once

#include "opendbc/safety/declarations.h"

static void chery_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
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
