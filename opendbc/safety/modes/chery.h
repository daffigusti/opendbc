#pragma once

#include "opendbc/safety/declarations.h"

static bool chery_acc_available = false;
static bool chery_acc_active = false;
static bool chery_stock_aeb = false;
static bool chery_acc_gas = false;
static bool chery_acc_stopped = false;
static bool chery_inhibited = false;
static bool chery_sensor_invalid = false;
static bool chery_longitudinal = false;
static const uint16_t CHERY_PARAM_LONG_CONTROL = 1U;
static int chery_current_angle_deg100 = 0;
static uint8_t chery_rx_seen_mask = 0U;
static bool chery_reauth_required = false;

static int chery_abs(int value) {
  return value < 0 ? -value : value;
}

static bool chery_health_ready(void) {
  return (chery_rx_seen_mask == 0x7FU) && !safety_rx_checks_invalid && !chery_sensor_invalid && !chery_inhibited;
}

// The stock ACC drops ACC_ACTIVE while it holds the car at standstill, then ignores ACC_CMD gas
// until a RES+ press. STOPPED keeps an existing engagement alive across that window. Requiring
// cruise_engaged_prev means STOPPED can never engage controls on its own, and ACC_STATE leaving
// the available range still tears the engagement down.
static bool chery_cruise_engaged(void) {
  return chery_acc_active || (chery_acc_stopped && cruise_engaged_prev);
}

static void chery_pcm_cruise_check(void) {
  if (!chery_cruise_engaged()) {
    chery_reauth_required = false;
    pcm_cruise_check(false);
  } else if (!chery_acc_available) {
    // ACC state becoming unavailable is not an explicit physical disengagement.
    pcm_cruise_check(false);
  } else if (!chery_health_ready()) {
    chery_reauth_required = true;
  } else if (!chery_reauth_required) {
    pcm_cruise_check(true);
  }
}

static void chery_update_gas(void) {
  // ENGINE_DATA.GAS is a drivetrain torque request, not pedal travel: across 943k moving frames
  // its distribution under ACC and under the driver is indistinguishable (38.8% vs 51.9% at
  // zero, both saturating above 26000), so no threshold on it separates the two. Reading it as
  // a driver press denied controls in 98%+ of ACC-engaged frames while protecting against
  // nothing. Only the camera's own driver-pedal bit is trusted. See KNOWN_GAPS.md.
  gas_pressed = chery_acc_gas;
}

static void chery_apply_inhibitors(void) {
  // A live condition, not a latch. The previous version cleared only when ACC_ACTIVE dropped, so
  // a single brake tap during a standstill hold -- where ACC_ACTIVE is already 0 and the
  // engagement is held alive by STOPPED -- blocked every transmission until the ACC was cycled.
  // The accelerator is an override, not a disengagement: it only blocks host longitudinal, which
  // the ACC_CMD transmit check enforces on its own.
  chery_inhibited = brake_pressed || chery_stock_aeb;
  if (chery_inhibited) {
    controls_allowed = false;
  }
}

static void chery_rx_hook(const CANPacket_t *msg) {
  const uint8_t seen_bit = (msg->addr == 0x03EU) ? 0U :
                           (msg->addr == 0x1D3U) ? 1U :
                           (msg->addr == 0x316U) ? 2U :
                           (msg->addr == 0x394U) ? 3U :
                           (msg->addr == 0x3A2U) ? 4U :
                           (msg->addr == 0x3A5U) ? 5U : 6U;
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
    chery_current_angle_deg100 = (raw * 10) - 78000;
    update_sample(&angle_meas, chery_current_angle_deg100);
  } else if (msg->addr == 0x394U) {
    const uint16_t raw = (uint16_t)(((msg->data[0] << 4U) | (msg->data[1] >> 4U)) & 0x0FFFU);
    update_sample(&torque_driver, to_signed(raw, 12));
  } else if (msg->addr == 0x03EU) {
    brake_pressed = GET_BIT(msg, 220U);
  } else if (msg->addr == 0x3A2U) {
    const uint8_t state = msg->data[1] & 0x03U;
    chery_acc_gas = GET_BIT(msg, 47U);
    // ACC_STATE drops to 1 while the driver overrides with the accelerator, with ACC_ACTIVE still
    // 1 and the pedal bit set. That is still an available ACC. Without the pedal bit, 1 is off.
    const bool gas_override = (state == 1U) && chery_acc_gas && chery_acc_active;
    chery_acc_available = (state == 2U) || (state == 3U) || gas_override;
    // acc_main_on is left false, as on Tesla and Rivian. The Omoda has no main switch, and
    // ACC_STATE reads 1 on 98% of brake-pressed frames, so deriving main from it would end MADS
    // lateral on every brake press. MADS engages on the ACC engagement edge instead.
    chery_acc_stopped = GET_BIT(msg, 10U);
    chery_pcm_cruise_check();
  } else if (msg->addr == 0x3A5U) {
    chery_acc_active = GET_BIT(msg, 20U);
    chery_pcm_cruise_check();
  } else if (msg->addr == 0x387U) {
    // SETTING.AEB_ACTIVE reads 3 when the stock AEB brakes. ACC.AEB_ACTIVE (0x3A5 bit 46) is the
    // collision warning: it also rises with AEB switched off, so it is not an inhibitor.
    chery_stock_aeb = ((msg->data[4] >> 6U) & 0x03U) == 3U;
  }
  chery_update_gas();
  chery_apply_inhibitors();
}

static bool chery_tx_hook(const CANPacket_t *msg) {
  static const AngleSteeringLimits CHERY_STEERING_LIMITS = {
    .max_angle = 37040,
    .angle_deg_to_can = 100,
    .frequency = 50U,
  };
  static const AngleSteeringParams CHERY_STEERING_PARAMS = {
    .slip_factor = -0.000637749883,
    .steer_ratio = 17.0,
    .wheelbase = 2.63,
  };

  if (msg->addr == 0x3A2U) {
    // Never allow host ACC traffic while RX health is untrusted. This keeps
    // startup, integrity faults, timeouts, and inhibitors fail-closed.
    if (!chery_health_ready()) {
      return false;
    }
    if (!chery_longitudinal || msg->bus != 0U || GET_LEN(msg) != 8U) {
      return false;
    }

    // ACC_CMD CMD is signed 10-bit Motorola: bits 6..15.
    const uint16_t cmd_raw = (uint16_t)(((msg->data[0] & 0x7FU) << 3U) | (msg->data[1] >> 5U));
    const int command = to_signed(cmd_raw, 10);
    const bool accel_on = GET_BIT(msg, 7U);
    const bool gas_pressed_cmd = GET_BIT(msg, 47U);
    const uint8_t aeb_req_stop = (msg->data[6] >> 4U) & 0x0FU;
    // CMD is a magnitude and ACCEL_ON its direction. Stock holds a stopped car with CMD=400,
    // ACCEL_ON=0, STOPPED=1, ACC_STATE=2 -- its maximum brake request. That pair is the one
    // exception to the sign agreement, and only while the car is already stopped: sent while
    // rolling it is a full-force brake application.
    const bool stopped_cmd = GET_BIT(msg, 10U);
    const bool state_holding = (msg->data[1] & 0x03U) == 2U;
    const bool full_stop_hold = (command == 400) && !accel_on && stopped_cmd && state_holding && !vehicle_moving;
    if (aeb_req_stop != 0U || command < -511 || command > 511) {
      return false;
    }
    if ((accel_on != (command >= 0)) && !full_stop_hold) {
      return false;
    }
    if (chery_stock_aeb) {
      return false;
    }
    // Inhibited and inactive states may only transmit stock's inactive command.
    // Check raw RX-derived inhibitors directly: test setters can override controls_allowed.
    if ((!controls_allowed || brake_pressed || gas_pressed) &&
        (command != -24 || gas_pressed_cmd)) {
      return false;
    }
    return true;
  }

  if (msg->addr == 0x360U) {
    if ((msg->bus != 2U) || (GET_LEN(msg) != 6U)) {
      return false;
    }
    const bool acc_button = GET_BIT(msg, 24U);
    const bool res_plus = GET_BIT(msg, 30U);
    const bool res_minus = GET_BIT(msg, 32U);
    const bool main_button = GET_BIT(msg, 26U);
    const bool gap_down = GET_BIT(msg, 43U);
    const bool gap_up = GET_BIT(msg, 45U);
    const bool other_buttons = main_button || gap_down || gap_up;
    // Cancel: the ACC button toggles the ACC, cancelling while active and engaging while not, so it
    // is only allowed alone and while ACC_ACTIVE is 1. It needs neither controls nor brake-free RX,
    // so a disengaged openpilot can still take the stock ACC down, but it does need trusted RX,
    // where ACC_ACTIVE cannot be stale.
    if (acc_button) {
      return !safety_rx_checks_invalid && chery_acc_active && !res_plus && !res_minus && !other_buttons;
    }
    // Resume taps and set-speed taps only, with controls authorized. RES+ is resume from a stopped
    // hold or +set speed while ACC is active. RES- is -set speed while active but SET (engage)
    // while not, so it requires an active ACC. The main bit is never host-sent.
    if (!chery_health_ready()) {
      return false;
    }
    // Gap taps sync the stock following distance to openpilot's personality. They only change the
    // stock ACC's own gap, which openpilot's ACC_CMD overrides, so they need openpilot longitudinal,
    // an active ACC and controls, one direction at a time, and nothing else pressed.
    if (gap_down || gap_up) {
      return chery_longitudinal && controls_allowed && chery_acc_active && (gap_down != gap_up) &&
             !main_button && !res_plus && !res_minus;
    }
    const bool plus_ok = !res_plus || chery_acc_active || !vehicle_moving;
    const bool minus_ok = !res_minus || chery_acc_active;
    return controls_allowed && !other_buttons && !(res_plus && res_minus) && plus_ok && minus_ok;
  }

  if ((msg->addr == 0x307U) || (msg->addr == 0x3FCU)) {
    // LKAS_STATE and HUD_ALERT are cluster indicators. They actuate nothing, but the stock copies
    // are blocked from forwarding, so openpilot has to be able to relay them.
    return chery_health_ready() && (msg->bus == 0U) && (GET_LEN(msg) == 8U);
  }

  if (msg->addr != 0x345U || GET_LEN(msg) != 8U || msg->bus != 0U) {
    return false;
  }

  // CMD is signed 13-bit Motorola: bits 6..18, with 0.1 degree resolution
  // and a -392 raw offset.
  const uint16_t raw = (uint16_t)(((msg->data[0] & 0x7FU) << 6U) | (msg->data[1] >> 2U));
  const int desired_angle = (to_signed(raw, 13) + 392) * 10;
  const bool steer_control_enabled = GET_BIT(msg, 9U);

  // Active commands have an explicit +/-360 degree cap before VM checks. Matches the controller's
  // STEER_ANGLE_MAX; the 13-bit encoding tops out at 370.4.
  if (steer_control_enabled && ((desired_angle > 36000) || (desired_angle < -36000))) {
    return false;
  }

  // Match the controller's low-speed 5 degree/frame fault avoidance limit.
  if (steer_control_enabled && (chery_abs(desired_angle - desired_angle_last) > 500)) {
    return false;
  }

  // Never enable lateral control while rack angle is outside the controller cap.
  if (steer_control_enabled && (chery_abs(chery_current_angle_deg100) > 36000)) {
    return false;
  }

  // Do not permit an inactive command to wrap or clamp across the signed-13
  // representable physical range.
  if (!steer_control_enabled && ((desired_angle > CHERY_STEERING_LIMITS.max_angle) ||
                                 (desired_angle < -CHERY_STEERING_LIMITS.max_angle))) {
    return false;
  }

  return !steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled,
                                    CHERY_STEERING_LIMITS, CHERY_STEERING_PARAMS);
}

static bool chery_fwd_hook(int bus_num, int addr) {
  if (bus_num == 2 && addr == 0x3A2U) {
    // Base mode always forwards. LONG_CONTROL blocks OEM ACC only after all
    // RX health checks are trusted; otherwise preserve OEM authority.
    return chery_longitudinal && chery_health_ready();
  }
  if ((bus_num == 2) && ((addr == 0x307U) || (addr == 0x3FCU))) {
    // openpilot re-emits LKAS_STATE and HUD_ALERT every 50ms -- its own state while steering or
    // overridden, the camera's verbatim otherwise -- so the camera's copies must not also reach the cluster.
    return chery_health_ready();
  }
  // Let stock steering pass through only when the measured rack angle is
  // outside the representable command range. Within range, block stock
  // steering while retaining forwarding for stock buttons and other frames.
  return (bus_num == 2) && (addr == 0x345U) && (chery_abs(chery_current_angle_deg100) <= 37040);
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
  chery_longitudinal = GET_FLAG(param, CHERY_PARAM_LONG_CONTROL);
  chery_acc_available = false;
  acc_main_on = false;
  chery_acc_active = false;
  chery_stock_aeb = false;
  chery_acc_gas = false;
  chery_acc_stopped = false;
  chery_inhibited = false;
  chery_sensor_invalid = false;
  chery_current_angle_deg100 = 0;
  chery_rx_seen_mask = 0U;
  chery_reauth_required = false;
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
    {.msg = {{0x387, 2, 8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
  };
  static const CanMsg chery_tx_msgs[] = {{0x345, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                         {0x307, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                         {0x3FC, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                         {0x360, 2, 6, .check_relay = false, .disable_static_blocking = true}};
  static const CanMsg chery_long_tx_msgs[] = {{0x345, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                              {0x307, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                              {0x3FC, 0, 8, .check_relay = true, .disable_static_blocking = true},
                                              {0x360, 2, 6, .check_relay = false, .disable_static_blocking = true},
                                              {0x3A2, 0, 8, .check_relay = true, .disable_static_blocking = true}};
  safety_config config = {
    .rx_checks = chery_rx_checks,
    .rx_checks_len = sizeof(chery_rx_checks) / sizeof(chery_rx_checks[0]),
    .tx_msgs = chery_longitudinal ? chery_long_tx_msgs : chery_tx_msgs,
    .tx_msgs_len = (int)(chery_longitudinal ? sizeof(chery_long_tx_msgs) / sizeof(chery_long_tx_msgs[0]) :
                                              sizeof(chery_tx_msgs) / sizeof(chery_tx_msgs[0])),
    .disable_forwarding = false,
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
