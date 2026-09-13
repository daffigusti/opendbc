"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.chery.cherycan import create_button_control
from opendbc.car.chery.values import CarControllerParams
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, CS, packer, frame, CAN) -> list[CanData]:
    """Tap RES+/RES- on the camera bus until the stock set speed matches the target.

    Taps rather than holds: a held RES+ auto-repeats by up to +14 kph, which would overshoot a
    closed loop that only reads the result back from the cluster. With ACC_ACTIVE at 0, RES-
    is SET and would engage the ACC, so nothing is sent unless it is already active.
    """
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    if self.ICBM.sendButton == SendButtonState.none or not CS.acc_active:
      self.button_frame = 0
      return can_sends
    if self.frame % CarControllerParams.BUTTONS_STEP != 0:
      return can_sends

    # ponytail: fixed tap cadence shared with resume, unmeasured while moving. Tune from a route
    # with driver +/- taps at ACC_ACTIVE 1 if presses are dropped or overshoot.
    if self.button_frame % CarControllerParams.RESUME_TAP_PERIOD < CarControllerParams.RESUME_TAP_FRAMES:
      increase = self.ICBM.sendButton == SendButtonState.increase
      can_sends.append(create_button_control(packer, CAN.camera, self.frame, CS.buttons_stock_values,
                                             resume=increase, decrease=not increase))
    self.button_frame += 1
    return can_sends
