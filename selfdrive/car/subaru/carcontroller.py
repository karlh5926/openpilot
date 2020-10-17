from common.numpy_fast import clip
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import DBC, PREGLOBAL_CARS
from opendbc.can.packer import CANPacker


class CarControllerParams():
  def __init__(self):
    self.STEER_MAX = 2047              # max_steer 4095
    self.STEER_STEP = 2                # how often we update the steer cmd
    self.STEER_DELTA_UP = 50           # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 70         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 60   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 10  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1       # from dbc

    self.RPM_MIN = 0                   # min cruise_rpm
    self.RPM_MAX = 3200                # max cruise_rpm
    self.RPM_BASE = 600                # cruise_rpm idle, from stock drive
    self.RPM_SCALE = 3000              # cruise_rpm, from testing

    self.RPM_DELTA_UP = 30
    self.RPM_DELTA_DOWN = 50

    self.THROTTLE_MIN = 0              # min cruise_throttle
    self.THROTTLE_MAX = 3400           # max cruise_throttle
    self.THROTTLE_BASE = 1818          # cruise_throttle, from stock drive
    self.THROTTLE_SCALE = 3000         # from testing

    self.THROTTLE_DELTA_UP = 30
    self.THROTTLE_DELTA_DOWN = 50

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_accel_cnt = -1
    self.es_rpm_cnt = -1
    self.es_lkas_cnt = -1
    self.fake_button_prev = 0
    self.steer_rate_limited = False

    self.cruise_rpm_last = 0
    self.cruise_throttle_last = 0

    self.params = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line):

    can_sends = []

    # *** steering ***
    if (frame % self.params.STEER_STEP) == 0:
      if (frame % (self.params.STEER_STEP * 10)) == 0:
        print("#### actuators.gas: ", actuators.gas)
      apply_steer = int(round(actuators.steer * self.params.STEER_MAX))

      # limits due to driver torque

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
      self.steer_rate_limited = new_steer != apply_steer

      if not enabled:
        apply_steer = 0

      if CS.CP.carFingerprint in PREGLOBAL_CARS:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, apply_steer, frame, self.params.STEER_STEP))
      else:
        can_sends.append(subarucan.create_steering_control(self.packer, apply_steer, frame, self.params.STEER_STEP))

      self.apply_steer_last = apply_steer

    # *** alerts and pcm cancel ***
    if CS.CP.carFingerprint in PREGLOBAL_CARS:

      if (frame % 100) == 0:
        print("#### leadCar, obstacleDistance, cruiseSetSpeed: ", CS.leadCar, CS.obstacleDistance, CS.out.cruiseState.speed)
        
      cruise_throttle = CS.es_accel_msg["Throttle_Cruise"]
      cruise_rpm = CS.es_rpm_msg["RPM"]

      P = self.params
      # slow down the signals change
      cruise_throttle = clip(cruise_throttle, self.cruise_throttle_last - P.THROTTLE_DELTA_DOWN, self.cruise_throttle_last + P.THROTTLE_DELTA_UP)
      cruise_rpm = clip(cruise_rpm, self.cruise_rpm_last - P.RPM_DELTA_DOWN, self.cruise_rpm_last + P.RPM_DELTA_UP)

      if self.es_rpm_cnt != CS.es_rpm_msg["Counter"]:
        can_sends.append(subarucan.create_es_rpm(self.packer, CS.es_rpm_msg, cruise_rpm))
        self.es_rpm_cnt = CS.es_rpm_msg["Counter"]
        self.cruise_rpm_last = cruise_rpm

      if self.es_accel_cnt != CS.es_accel_msg["Counter"]:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          fake_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          fake_button = 1
        else:
          fake_button = CS.button

        # unstick previous mocked button press
        if fake_button == 1 and self.fake_button_prev == 1:
          fake_button = 0
        self.fake_button_prev = fake_button

        can_sends.append(subarucan.create_es_throttle_control(self.packer, CS.es_accel_msg, fake_button, cruise_throttle))
        self.es_accel_cnt = CS.es_accel_msg["Counter"]
        self.cruise_throttle_last = cruise_throttle

    else:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        can_sends.append(subarucan.create_es_lkas(self.packer, CS.es_lkas_msg, visual_alert, left_line, right_line))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]

    return can_sends
