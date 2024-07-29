# Plan to add interpolated scaling factors
from collections import deque #used for moving averages
import numpy as np # used for calculationg predicted curvatures
from cereal import car # needed for pretty much everything in comma
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL # used for timing based calculations
from opendbc.can.packer import CANPacker # used for canbus messages
from openpilot.common.numpy_fast import clip, interp #used for lots of interpolations and clipping
from openpilot.selfdrive.car import apply_std_steer_angle_limits # for applying safety limits
from openpilot.selfdrive.car.ford import fordcan # ford specific canbus messages
from openpilot.selfdrive.car.ford.values import CarControllerParams, FordFlags, FordFlagsSP, FORD_VEHICLE_TUNINGS
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N # for control libraries
from openpilot.selfdrive.modeld.constants import ModelConstants # for calculations
from openpilot.common.params import Params

LongCtrlState = car.CarControl.Actuators.LongControlState # is long control engaged
VisualAlert = car.CarControl.HUDControl.VisualAlert # alerts on screen

# define the fuction to calculate curvature limits for Ford vehicles
def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    # clip the curvature value based on current curvature +/- the curvature error
    apply_curvature = clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                           current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit based on speed
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CarControllerParams)
  # ensure that apply_curvature does not exceed the min and max the car will accept.
  return clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)

# define the function for applying hysterisis to a value.
def hysteresis(current_value, old_value, target, stdDevLow: float, stdDevHigh: float):
  if target - stdDevLow < current_value < target + stdDevHigh:
    result = old_value
  elif current_value <= target - stdDevLow:
    result = 1
  elif current_value >= target + stdDevHigh:
    result = 0

  return result

# define the brake / gas actuator which utilizes a hysterisis to avoid brake spamming
def actuators_calc(self, brake):
  ts = self.frame * DT_CTRL
  brake_actuate = hysteresis(brake, self.brake_actuate_last, self.brake_actutator_target, self.brake_actutator_stdDevLow, self.brake_actutator_stdDevHigh)
  self.brake_actuate_last = brake_actuate

  # define the brake pre-charge actuator, also apply a hysterisis
  precharge_actuate = hysteresis(brake,
                                 self.precharge_actuate_last,
                                 self.precharge_actutator_target,
                                 self.precharge_actutator_stdDevLow,
                                 self.precharge_actutator_stdDevHigh)
  if precharge_actuate and not self.precharge_actuate_last:
    self.precharge_actuate_ts = ts
  elif not precharge_actuate:
    self.precharge_actuate_ts = 0

  # determine whether to apply brake pre-charge
  if (
      precharge_actuate and
      not brake_actuate and
      self.precharge_actuate_ts > 0 and
      brake > (self.precharge_actutator_target - self.precharge_actutator_stdDevLow) and
      (ts - self.precharge_actuate_ts) > (200 * DT_CTRL)
    ):
    precharge_actuate = False

  self.precharge_actuate_last = precharge_actuate

  return precharge_actuate, brake_actuate

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = fordcan.CanBus(CP)
    self.frame = 0

    self.apply_curvature_last = 0 # previous value of apply_curvature
    self.main_on_last = False # previous main cruise control state
    self.lkas_enabled_last = False # previous lkas state
    self.steer_alert_last = False # previous status of steering alert
    self.gac_tr_cluster_last = -1 # previous state of ui elements
    self.gac_tr_cluster_last_ts = 0 # prevous state of ui elements
    self.brake_actuate_last = 0 # previous state of brake actuator
    self.precharge_actuate_last = 0 # previous state of pre-charge actuator
    self.precharge_actuate_ts = 0 # previous state of pre-charge actuator
    self.lead_distance_bars_last = None
    self.last_log_frame = 0
    self.log_frames = 0

    # Activates at self.brake_actutator_target - self.brake_actutator_stdDevLow
    self.brake_actutator_target = -0.45 # Default: -0.5
    self.brake_actutator_stdDevLow = 0.05 # Default: -0.5

    # Deactivates at self.brake_actutator_target + self.brake_actutator_stdDevHigh
    self.brake_actutator_stdDevHigh = 0.45 # Default: 0

    self.precharge_actutator_target = -0.2
    self.precharge_actutator_stdDevLow = 0.05
    self.precharge_actutator_stdDevHigh = 0.2

    self.brake_0_point = -0.1
    self.brake_converge_at = -1.5
    self.testing_active = False

    # Deactivates at self.precharge_actutator_target + self.precharge_actutator_stdDevHigh
    self.target_speed_multiplier = 1 # Default: 0

    # set braking tuning based on which vehicle is being driven
    print(f'Car Fingerprint: {self.CP.carFingerprint}')

    # Ford Model Specific Tuning
    if self.CP.flags & FordFlags.CANFD:
      self.testing_active = True
      self.path_lookup_time = 0.75 #lookup time for path_angle and path_offset, needs to be further into future than curvature and curvature rate

      # Check FORD_VEHICLE_TUNINGS has a key for the carFingerprint
      if self.CP.carFingerprint in FORD_VEHICLE_TUNINGS:
        print(f'Matched carFingerprint in CarController | FingerPrint: {self.CP.carFingerprint}')
        self.path_lookup_time =               FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["path_lookup_time"]
        self.reset_lookup_time =              FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["reset_lookup_time"]
        self.brake_actutator_target =         FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["brake_actutator_target"]
        self.brake_actutator_stdDevLow =      FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["brake_actutator_stdDevLow"]
        self.brake_actutator_stdDevHigh =     FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["brake_actutator_stdDevHigh"]
        self.precharge_actutator_target =     FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["precharge_actutator_target"]
        self.precharge_actutator_stdDevLow =  FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["precharge_actutator_stdDevLow"]
        self.precharge_actutator_stdDevHigh = FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]["precharge_actutator_stdDevHigh"]
        print(f'Fingerprint Specific Tuning Values: {FORD_VEHICLE_TUNINGS[self.CP.carFingerprint]}')

    self.brake_clip = self.brake_actutator_target - self.brake_actutator_stdDevLow


    # lateral control parameters
    self.lat_mode = 1 # initialize lateral control mode. 0 = None, 1 = PathFollowingLimitedMode
    self.apply_curvature_filtered = FirstOrderFilter(0., 0.3, 0.05) # filter for apply_curvature
    self.curvature_lookup_time = CP.steerActuatorDelay + 0.3  # how far into the future do we need to look for curvature signal
    self.reset_lookup_time = 0.5 # how long does it take the steering wheel to unwind to zero from the maximum predicted curvature
    self.path_lookup_time = 0.5 # how far into the future to we need to look for our path_angle signals.
    self.curvature_rate_delta_t = 0.3 # [s] used in denominator for curvature rate calculation
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t/0.05))) # 0.3 seconds at 20Hz
    # These parameters decide how early and how hard to ramp in path_angle and path_offset signals based on the predicted curvature.
    # These parameters exists so that path_angle and path_offset are not affecting control at low curvature levels.
    self.po_bp = [0.0145, 0.0165] # path_offset scaling range versus curvature in /m
    self.po_v = [0.0, 1.0] # path_offset scaling factor versus curvature in /m
    self.pa_bp = [0.0145, 0.0165] # path_offset scaling range versus curvature in /m
    self.pa_v = [0.0, 1.0] # path_offset scaling factor versus curvature in /m
##### These parameters determine how far into the future we look to calculate our control variables. higher curvatures need longer lead times
    self.reset_lookup_time_min = 0.25 # minimum lookup time
    self.reset_lookup_time_max = 2.0 + self.reset_lookup_time # maximum future time to consider for resetting steering
    self.path_lookup_time_min = 0.5 # minimum future time to consider for path angle and offset variables
    self.path_lookup_time_max = 0.75 + self.path_lookup_time # maximum future time to consider for path variables
    # At what curvature do we start using a longer lead time to calcualte our control variables
    self.reset_lookup_time_bp = [0.02, 0.06] # curvature lookup time versus curvature in /m
    self.path_lookup_time_bp = [0.018, 0.04] # path_offset lookup time versus curvature in /m
    # calculate the lookup times based on the predicted curvature
    self.reset_lookup_time_v = [self.reset_lookup_time_min, self.reset_lookup_time_max] # reset lookup time versus curvature in /m
    self.path_lookup_time_v = [self.path_lookup_time_min, self.path_lookup_time_max] # path_offset lookup time versus curvature in /m
    self.path_angle_bp = [0.0135, 0.0155] # path_offset_curvature 1/m
    self.path_angle_v = [0.0, 1.0] # path_angle_ramp goes from 0% applied to 100% as curvature goes from 0.0135 to 0.0155
    self.large_curve_active = False # is a large curve active
    self.large_curve_reset = False
    self.lane_change_factor = 0.65
    self.precision_type = 1
    self.pc_blend_ratio_bp = [17.8, 31.3] # 39.8 mph & 70mph
    self.pc_blend_ratio_v = [0.6, 0.4] # 60% to 40% predicted curvature from 39.8 to 70 mph

    # max absolute values for all four signals
    self.path_angle_max = 0.5 # from dbc files
    self.path_offset_max = 5.11 # from dbc files
    self.curvature_max = 0.02 # from dbc files
    self.curvature_rate_max = 0.001023 # from dbc files

    # human turn detection parameters
    self.human_turn_dur_threshold = 0.5 # how long does a human need to be turning the wheel before we consider it an over-ride instead of an adjustment
    self.human_turn_dur_frames = self.human_turn_dur_threshold / (DT_CTRL * 5) # convert time to scans
    self.human_turn_frames = 0 # how many scans has a human been turning the wheel
    self.human_turn = False # have we detected a human override in a turn

    # values from previous frame
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate = 0 # initialize curvature_rate

    # Count cycles for logging
    self.cycle_count = 0

    # Define the Ford Variables object
    self.fordVariables = None

  def update(self, CC, CS, now_nanos, model_data=None):
    can_sends = []

    actuators = CC.actuators # create a shortcut for actuators
    hud_control = CC.hudControl # create a shortcut for hud
    if self.fordVariables is None:
      act = actuators.copy()
      self.fordVariables = act.fordVariables

    main_on = CS.out.cruiseState.available # main cruise control button status
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw) # alerts on screen and dashboard
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw # alert on screen and dashboard

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:

      if CC.latActive:
        self.precision_type = 1
        # calculate current curvature and model desired curvature
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1) # use canbus data to calculate current_curvature
        self.fordVariables.currentCurvature01 = float(current_curvature)
        desired_curvature = actuators.curvature # get desired curvature from model
        self.fordVariables.desiredCurvature01 = float(desired_curvature)

        # use the model position data to calculate the predicted path curvature at different time intervals
        if model_data is not None and len(model_data.orientation.x) >= CONTROL_N:
          # compute curvature from model predicted orientationRate, and blend with desired curvature based on max predicted curvature magnitude
          curvatures = np.array(model_data.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
          max_abs_predicted_curvature = max(np.abs(curvatures[:CONTROL_N])) # max curvature magnitude over next 2.5s
          self.fordVariables.maxAbsPredictedCurvature01 = float(max_abs_predicted_curvature)

          # calculate lookup time based on the max predicted curvature
          self.reset_lookup_time = interp(max_abs_predicted_curvature, self.reset_lookup_time_bp, self.reset_lookup_time_v)
          self.reset_lookup_time = 0.5 #hard code to see if lat_mode = 0 is a fixed delay time
          self.fordVariables.resetLookupTime01 = float(self.curvature_lookup_time)
          self.path_lookup_time = interp(max_abs_predicted_curvature, self.path_angle_bp, self.path_lookup_time_v)
          self.fordVariables.pathLookupTime01 = float(self.path_lookup_time)

          # calculate predicted curvature used for the curvature and curvature_rate variables
          predicted_curvature = interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
          self.fordVariables.predictedCurvature01 = float(predicted_curvature)

          # calculate predicted curvature used for resetting variables during a large curve
          reset_curvature = interp(self.reset_lookup_time, ModelConstants.T_IDXS, curvatures)
          self.fordVariables.resetCurvature01 = float(predicted_curvature)

          # calcualte the curvature used for the path_angle and path_offset variables.
          predicted_path_curvature = interp(self.path_lookup_time, ModelConstants.T_IDXS, curvatures)
          self.fordVariables.predictedPathCurvature01 = float(predicted_path_curvature)

          # capture if a large curve or turn is active
          if abs(predicted_path_curvature) > 0.02:
            self.large_curve_active = True
          self.fordVariables.largeCurveActive01 = float(self.large_curve_active)

          # clear large curve active flag when acrtual curvature is less than 0.015
          if self.large_curve_active:
            if abs(current_curvature) < 0.015:
              self.large_curve_active = False
              self.large_curve_reset = False
              self.fordVariables.largeCurveActive02 = float(self.large_curve_active)

          # equate apply_curvature to a blend of desired and predicted_curvature and apply curvature limits
          pc_blend_ratio = interp(CS.out.vEgoRaw, self.pc_blend_ratio_bp, self.pc_blend_ratio_v)
          apply_curvature = (predicted_curvature * pc_blend_ratio) + (desired_curvature * (1 - pc_blend_ratio))
          apply_curvature = self.apply_curvature_filtered.update(apply_curvature)
          self.fordVariables.applyCurvature01 = float(apply_curvature)

          # compute curvature rate
          self.curvature_rate_deque.append(apply_curvature)
          if len(self.curvature_rate_deque) > 1:
            delta_t = self.curvature_rate_delta_t if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen else (len(self.curvature_rate_deque) - 1) * 0.05
            desired_curvature_rate = (self.curvature_rate_deque[-1] - self.curvature_rate_deque[0]) / delta_t / max(0.01, CS.out.vEgoRaw)
          else:
            desired_curvature_rate = 0.0
          self.fordVariables.desiredCurvatureRate01 = float(desired_curvature_rate)

          # path_offset and path_angle signals have a linear relationship with the amount of curvature
          # they produce, so they are calculated based on the desired curvature.
          # These values were determined from injection testing data in a F-150.
          po_scaling_factor = interp(abs(predicted_path_curvature), self.po_bp, self.po_v)
          self.fordVariables.poScalingFactor01 = float(po_scaling_factor)
          pa_scaling_factor = interp(abs(predicted_path_curvature), self.pa_bp, self.pa_v)
          self.fordVariables.paScalingFactor01 = float(pa_scaling_factor)
          path_offset = (68.44 * predicted_path_curvature - 1.17) * po_scaling_factor
          path_angle = (6.84 * predicted_path_curvature - 0.117) * pa_scaling_factor
          self.fordVariables.pathOffset01 = float(path_offset)
          self.fordVariables.pathAngle01 = float(path_angle)

          #clip all values
          apply_curvature = clip(apply_curvature, -self.curvature_max, self.curvature_max)
          desired_curvature_rate = clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max)
          path_offset = clip(path_offset, -self.path_offset_max, self.path_offset_max)
          path_angle = clip(path_angle, -self.path_angle_max, self.path_angle_max)

          self.fordVariables.applyCurvature02 = float(apply_curvature)
          self.fordVariables.desiredCurvatureRate02 = float(desired_curvature_rate)
          self.fordVariables.pathOffset02 = float(path_offset)
          self.fordVariables.pathAngle02 = float(path_angle)

          # if large curve is active, and path curvature is lower than predicted curvature, we are near peak, zero out all signals when the reset_curvature is less than 0.01
          if self.large_curve_active:
            if abs(interp(self.path_lookup_time, ModelConstants.T_IDXS, curvatures)) < abs(interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures)):
              if reset_curvature < 0.005:
                self.large_curve_reset = True

          if self.large_curve_reset:
            self.lat_mode = 0
          else:
            self.lat_mode = 1

          # determine if a lane change is active
          if model_data.meta.laneChangeState == 1 or model_data.meta.laneChangeState == 2:
            self.lane_change = True
          else:
            self.lane_change = False

          # if changing lanes, reduce apply_curvature by a factor smooth out the lane change.
          if self.lane_change:
            apply_curvature = apply_curvature * self.lane_change_factor
            self.precision_type = 0 # comfort for lane change

          # Determine if a human is making a turn and trap the value
          steeringPressed = CS.out.steeringPressed
          steeringAngleDeg = CS.out.steeringAngleDeg
          if steeringPressed and (abs(steeringAngleDeg) > 60):
            self.human_turn_frames += 1
          else:
            self.human_turn_frames = 0
          if self.human_turn_frames >= self.human_turn_dur_frames:
            self.human_turn = True
          self.fordVariables.humanTurn01 = float(self.human_turn)

          if self.human_turn:
            if not steeringPressed and abs(steeringAngleDeg) < 10:
              self.human_turn = False
              self.fordVariables.humanTurn02 = float(self.human_turn)

          #Determine when to reset steering
          if self.human_turn or self.large_curve_reset:
            reset_steering = 1
          else:
            reset_steering = 0

          # reset steering by setting all values to 0 and ramp_type to immediate
          if reset_steering == 1:
            apply_curvature = 0
            path_offset = 0
            path_angle = 0
            desired_curvature_rate = 0
            ramp_type = 3
            self.apply_curvature_filtered.x = 0.0
          else:
            ramp_type = 2
        else:
          desired_curvature_rate = 0.0
          path_offset = 0.0
          path_angle = 0.0
          self.apply_curvature_filtered.x = 0.0
          ramp_type = 0
      else:
        apply_curvature = 0.
        desired_curvature_rate = 0.0
        path_offset = 0.0
        path_angle = 0.0
        self.apply_curvature_filtered.x = 0.0
        ramp_type = 0

      self.apply_curvature_last = apply_curvature
      self.curvature_rate_last = desired_curvature_rate
      self.path_offset_last = path_offset
      self.path_angle_last = path_angle

      if self.CP.flags & FordFlags.CANFD:
        mode = 1 if CC.latActive else 0
        if self.lat_mode == 0:
          mode = 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        if self.CP.spFlags & FordFlagsSP.SP_ENHANCED_LAT_CONTROL.value:
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode,
                                                      ramp_type,
                                                      self.precision_type,
                                                      -path_offset,
                                                      -path_angle,
                                                      -apply_curvature,
                                                      -desired_curvature_rate,
                                                      counter))
        else:
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -apply_curvature, 0., counter))

      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive,
                                                    ramp_type,
                                                    1,
                                                    -path_offset,
                                                    -path_angle,
                                                    -apply_curvature,
                                                    -desired_curvature_rate))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # Both gas and accel are in m/s^2, accel is used solely for braking
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      gas = accel
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS
      stopping = CC.actuators.longControlState == LongCtrlState.stopping

      precharge_actuate, brake_actuate = actuators_calc(self, accel)
      brake = accel
      if brake < 0 and brake_actuate:
        brake = interp(accel, [ CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_clip], [CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_0_point])

      # Calculate targetSpeed
      targetSpeed = clip(actuators.speed * self.target_speed_multiplier, 0, V_CRUISE_MAX)
      if not CC.longActive and hud_control.setSpeed:
        targetSpeed = hud_control.setSpeed

      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, brake, stopping, brake_actuate, precharge_actuate, v_ego_kph=targetSpeed))

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, hud_control,
                                                 CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.copy()
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.fordVariables = self.fordVariables

    self.frame += 1
    return new_actuators, can_sends