# PID attitude rate control for quadcopters

import numpy as np
from controllers.pid_params import *
from datatypes.stabilizer_types import State_t, Attitude_t, Setpoint_t, Velocity_t

DEFAULT_PID_INTEGRATION_LIMIT = 5000.0
DEFAULT_PID_OUTPUT_LIMIT = None

class PidObject:
    def __init__(self, kp, ki, kd, kff, dt):
        self.error = 0
        self.prev_error = 0
        self.integ = 0
        self.deriv = 0
        self.desired = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.i_limit = DEFAULT_PID_INTEGRATION_LIMIT
        self.output_limit = DEFAULT_PID_OUTPUT_LIMIT
        self.dt = dt
        
    def update(self, measured: float, update_error: bool):
        output = 0.0

        if update_error:
            self.error = self.desired - measured

        output += self.kp * self.error

        self.deriv = (self.error - self.prev_error) / self.dt

        output += self.kd * self.deriv

        self.integ = self.error * self.dt

        if self.i_limit != 0:
            self.integ = np.clip(self.integ, -self.i_limit, self.i_limit)
        
        output += self.ki * self.integ

        output += self.kff * self.desired

        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)
        
        self.prev_error = self.error

        return output
    
    def reset(self):
        self.error = 0
        self.prev_error = 0
        self.integ = 0
        self.deriv = 0

    def run(self, input: float, setpoint: float):
        self.desired = setpoint
        return self.update(input, True)


class PidControl:
    def __init__(self):
        self.dt = 0.01
        self.pid_x = PidObject(PID_POS_X_KP, PID_POS_X_KI, PID_POS_X_KD, PID_POS_X_KFF, self.dt)
        self.pid_y = PidObject(PID_POS_Y_KP, PID_POS_Y_KI, PID_POS_Y_KD, PID_POS_Y_KFF, self.dt)
        self.pid_z = PidObject(PID_POS_Z_KP, PID_POS_Z_KI, PID_POS_Z_KD, PID_POS_Z_KFF, self.dt)
        self.pid_vx = PidObject(PID_VEL_X_KP, PID_VEL_X_KI, PID_VEL_X_KD, PID_VEL_X_KFF, self.dt)
        self.pid_vy = PidObject(PID_VEL_Y_KP, PID_VEL_Y_KI, PID_VEL_Y_KD, PID_VEL_Y_KFF, self.dt)
        self.pid_vz = PidObject(PID_VEL_Z_KP, PID_VEL_Z_KI, PID_VEL_Z_KD, PID_VEL_Z_KFF, self.dt)
        self.pid_roll = PidObject(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_ROLL_KFF, self.dt)
        self.pid_pitch = PidObject(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_KFF, self.dt)
        self.pid_yaw = PidObject(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_KFF, self.dt)

        self.pid_roll.i_limit = PID_ROLL_INTEGRATION_LIMIT
        self.pid_pitch.i_limit = PID_PITCH_INTEGRATION_LIMIT
        self.pid_yaw.i_limit = PID_YAW_INTEGRATION_LIMIT

    def position_controller(self, setpoint: Setpoint_t, state: State_t):
        self.pid_x.output_limit = PID_POS_VEL_X_MAX * 1.1
        self.pid_y.output_limit = PID_POS_VEL_Y_MAX * 1.1
        self.pid_z.output_limit = max(PID_POS_VEL_Z_MAX, 0.5) * 1.1

        cosyaw = np.cos(state.attitude.yaw * np.pi / 180.0)
        sinyaw = np.sin(state.attitude.yaw * np.pi / 180.0)

        setp_body_x = setpoint.position.x * cosyaw + setpoint.position.y * sinyaw
        setp_body_y = -setpoint.position.x * sinyaw + setpoint.position.y * cosyaw

        state_body_x = state.position.x * cosyaw + state.position.y * sinyaw
        state_body_y = -state.position.x * sinyaw + state.position.y * cosyaw

        setpoint_velocity = Velocity_t(0, 0, 0, 0)
        # WARNING: we assume that setpoint->mode.x == modeAbs, which is normal operation for the Crazyflie
        setpoint_velocity.x = self.pid_x.run(state_body_x, setp_body_x)
        setpoint_velocity.y = self.pid_y.run(state_body_y, setp_body_y)
        setpoint_velocity.z = self.pid_z.run(state.position.z, setpoint.position.z)

        thrust, attitude_des = self.velocity_controller(setpoint_velocity, state)
        attitude_rate = self.attitude_controller(state.attitude.roll, state.attitude.pitch, state.attitude.yaw, 
                                                 attitude_des.roll, attitude_des.pitch, attitude_des.yaw)
        return thrust, attitude_rate

    def velocity_controller(self, setpoint_velocity: Velocity_t, state: State_t):
        self.pid_vx.output_limit = PID_VEL_PITCH_MAX * 1.1        
        self.pid_vy.output_limit = PID_VEL_ROLL_MAX * 1.1
        self.pid_vz.output_limit = 65535 / 2 / 1000
        
        cosyaw = np.cos(state.attitude.yaw * np.pi / 180.0)
        sinyaw = np.sin(state.attitude.yaw * np.pi / 180.0)

        state_body_vx = state.velocity.x * cosyaw + state.velocity.y * sinyaw
        state_body_vy = -state.velocity.x * sinyaw + state.velocity.y * cosyaw

        attitude = Attitude_t(0, 0, 0, 0)
        attitude.pitch = -self.pid_vx.run(state_body_vx, setpoint_velocity.x)
        attitude.roll = -self.pid_vy.run(state_body_vy, setpoint_velocity.y)

        thrust_raw = self.pid_vz.run(state.velocity.z, setpoint_velocity.z)

        thrust = thrust_raw * 1000 + PID_VEL_THRUST_BASE

        thrust = np.clip(thrust, PID_VEL_THRUST_MIN, 65535)

        return thrust, attitude
    
    def attitude_controller(self, roll, pitch, yaw, roll_des, pitch_des, yaw_des):
        roll_rate_des = self.pid_roll.run(roll, roll_des)
        pitch_rate_des = self.pid_pitch.run(pitch, pitch_des)
        
        yaw_error = yaw_des - yaw
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360
        
        self.pid_yaw.error = yaw_error
        yaw_rate_des = self.pid_yaw.update(yaw, False)

        return Attitude_t(0, roll_rate_des, pitch_rate_des, yaw_rate_des)