'''
Good PD Values without Disturbance
python3 main.py kp=5 ki=0 kd=5 sim_time=10 disturbance_flag=False

Good PID Values with Disturbance
python3 main.py kp=700 ki=0 kd=7500 sim_time=10 disturbance_flag=True

'''

import utils
import time

class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd
        self.params = utils.CrazyflieParams
        self.E_last = 0
        self.E_accum = 0

    def compute_commands(self, setpoint, state, time_delta):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """

        E = setpoint.z_pos - state.z_pos

        # Two ways of getting E_dot
        # E_dot = (E-self.E_last)/time_delta # if we use this divide it by the elapsed time
        E_dot = setpoint.z_vel - state.z_vel

        self.E_last = E
        self.E_accum += E

        U = 0 # force for upward thrusts
        # PD
        # z_accel = self.kd_z*E_dot+self.kp_z*E
        #PID 
        z_accel = self.kd_z*E_dot+self.kp_z*E+self.ki_z*self.E_accum
        U = self.params.mass*(z_accel+self.params.g)

        return U
