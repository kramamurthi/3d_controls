"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""
        # Body Rate P-controller Gains
        # Units: Newton-m-s^2/radian
        self.k_p_p = 17.5
        self.k_p_q = 17.5
        self.k_p_r = 17.5

        # Altitude Controller PD(ff) Gains
        self.z_k_p = 50.0
        self.z_k_d = 10.0

        # Yaw Controller P Gain
        self.k_p_yaw = 5.0
 
        # Roll Pitch Controller
        # Units: Rad*s/m 
        self.k_p_roll = 1 
        self.k_p_pitch = 1 
 
        # Lateral Position Controller
        self.x_k_p = 5.0
        self.x_k_d = 1.5
        self.y_k_p = 5.0
        self.y_k_d = 1.5

        # Global G
        self.g = -GRAVITY
        self.m = DRONE_MASS_KG
        return    

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """

        x_target, y_target = local_position_cmd
        x_dot_target, y_dot_target = local_velocity_cmd
        x_actual, y_actual = local_position
        x_dot_actual, y_dot_actual = local_velocity
        x_dot_dot_ff, y_dot_dot_ff = acceleration_ff

        e_x = x_target - x_actual
        e_x_dot = x_dot_target - x_dot_actual
        x_dot_dot = self.x_k_p * e_x + self.x_k_d * e_x_dot + x_dot_dot_ff
    
        e_y = y_target - y_actual
        e_y_dot = y_dot_target - y_dot_actual
        y_dot_dot = self.y_k_p * e_y + self.y_k_d * e_y_dot + y_dot_dot_ff
        
        return np.array([x_dot_dot, y_dot_dot])
        
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """
        # Since all values are provided (+up) and required thrust is also (+up) no need to negate 
        # values in accordance with usual convention. Rather negate g.

        error = altitude_cmd - altitude
        error_dot = vertical_velocity_cmd - vertical_velocity
        # Attitude (world space) 
        roll, pitch, yaw = attitude
        rotation_matrix = euler2RM(roll, pitch, yaw)

        # Component of rotation matrix along z direction
        b_z = rotation_matrix[2][2]

        # Feed-forward PD Controller
        u_bar_1 = self.z_k_p * error + self.z_k_d * error_dot + acceleration_ff

        # make u_bar_1 (+down)
        u_bar_1 = -u_bar_1

        # Get the thrust command (+down)
        c = self.m*(u_bar_1)/b_z

        # Return thrust command (+up)
        return np.clip(-c,0,MAX_THRUST)
        
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        x_target, y_target = acceleration_cmd
        roll, pitch, yaw = attitude
        
        rot_mat = euler2RM(roll, pitch, yaw)

        # Divide by mass to get units in acceleration for horizontal thrusts.
        x_actual = rot_mat[0][2] *  (-thrust_cmd/self.m)
        y_actual = rot_mat[1][2] *  (-thrust_cmd/self.m)
    
        b_x_dot = self.k_p_roll*(x_target - x_actual)
        b_y_dot = self.k_p_pitch*(y_target - y_actual)
    
        t_matrix = np.matrix([
                             [rot_mat[1][0], -rot_mat[0][0]],
                             [rot_mat[1][1], -rot_mat[0][1]]
                            ])
    
        a = np.array(t_matrix * np.array([b_x_dot, b_y_dot]).reshape(2,1))/rot_mat[2][2] 
        p = -float(a[0])
        q = -float(a[1])
    
        return np.array([p, q])


    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        p_cmd, q_cmd, r_cmd = body_rate_cmd
        p_actual, q_actual, r_actual = body_rate

        # In Rad/s^2
        p_e = p_cmd - p_actual
        q_e = q_cmd - q_actual
        r_e = r_cmd - r_actual

        u_bar_p = self.k_p_p * p_e
        u_bar_q = self.k_p_q * q_e
        u_bar_r = self.k_p_r * r_e
       
        u_p = np.clip(u_bar_p * MOI[0], -MAX_TORQUE, MAX_TORQUE)
        u_q = np.clip(u_bar_q * MOI[1], -MAX_TORQUE, MAX_TORQUE) 
        u_r = np.clip(u_bar_r * MOI[2], -MAX_TORQUE, MAX_TORQUE) 

        return np.array([u_p, u_q, u_r])
    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """
        # Find the shorter direction of rotation.
        # It could be one revolution ahead or behind
        e1 = yaw_cmd - yaw
        e2 = yaw_cmd + 2*np.pi - yaw
        e3 = yaw_cmd - 2*np.pi - yaw

        errors = [e1, e2, e3]
        abs_errors = [abs(x) for x in errors]
        min_loc = abs_errors.index(min(abs_errors))
        min_yaw_error = errors[min_loc]

        r_c = self.k_p_yaw * min_yaw_error
        return r_c 
    
