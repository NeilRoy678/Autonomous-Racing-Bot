import rclpy
from rclpy.node import Node

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nxp_cup_interfaces.msg import PixyVector
from time import sleep
from datetime import datetime
import numpy as np
prev_error=0

class LineFollow(Node):

    def __init__(self):
        super().__init__('aim_line_follow')
        
        
        self.start_delay = 1.0

        self.camera_vector_topic = "/cupcar0/PixyVector"

        self.linear_velocity = 1.0

        self.angular_velocity = 0.175

        self.single_line_steer_scale = 1.0

         # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        self.start_time = datetime.now().timestamp()
        self.restart_time = True

        # Subscribers
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

        # Timer setup
        # timer_period = 0.5 #seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def get_num_vectors(self, msg):
        num_vectors = 0
        if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
            num_vectors = num_vectors + 1
        if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
            num_vectors = num_vectors + 1
        return num_vectors

    # def timer_callback(self):
    #     #TODO

    def listener_callback(self, msg):
        #TODO
        
        # =============================================PID Variables====================================================
        
        # prev_error is a global variable that stores the error value of the previous iteration, and it is updated after
        # each iteration.
        # It is used to calculate the steer value in the final PID algorithm equation
        global prev_error
        
        # Kp is the proportional gain
        # Increasing the proportional gain (Kp) proportionally increases the control signal for the same level of error.
        # More Kp for the given level of error tends to cause the closed-loop system to react more quickly, but also
        # to overshoot more. Kp reduces, but not eliminate, the steady-state error. 
        kp=0.05
        
        # Kd is the derivative gain
        # With derivative control, the control signal can become large if the error begins sloping upward, even while
        # the magnitude of the error is still relatively small. This adds damping to the system, thereby decreasing 
        # overshoot. The addition of a derivative term, however, has no effect on the steady-state error. 
        kd=0.9
        
        # Ki is the Integral gain
        # The addition of an integral term to the controller Ki tends to help reduce steady-state error. If there is 
        # a persistent, steady error, the integrator builds and builds, thereby increasing the control signal and 
        # driving the error down. A drawback of the integral term, however, is that it can make the system more sluggish
        # (and oscillatory) since when the error signal changes sign, it may take a while for the integrator to unwind.
        # The Ki value is set to 0.0 since there is no persistant error in our system
        ki=0.0
        
        # ==============================================================================================================
        
        current_time = datetime.now().timestamp()
        frame_width = 72
        frame_height = 52
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 0
        num_vectors = self.get_num_vectors(msg)
        if(num_vectors == 0):
            if self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = False
            if (self.start_time+4.0) > current_time:
                speed = self.linear_velocity * (4.0-(current_time-self.start_time))/4.0
            if (self.start_time+4.0) <= current_time:
                speed = 0
            steer = 0

        if(num_vectors == 1):
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            if(msg.m0_x1 > msg.m0_x0):
                x = (msg.m0_x1 - msg.m0_x0) / frame_width
                y = (msg.m0_y1 - msg.m0_y0) / frame_height
            else:
                x = (msg.m0_x0 - msg.m0_x1) / frame_width
                y = (msg.m0_y0 - msg.m0_y1) / frame_height
            if(msg.m0_x0 != msg.m0_x1 and y != 0):
                steer = (self.angular_velocity) * (x / y) * self.single_line_steer_scale
                if (self.start_time+4.0) > current_time:
                    speed = self.linear_velocity * ((current_time-self.start_time)/4.0)
                if (self.start_time+4.0) <= current_time:
                    speed = self.linear_velocity
            else:
                steer = 0
                if (self.start_time+4.0) > current_time:
                    speed = self.linear_velocity * ((current_time-self.start_time)/4.0)*0.4
                if (self.start_time+4.0) <= current_time:
                    # The speed is scaled to 70% of linear velocity
                    speed = self.linear_velocity*0.7

        if(num_vectors == 2):
            
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
                
            #this logic is used for extreme right turn when m1_y1 >0
            #the steer value is scaled dynamically with the value of m1_y1
            if (msg.m1_y1!= 0):
                steer = -(self.angular_velocity*msg.m1_y1)/10
	    
	    # =====================================PID ALgorithm implementation=============================================
	    
	    # --------------------------------------------------------------------------------------------------------------
	    # m0_xi and m1_xi are initial values of m0x1 and m1_x1 which are assumed to be at the center of the track
	    # The error is calculated as the difference between the current values of m0_x1 and m1_x1, and the ideal values
	    # The error value is the sum of the differences from m0 and m1 vectors
            m0_xi=23
            m1_xi=48
            error1=(m0_xi-msg.m0_x1)
            error2=(m1_xi-msg.m1_x1)
            error=error1+error2
            # --------------------------------------------------------------------------------------------------------------
            
            # --------------------------------------------------------------------------------------------------------------
            # the total error is calculated with the help of PID equation 
            # Output=Kp×error+Kd×δerror/δt+Ki×∑error
            # Kp, Kd and Ki are the proportional gain, derivative gain and the Integral gain respectively
            # The values of Kp Kd and Ki are decided after rigourous testing and tuning
            
            total_error = kp * (error) + kd * (error - prev_error) + ki * (error + prev_error)
            prev_error=error
            # --------------------------------------------------------------------------------------------------------------
            
            # ==============================================================================================================
            steer = self.angular_velocity * total_error
            if (self.start_time+4.0) > current_time:
                speed = self.linear_velocity * ((current_time-self.start_time)/9.0)
            if (self.start_time+4.0) <= current_time:
                speed = self.linear_velocity*0.7
            # condition for extreme right turn
            if (msg.m1_y1!= 0):
                steer = -(self.angular_velocity*msg.m1_y1)/10
        
        self.speed_vector.x = float(speed*(1-np.abs(0.01*steer)))
        # slight delay for avoiding going off track in extreme left turns
        if(steer>0.6):
                sleep(0.07)
        self.steer_vector.z = float(steer)

        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector

        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    line_follow = LineFollow()

    rclpy.spin(line_follow)

    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()