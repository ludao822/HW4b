# Simple example of taxiing as scripted behavior
# import Pilot; a=Pilot.Pilot(); a.start()

import Ckpt as Ckpt, math, numpy
import imp, sys, Utilities


def rel():
    imp.reload(sys.modules['Pilot'])


class Pilot (Ckpt.Ckpt):  # subclass of the class Ckpt in the file Ckpt

    def __init__(self, tsk='HW4a', rc=False, gui=False):
        super().__init__(tsk, rc, gui)
        self.strtTime = None
        self.duration = None

        self.throttle_gains = {'P': 1.0/400, 'I': 1.0/1000, 'D': 1.0/800}    # PID gains for the throttle
        self.rudder_gains = {'P': 0.03, 'I': 0.0, 'D': 0.00}                # PID gains for the rudder
        self.pitch_gains_down = {'P': -0.03, 'I': -0.01, 'D': 0.01}        # PID gains for the elevator when going down
        self.pitch_gains_up = {'P': -0.06, 'I': -0.015, 'D': 0.006}          # PID gains for the elevator when going up
        self.roll_gains = {'P': 0.02, 'I': 0.001, 'D': 0.001}               # PID gains for the elevator
        self.pitch_Hold = {'P': -0.04, 'I': -0.01, 'D': -0.001}

        self.throttle_PID = None
        self.rudder_PID = None
        self.pitch_PID = None
        self.roll_PID = None

        self.state = "Initialize"
        self.start_speed = 0
        self.desired_pitch = 0
        self.prev_time = 0
        self.prev_altitude = 0
        self.gamma = []

        self.desired_altitude = 0
        self.desired_gamma = 0 
        self.desired_speed = 0 
        self.stabilize_flag = 0
        self.stabilize_time = None
        self.going_up = False
        self.set_altitude_flag = False
        self.overshoot = 0
        self.holdFlag = False

        self.PC_flag = True

        self.myflag = 0

    def ai(sf,fDat,fCmd,desired_altitude_change = -1200, desired_gamma = -10, desired_velocity=200):
        if not fDat.running:
            if sf.PC(fDat, desired_altitude_change, desired_gamma, desired_velocity) != "OK":
                desired_altitude_change, desired_gamma, desired_velocity = sf.PC(fDat, desired_altitude_change, desired_gamma, desired_velocity)
            sf.PLAN(fDat,desired_altitude_change, desired_gamma, desired_velocity)
            
            sf.strtTime = fDat.time
            fCmd.throttle = 0.9
            
        else:
            sf.DO(fDat,fCmd)
    '''
    def ai(self, fDat, fCmd):

        if not self.set_altitude_flag:
            if self.desired_altitude:
                self.desired_altitude += fDat.altitude
                self.set_altitude_flag = True
    '''    

        # Implementing a simple throttle hold
        #fCmd.throttle = self.throttle_PID.control(fDat.kias, [self.start_speed, 0], fDat.time) + 0.7 if self.myflag == 1 else 0.8

        # Implementing a simple roll PID
       
    def PC(self,fDat,desired_altitude_change, desired_gamma, desired_velocity):
        self.desired_gamma = desired_gamma
        self.desired_altitude = fDat.altitude + desired_altitude_change
        self.desired_speed = desired_velocity
        print(str(self.desired_altitude))
        self.desired_velocity = desired_velocity
        # Some basic attainable values. Will be overwritten by respective cases
        reachable_gamma = self.desired_gamma
        reachable_altitude = self.desired_altitude
        reachable_speed = self.desired_speed

        # Can't go up if desired flight path angle is negative and vice versa
        if self.desired_gamma*self.desired_altitude < 0:
            self.PC_flag = False
            reachable_altitude = -self.desired_altitude

        # Its not recommended to go beyond this desired flight path angle
        if abs(self.desired_gamma) > 25:
            self.PC_flag = False
            if self.desired_gamma < 0:
                reachable_gamma = -25
            else:
                reachable_gamma = 25

        if self.desired_velocity <= 0:
            self.PC_flag = False

            # Artificially bound the lower limit. Might be slightly higher or lower depending on aircraft
            reachable_speed = 50

        if self.desired_velocity >= 300:
            self.PC_flag = False
            reachable_speed = 300

        if self.PC_flag:
            return 'OK'
        else:
            return [reachable_gamma, reachable_altitude, reachable_speed]

    def PLAN(self,fDat,desired_altitude_change, desired_gamma, desired_velocity):
        self.throttle_gains = {'P': 1.0/400, 'I': 1.0/1000, 'D': 1.0/800}    # PID gains for the throttle
        self.rudder_gains = {'P': 0.03, 'I': 0.0, 'D': 0.00}                # PID gains for the rudder
        self.pitch_gains_down = {'P': -0.03, 'I': -0.01, 'D': 0.01}        # PID gains for the elevator when going down
        self.pitch_gains_up = {'P': -0.06, 'I': -0.015, 'D': 0.006}          # PID gains for the elevator when going up
        self.roll_gains = {'P': 0.02, 'I': 0.001, 'D': 0.001}               # PID gains for the elevator
        self.pitch_Hold = {'P': -0.04, 'I': -0.01, 'D': -0.001}

        self.throttle_PID = None
        self.rudder_PID = None
        self.pitch_PID = None
        self.roll_PID = None

        self.state = "Initialize"
        self.start_speed = 0
        self.desired_pitch = 0
        self.prev_time = 0
        self.prev_altitude = 0
        self.gamma = []
        self.stabilize_flag = 0
        self.stabilize_time = None
        self.going_up = False
        self.set_altitude_flag = False
        self.overshoot = 0
        self.holdFlag = False

        self.PC_flag = True

        self.myflag = 0
        self.strtTime = fDat.time

        # If climb angle is given, use that. Else read from file
        if self.desired_gamma is None:
            f = open('pitch.txt', 'r')
            self.desired_pitch = int(f.readlines()[0].strip())
            f.close()
        else:
            # Read in the file which relates gamma to the desired flight path angle. Find the best fit line.
            # Find the desired pitch given the desired gamma by inverting this.
            gamma_pitch = numpy.loadtxt('gamma.txt')
            gamma = gamma_pitch[:, 0]
            pitch = gamma_pitch[:, 1]

            A = numpy.vstack([pitch, numpy.ones(len(pitch))]).T
            m, c = numpy.linalg.lstsq(A, gamma)[0]

            self.desired_pitch = (self.desired_gamma - c)/m

        # Set the flag to whether the aircraft is going up or not. Uses a different set of parameters for both
        if self.desired_altitude:
            if self.desired_altitude > fDat.altitude:
                self.pitch_gains = self.pitch_gains_up
                self.going_up = True
            else:
                self.pitch_gains = self.pitch_gains_down
        else:
            if self.desired_pitch < 0:
                self.pitch_gains = self.pitch_gains_down
            else:
                self.pitch_gains = self.pitch_gains_up

        # Initialize the PIDs
        self.pitch_PID = PID(self.pitch_gains, self.strtTime)
        self.roll_PID = PID(self.roll_gains, self.strtTime)
        self.throttle_PID = PID(self.throttle_gains, self.strtTime)   # The PID controller for throttle

        # If desired speed is given, use that. Else use the current speed
        if not self.desired_speed:
            self.start_speed = fDat.kias
        else:
            self.start_speed = self.desired_speed

        self.prev_time = fDat.time
        self.prev_altitude = fDat.altitude

        self.duration = fDat.time - self.strtTime
        print(str(self.desired_altitude)+"abc")

    def DO(self,fDat,fCmd):
        # If the plan flag is set, just run it once and compute the PID values and stop.
        print(str(self.desired_altitude)+ "!!")
        if self.state is "Initialize":
            fCmd.elevator = self.pitch_PID.control(fDat.pitch, [self.desired_gamma, 0], fDat.time)
            fCmd.throttle = self.throttle_PID.control(fDat.kias, [self.start_speed, 0], fDat.time)
            fCmd.aileron = self.roll_PID.control(fDat.roll, [0, 0], fDat.time)
            self.state = "AdjustAngle"


        if self.state is "AdjustAngle":
            dt = fDat.time - self.prev_time

            if self.desired_pitch - fDat.pitch > 2:
                commanded_pitch = fDat.pitch + 2
            elif self.desired_pitch - fDat.pitch < -2:
                commanded_pitch = fDat.pitch - 2
            else:
                commanded_pitch = self.desired_pitch
            
            if dt != 0:
                self.myflag = 1
            fCmd.elevator = self.pitch_PID.control(fDat.pitch, [commanded_pitch, 0], fDat.time) if self.myflag == 1 else 0
            if self.desired_speed - fDat.kias > 20:
                fCmd.throttle = 1.0
            elif self.desired_speed - fDat.kias < 20 and self.desired_speed - fDat.kias > 0: 
                fCmd.throttle = 0.7
            else:
                fCmd.throttle = 0.4 if self.desired_altitude > fDat.altitude else 0.0

            if (abs(fDat.pitch - self.desired_pitch) < 0.7):
                if self.overshoot > 5:
                    if self.holdFlag is False:
                        self.hold_PID = PID(self.pitch_Hold,fDat.time)
                        self.state = "Hold"
                    else:
                        self.state = "Finalize"
                self.overshoot += 1

            if (abs(self.desired_altitude - fDat.altitude) < 50) and self.holdFlag is False:
                self.desired_pitch = 0.3 if self.desired_altitude > fDat.altitude else -0.1 
                self.state = "Finalize"

        if self.state is "Hold":
            self.overshoot = 0
            commanded_pitch = self.desired_pitch
            fCmd.elevator = self.pitch_PID.control(fDat.pitch, [commanded_pitch, 0], fDat.time) if self.myflag == 1 else 0
            #fCmd.elevator = self.hold_PID.control(fDat.pitch, [commanded_pitch, 0], fDat.time) if self.myflag == 1 else 0
            if self.desired_speed - fDat.kias > 20:
                fCmd.throttle = 1.0
            elif self.desired_speed - fDat.kias < 20 and self.desired_speed - fDat.kias > 0: 
                fCmd.throttle = 0.7
            else:
                fCmd.throttle = 0.4 if self.desired_altitude > fDat.altitude else 0.0
            self.holdFlag = True
   
            if (abs(self.desired_altitude - fDat.altitude) < 100):
                self.desired_pitch = 0.3 if self.desired_altitude > fDat.altitude else -0.1 
                #self.pitch_PID = PID(self.pitch_gains, fDat.time)
                self.state = "AdjustAngle"

                
        if self.state is "Finalize":
            commanded_pitch = self.desired_pitch
            fCmd.elevator = self.pitch_PID.control(fDat.pitch, [commanded_pitch, 0], fDat.time) if self.myflag == 1 else 0
            if self.desired_speed - fDat.kias > 20:
                fCmd.throttle = 1.0
            elif self.desired_speed - fDat.kias < 20 and self.desired_speed - fDat.kias > 0: 
                fCmd.throttle = 0.7
            elif abs(self.desired_speed - fDat.kias) < 1:
                return 'Done'
            else:
                fCmd.throttle = 0.0

        fCmd.aileron = self.roll_PID.control(fDat.roll, [0, 0], fDat.time) 

        print('Current State: ', self.state, '\n',
              'Current altitude:', fDat.altitude, 'Desired Altitude:', self.desired_altitude, '\n',
              'Current flight path angle:', fDat.pitch, 'Desired flight path angle:', self.desired_pitch, '\n',
              'Current throttle is set to', fCmd.throttle, '\n',
              'Current speed:', fDat.kias, 'Desired speed:', self.desired_speed, '\n',
              'Current time:', fDat.time, '\n') 
          
           

        self.prev_time = fDat.time


class PID:

    def __init__(self, gains, start_time):
        self.P = gains['P']
        self.I = gains['I']
        self.D = gains['D']
        self.integral = 0
        self.prev_time = start_time
        self.prev_current = None
        self.velocity = 0

    def control(self, current, ref, time):

        dt = time - self.prev_time

        if current is list:                 # If velocity is given
            error = ref[0] - current[0]     # Error in position
            self.velocity = current[1]
        else:                               # Else, compute the current velocity
            error = ref[0] - current        # Error in position

            if self.prev_current is None:
                self.prev_current = current

            if dt != 0:                     # If time difference between calls is 0, keep old velocity
                self.velocity = (current - self.prev_current)/dt
                self.prev_current = current

        error_dot = ref[1] - self.velocity

        self.integral += self.I*error*dt    # Integral build up
        u = self.P*error + self.D*error_dot + self.integral

        self.prev_time = time

        return u
