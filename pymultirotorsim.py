#!/usr/bin/env python

'''
pymultirotorsim.py - Automatically-launched Python server script for PyHexSim

Translates simulation values from V-REP to sensor values for quadrotor model

    Inspired from Simon D. Levy

'''

# Simulation parameters ===========================================================

# Timeout for receiving data from client
TIMEOUT_SEC      = 0.250

# Other imports ===================================================================

from sys import argv, exit
from math import pi
import numpy as np
import struct
import time
import random
import colormap
import rotations
import actuators
import Tkinter, tkMessageBox
import yaml

from socket_server import serve_socket
from multirotor import Multirotor
from six_dof_double_integral_controller import SixDofDoubleIntegralController
from four_dof_dcm_controller import FourDofDCMController
from coordinates import CoordinateCalculator
from geometry import rotate

# Helper functions ================================================================

def sendFloats(client, data):

    client.send(struct.pack('%sf' % len(data), *data))

def unpackFloats(msg, nfloats):

    return struct.unpack('f'*nfloats, msg)

def receiveFloats(client, nfloats):
 
    # We use 32-bit floats
    msgsize = 4 * nfloats

    # Implement timeout
    start_sec = time.time()
    remaining = msgsize
    msg = ''
    while remaining > 0:
        msg += client.recv(remaining)
        remaining -= len(msg)
        if (time.time()-start_sec) > TIMEOUT_SEC:
            return None

    return unpackFloats(msg, nfloats)
    
def receiveString(client):
    
    return client.recv(int(receiveFloats(client, 1)[0]))

def scalarTo3D(s, a):

    return [s*a[2], s*a[6], s*a[10]]

# Initialization ==========================================================================================================

# Serve a socket on the port indicated in the first command-line argument
print("Opening V-REP <--> Python server on socket " + argv[1])
client = serve_socket(int(argv[1]))

# hitlOutputServer = serve_socket(9000)

# Receive working directory path from client
pyquadsim_directory = receiveString(client)

# Receive particle info from client
particleInfo = receiveFloats(client, 2)
numMotors = int(particleInfo[0])
gravity_ned = -particleInfo[1] # Negative sign because it's expressed in NED

particleInfo = receiveFloats(client, numMotors)
particleSizes = particleInfo[0:numMotors]

particleInfo = receiveFloats(client, 3)
particleDensity        = particleInfo[0]
particleCountPerSecond = particleInfo[1]
totalVehicleMass       = particleInfo[2]

# Instantiate output array
numOutputFloats = 10
headerOffset = 6
forcesAndTorques = [0] * (numMotors * numOutputFloats + headerOffset)

# Load multirotor
with open('/Users/kenz/Documents/PyHexSim/PARC_parameters.yml') as infile:
    multirotor = yaml.load(infile)

# Create motor objects
min_speed_R = multirotor['min_speed_R']
max_speed_R = multirotor['max_speed_R']
# max_speed_R = 20500*pi/30*100  # Convert to rad/s!
motors     = [actuators.Motor('Speed', 0, min_speed_R, max_speed_R) for i in range(numMotors)]

# Create propeller objects
a0 = multirotor['propeller']['thrust2rpm']['a0']
a1 = multirotor['propeller']['thrust2rpm']['a1']
a2 = multirotor['propeller']['thrust2rpm']['a2']
a3 = multirotor['propeller']['thrust2rpm']['a3']

b0 = multirotor['propeller']['rpm2thrust']['b0']
b1 = multirotor['propeller']['rpm2thrust']['b1']
b2 = multirotor['propeller']['rpm2thrust']['b2']
b3 = multirotor['propeller']['rpm2thrust']['b3']

propellers = [actuators.Propeller(a0, a1, a2, a3, b0, b1, b2, b3) for i in range(numMotors)]

# Initialize propeller transformation matrices. These are 3x4 transformation
# matrices (the last row is missing). See http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#simGetObjectMatrix
propellerMatrices = [[0.0]*12 for i in range(numMotors)]

is6DOF = False
# is6DOF = True

if is6DOF:
    hexaMixerMatrix = np.array(multirotor['mixerMatrix6dof'])
else:
    hexaMixerMatrix = np.array(multirotor['mixerMatrix4dof'])

# Create multirotor object
hexacopter = Multirotor(hexaMixerMatrix, numMotors, propellers[0].rpm2thrust(min_speed_R * 30/pi), propellers[0].rpm2thrust(max_speed_R * 30/pi))

# Create controller object
if is6DOF:
    hexacontroller = SixDofDoubleIntegralController('PID')
else:
    hexacontroller = FourDofDCMController('PID')

# Calculate feed forward thrust required to keep hovering
hexacontroller.set_feed_forward_Z_force(totalVehicleMass, gravity_ned)

# Transform from ENU to NED
Rot_ned_from_enu = np.array([[0, 1, 0],
                             [1, 0, 0],
                             [0,0,-1]])

# Rotate ENU_body to NED_body
Rot_ned_body_from_enu_body = np.array([[1, 0, 0],
                                       [0,-1, 0],
                                       [0, 0,-1]])


# Open logfile named by current date, time
# logfile = LogFile(pyquadsim_directory + '/logs')

# Create a quadrotor object for  pitch, roll, yaw, altitude correction.  
# Pass it the logfile object in case it needs to write to the logfile.
# quad = Quadrotor(logfile)

# Create coordinate calculator for GPS simulation
coordcalc = CoordinateCalculator()

# Loop ==========================================================================================================

# root = Tkinter.Tk()
# root.withdraw()
# tkMessageBox.showinfo("my dialog title", "my dialog message")

# Forever loop will be halted by VREP client or by exception
loop_time = 0
while True:

    try:
        # Get core data from client
        clientData = receiveFloats(client, 19 + 12*(numMotors+1))

        if not clientData:
            print 'Timeout'
            break

        # Receive actuator commands from HiTL
        # hitlOutputServer.recv_bytes()

        ## Unpack IMU data
        delT = clientData[0]
        loop_time = loop_time + delT

        # Position and velocity in ENU coordinates
        setpoint_enu              = np.array([clientData[ 1], clientData[ 2], clientData[ 3]])
        position_enu              = np.array([clientData[ 4], clientData[ 5], clientData[ 6]])
        velocity_enu              = np.array([clientData[ 7], clientData[ 8], clientData[ 9]])
        acceleration_body         = np.array([clientData[10], clientData[11], clientData[12]])
        angular_acceleration_body = np.array([clientData[13], clientData[14], clientData[15]])

        # quaternion rotation vector, about ENU world coordinate frame
        offset = 16
        vehicleTransformationMatrix = [[0.0]*12]
        vehicleTransformationMatrix = clientData[offset:offset+12]

        # Angular velocities in ENU inertial frame
        offset = offset + 12
        omega_enu = np.array([clientData[offset+0], clientData[offset+1], clientData[offset+2]])

        # Unpack propeller transformation matrices
        offset = offset + 3
        for i in range(numMotors):
            propellerMatrices[i] = clientData[offset:offset+12]
            offset = offset+12

        # Rotation expressing attitude in the ENU inertial frame
        attitude_enu = np.array([[vehicleTransformationMatrix[0], vehicleTransformationMatrix[1], vehicleTransformationMatrix[2]],
                                 [vehicleTransformationMatrix[4], vehicleTransformationMatrix[5], vehicleTransformationMatrix[6]],
                                 [vehicleTransformationMatrix[8], vehicleTransformationMatrix[9], vehicleTransformationMatrix[10]]])

        # print 'Att: ' + repr(attitude_enu)

        # Calculate attitude rotation in NED frame
        Rot_attitude_ned = np.dot(Rot_ned_from_enu, attitude_enu)

        # # Add some Gaussian noise to position measurement
        # positionXMeters = random.gauss(positionXMeters, GPS_NOISE_METERS)
        # positionYMeters = random.gauss(positionYMeters, GPS_NOISE_METERS)

        # Calculate the rotation from NED to body frame
        Rot_body_from_ned = np.dot(Rot_attitude_ned, Rot_ned_body_from_enu_body).transpose()

        # Convert Euler angles to pitch, roll, yaw
        # See http://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft) for positive/negative orientation
        yaw_R, pitch_R, roll_R = rotations.mat2euler(Rot_body_from_ned)

        # Rotate position and velocity into NED frame
        position_ned = np.dot(Rot_ned_from_enu, position_enu)
        velocity_ned = np.dot(Rot_ned_from_enu, velocity_enu)
        omega_ned    = np.dot(Rot_ned_from_enu, omega_enu)

        # Rotate position, velocity, and angular velocity into body frame
        position_body = np.dot(Rot_body_from_ned, position_ned)
        velocity_body = np.dot(Rot_body_from_ned, velocity_ned)
        omega_body = np.dot(Rot_body_from_ned, omega_ned)

        # Send IMU data to HiTL
        # hitlOutputServer.send_bytes()

        # Update setpoint: [roll, pitch, yaw], [N,E,D]). Angles are in [deg], positions in [m]
        setpoint_ned = np.dot(Rot_ned_from_enu, setpoint_enu)
        setpoint_body = np.dot(Rot_body_from_ned, setpoint_ned)

        if is6DOF:
            hexacontroller.update_setpoint([0, 0, loop_time * 30], setpoint_body)
        else:
            hexacontroller.update_setpoint(loop_time * 30, setpoint_body)

        # Poll dynamics controller
        requested_dynamics = hexacontroller.get_control(np.array([roll_R, pitch_R, yaw_R]), omega_body, position_body, velocity_body, Rot_body_from_ned, delT)

        # Calculate the required motor thrusts based on the desired dynamics response
        thrusts = hexacopter.get_required_thrusts(requested_dynamics)

        # Saturate motors. Don't forget that thrusts are normally negative (because of the NED coordinate frame)
        for i in range(numMotors):
            if thrusts[i] > 0:
                thrusts[i] = 0

        # Convert the propeller thrusts into rpm, and update the motor set point. Don't forget to convert from rpm into rad/s!
        for i in range(numMotors):
            motors[i].update_setpoint_R((propellers[i].thrust2rpm(thrusts[i])) * pi/30, loop_time)

        # Cause motor to fail. Augment motor thrust in case of motor out. Currently, this doesn't work automatically.
        # if loop_time > 2:
        #     motors[0].isEnabled = False
        #     hexacontroller.set_feed_forward_Z_force(totalVehicleMass/4*6, gravity_ned)


        # Compute drag forces on vehicle.
        drag_translational_body, drag_rotational_body = hexacopter.get_air_drag(velocity_body, omega_body)

        # Rotate drag forces and torques back into inertial frame
        # omega_body = np.dot(Rot_body_from_ned, np.dot(Rot_ned_from_enu, omega_enu))
        drag_translational_enu = np.dot(Rot_ned_from_enu.transpose(), np.dot(Rot_body_from_ned.transpose(), drag_translational_body))
        drag_rotational_enu  = np.dot(Rot_ned_from_enu.transpose(), np.dot(Rot_body_from_ned.transpose(), drag_rotational_body))

        # Apply wind. Currently it's just Xm/s from the East (i.e. going to the West). The signs
        # are reversed because airspeed is relative, so this is the same drag as if the vehicle
        # were flying east at Xm/s
        wind_enu, dummy_tmp = hexacopter.get_air_drag(np.array([0,0,0]), np.array([0,0,0]))

        # Apply vehicle drag forces and torques
        forcesAndTorques[0:3]            = drag_translational_enu + wind_enu
        forcesAndTorques[3:headerOffset] = drag_rotational_enu

        # print "loop_time : " + repr(loop_time)
        # print "control  : " + repr(requested_dynamics)
        # print "thrust_z : " + repr(thrusts)
        # print "ENU  position: " + repr(position_enu)
        # print "NED  position: " + repr(position_ned)
        # print "NED  velocity: " + repr(velocity_ned)
        # print "ENU  velocity: " + repr(velocity_enu)
        # print "ENU  drag    : " + repr(drag_translational_enu)
        # print "ENU  omega       : " + repr(omega_enu)
        # print "ENU  angular drag: " + repr(drag_rotational_enu)
        # print "Body velocity: " + repr(velocity_body)
        # print "Body drag    : " + repr(drag_translational_body)
        # print "Body omega: " + repr(omega_body * 180/pi)
        # print "Body angular drag: " + repr(drag_rotational_body)
        # print "ENU  rotation: " + repr(attitude_enu)
        # print "NED  rotation: " + repr(Rot_body_from_ned)
        # yaw_enu_R, pitch_enu_R, roll_enu_R = rotations.mat2euler(attitude_enu)
        # print "ENU  attitude: " + repr(np.array([roll_enu_R * 180/pi, pitch_enu_R * 180/pi, yaw_enu_R * 180/pi]))
        print "Body attitude: " + repr(np.array([roll_R * 180/pi, pitch_R * 180/pi, yaw_R * 180/pi]))
        # print "Body torque  : " + repr(np.array([angular_acceleration_body[0], angular_acceleration_body[1], angular_acceleration_body[2]]))
        # print ""

        # Calculate forces and torques generated by propellers
        for i in range(numMotors):
            # Net force is the force exerted on all the particles divided by the time step
            # thrust_z  = thrusts[i] # * netParticleMass / delT
            thrust_z  = propellers[i].rpm2thrust(motors[i].get_rpm(loop_time)) # * netParticleMass / delT
            torque_z = ((-1)**(i))*1.0/43 * thrust_z # The magic number of 1/45 comes from empirical tests

            # Update packet payload offset
            j = i * numOutputFloats + headerOffset

            # Rotate body-frame forces and torques into world frame. No need for vector math
            # since the rotated vector is only along the z axis
            forcesAndTorques[j:j+3]   = scalarTo3D(-thrust_z, propellerMatrices[i])
            forcesAndTorques[j+3:j+6] = scalarTo3D(-torque_z, propellerMatrices[i])

            # Include coloring and propeller angular velocity information
            forcesAndTorques[j+6:j+9] = colormap.floatRgb(abs(thrust_z), 10,17)
            forcesAndTorques[j+9] = ((-1)**(i+1)) * motors[i].get_rpm(loop_time)

            # print "Motor "  + str(i) + " speed: " + str(forcesAndTorques[j+9])
            print "Input: " + repr(np.array([i+1, thrust_z, torque_z]))
            # print "Thrust: " + repr(forcesAndTorques[j:j+3])
            # print "Torque: " + repr(forcesAndTorques[j+3:j+6])
            # print "Angle: " + repr(propellerMatrices[i])
            # print ""


        # print "Floats: " + repr(forcesAndTorques)

        # Send forces and torques to client
        sendFloats(client, forcesAndTorques)

    except Exception:

        # Inform and exit on exception
        exit(0)
