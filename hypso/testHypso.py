
#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the satelite and gravity modules illustrating
#           how the satelite should be located and rotated to take a picture

import math
import os
from multiprocessing import Process

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import pinholeCamera
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.simulation import vizInterface
from Basilisk.utilities import RigidBodyKinematics




import tkinter as tk
import time
import threading

from simulationTools import *
from datetime import datetime
from datetime import timedelta
import spiceypy as spice

from Basilisk.architecture.sysModel import *

import socketserver
import socket
from contextlib import closing
import threading
from Basilisk.architecture import messaging, sim_model

from orientation import process_command
import zmq
import json
#from attitude import RealTimeAttitudeControl

from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import mrpFeedback

from PIL import Image, ImageTk

global Sigma
Sigma = np.array([0.0, 0.0, 0.0])  # Orientation in MRP
global SigInit
SigInit = np.array([0.0, 0.0, 0.0])  # Initial orientation in MRP

global direction
direction = "down"


def attitude_listener(att_ref_msg_writer):
    """Listener for attitude commands from the GUI via ZMQ. It transmits the commands to Basilisk."""
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5559")  # Port used by GUI
    socket.setsockopt_string(zmq.SUBSCRIBE, '')
    
    while True:
        try:


            message = socket.recv_string()
            data = json.loads(message)

            if data.get("type") == "attitude_command":
                roll = float(data["roll"])
                pitch = float(data["pitch"])
                yaw = float(data["yaw"])
                
                sigma = RigidBodyKinematics.euler3122MRP([
                    math.radians(yaw),
                    math.radians(roll),
                    math.radians(pitch)
                ])

                global direction
                if yaw%360 > 315 and yaw%360 < 45:
                    direction = "down"
                if 45 < yaw%360 < 135:
                    direction = "right"
                if 135 < yaw%360 < 225:
                    direction = "up"
                if 225 < yaw%360 < 315:
                    direction = "left"
                if yaw%360 == 45 or yaw%360 == 225:
                    direction = "diagonal"
                if yaw%360 == 135 or yaw%360 == 315:
                    direction = "diagonalinverse"
                
                global Sigma
                global SigInit
                dcm_BN = RigidBodyKinematics.MRP2C(SigInit)
                dcm_BL = RigidBodyKinematics.MRP2C(sigma)
                dcm_BN = dcm_BL @ dcm_BN  # Update the DCM with the new orientation
                Sigma = RigidBodyKinematics.C2MRP(dcm_BN)


                msg = messaging.AttRefMsgPayload()
                msg.sigma_RN = Sigma + SigInit # Orientation in MRP
                att_ref_msg_writer(msg)

                print(f"[Basilisk] Orientation updated : {roll}, {pitch}, {yaw}")

        except Exception as e:
            print(f"ZMQ Basilisk error: {e}")
            socket.close()
            context.term()


class CameraInterface:
    """
    Class to create a simple GUI for displaying three positions corresponding to the target of the camera and its up left and bottom right corners on a canvas.
    """
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Satellite and camera informations")
        self.root.geometry("800x600")
        self.position_label = tk.Label(self.root, text=f"Position of the satellite: N/A", font=("Arial", 12))
        self.position_label.pack(pady=10)
        self.orientation_label = tk.Label(self.root, text=f"Orientation of the satellite in the LVLH frame: N/A \n and in the ECI frame: N/A", font=("Arial", 12))
        self.orientation_label.pack(pady=10)

        # Dictionary to store the three positions
        self.positions = {
            "center": {"lat": 0.0, "lon": 0.0},
            "top_left": {"lat": 0.0, "lon": 0.0},
            "top_right": {"lat": 0.0, "lon": 0.0},
            "bottom_left": {"lat": 0.0, "lon": 0.0},
            "bottom_right": {"lat": 0.0, "lon": 0.0}
        }
        
        self.setup_ui()
    
    def setup_ui(self):
        """Initiates the interface"""
        self.canvas = tk.Canvas(self.root, width=400, height=300, bg='#0a2e5a')
        self.canvas.pack(fill="both", expand=True)
        
        self.canvas.config(bg='#669999')
        
        self.update_display()
    
    def update_target(self, position_key, lat, lon):
        """Update a specific position with latitude and longitude"""
        if position_key in self.positions:
            self.positions[position_key]["lat"] = lat
            self.positions[position_key]["lon"] = lon
        else:
            raise ValueError(f"Position inconnue: {position_key}. Options: center, top_left, bottom_right")
    
    def update_position(self, position):
        """Update the satellite position displayed"""
        self.position_label.config(text=f"Position of the satellite: altitude {float(position[2]):.4f}, \n latitude {float(position[0]):.4f} and longitude {float(position[1]):.4f}")

    def update_orientation(self, quaternions):
        """Update the satellite orientation displayed"""
        self.orientation_label.config(text=f"Orientation of the satellite in the LVLH frame: {quaternions[0]} \n and in the ECI frame: {quaternions[1]}")

    def update_display(self):
        """Update the display with the current positions"""
        try:
            self.canvas.delete("coords")  # Clean the previous display
        
            width = self.canvas.winfo_width()
            height = self.canvas.winfo_height()
            
            # Configuration of the display texts
            display_config = {
                "center": {
                    "coords": (width/2, height/2),
                    "font": ('Arial', 20, 'bold'),
                    "anchor": "center",
                    "color": "white"
                },
                "top_left": {
                    "coords": (10, 10),
                    "font": ('Arial', 20),
                    "anchor": "nw",
                    "color": "yellow"
                },
                "top_right": {
                    "coords": (width-10, 10),
                    "font": ('Arial', 20),
                    "anchor": "ne",
                    "color": "magenta"
                },
                "bottom_left": {
                    "coords": (10, height-10),
                    "font": ('Arial', 20),
                    "anchor": "sw",
                    "color": "green"
                },
                "bottom_right": {
                    "coords": (width-10, height-10),
                    "font": ('Arial', 20),
                    "anchor": "se",
                    "color": "cyan"
                }
            }
            
            # Define the configuration for each position
            for key, config in display_config.items():
                pos_data = self.positions[key]
                if pos_data["lat"] == 1000.0 and pos_data["lon"] == 0.0:
                    text = f"{key.replace('_', ' ').title()}\n points at space"
                else:
                    text = f"{key.replace('_', ' ').title()}\nLat: {pos_data['lat']:.4f}°\nLon: {pos_data['lon']:.4f}°"
                
                self.canvas.create_text(
                    *config["coords"],
                    text=text,
                    fill=config["color"],
                    font=config["font"],
                    anchor=config["anchor"],
                    tags="coords"
                )
        except Exception as e:
            pass
    
    def display_total_image(self):
        try:
            self.canvas.delete("coords")  # Clean the previous display
        
            width = self.canvas.winfo_width()
            height = self.canvas.winfo_height()
            
            # Configuration of the display texts
            display_config = {
                "top_left": {
                    "coords": (10, 10),
                    "font": ('Arial', 20),
                    "anchor": "nw",
                    "color": "yellow"
                },
                "top_right": {
                    "coords": (width-10, 10),
                    "font": ('Arial', 20),
                    "anchor": "ne",
                    "color": "magenta"
                },
                "bottom_left": {
                    "coords": (10, height-10),
                    "font": ('Arial', 20),
                    "anchor": "sw",
                    "color": "green"
                },
                "bottom_right": {
                    "coords": (width-10, height-10),
                    "font": ('Arial', 20),
                    "anchor": "se",
                    "color": "cyan"
                }
            }
            
            # Define the configuration for each position
            for key, config in display_config.items():
                pos_data = self.positions[key]
                if pos_data["lat"] == 1000.0 and pos_data["lon"] == 0.0:
                    text = f"{key.replace('_', ' ').title()}\n points at space"
                else:
                    text = f"{key.replace('_', ' ').title()}\nLat: {pos_data['lat']:.4f}°\nLon: {pos_data['lon']:.4f}°"
                
                self.canvas.create_text(
                    *config["coords"],
                    text=text,
                    fill=config["color"],
                    font=config["font"],
                    anchor=config["anchor"],
                    tags="coords"
                )
        except Exception as e:
            pass
    
    def display_error(self):
        try:
            self.canvas.delete("coords")

            width = self.canvas.winfo_width()
            height = self.canvas.winfo_height()
            
            self.canvas.create_text(
                    (width/2, height/2),
                    anchor="center",
                    text=f"The camera does not \n see the Earth",
                    fill="blue",
                    font=("Helvetica", 48, "bold"),
                    tags="coords"
                )
        except Exception as e:
            pass

    def run(self):
        """Launch the interface"""
        self.root.mainloop()





bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, initOrbit, duration, image, fieldOfView=120, total_image = False, mode = "sliders",  N = 100):
    """
    Main function to run the simulation of the satellite in orbit around the Earth.
    This function initializes the simulation environment, sets up the satellite's orbit and attitude, and runs the simulation.
    It also creates a simple GUI to visualize the satellite's position and orientation in real-time and its target positions on the Earth.

    The simulation will run until the user stops it using the GUI button STOP.


    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        initOrbit (tab[float]) : Corresponds to the few initial parameters that will allow to place the satellite where we want
            ======  ============================
            Index    Definition
            ======  ============================
            0       satellite altitude (in kilometers)
            1       orbital initial inclination (in degrees)
            2       longitude of the ascending node (in degrees)
            3       true anomaly (in degrees)
            ======  ============================
        start (float): Start time of the simulation in seconds
        end (float): End time of the simulation in seconds
        fieldOfView (float): Field of view of the camera in degrees, default is 120° which allows to see a large area of the Earth.

    """

    from tkinter import font
    
    Simulation = True
    class stop_simulation:
        def __init__(self,root):
            self.root = root
            self.frame = tk.Frame(root, width=400, height=300)
            self.frame.pack_propagate(False)  # empêche la frame de changer de taille à cause du contenu
            self.frame.pack(padx=5, pady=5)

            self.stop = tk.Button(self.frame , font=font.Font(family="Arial", size=80, weight="bold"), bg="red", fg="white", text="Stop", command=self.stop_simulation)
            self.stop.pack(fill="both", expand=True)

        def stop_simulation(self):
            nonlocal Simulation
            Simulation = False
            self.root.quit()
            self.root.destroy()
    
    def control_sim():
        root2 = tk.Tk()
        root2.title("Stop Simulation")
        stop = stop_simulation(root2)
        root2.mainloop()

    t1 = threading.Thread(target=control_sim, daemon=True).start()

    msg_data = messaging.AttRefMsgPayload()
    msg_data.sigma_RN = [0, 0, 0]  # Initial orientation
    msg_data.omega_RN_N = [0, 0, 0]  # Initial angular velocity
    att_ref_msg = messaging.AttRefMsg().write(msg_data)
    att_ref_msg_writer = att_ref_msg.write

    # Launch the thread to listen ZMQ
    t2 = threading.Thread(target=attitude_listener, args=(att_ref_msg_writer, ), daemon=True).start()

    interface = CameraInterface()
    

    while Simulation:
        global Sigma
        global SigInit
        # Create simulation variable names
        simTaskName = "simTask"
        simProcessName = "simProcess"

        #  Create a sim module as an empty container
        scSim = SimulationBaseClass.SimBaseClass()

        #
        #  create the simulation process
        #
        dynProcess = scSim.CreateNewProcess(simProcessName)

        # create the dynamics task and specify the integration update time
        simulationTimeStep = macros.sec2nano(1.)
        dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

        #
        #   setup the simulation tasks/objects
        #

        # initialize spacecraft object and set properties
        hypso = spacecraft.Spacecraft()

        hypso.ModelTag = "spacecraftBody"

        # add spacecraft object to the simulation process
        scSim.AddModelToTask(simTaskName, hypso)

        # setup Gravity Body and SPICE definitions
        gravFactory = simIncludeGravBody.gravBodyFactory()
        earth = gravFactory.createEarth()
        earth.isCentralBody = True  # ensure this is the central gravitational body

        # attach gravity model to spacecraft
        gravFactory.addBodiesTo(hypso)

        # setup spice library for Earth ephemeris
        timeInit =  datetime(2024, 9, 21, 21, 00, 0)
        timeInitString = "2024 September 21, 21:00:00.0 TDB"
        spiceObject = gravFactory.createSpiceInterface(time=timeInitString, epochInMsg=True)
        spiceObject.zeroBase = 'Earth'

        # calculate the Greenwich sidereal angle at the given date
        
        gast_deg = greenwich_sidereal_angle_deg(timeInit)

        # print(f"Greenwich sidereal angle (GAST) : {gast_deg:.6f}°")

        # need spice to run before spacecraft module as it provides the spacecraft translational states
        scSim.AddModelToTask(simTaskName, spiceObject)

        simulationTime = macros.sec2nano(duration)

        
        #
        #   setup orbit and simulation time
        #
        # setup the orbit using classical orbit elements
        oe = orbitalMotion.ClassicElements()
        rLEO = (initOrbit[0]+6371.) * 1000  # meters
        oe.a = rLEO
        oe.e = 0.0001
        if initOrbit[1]<0 :
            oe.i = (-initOrbit[1])*macros.D2R
        elif initOrbit[1]>180. :
            oe.i = (360-initOrbit[1])*macros.D2R
        else:
            oe.i = initOrbit[1] * macros.D2R
        oe.Omega = ((initOrbit[2] + gast_deg)%360) * macros.D2R 

        #print(f"{initOrbit[2]}")
        #print(f"{oe.Omega*macros.R2D}")
        #print(f"{initOrbit[3]}")
        #print(oe.i*macros.R2D)


        oe.omega = 0.0 * macros.D2R
        oe.f = initOrbit[3] * macros.D2R
        rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
        hypso.hub.r_CN_NInit = rN  # m - r_CN_N
        hypso.hub.v_CN_NInit = vN  # m - v_CN_N

        z_B = rN / np.linalg.norm(rN)  # Z axis in the inertial frame
        y_B = np.cross(rN , vN) / np.linalg.norm(np.cross(rN,vN))  # Y axis in the inertial frame
        x_B = np.cross(y_B, z_B) / np.linalg.norm(np.cross(y_B, z_B))  # X axis in the inertial frame
        dcm_BN = np.array([x_B, y_B, z_B])
        if mode == "sliders":

            if np.all(Sigma == 0.0) or np.all(SigInit == 0.0):
                SigInit = RigidBodyKinematics.C2MRP(dcm_BN)
                angles = angular_velocity(rN, vN, initOrbit[1]*macros.D2R)
                sig = RigidBodyKinematics.MRP2C(Sigma + SigInit)
                hypso.hub.omega_BN_BInit = sig @ angles[0] # Initial angular velocity
                hypso.hub.sigma_BNInit = Sigma + SigInit# Initial orientation in MRP

            else :
                SigInit = RigidBodyKinematics.C2MRP(dcm_BN)
                angles = angular_velocity(rN, vN, initOrbit[1]*macros.D2R)
                sig = RigidBodyKinematics.MRP2C(Sigma)
                hypso.hub.omega_BN_BInit = sig @ angles[0] # Initial angular velocity
                hypso.hub.sigma_BNInit = Sigma # Initial orientation in MRP

        elif mode == "quaternion":
            
            target = interpolate_image(image[0], image[1], int(simulationTime/simulationTimeStep))
            pos = propagate_kepler(oe, macros.NANO2SEC*simulationTime/2)[0]  # Initial attitude in quaternion

            if len(target)%2==0:
                targ =(np.array(target[len(target)//2]) + np.array(target[len(target)//2+1]))/2

            else:
                targ = np.array(target[len(target)//2 + 1])

            dt = timeInit + timedelta(seconds=macros.NANO2SEC * simulationTime/2)
            gast = greenwich_sidereal_angle_deg(dt)

            print(eci_to_geodetic(pos,gast))
            attitude = get_att(pos, geodetic_to_eci(targ, gast))
            
            hypso.hub.omega_BN_BInit = [0.0, 0.0, 0.0]  # Initial angular velocity
            hypso.hub.sigma_BNInit = attitude  # Initial orientation in MRP
        else:
            raise ValueError("Invalid mode. Use 'sliders' or 'quaternion'.")


        # set the simulation time
        n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
        P = 2. * np.pi / n # the orbital period in seconds

        


        #
        #   Setup data logging before the simulation is initialized
        #
        numDataPoints = int(simulationTime/simulationTimeStep)
        samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
        dataRec = hypso.scStateOutMsg.recorder(samplingTime)
        scSim.AddModelToTask(simTaskName, dataRec)



        
        # Use a file name instead of None
        vizFile = "LiveScenario"  
        viz = vizSupport.enableUnityVisualization(
            scSim,
            simTaskName,
            hypso,
            saveFile=vizFile,  # Ignored in live stream
            liveStream=True,   # Live stream mode
            oscOrbitColorList=[vizSupport.toRGBA255("yellow")],
            trueOrbitColorList=[vizSupport.toRGBA255("turquoise")]
        )
        # Create a camera to visualize what the spacecraft sees on the ground
        cam = vizSupport.createStandardCamera(
            spacecraftName=hypso.ModelTag,  # Have to match the spacecraft.ModelTag
            setMode=1,  # 1 = mode pointing vector (necessary)
            pointingVector_B=get_direction([0,0,-1], Sigma),  # Initially aim Nadir (Z negative)
            fieldOfView=fieldOfView * macros.D2R,  # 120° in radians
            position_B=[0, 0, 0],  # Position in the satellite referencial
            viz = viz
        )

        vizSupport.createConeInOut(
            viz=viz,
            fromBodyName=hypso.ModelTag,
            toBodyName="earth",
            coneColor=[255, 255, 0, 150],             # Jaune semi-transparent
            isKeepIn=True,
            position_B=[0.0, 0.0, 0.0],
            normalVector_B=[0, 0, -1],                # Même direction que la caméra
            incidenceAngle=fieldOfView/2 * macros.D2R,           # Angle d’ouverture
            coneHeight=20.0,                        
            coneName="CameraCone"
        )

        print(f"Position initiale de la caméra : {cam.position_B}")

        
        viz.settings.mainCameraTarget = "earth"
        viz.settings.showSpacecraftLabels = 1
        viz.settings.trueTrajectoryLinesOn = 3

        
        scSim.InitializeSimulation()
        
        print("Orientation initiale (sigma_BN):", hypso.hub.sigma_BNInit)
        print("Vitesse angulaire initiale (omega_BN_B):", hypso.hub.omega_BN_BInit)
        # Reference objects for position and velocity
        posRef = hypso.dynManager.getStateObject(hypso.hub.nameOfHubPosition)
        velRef = hypso.dynManager.getStateObject(hypso.hub.nameOfHubVelocity)

        list_r = []
        
        tot = np.zeros((4,3))

        global direction
        # Execution of the simulation
        for i in range(int(simulationTime/simulationTimeStep)):
            stateVector = hypso.scStateOutMsg.read()
            dt = timeInit + timedelta(seconds=macros.NANO2SEC * i * simulationTimeStep)
            gast = greenwich_sidereal_angle_deg(dt)
            
            
            r_BN_N = np.array(stateVector.r_BN_N)
            v_BN_N = np.array(stateVector.v_BN_N)

            list_r.append(r_BN_N)
        
            coord = get_camera_frame(r_BN_N, v_BN_N, Sigma, [fieldOfView, fieldOfView])
            
            
            if coord!=None:
                coord_to_display = [eci_to_geodetic(coord[0],gast*macros.D2R), eci_to_geodetic(coord[1],gast*macros.D2R), eci_to_geodetic(coord[2],gast*macros.D2R), eci_to_geodetic(coord[3],gast*macros.D2R), eci_to_geodetic(coord[4],gast*macros.D2R)]
                # Update the interface
                interface.update_target("center", coord_to_display[2][0], coord_to_display[2][1])
                interface.update_target("top_left", coord_to_display[0][0], coord_to_display[0][1])
                interface.update_target("top_right", coord_to_display[1][0], coord_to_display[1][1])
                interface.update_target("bottom_left", coord_to_display[3][0], coord_to_display[3][1])
                interface.update_target("bottom_right", coord_to_display[4][0], coord_to_display[4][1])
                if i == 0:
                    if total_image:
                        match direction:
                            case "down":
                                tot[1] = coord_to_display[1]
                                tot[3] = coord_to_display[4]
                            case "up":
                                tot[0] = coord_to_display[0]
                                tot[2] = coord_to_display[3]
                            case "right":
                                tot[2] = coord_to_display[3]
                                tot[3] = coord_to_display[4]
                            case "left":
                                tot[0] = coord_to_display[0]
                                tot[1] = coord_to_display[1]
                            case "diagonal":
                                tot[0] = coord_to_display[0]
                                tot[3] = coord_to_display[4]
                            case "diagonalinverse":
                                tot[1] = coord_to_display[1]
                                tot[2] = coord_to_display[3]

                if i == int(simulationTime/simulationTimeStep)-1:
                    if total_image:
                        match direction:
                            case "down":
                                tot[0] = coord_to_display[0]
                                tot[2] = coord_to_display[3]
                            case "up":
                                tot[1] = coord_to_display[1]
                                tot[3] = coord_to_display[4]
                            case "right":
                                tot[0] = coord_to_display[0]
                                tot[1] = coord_to_display[1]
                            case "left":
                                tot[2] = coord_to_display[3]
                                tot[3] = coord_to_display[4]
                            case "diagonal":
                                tot[1] = coord_to_display[1]
                                tot[2] = coord_to_display[3]
                            case "diagonalinverse":
                                tot[0] = coord_to_display[0]
                                tot[3] = coord_to_display[4]
        
                interface.update_display()
            else:
                interface.display_error()
            
            att = RigidBodyKinematics.MRP2Euler312(Sigma)
            interface.update_orientation(EulerToQuat(att, r_BN_N, v_BN_N))
            interface.update_position(eci_to_geodetic(r_BN_N, gast*macros.D2R))
            
            interface.root.update()

            # Execute a step of the simulation
            scSim.ConfigureStopTime((i+1) * simulationTimeStep)
            scSim.ExecuteSimulation()

            # time.sleep(0.1)  # Sleep to allow the GUI to update and the user to visualize the changes
        
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()

        if len(tot)!= 4 and total_image:
            interface.display_error()
        else:
            interface.update_target("bottom_left", tot[0][0], tot[0][1])
            interface.update_target("bottom_right", tot[1][0], tot[1][1])
            interface.update_target("top_left", tot[2][0], tot[2][1])
            interface.update_target("top_right", tot[3][0], tot[3][1])

            interface.display_total_image()
        
        interface.root.update()
        
        if mode == "quaternion":
            Simulation = False
        # Break for 10 seconds to allow the simulation to run
        time.sleep(10)
        
    list_quat = calculate_quat_imaging(numDataPoints, list_r, target)
    T = []
    # compute maneuver Delta_v's

    # unload Spice kernel
    gravFactory.unloadSpiceKernels()

    #
    #   retrieve the logged data
    #
    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(dataRec.times() * macros.NANO2HOUR, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + str(1)
    figureList[pltName] = plt.figure(1)

    # show inclination angle
    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    iData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(earth.mu, posData[idx], velData[idx])
        iData.append(oeData.i * macros.R2D)
    plt.plot(dataRec.times() * macros.NANO2HOUR, np.ones(len(posData[:, 0])) * 8.93845, '--', color='#444444'
                )
    plt.plot(dataRec.times() * macros.NANO2HOUR, iData, color='#aa0000'
                )
    plt.ylim([-1, 90])
    plt.xlabel('Time [h]')
    plt.ylabel('Inclination [deg]')

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")


    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    dataPos = posRef.getState()
    dataPos = [[0.0, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]

    return figureList




if __name__ == "__main__":
    

    # Execute the main function with the initial parameters
    run(
        True,
        initialCoordinate(1500., 59.9115,  10.7579,  65., False),
        200.,
        [[59.9115, 10.7579], [61.1115, 10.9579]],  # Image to capture from a target point to another
        10.,
        True,
        "quaternion"   # "sliders" option to use the sliders and try finding an orientation that fits for the image 
                    # or "quaternion" to get the quaternions necessary to capture the wished image
    )
