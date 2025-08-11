
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

from simulationTools import *
from datetime import datetime
from datetime import datetime, timezone
import spiceypy as spice

from Basilisk.architecture.sysModel import *




bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, initOrbit, nbInclinations, inclinations, durations):
    """
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
        nbInclination (int): Describes the number of inclinations we want for the satellite's orbit
        inclinations (tab[float]): Gives in a temporal order the different inclinations in degrees
        durations (tab[float]) : Gives in a temporal order the durations of each inclinations in orbital period

    """

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
    simulationTimeStep = macros.sec2nano(10.)
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

    #print(f"Angle sidéral de Greenwich (GAST) : {gast_deg:.6f}°")

    # need spice to run before spacecraft module as it provides the spacecraft translational states
    scSim.AddModelToTask(simTaskName, spiceObject)

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
    print(oe.i*macros.R2D)


    oe.omega = 0.0 * macros.D2R
    oe.f = initOrbit[3] * macros.D2R
    rN, vN = orbitalMotion.elem2rv(earth.mu, oe)
    hypso.hub.r_CN_NInit = rN  # m - r_CN_N
    hypso.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(earth.mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(0.25 * P)


    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataRec = hypso.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)

   


    

    


    if vizSupport.vizFound:
        # Utilisez un nom de fichier fixe au lieu de None
        vizFile = "LiveScenario"  
        


        

        viz = vizSupport.enableUnityVisualization(
            scSim,
            simTaskName,
            hypso,
            saveFile=vizFile,  # Nom obligatoire mais ignoré en liveStream
            liveStream=True,   # Flux temps réel prioritaire
            oscOrbitColorList=[vizSupport.toRGBA255("yellow")],
            trueOrbitColorList=[vizSupport.toRGBA255("turquoise")],
        )
        cam = vizSupport.createStandardCamera(
            spacecraftName=hypso.ModelTag,  # Doit matcher votre spacecraft.ModelTag
            setMode=1,  # 1 = mode vecteur de pointage (obligatoire)
            pointingVector_B=[0, 0, -1],  # Vise vers Nadir (Z négatif)
            fieldOfView=10 * macros.D2R,  # 10° en radians
            position_B=[0, 0, 0],  # Position dans le repère satellite
            viz = viz
        )
        # Configuration visuelle
        viz.settings.mainCameraTarget = "earth"
        viz.settings.showSpacecraftLabels = 1
        viz.settings.trueTrajectoryLinesOn = 3
        


        
        print(f"Vizard prêt - Fichier: {vizFile}.bin (ignoré en mode stream)")

    
    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #  get access to dynManager translational states for future access to the states
    #
    posRef = hypso.dynManager.getStateObject(hypso.hub.nameOfHubPosition)
    velRef = hypso.dynManager.getStateObject(hypso.hub.nameOfHubVelocity)

    # The dynamics simulation is setup using a Spacecraft() module with the Earth's
    # gravity module attached.  Note that the rotational motion simulation is turned off to leave
    # pure 3-DOF translation motion simulation.  After running the simulation for 1/4 of a period
    # the simulation is stopped to apply impulsive changes to the inertial velocity vector.
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    
    T = []
    # compute maneuver Delta_v's
    for k in range(nbInclinations):
        # We repeat the same process for each inclinations wanted

        # Next, the state manager objects are called to retrieve the latest inertial position and
        # velocity vector components:
        rVt = unitTestSupport.EigenVector3d2np(posRef.getState())
        vVt = unitTestSupport.EigenVector3d2np(velRef.getState())

        # inclination change
        Delta_i = inclinations[k] * macros.D2R
        rHat = rVt / np.linalg.norm(rVt)
        hHat = np.cross(rVt, vVt)
        hHat = hHat / np.linalg.norm(hHat)
        vHat = np.cross(hHat, rHat)
        v0 = np.dot(vHat, vVt)
        vVt = vVt - (1. - np.cos(Delta_i)) * v0 * vHat + np.sin(Delta_i) * v0 * hHat

        # After computing the maneuver specific Delta_v's, the state managers velocity is updated through
        velRef.setState(vVt)
        T.append(macros.sec2nano(P * durations[k]))

        # To start up the simulation again, note that the total simulation time must be provided,
        # not just the next incremental simulation time.
        scSim.ConfigureStopTime(simulationTime + sum(T))
        scSim.ExecuteSimulation()

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

    attRefRecorder = messaging.AttRefMsg().recorder()
    scSim.AddModelToTask(simTaskName, attRefRecorder)

    # Exécuter la simulation
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    return figureList





#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#





if __name__ == "__main__":
    run(
        True,  # show_plots
        initialCoordinateTraject(590., 10.7579, 59.9115,  -51.205, 46.734), # the initial situation of the satellite
        3, # we want 3 inclinations
        [8.0, 4.0, -12.0], # the three inclinations wanted
        [0.25, 0.4, 0.25] # the durations of these inclinations
    )






























    