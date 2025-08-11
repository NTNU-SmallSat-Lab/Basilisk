from Basilisk.architecture import messaging
import threading
import zmq
import json
import math
import time

from Basilisk.utilities import RigidBodyKinematics
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import mrpFeedback

class RealTimeAttitudeControl:
    def __init__(self, scSim, scObject, simTaskName):
        self.scSim = scSim
        self.scObject = scObject
        self.simTaskName = simTaskName
        
        # Création du message de référence
        self.att_ref_msg = messaging.AttRefMsg_C()
        self.att_ref_msg_obj = self.att_ref_msg.init()
        
        # Configuration ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5557")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Initialisation du contrôle d'attitude
        self.setup_attitude_control()
        
        # Thread pour le contrôle en temps réel
        self.running = True
        self.control_thread = threading.Thread(target=self.attitude_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

    def setup_attitude_control(self):
        """Configure les modules de contrôle d'attitude"""
        # Module de navigation
        self.nav = simpleNav.SimpleNav()
        self.nav.ModelTag = "SimpleNav"
        self.scSim.AddModelToTask(self.simTaskName, self.nav)
        self.nav.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        
        # Calcul d'erreur d'attitude
        self.attError = attTrackingError.attTrackingError()
        self.attError.ModelTag = "AttError"
        self.scSim.AddModelToTask(self.simTaskName, self.attError)
        self.attError.attRefInMsg.subscribeTo(self.att_ref_msg_obj)
        self.attError.attNavInMsg.subscribeTo(self.nav.attOutMsg)
        
        # Contrôleur MRP
        self.mrpControl = mrpFeedback.mrpFeedback()
        self.mrpControl.ModelTag = "mrpFeedback"
        self.scSim.AddModelToTask(self.simTaskName, self.mrpControl)
        self.mrpControl.guidInMsg.subscribeTo(self.attError.attGuidOutMsg)
        
        # Configuration de l'inertie (adaptée à votre version de Basilisk)
        vehicleConfigMsg = messaging.VehicleConfigMsg_C()
        vehicleConfigOutMsg = messaging.VehicleConfigMsgPayload()
        if hasattr(self.scObject.hub, 'I_sc'):
            I = self.scObject.hub.I_sc
        else:
            I = [[1000,0,0],[0,800,0],[0,0,800]]  # Valeurs par défaut
            
        vehicleConfigOutMsg.ISCPntB_B = [I[0][0], I[1][1], I[2][2], I[0][1], I[0][2], I[1][2]]
        vehicleConfigMsg.write(vehicleConfigOutMsg)
        self.mrpControl.vehConfigInMsg.subscribeTo(vehicleConfigMsg)
        
        # Paramètres du contrôleur
        self.mrpControl.K = 3.5
        self.mrpControl.Ki = -1.0
        self.mrpControl.P = 30.0
        
        # Application du couple
        self.extFTObject = extForceTorque.ExtForceTorque()
        self.extFTObject.ModelTag = "externalDisturbance"
        self.extFTObject.extTorquePntB_B = [0., 0., 0.]
        self.scObject.addDynamicEffector(self.extFTObject)
        self.scSim.AddModelToTask(self.simTaskName, self.extFTObject)
        self.extFTObject.cmdTorqueInMsg.subscribeTo(self.mrpControl.cmdTorqueOutMsg)

    def attitude_control_loop(self):
        """Boucle de contrôle en temps réel"""
        last_update = 0
        update_interval = 0.1  # 10 Hz
        
        while self.running:
            try:
                # Réception non-bloquante des commandes
                try:
                    message = self.socket.recv_string(flags=zmq.NOBLOCK)
                    data = json.loads(message)
                    
                    # Vérification du taux de mise à jour
                    current_time = time.time()
                    if current_time - last_update >= update_interval:
                        last_update = current_time
                        
                        # Conversion des angles d'Euler en MRP
                        roll = math.radians(float(data['roll']))
                        pitch = math.radians(float(data['pitch']))
                        yaw = math.radians(float(data['yaw']))
                        
                        sigma = RigidBodyKinematics.euler3122MRP([yaw, pitch, roll])
                        
                        # Mise à jour de la référence d'attitude
                        msg = messaging.AttRefMsgPayload()
                        msg.sigma_RN = sigma
                        msg.omega_RN_N = [0.0, 0.0, 0.0]
                        self.att_ref_msg_obj.write(msg)
                        
                except zmq.Again:
                    pass  # Pas de nouvelle commande
                    
                time.sleep(0.01)  # Réduire la charge CPU
                
            except Exception as e:
                print(f"Erreur dans la boucle de contrôle: {str(e)}")
                time.sleep(1)

    def stop(self):
        """Arrête le contrôle en temps réel"""
        self.running = False
        self.control_thread.join()
        self.socket.close()
        self.context.term()