import tkinter as tk
import socket
import time
import zmq
import math
import json
from multiprocessing import Process

import os
from Basilisk import __path__ 
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
from Basilisk.architecture import messaging
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import vizProtobuffer
import psutil
import platform




PUB_PORT = 5559  # Commandes vers Basilisk
SUB_PORT = 5558  # Retours de Basilisk


class ZMQManager:
    def __init__(self):
        self.context = zmq.Context()
        

        # Publisher pour les commandes
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{PUB_PORT}")
        
        # Subscriber pour les retours
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{SUB_PORT}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')
    
    def send_command(self, data):
        try:
            self.pub_socket.send_string(json.dumps(data))
        except zmq.ZMQError as e:
            print(f"Sending error: {e}")



def listen_zmq():
    print("Subscriber ZMQ ready")
    global zmq_mgr
    while True:
        try:
            message = zmq_mgr.sub_socket.recv_string()
            data = json.loads(message)
            print(f"Received from Basilisk: {data}")  # Recevoir les données brutes
            if not data:
                break
            print(f"Received : {data}")
            _, roll, pitch, yaw = data.split()
            roll, pitch, yaw = float(roll), float(pitch), float(yaw)
            # Mettre à jour la simulation et envoyer à Vizard
            process_command("ORIENTATION {} {} {}".format(roll, pitch, yaw))
            
        except Exception as e:
            print(f"Error in the handler : {e}")
            break


def process_command(command):
    global vizard_client
    port = 5558
    server = None
    
    if command.startswith("ORIENTATION"):
        print(f"Message received : sigma_RN={command}")
        try:
            _, roll, pitch, yaw = command.split()
            current_attitude = [float(roll), float(pitch), float(yaw)]

            # Convertir en MRP (Modified Rodrigues Parameters)
            sigma = RigidBodyKinematics.euler3122MRP(
                [math.radians(float(yaw)),  # Yaw
                math.radians(float(pitch)),  # Pitch
                math.radians(float(roll))]  # Roll
            )

            # Mettre à jour le message Basilisk
            msg = messaging.AttRefMsgPayload()
            msg.sigma_RN = sigma
            msg.omega_RN_N = [0.0, 0.0, 0.0]  # Pas de vitesse angulaire
            
            attRefMsg = messaging.AttRefMsg().write(msg)
            print(f"Simulation updated : roll={roll}, pitch={pitch}, yaw={yaw}")

            # Envoyer les données à Vizard
            try:
                vizard_client.send_orientation(float(roll), float(pitch), float(yaw))
            except Exception as e:
                print(f"Error sending to Vizard : {e}")

        except Exception as e:
            print(f"Error processing order: {e}")

    return attRefMsg






class ControlApp:
    def __init__(self, root):
        global zmq_mgr
        self.root = root
        self.zmq_mgr = zmq_mgr
        self.host = 'localhost'
        self.port_basilisk = 5557
        self.sock = None
        self.socket = None
        self.port = 5556

        message = {"roll": 10.0, "pitch": 20.0, "yaw": 30.0}
        self.zmq_mgr.send_command(message)

        #  # Initialiser le client ZMQ
        # self.context = zmq.Context()
        # self.socket = self.context.socket(zmq.PUB)
        # self.socket.bind(f"tcp://localhost:5556")
        # print(f"Connecté à Vizard via ZMQ sur tcp://localhost:5556")


        payload = messaging.AttRefMsgPayload()
        payload.sigma_RN = [0.0, 0.0, 0.0]  # Orientation initiale en MRP
        payload.omega_RN_N = [0.0, 0.0, 0.0]  # Vitesse angulaire initiale

         # Initialiser le message Basilisk
        self.att_ref_msg_object = messaging.AttRefMsg()
        self.att_ref_msg = self.att_ref_msg_object.write

        # Créer les curseurs avec un événement lié au mouvement
        self.roll = tk.Scale(root, length=720, width=20, from_=-180, to=180, label="Roll", orient='horizontal', command=self.send_command)
        self.pitch = tk.Scale(root, length=720, width=20, from_=-180, to=180, label="Pitch", orient='horizontal', command=self.send_command)
        self.yaw = tk.Scale(root, length=720, width=20, from_=-180, to=180, label="Yaw", orient='horizontal', command=self.send_command)
        self.roll.pack()
        self.pitch.pack()
        self.yaw.pack()


        

    def send_command(self,_):
        try:

            # Récupérer les valeurs des curseurs
            roll = float(self.roll.get())
            pitch = float(self.pitch.get())
            yaw = float(self.yaw.get())

            # Convertir les angles en MRP pour Basilisk
            sigma = RigidBodyKinematics.euler3122MRP([
                math.radians(yaw),   # Yaw
                math.radians(roll), # Roll
                math.radians(pitch)   # Pitch
            ])
            
            #  # Créer un message pour Basilisk
            # msg = messaging.AttRefMsgPayload()
            # msg.sigma_RN = sigma
            # msg.omega_RN_N = [0.0, 0.0, 0.0]  # Pas de vitesse angulaire
            # self.att_ref_msg(msg)  # Envoyer le message à Basilisk
            # print(f"Envoyé à Basilisk : sigma_RN={sigma}")

            # Créer un message pour Vizard
            rotations = {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "type": "attitude_command"
            }
            self.zmq_mgr.send_command(rotations)  # Envoyer les données à Vizard
            print(f"Sending to Vizard : {rotations}")

        except Exception as e:
            print(f"Error sending : {e}")


if __name__ == "__main__":

    global zmq_mgr
    zmq_mgr = ZMQManager()
    zmq_proc = Process(target=listen_zmq, daemon=True)
    zmq_proc.start()

    root = tk.Tk()
    root.title("Satellite orientation control")
    app = ControlApp(root)
    
    try:
        root.mainloop()
    finally:
        print("Application closing...")
        zmq_proc.terminate()
        zmq_proc.join()
        zmq_mgr.pub_socket.close()
        zmq_mgr.sub_socket.close()
        zmq_mgr.context.term()
        root.quit()

    

    