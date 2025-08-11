# import socketserver
# import socket
# import threading
# import math
# import time
# import zmq
# from Basilisk.utilities import RigidBodyKinematics
# from Basilisk.architecture import messaging

# # Adresse et ports






# class VizardZMQClient:
#     def __init__(self, host='localhost', port=5556):
#         self.host = host
#         self.port = port
#         self.context = zmq.Context()
#         self.socket = self.context.socket(zmq.PUB)  # Utilise le mode Publisher
#         self.socket.connect(f"tcp://{self.host}:{self.port}")
#         print(f"Connecté à Vizard via ZMQ sur tcp://{self.host}:{self.port}")

#     def send_orientation(self, roll, pitch, yaw):
#         """Envoie les angles d'orientation à Vizard."""
#         try:
#             # Convertir les angles Euler en radians
#             roll_rad = math.radians(roll)
#             pitch_rad = math.radians(pitch)
#             yaw_rad = math.radians(yaw)

#             # Construire le message à envoyer
#             message = f"ORIENTATION {roll_rad} {pitch_rad} {yaw_rad}"
#             self.socket.send_string(message)
#             print(f"Commande envoyée : {message}")
#         except Exception as e:
#             print(f"Erreur lors de l'envoi à Vizard : {e}")

# # class VizardClient:
# #     """Client TCP pour envoyer les données à Vizard."""
# #     def __init__(self, host, port):
# #         self.host = host
# #         self.port = port
# #         self.sock = None
# #         self.lock = threading.Lock()

# #     def connect(self):
# #         """Établit une connexion avec Vizard."""
# #         with self.lock:
# #             if self.sock:
# #                 self.sock.close()
# #             try:
# #                 self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# #                 self.sock.settimeout(5.0)  # Augmenter le délai de timeout
# #                 self.sock.connect((self.host, self.port))
# #                 print(f"Connecté à Vizard {self.host}:{self.port}")
# #             except Exception as e:
# #                 print(f"Erreur connexion Vizard: {e}")
# #                 self.sock = None
# #                 time.sleep(1)  # Attendre avant de réessayer

    

# #     def retry_send(self, msg):
# #         """Réessaye d'envoyer un message après une reconnexion."""
# #         try:
# #             self.connect()
# #             with self.lock:
# #                 self.sock.sendall(msg.encode())
# #             print(f"Message réenvoyé à Vizard : {msg.strip()}")
# #         except Exception as e:
# #             print(f"Échec lors du réenvoi à Vizard : {e}")


# # Instance globale du client Vizard
# vizard_client = VizardZMQClient()


# def update_simulation(roll, pitch, yaw):
#     """Met à jour la simulation Basilisk et envoie les données à Vizard."""
#     try:
#         # Convertir les angles en MRP (Modified Rodrigues Parameters)
#         sigma = RigidBodyKinematics.euler3122MRP([
#             math.radians(yaw),  # Yaw
#             math.radians(pitch),  # Pitch
#             math.radians(roll)   # Roll
#         ])

#         # Construire le message pour Basilisk
#         msg = messaging.AttRefMsgPayload()
#         msg.sigma_RN = sigma
#         msg.omega_RN_N = [0.0, 0.0, 0.0]  # Pas de vitesse angulaire
#         msg.domega_RN_N = [0.0, 0.0, 0.0]  # Pas d'accélération angulaire

#         # Mettre à jour le message partagé
#         CommandTCPHandler.attRefMsg.write(msg)
#         print(f"Simulation mise à jour : roll={roll}, pitch={pitch}, yaw={yaw}")

#         # Envoyer les données à Vizard
#         vizard_client.send_orientation(roll, pitch, yaw)
#     except Exception as e:
#         print(f"Erreur lors de la mise à jour de la simulation : {e}")
