'''

import tkinter as tk
from Basilisk.utilities import macros as mc

def update_pointing():
    # Récupérer les valeurs des curseurs
    lat = lat_slider.get()
    lon = lon_slider.get()
    
    # Écrire les données dans un fichier (pour Vizard)
    with open("viz_command.txt", "w") as f:
        f.write(f"TARGET_LAT_LON {lat} {lon}")

# Fenêtre Tkinter
root = tk.Tk()
root.title("Contrôle d'orientation")

# Curseurs
lat_slider = tk.Scale(root, from_=-90, to=90, orient="horizontal", label="Latitude (°)")
lon_slider = tk.Scale(root, from_=-180, to=180, orient="horizontal", label="Longitude (°)")
lat_slider.pack()
lon_slider.pack()

# Bouton pour envoyer les commandes
tk.Button(root, text="Mettre à jour Vizard", command=update_pointing).pack()

root.mainloop()








import viz
import time

viz.go()  # Lancer Vizard

while True:
    try:
        with open("viz_command.txt", "r") as f:
            data = f.read().strip()
            if data.startswith("TARGET_LAT_LON"):
                lat, lon = map(float, data.split()[1:])
                viz.MainView.setEuler(lon, lat, 0)  # Mettre à jour la caméra/satellite
    except:
        pass
    time.sleep(0.1)




















'''