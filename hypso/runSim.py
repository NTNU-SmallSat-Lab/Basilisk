#
# Python script 
#
# Purpose:  Kills any process running on the specified ports 
#           and launches the Vizard, Basilisk, and interface components in separate threads.



import subprocess
import threading
import time
import sys
import platform
import os
import socket

def kill_process_on_port(port):
    if platform.system() == "Windows":
        os.system(f"taskkill /F /PID $(netstat -ano | findstr :{port} | awk '{{print $5}}')")
    else:
        os.system(f"kill -9 $(lsof -t -i:{port})")


def launch_vizard():
    print("Launching Vizard...")
    subprocess.Popen(["bash", "vizard_launcher.sh"])

def launch_basilisk():
    time.sleep(5)
    print("Launching Basilisk...")
    subprocess.Popen([sys.executable, "testHypso.py"])

def launch_interface():
    time.sleep(15)
    print("Launch the interface...")
    subprocess.Popen([sys.executable, "orientation.py"])

if __name__ == "__main__":
    # Assures that the port used are free to be used
    kill_process_on_port(5559)
    kill_process_on_port(5558)
    kill_process_on_port(5557)

    # Launch each component in a separate thread
    threads = [
        threading.Thread(target=launch_vizard),
        threading.Thread(target=launch_basilisk),
        threading.Thread(target=launch_interface)
    ]

    for t in threads:
        t.start()

    for t in threads:
        t.join() 


    
    while True:
        time.sleep(0.1)