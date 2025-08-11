#!/bin/bash
# Lancer Vizard en mode live (__live__)

cd "./vizard/Vizard_Linux" || { echo "❌ Échec du cd vers .vizard/Vizard_Linux"; exit 1; }
./Vizard.x86_64 --liveStream