import cv2
import os
import time
from datetime import datetime

# Paramètres
interval_seconds = 5  # Intervalle de temps entre chaque capture en secondes
save_directory = "captured_images"  # Dossier où les images seront enregistrées

# Création du dossier pour stocker les images, s'il n'existe pas
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# Initialisation de la caméra (0 pour la caméra par défaut, essayez 1 si vous utilisez une autre webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Erreur: Impossible d'accéder à la webcam.")
    exit()

try:
    print("Press Ctrl+C to stop capturing images.")
    while True:
        # Capture de l'image
        ret, frame = cap.read()
        if not ret:
            print("Erreur: Impossible de lire une frame.")
            break

        # Génération d'un nom unique pour l'image avec horodatage
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(save_directory, f"image_{timestamp}.jpg")

        # Sauvegarde de l'image
        cv2.imwrite(image_path, frame)
        print(f"Image capturée et enregistrée : {image_path}")

        # Pause pour l'intervalle spécifié
        time.sleep(interval_seconds)

except KeyboardInterrupt:
    print("Capture d'images arrêtée par l'utilisateur.")

# Libération de la caméra et fermeture des fenêtres
cap.release()
cv2.destroyAllWindows()
