import cv2
import numpy as np
import json
import os

def select_roi(image_path):
    """
    Laat de gebruiker een ROI (Region of Interest) selecteren in de afbeelding.
    """
    image = cv2.imread(image_path)
    if image is None:
        print("Fout: Kan afbeelding niet laden.")
        return None
    
    r = cv2.selectROI("Selecteer het referentie-object", image, fromCenter=False, showCrosshair=True)
    cv2.destroyAllWindows()
    
    x, y, w, h = r  # ROI-coördinaten
    
    # Toon de geselecteerde ROI en sla de afbeelding op
    roi_image = image.copy()
    cv2.rectangle(roi_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow("Geselecteerde ROI", roi_image)
    cv2.imwrite("detected_roi.jpg", roi_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return w  # Breedte van het object in pixels

def calculate_focal_length_pixels(image_path, reference_object_width_mm, reference_object_distance_mm):
    """
    Berekent de focal length in pixels op basis van een referentie-object.
    """
    object_width_pixels = select_roi(image_path)
    if object_width_pixels is None or object_width_pixels == 0:
        print("Fout: Geen geldige ROI geselecteerd.")
        return None
    
    focal_length_pixels = (object_width_pixels * reference_object_distance_mm) / reference_object_width_mm
    return focal_length_pixels

def save_focal_length(focal_length, file_path="focal_length.json"):
    """
    Slaat de berekende focal length op in een JSON-bestand.
    """
    data = {"focal_length_pixels": focal_length}
    with open(file_path, "w") as f:
        json.dump(data, f)
    print(f"Focal length opgeslagen in {file_path}")

def main():
    """
    Voert de kalibratie uit en slaat de focal length op.
    """
    image_path = "my_photo-17.jpg"#"my_photo-5.jpg"  # Pas dit aan naar de juiste afbeelding
    reference_object_width_mm = 135 #297  # Bijvoorbeeld A4-papier breedte (landscape)
    reference_object_distance_mm = 1000#650  # Afstand tot object in mm
    
    focal_length = calculate_focal_length_pixels(image_path, reference_object_width_mm, reference_object_distance_mm)
    
    if focal_length:
        print(f"Berekende focal length: {focal_length:.2f} pixels")
        save_focal_length(focal_length)
    else:
        print("Kalibratie mislukt.")

if __name__ == "__main__":
    main()