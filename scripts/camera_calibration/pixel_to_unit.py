import cv2
import numpy as np
import matplotlib.pyplot as plt

# Bestandspad van de afbeelding
df_image_path = "my_photo-13.jpg"  # Pas dit aan naar de juiste bestandsnaam

# Werkelijke breedte van het doosje in mm
known_width_mm = 135  # 13.5 cm omgezet naar mm

# Lijst om de geselecteerde punten op te slaan
selected_points = []

# Callback-functie om muisklikken op te vangen en punten op te slaan
def select_points(event, x, y, flags, param):
    global selected_points, image_copy
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_points.append((x, y))
        cv2.circle(image_copy, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Selecteer de hoeken van het doosje", image_copy)

# Laad de afbeelding en controleer of deze correct geladen is
image = cv2.imread(df_image_path)
if image is None:
    raise FileNotFoundError(f"Kan afbeelding niet laden: {df_image_path}")
image_copy = image.copy()

# Toon de afbeelding en wacht op vier muisklikken
cv2.imshow("Selecteer de hoeken van het doosje", image_copy)
cv2.setMouseCallback("Selecteer de hoeken van het doosje", select_points)

# Wacht tot vier punten zijn geselecteerd
while len(selected_points) < 4:
    if cv2.waitKey(1) & 0xFF == 27:  # Druk op ESC om af te breken
        cv2.destroyAllWindows()
        exit()

cv2.destroyAllWindows()

# Controleer of voldoende punten zijn geselecteerd
if len(selected_points) == 4:
    x_coords = [p[0] for p in selected_points]
    y_coords = [p[1] for p in selected_points]
    
    # Bereken de breedte van het geselecteerde object (ROI)
    pixel_width = max(x_coords) - min(x_coords)
    pixel_height = max(y_coords) - min(y_coords)
    
    # Bereken pixels per mm
    pixels_per_mm = pixel_width / known_width_mm
    print(f"Breedte in pixels: {pixel_width:.2f}")
    print(f"Pixels per mm: {pixels_per_mm:.4f}")
    
    # Teken de ROI op de afbeelding
    roi_image = image.copy()
    cv2.rectangle(roi_image, (min(x_coords), min(y_coords)), (max(x_coords), max(y_coords)), (255, 0, 0), 2)
    
    # Toon de afbeelding met de geselecteerde ROI
    plt.figure(figsize=(10, 6))
    plt.imshow(cv2.cvtColor(roi_image, cv2.COLOR_BGR2RGB))
    plt.title("Geselecteerde ROI van het doosje")
    plt.axis("off")
    plt.show()
else:
    print("Niet genoeg punten geselecteerd!")