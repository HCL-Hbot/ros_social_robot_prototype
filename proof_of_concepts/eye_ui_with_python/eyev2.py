import pygame

# Initialiseer Pygame
pygame.init()

# Stel HDMI-schermen in (fullscreen mode)
screen1 = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=0)  # HDMI-scherm 1
#screen2 = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=1)  # HDMI-scherm 2

# Haal de schermresoluties op
screen1_width, screen1_height = screen1.get_size()
#screen2_width, screen2_height = screen2.get_size()

# Basisinstellingen
running = True
clock = pygame.time.Clock()

# Pupilpositie (start in het midden van het oog)
pupil_offset_x = 0
pupil_offset_y = 0

# Dynamische schaal (oog en pupil gebaseerd op schermresolutie)
eye_radius = min(screen1_width, screen1_height) // 4  # Grootte van het oog
pupil_radius = eye_radius // 3 # Grootte van de pupil
eye_center_x = screen1_width // 2
eye_center_y = screen1_height // 2
pupil_range = eye_radius - pupil_radius  # Beweging van pupil binnen het oog

# Functie voor tekenen van een oog
def draw_eye(screen, x, y, pupil_offset_x, pupil_offset_y, eye_radius, pupil_radius):
    # Teken het oogwit
    pygame.draw.circle(screen, (255, 255, 255), (x, y), eye_radius)  
    # Teken de pupil
    pygame.draw.circle(screen, (0, 0, 0), 
                       (x + pupil_offset_x, y + pupil_offset_y), 
                       pupil_radius)

# Hoofdloop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            # Pijltoetsen input
            if event.key == pygame.K_UP:
                pupil_offset_y = -pupil_range
            elif event.key == pygame.K_DOWN:
                pupil_offset_y = pupil_range
            elif event.key == pygame.K_LEFT:
                pupil_offset_x = -pupil_range
            elif event.key == pygame.K_RIGHT:
                pupil_offset_x = pupil_range
        if event.type == pygame.KEYUP:
            # Reset pupil naar midden
            if event.key in [pygame.K_UP, pygame.K_DOWN]:
                pupil_offset_y = 0
            if event.key in [pygame.K_LEFT, pygame.K_RIGHT]:
                pupil_offset_x = 0

    # Clear schermen
    screen1.fill((0, 0, 0))
    #screen2.fill((0, 0, 0))

    # Teken ogen
    draw_eye(screen1, eye_center_x, eye_center_y, pupil_offset_x, pupil_offset_y, eye_radius, pupil_radius)
    #draw_eye(screen2, eye_center_x, eye_center_y, pupil_offset_x, pupil_offset_y, eye_radius, pupil_radius)

    # Update schermen
    pygame.display.update()
    clock.tick(30)

pygame.quit()
