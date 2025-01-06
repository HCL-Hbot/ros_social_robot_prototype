import pygame

# Initialiseer Pygame
pygame.init()

# Stel HDMI-schermen in
screen1 = pygame.display.set_mode((800, 600), display=0)  # HDMI-scherm 1
#screen2 = pygame.display.set_mode((800, 600), display=1)  # HDMI-scherm 2

# Basisinstellingen
running = True
clock = pygame.time.Clock()

# Pupilpositie (start in het midden van het oog)
pupil_offset_x = 0
pupil_offset_y = 0

# Functie voor tekenen van een oog
def draw_eye(screen, x, y, pupil_offset_x, pupil_offset_y):
    pygame.draw.circle(screen, (255, 255, 255), (x, y), 100)  # Oogwit
    pygame.draw.circle(screen, (0, 0, 0), (x + pupil_offset_x, y + pupil_offset_y), 30)  # Pupil

# Hoofdloop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            # Pijltoetsen input
            if event.key == pygame.K_UP:
                pupil_offset_y = -30
            elif event.key == pygame.K_DOWN:
                pupil_offset_y = 30
            elif event.key == pygame.K_LEFT:
                pupil_offset_x = -30
            elif event.key == pygame.K_RIGHT:
                pupil_offset_x = 30
        if event.type == pygame.KEYUP:
            # Reset pupil naar midden
            if event.key in [pygame.K_UP, pygame.K_DOWN]:
                pupil_offset_y = 0
            if event.key in [pygame.K_LEFT, pygame.K_RIGHT]:
                pupil_offset_x = 0

    # Clear schermen
    screen1.fill((0, 0, 0))
#    screen2.fill((0, 0, 0))

    # Teken ogen
    draw_eye(screen1, 400, 300, pupil_offset_x, pupil_offset_y)
 #   draw_eye(screen2, 400, 300, pupil_offset_x, pupil_offset_y)

    # Update schermen
    pygame.display.update()
    clock.tick(30)

pygame.quit()
