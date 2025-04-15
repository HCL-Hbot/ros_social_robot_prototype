import pygame

# Initialize Pygame
pygame.init()

# Set up HDMI screens
screen1 = pygame.display.set_mode((800, 600), display=0)  # HDMI screen 1
#screen2 = pygame.display.set_mode((800, 600), display=1)  # HDMI screen 2 (commented out, optional)

# Basic settings
running = True  # Variable to keep the main loop running
clock = pygame.time.Clock()  # Clock object to control the frame rate

# Pupil position (starting in the center of the eye)
pupil_offset_x = 0  # Horizontal offset of the pupil
pupil_offset_y = 0  # Vertical offset of the pupil

# Function to draw an eye
def draw_eye(screen, x, y, pupil_offset_x, pupil_offset_y):
    pygame.draw.circle(screen, (255, 255, 255), (x, y), 100)  # Draw the outer eye (white)
    pygame.draw.circle(screen, (0, 0, 0), (x + pupil_offset_x, y + pupil_offset_y), 30)  # Draw the pupil

# Main loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # Check if the window is closed
            running = False
        if event.type == pygame.KEYDOWN:  # Check if a key is pressed
            # Move the pupil with arrow keys
            if event.key == pygame.K_UP:
                pupil_offset_y = -30  # Move pupil up
            elif event.key == pygame.K_DOWN:
                pupil_offset_y = 30  # Move pupil down
            elif event.key == pygame.K_LEFT:
                pupil_offset_x = -30  # Move pupil left
            elif event.key == pygame.K_RIGHT:
                pupil_offset_x = 30  # Move pupil right
        if event.type == pygame.KEYUP:  # Check if a key is released
            # Reset the pupil to the center when arrow keys are released
            if event.key in [pygame.K_UP, pygame.K_DOWN]:
                pupil_offset_y = 0
            if event.key in [pygame.K_LEFT, pygame.K_RIGHT]:
                pupil_offset_x = 0

    # Clear the screens
    screen1.fill((0, 0, 0))  # Clear screen 1 with black
#    screen2.fill((0, 0, 0))  # Clear screen 2 with black (optional)

    # Draw the eyes
    draw_eye(screen1, 400, 300, pupil_offset_x, pupil_offset_y)  # Draw an eye on screen 1
#    draw_eye(screen2, 400, 300, pupil_offset_x, pupil_offset_y)  # Draw an eye on screen 2 (optional)

    # Update the screens
    pygame.display.update()
    clock.tick(30)  # Limit the frame rate to 30 FPS

# Quit Pygame
pygame.quit()
