import pygame

# Initialize Pygame
pygame.init()

# Set up HDMI screens (fullscreen mode)
screen1 = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=0)  # HDMI screen 1
#screen2 = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, display=1)  # HDMI screen 2 (optional)

# Get screen resolutions
screen1_width, screen1_height = screen1.get_size()  # Resolution of screen 1
#screen2_width, screen2_height = screen2.get_size()  # Resolution of screen 2 (optional)

# Basic settings
running = True  # Variable to keep the main loop running
clock = pygame.time.Clock()  # Clock object to control the frame rate

# Pupil position (starting in the center of the eye)
pupil_offset_x = 0  # Horizontal offset of the pupil
pupil_offset_y = 0  # Vertical offset of the pupil

# Dynamic scaling (eye and pupil size based on screen resolution)
eye_radius = min(screen1_width, screen1_height) // 4  # Size of the eye (proportional to screen size)
pupil_radius = eye_radius // 3  # Size of the pupil (proportional to the eye size)
eye_center_x = screen1_width // 2  # Horizontal center of the eye
eye_center_y = screen1_height // 2  # Vertical center of the eye
pupil_range = eye_radius - pupil_radius  # Range of pupil movement within the eye

# Function to draw an eye
def draw_eye(screen, x, y, pupil_offset_x, pupil_offset_y, eye_radius, pupil_radius):
    # Draw the white part of the eye
    pygame.draw.circle(screen, (255, 255, 255), (x, y), eye_radius)  
    # Draw the pupil
    pygame.draw.circle(screen, (0, 0, 0), 
                       (x + pupil_offset_x, y + pupil_offset_y), 
                       pupil_radius)

# Main loop
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # Check if the window is closed
            running = False
        if event.type == pygame.KEYDOWN:  # Check if a key is pressed
            # Handle arrow key input to move the pupil
            if event.key == pygame.K_UP:
                pupil_offset_y = -pupil_range  # Move pupil up
            elif event.key == pygame.K_DOWN:
                pupil_offset_y = pupil_range  # Move pupil down
            elif event.key == pygame.K_LEFT:
                pupil_offset_x = -pupil_range  # Move pupil left
            elif event.key == pygame.K_RIGHT:
                pupil_offset_x = pupil_range  # Move pupil right
        if event.type == pygame.KEYUP:  # Check if a key is released
            # Reset the pupil to the center when arrow keys are released
            if event.key in [pygame.K_UP, pygame.K_DOWN]:
                pupil_offset_y = 0
            if event.key in [pygame.K_LEFT, pygame.K_RIGHT]:
                pupil_offset_x = 0

    # Clear the screens
    screen1.fill((0, 0, 0))  # Clear screen 1 with black
    #screen2.fill((0, 0, 0))  # Clear screen 2 with black (optional)

    # Draw the eyes
    draw_eye(screen1, eye_center_x, eye_center_y, pupil_offset_x, pupil_offset_y, eye_radius, pupil_radius)  # Draw eye on screen 1
    #draw_eye(screen2, eye_center_x, eye_center_y, pupil_offset_x, pupil_offset_y, eye_radius, pupil_radius)  # Draw eye on screen 2 (optional)

    # Update the screens
    pygame.display.update()
    clock.tick(30)  # Limit the frame rate to 30 FPS

# Quit Pygame
pygame.quit()
