"""
Joshua Katz
9/10/2020

This is a test to use a wired ps3 controller on windows with threading
"""

import pygame
import time

pygame.init()

# Find game controller
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    # No joysticks!
    print("Error, I didn't find any joysticks.")
    exit()
else:
    # Use joystick #0 and initialize it
    my_joystick = pygame.joystick.Joystick(0)
    my_joystick.init()

done = False
horiz_axis_pos = 0
vert_axis_pos = 0
while not done:
    start_time = time.time()

    for event in pygame.event.get():  # User did something
        print(event)

        if event.type == pygame.JOYBUTTONDOWN:  # Game Controller Button detection
            pass

    if joystick_count != 0:  # joystick detection
        horiz_axis_pos = my_joystick.get_axis(
            0)  # 0&1 for left stick, 2&3 for right stick
        vert_axis_pos = my_joystick.get_axis(1)

    #print(round(horiz_axis_pos,2))
    #print(round(vert_axis_pos,2))
    #print("")
    time.sleep(0.01)
