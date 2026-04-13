import pygame
pygame.init()
pygame.joystick.init()
j = pygame.joystick.Joystick(0)
j.init()
print('buttons:', j.get_numbuttons())
while True:
    pygame.event.pump()
    for i in range(j.get_numbuttons()):
        if j.get_button(i):
            print('Button pressed:', i)
