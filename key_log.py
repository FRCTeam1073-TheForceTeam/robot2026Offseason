# import winsound
import pygame
import os

running = True

# winsound.PlaySound("C:/Users/FRC1073/Desktop/Mario-coin-sound.wav", winsound.SND_FILENAME)
# pygame docs: https://www.pygame.org/docs/ref/joystick.html
# work for bluetooth connected controller but not hard connect

pygame.init()
pygame.joystick.init()
pygame.mixer.init()

name = "log_file0"
found = False
count = 1

while(not found):
	if(name in os.listdir("C:/Users/FRC1073/Documents/GitHub/robot2025")):
		name = name[0:-1] + str(count)
		count += 1
		continue
	break

with open(name, "x") as file:
	joystick_count = pygame.joystick.get_count()
	file.write(f'Joystick Count: {joystick_count}\n')

	if joystick_count == 0:
		file.write("No joysticks found.\n")
		pygame.QUIT
	else:
		joystickDriver = pygame.joystick.Joystick(0)
		joystickPrimary = pygame.joystick.Joystick(1)
		joystickSecondary = pygame.joystick.Joystick(2)
		joystickDriver.init()
		joystickPrimary.init()
		joystickSecondary.init()
		file.write(f"Joystick Driver {joystickDriver.get_name()} initialized\n")
		file.write(f"Joystick Primary {joystickPrimary.get_name()} initialized\n")
		file.write(f"Joystick Secondary {joystickSecondary.get_name()} initialized\n")

	while running:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
				file.write("closing...\n")
				file.close()

			if event.type == pygame.JOYBUTTONDOWN:
				file.write(f'Button pressed: {event.button}, Joystick: {event.joy}\n')

			if event.type == pygame.JOYBUTTONUP:
				file.write(f'Button released: {event.button}\n')
				
			if event.type == pygame.JOYAXISMOTION:
				file.write(f'Axis moved: {event.axis}, Value: {event.value}\n')

pygame.QUIT
