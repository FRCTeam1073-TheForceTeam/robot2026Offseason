# import winsound
import pygame

running = True

# winsound.PlaySound("C:/Users/FRC1073/Desktop/Mario-coin-sound.wav", winsound.SND_FILENAME)
# pygame docs: https://www.pygame.org/docs/ref/joystick.html
# work for bluetooth connected controller but not hard connect

pygame.init()
pygame.joystick.init()
pygame.mixer.init()
sound = pygame.mixer.Sound("C:/Users/FRC1073/Desktop/Mario-coin-sound.mp3")
arjun_sound = pygame.mixer.Sound("C:/Users/FRC1073/Desktop/Arjun_noise.mp3")
spencer_sound = pygame.mixer.Sound("C:/Users/FRC1073/Desktop/spencer.mp3")
viraj_sound = pygame.mixer.Sound("C:/Users/FRC1073/Desktop/Viraj.mp3")
hyper_sound = pygame.mixer.Sound("C:/Users/FRC1073/Desktop/hyper.mp3")
held = False
count = 0


joystick_count = pygame.joystick.get_count()
print(f'Joystick Count: {joystick_count}')

if joystick_count == 0:
	print("No joysticks found.")
	pygame.QUIT
else:
	joystickDriver = pygame.joystick.Joystick(0)
	joystickPrimary = pygame.joystick.Joystick(1)
	joystickSecondary = pygame.joystick.Joystick(2)
	joystickDriver.init()
	joystickPrimary.init()
	joystickSecondary.init()
	print(f"Joystick Driver {joystickDriver.get_name()} initialized")
	print(f"Joystick Primary {joystickPrimary.get_name()} initialized")
	print(f"Joystick Secondary {joystickSecondary.get_name()} initialized")

while running:
	count += 1
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			running = False
		if event.type == pygame.JOYBUTTONDOWN:
			print(f'Button pressed: {event.button}, Joystick: {event.joy}')

			if(event.button == 6 and event.joy == 2):
				sound.play()
			elif(event.button == 9 and event.joy == 2):
				arjun_sound.play()
			elif(event.button == 1 and event.joy == 1):
				spencer_sound.play()
			elif(event.button == 0 and event.joy == 2):
				viraj_sound.play()


		if event.type == pygame.JOYBUTTONUP:
			print("Button released:", event.button)
		
		if(joystickDriver.get_axis(4) <= -0.5 and joystickDriver.get_axis(5) <= -0.5):
			held = False
		
		if event.type == pygame.JOYAXISMOTION:
			print("Axis moved:", event.axis, "Value:", event.value)
			
		if((joystickDriver.get_axis(4) >= 0.25 or joystickDriver.get_axis(5) >= 0.25) and held == False and count > 10):
				held = True
				count = 0
				hyper_sound.play()

		if event.type == pygame.JOYHATMOTION:
			print("Hat switch moved:", event.hat, "Value:", event.value)

pygame.QUIT
