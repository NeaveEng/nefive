# This isn't used by the vision package but I keep losing the script and this seemed a sensible place to save it...

import board
import neopixel
import time
import random
import os

user = os.getuid()
if user != 0:
    print("This program requires root privileges to control the NeoPixels, run as root using 'sudo'.")
    exit()

pixels = neopixel.NeoPixel(board.D12, 32)

def larsonScanner():
	while True:
		# print("Ascending...")
		for x in range(25, 32):
			next = x - 1
			if next == -1:
				next = 31
			
			# print(f"X: {x}, Next: {next}")

			pixels[next] = (0, 0, 0)
			pixels[x] = (255, 0, 0)

			time.sleep(0.05)

		# print("Decending...")
		for y in range(31, 23, -1):
			next = y + 1
			if next == 32:
				next = 24
				
			# print(f"Y: {y}, Next: {next}")

			pixels[next] = (0, 0, 0)
			pixels[y] = (255, 0, 0)

			time.sleep(0.05)


		pixels[24] = (0, 0, 0)
		time.sleep(0.5)

def setRowColour(row, colour):
	pixels[row[0]] = colour
	pixels[row[1]] = colour

def blinking(colour):
	print("blinking eyes...")
	blink_interval_min = 4
	blink_interval_max = 8

	off = (0,0,0)

	while True:
		length = 0.1
		for i in [0, 1, 2]:
			setRowColour(left_eye[i], off)
			setRowColour(left_eye[5-i], off)
			setRowColour(right_eye[i], off)
			setRowColour(right_eye[5-i], off)
			time.sleep(length / 2)

		for i in [2, 1, 0]:
			setRowColour(left_eye[i], colour)
			setRowColour(left_eye[5-i], colour)
			setRowColour(right_eye[i], colour)
			setRowColour(right_eye[5-i], colour)
			time.sleep(length / 2)


		interval = random.randint(blink_interval_min, blink_interval_max)
		time.sleep(interval)



left_eye = [[0, 11], [1, 10], [2, 9], [3, 8], [4, 7], [5, 6]]

right_eye = [[12, 13], [14, 23], [15, 22], [16, 21], [17, 20], [18, 19]]


brightness = 135

# for x in range(24, 32):
pixels.fill((25, 0, 0))
# pixels[0] = (255, 0, 0)

# blinking((75, 75, 75))
