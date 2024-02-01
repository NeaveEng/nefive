# This file runs at power-up. It and files in the /lib directory get added to a LittleFS filesystem and appended to the firmware
# Overide this file with any of your own code that you wish to run at power-up, or remove the file to not have anything run.

from nefive_yukon import NEFive

"""
A program to show that Yukon is active, by flashing the onboard LEDs in sequence.
"""

nefive = NEFive()
nefive.run()