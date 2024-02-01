import random


class Blinking:
    def __init__(self):
        self.left_eye = [[0], [1, 11], [2, 10], [3, 9], [4, 8], [5, 7], [6]]
        self.right_eye = [[12], [13, 23], [14, 22], [15, 21], [16, 20], [17, 19], [18]]

        self.BlinkStates = {
            'Open'   : 0,
            'Opening': 1,
            'Closed' : 2,
            'Closing': 3
        }

        self.EyeStates = {
            'Closed'           : 0,
            'QuarterOpen'      : 1,
            'HalfOpen'         : 2,
            'ThreeQuarterOpen' : 3,
            'Open'             : 4
        }

        self.eyeState = self.EyeStates['Closed']
        self.nextEyeState = self.EyeStates['QuarterOpen']

        self.blinkState = self.BlinkStates['Closed']
        self.nextBlinkState = self.BlinkStates['Opening']

        self.lastBlinkAction = 0
        self.blinkActionInterval = 1000
        self.blinkIntervalMin = 2000
        self.blinkIntervalMax = 4000
        self.blinkLength = 0.2

    def what_leds_should_be_on(self):
        retVal = []

        if self.eyeState == self.EyeStates['Closed']:
            retVal = []
        elif self.eyeState == self.EyeStates['QuarterOpen']:
            retVal = [3, 9, 15, 21]        
        elif self.eyeState == self.EyeStates['HalfOpen']:
            retVal = [3, 9, 15, 21, 2, 10, 4, 8, 14, 22, 16, 20]        
        elif self.eyeState == self.EyeStates['ThreeQuarterOpen']:
            retVal = [3, 9, 15, 21, 2, 10, 4, 8, 14, 22, 16, 20, 1, 11, 5, 7, 13, 23, 17, 19]
        elif self.eyeState == self.EyeStates['Open']:
            retVal = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23]
        
        return retVal

    def update_eyelids(self, current_time):
        r, g, b = 150, 150, 150

        if current_time - self.lastBlinkAction > self.blinkActionInterval:
            self.lastBlinkAction = current_time

            # print(f"{current_time}:= blinkState: {self.blinkState}, nextBlinkState: {self.nextBlinkState}, eyeState: {self.eyeState}, nextEyeState: {self.nextEyeState}, interval: {self.blinkActionInterval}")

            # First, if we're open or closed, we need to switch to the next blinking and eye states
            # Finally, we need to switch to the next eye or blinking states
            if self.blinkState == self.BlinkStates['Closed']:
                self.blinkState = self.BlinkStates['Opening']

            elif self.blinkState == self.BlinkStates['Opening']:
                if self.eyeState == self.EyeStates['ThreeQuarterOpen']:
                    self.blinkState = self.BlinkStates['Open']
                    self.eyeState = self.EyeStates['Open']
                    self.blinkActionInterval = random.randint(self.blinkIntervalMin, self.blinkIntervalMax)
                else:
                    self.eyeState = self.eyeState + 1
                    self.blinkActionInterval = self.blinkLength / 2
            
            elif self.blinkState == self.BlinkStates['Open']:
                self.blinkState = self.BlinkStates['Closing']
            
            elif self.blinkState == self.BlinkStates['Closing']:
                if self.eyeState == self.EyeStates['QuarterOpen']:
                    self.blinkState = self.BlinkStates['Closed']
                    self.eyeState = self.EyeStates['Closed']
                    self.blinkActionInterval = self.blinkLength / 2
                else:
                    self.eyeState = self.eyeState - 1
                    self.blinkActionInterval = self.blinkLength / 2

            leds_to_turn_on = self.what_leds_should_be_on()
            return leds_to_turn_on
        
        else:
            return None
