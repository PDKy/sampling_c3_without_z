import pygame
import dairlib.lcmt_radio_out
import lcm
import numpy as np

# colors
cassie_blue = (6, 61, 128)
white = (255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 30)

    def print(self, screen, textString):
        textList = textString.split("\n")
        for string in textList:
            textBitmap = self.font.render(string, True, white)
            screen.blit(textBitmap, [self.x, self.y])
            self.y += self.line_height
        self.y += self.new_line_height


    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 30
        self.new_line_height = 40

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

def main():
    publisher = lcm.LCM()

    pygame.init()

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()
    textPrint = TextPrint()

    latching_switch = 0
    # radio_msg.channel[14] = 0

    if (pygame.joystick.get_count() != 1):
        raise RuntimeError("Please connect exactly one controller")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    done = False
    i = 0
    latching_switch_a = 1
    latching_switch_b = 1
    latching_switch_x = 0
    latching_switch_y = 0
    latching_switch_start = 0
    latching_switch_back = 0
    print("Teleop Status: " + str(latching_switch_a))
    print("Move C3 Target with Remote Status: " + str(latching_switch_b))
    print("Force Tracking Status: " + str(not latching_switch_x))


    while not done:
        # DRAWING STEP
        # First, clear the screen to blue. Don't put other drawing commands
        # above this, or they will be erased with this command.
        # textPrint.reset()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                # use this print statement to find the button number
                # print(f"Button {event.button} pressed")
                if event.button == 0:
                    latching_switch_a = not latching_switch_a
                    print("Teleop Status: " + str(latching_switch_a))
                if event.button == 1:
                    latching_switch_b = not latching_switch_b
                    print("Move C3 Target with Remote Status: " + str(latching_switch_b))
                if event.button == 2:
                    latching_switch_x = not latching_switch_x
                    print("Force Tracking Status: " + str(not latching_switch_x))
                if event.button == 3:
                    latching_switch_y = not latching_switch_y
                    print("Force C3 Mode Status: " + str(latching_switch_y))
                if event.button == 7:
                    latching_switch_start = not latching_switch_start  # Send signal when pressed
                    print("Print cost breakdown status: " + str(latching_switch_start))
                if event.button == 6:
                    latching_switch_back = not latching_switch_back  # Send signal when pressed
                    print("Print current rot and pos cost status: " + str(latching_switch_back))

        # Send LCM message
        radio_msg = dairlib.lcmt_radio_out()
        radio_msg.channel[0] = -joystick.get_axis(1)
        radio_msg.channel[1] = -joystick.get_axis(0)
        radio_msg.channel[2] = -joystick.get_axis(4)
        radio_msg.channel[3] = joystick.get_axis(3)
        # radio_msg.channel[2] = -joystick.get_axis(3)
        # radio_msg.channel[3] = joystick.get_axis(2)
        radio_msg.channel[6] = latching_switch_back
        radio_msg.channel[7] = latching_switch_start
        radio_msg.channel[13] = latching_switch_b
        radio_msg.channel[14] = latching_switch_a
        radio_msg.channel[11] = latching_switch_x
        radio_msg.channel[12] = latching_switch_y
        radio_msg.channel[15] = -1 * np.rint(joystick.get_axis(5))


        publisher.publish("SAMPLING_C3_RADIO", radio_msg.encode())

        # Limit to 20 frames per second
        clock.tick(60)
        i += 1

    pygame.quit()

if __name__ == '__main__':
    main()
