from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *
from runs import *

hub = PrimeHub(Axis.Y, Axis.Z)

hub.display.orientation(Side.BOTTOM)

while True:
    hub.display.char("1")
    selected = None

    # Polling loop waits for user input
    while True:
        pressed = hub.buttons.pressed()
        hub.system.set_stop_button(None)

        if Button.CENTER in pressed:
            while hub.buttons.pressed():
                wait(10)
            selected = "1"
            break

        elif Button.LEFT in pressed:
            while hub.buttons.pressed():
                wait(10)
            selected = hub_menu("2", "4", "3")
            break

        elif Button.RIGHT in pressed:
            while hub.buttons.pressed():
                wait(10)
            selected = hub_menu("5", "6", "7", "8", "9")
            break

        wait(10)

    # Execution block runs the selected option
    hub.system.set_stop_button(Button.CENTER)
    if selected == "1":
        run1()
        break
    elif selected == "2":
        run2point5()
        break
    elif selected == "3":
        run3()
        break
    elif selected == "4":
        run4()
        break
    elif selected == "5":
        run5()
        break
    elif selected == "6":
        # run6()
        run6point5()
        break
    elif selected == "7":
        run7()
        break
    elif selected == "8":
        run8()
        break
    elif selected == "9":
        run8point5()
        break
