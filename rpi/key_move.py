"""
This code is a Python script for controlling an Arduino-based robot using keyboard inputs.
The script uses the `curses` library to read key presses in real-time and sends commands
to the Arduino via serial communication. 
The program runs in an infinite loop until the 'q' key is pressed to quit.

Key Functionality:
- 'q': Quit the program.
- UP Arrow or 'w': (Move forward) sending the 'F' command to the Arduino, followed by a 'S' command to stop.
- DOWN Arrow or 's': (Move backward) sending the 'B' command, followed by a 'S' command to stop.
- LEFT Arrow or 'a': (Turn left) sending the 'L' command, followed by a 'S'command to stop.
- RIGHT Arrow or 'd': (Turn right), sending the 'R' command, followed by a 'S' command to stop.
- Space bar or 's': Stop the robot by sending the 'S' command.

Note:
- `time.sleep()` calls are used to control the duration of each movement before stopping.
- In case the Arduino functions for forward, back, left and right contain a delay and stopping functions, 
  time.sleep(..) and arduino.write(b'S') in every condition can be removed.
- Commands are sent to the Arduino via serial communication (`arduino.write`). 
  The port and baudrate must be changed as required.

"""

import serial
import curses
import time
from automation2 import automate_inputs
from constants import *

arduino = serial.Serial('/dev/ttyACM0', 9600)  # Port and baudrate

screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

screen.move(0, 0)
screen.addstr("Starting program... Press 'q' to quit.\n")
screen.addstr("Press arrow keys to move the robot.\n")
screen.addstr("Press ' ' or 's' to brake.\n")
screen.refresh()


def bytes_pose_to_tuple(b: bytes):
    b = b.decode("utf-8").strip().split(" ")
    b[0] = int(b[0])
    b[1] = int(b[1])
    return b

def front_grid():
  arduino.write(b'F')
  time.sleep(1.92)
  arduino.write(b'S')

def back_grid():
  arduino.write(b'B')
  time.sleep(1.89)
  arduino.write(b'S')

def right_grid():
  arduino.write(b'R')
  time.sleep(1.72)
  arduino.write(b'S')

try:
  while True:
    char = screen.getch()
    if char == ord('q'):
      arduino.write(b'S')  # brake
      break

    elif char == curses.KEY_UP or char == ord('w'):
      arduino.write(b'F') 

    elif char == curses.KEY_DOWN or char == ord('s'):
      arduino.write(b'B')

    elif char == curses.KEY_LEFT or char == ord('a'):
      arduino.write(b'L')

    elif char == curses.KEY_RIGHT or char == ord('d'):
      arduino.write(b'R')

    elif char == ord(' ') or char == ord('s'):
      arduino.write(b'S')  # brake

    elif char == ord('j'):
      front_grid()

    elif char == ord('k'):
      back_grid()

    elif char == ord('l'):
      right_grid()

    elif char == ord('h'):
      arduino.write(b'L')
      time.sleep(2)
      arduino.write(b'S')

    elif char == ord('x'):
      screen.addstr('Enter current pose in format: x y theta(U/D/L/R) (matrix indices)')
      curr_pose = bytes_pose_to_tuple(screen.getstr())

      screen.addstr('Enter final pose in format: x y (matrix indices)')
      final_pose = bytes_pose_to_tuple(screen.getstr())

      screen.addstr(f'Starting at (({curr_pose[0]},{curr_pose[1]}),{curr_pose[2]}), \nFinishing at ({final_pose[0]},{final_pose[1]})\n')  
      
      poses, curr_theta = automate_inputs(((curr_pose[0],curr_pose[1]),curr_pose[2]),(final_pose[0],final_pose[1]),(GRID_X,GRID_Y))

      screen.addstr(f'Actions being done {poses}\n')

      for pose in poses:
        if pose == 'f':
          front_grid()
        if pose == 'r':
          right_grid()
        if pose == 'b':
          back_grid()
        time.sleep(1)


except curses.error:
    print('Cursor has crashed due to not being able to print properly')
finally:
  curses.nocbreak()
  screen.keypad(False)
  curses.echo()
  curses.endwin()
  arduino.close()
