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

arduino = serial.Serial('/dev/ttyUSB0', 28800)  # Port and baudrate

screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

try:
  while True:
    char = screen.getch()
    if char == ord('q'):
      break
    elif char == curses.KEY_UP or char == ord('w'):
      arduino.write(b'F') 
      time.sleep(3.589)
      arduino.write(b'S')
    elif char == curses.KEY_DOWN or char == ord('s'):
      arduino.write(b'B')
      time.sleep(3.589)
      arduino.write(b'S')
    elif char == curses.KEY_LEFT or char == ord('a'):
      arduino.write(b'L')
      time.sleep(0.60)
      arduino.write(b'S')
    elif char == curses.KEY_RIGHT or char == ord('d'):
      arduino.write(b'R')
      time.sleep(0.68)
      arduino.write(b'S')
    elif char == ord(' ') or char == ord('s'):
      arduino.write(b'S')  # brake
 
      
finally:
  curses.nocbreak()
  screen.keypad(False)
  curses.echo()
  curses.endwin()
  arduino.close()
