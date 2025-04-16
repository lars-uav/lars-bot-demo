import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tanayrs/groundbot_ws/src/lars-bot-demo/install/arduino_serial_publisher'
