# lars-bot-demo

### Setup Instructions

run the dashboard using the command
` streamlit run stream_client.py`. The file is located in the camera-stuff folder.

On the Drone's Raspberry Pi, run the command `python3 stream_server.py` to start the video streaming server

On the Ground Bot's Raspberry Pi, run the command `python3 robot_server.py` to start the ground bot communication.

Make sure you're on Guest Wifi.

On the streamlit dashboard, enter the IP address of the drone and the ground bot. The port for the drone is 8000 and the port for the ground bot is 8001.

If using the big bot, make sure device is set to `/dev/ttyACM0` and on the small bot make sure it is set to `/dev/ttyUSB0`. This change will need to be made in the streamlit dashboard code as well as the robot server.

The rest of the instructions will be on the dashboard UI
