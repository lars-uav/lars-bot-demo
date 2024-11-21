import streamlit as st
import cv2
import socket
import json
import time
import threading
import queue
import numpy as np
import requests
import PIL.Image as Image
import os
import struct
import pickle
from enum import Enum

class GridState(Enum):
    TRANSPARENT = 1
    REFLECTANCE = 2
    STRESSED = 3
    END = 4

class GridOverlayManager:
    def __init__(self):
        # Grid paths - store as relative paths or update with your paths
        self.grid_paths = {
            'transparent': "camera-stuff/transparent_grid.png",
            'reflectance': "camera-stuff/reflectance_grid.png",
            'stressed': "camera-stuff/reflectance_grid_stressed.png"
        }
        
        # Status messages
        self.status_messages = {
            GridState.TRANSPARENT: "Transparent Grid Active",
            GridState.REFLECTANCE: "Reflectance Grid Active",
            GridState.STRESSED: "Stress Analysis Grid Active"
        }
        
        # Initialize state
        self.state = GridState.TRANSPARENT
        self.transparency_factor = 0.8
        
        # Initialize grids
        self.grids = {}
        self.has_alpha = False
        self._load_grids()
    
    def _load_grids(self):
        """Load and verify grid images"""
        missing_files = []
        for name, path in self.grid_paths.items():
            if not os.path.exists(path):
                missing_files.append(path)
                continue
                
            grid = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if grid is None:
                missing_files.append(path)
                continue
                
            self.grids[name] = grid
        
        if missing_files:
            st.warning(f"Missing grid files: {', '.join(missing_files)}")
            return False
            
        self.has_alpha = all(grid.shape[2] == 4 for grid in self.grids.values())
        return True

    def overlay_grid(self, frame):
        """Apply grid overlay to frame"""
        if not self.grids:
            return frame

        result = frame.copy()
        height, width = frame.shape[:2]
        
        # Calculate grid dimensions - Now full frame size
        grid_size = min(height, width)
        x_offset = (width - grid_size) // 2
        y_offset = (height - grid_size) // 2
        
        # Get current grid based on state
        current_grid = None
        if self.state == GridState.TRANSPARENT:
            current_grid = self.grids.get('transparent')
        elif self.state == GridState.REFLECTANCE:
            current_grid = self.grids.get('reflectance')
        elif self.state == GridState.STRESSED:
            current_grid = self.grids.get('stressed')
            
        if current_grid is None:
            return result
            
        try:
            # Resize grid to frame size
            overlay_resized = cv2.resize(current_grid, (grid_size, grid_size))
            y1, y2 = y_offset, y_offset + grid_size
            x1, x2 = x_offset, x_offset + grid_size
            
            # Apply overlay with alpha channel if available
            if self.has_alpha and overlay_resized.shape[2] == 4:
                overlay_rgb = overlay_resized[:, :, :3]
                overlay_alpha = (overlay_resized[:, :, 3] / 255.0) * self.transparency_factor
                alpha_3d = np.stack([overlay_alpha] * 3, axis=2)
                result[y1:y2, x1:x2] = (
                    overlay_rgb * alpha_3d + 
                    result[y1:y2, x1:x2] * (1 - alpha_3d)
                ).astype(np.uint8)
            else:
                # Fallback to simple overlay if no alpha channel
                result[y1:y2, x1:x2] = cv2.addWeighted(
                    overlay_resized, self.transparency_factor,
                    result[y1:y2, x1:x2], 1 - self.transparency_factor, 
                    0
                )
            
            # Add status message at the bottom of the frame
            if self.state in self.status_messages:
                message = self.status_messages[self.state]
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.0
                thickness = 2
                text_size = cv2.getTextSize(message, font, font_scale, thickness)[0]
                text_x = (width - text_size[0]) // 2
                text_y = height - 30  # Position at bottom
                
                # Add text shadow/outline for better visibility
                cv2.putText(result, message, (text_x, text_y), font, font_scale, (0, 0, 0), thickness + 2)
                cv2.putText(result, message, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)
                
            return result
            
        except Exception as e:
            st.error(f"Error applying overlay: {str(e)}")
            return frame

    def set_state(self, new_state):
        """Set the current grid state"""
        if new_state in GridState:
            self.state = new_state

class RobotClient:
    def __init__(self, host, port=8001):
        """Initialize robot client with separate host and port"""
        self.base_url = f"http://{host}:{port}"
        self.session = requests.Session()
        self.connected = False
        
    def connect(self):
        """Establish connection to robot server"""
        try:
            response = self.session.get(f"{self.base_url}/", timeout=5)
            data = response.json()
            self.connected = data.get('connected', False)
            if self.connected:
                st.success("Connected to robot")
            else:
                st.error("Robot controller not responding")
            return self.connected
        except Exception as e:
            st.error(f"Robot connection failed: {str(e)}")
            self.connected = False
            return False
            
    def disconnect(self):
        """Disconnect from robot server"""
        self.connected = False
        self.session.close()
        
    def send_command(self, command):
        """Send single-letter command to robot"""
        if not self.connected:
            st.error("Not connected to robot")
            return False
            
        try:
            response = self.session.post(
                f"{self.base_url}/command",
                json={'command': command},
                timeout=5
            )
            response.raise_for_status()
            result = response.json()
            return result['status'] == 'success'
        except Exception as e:
            st.error(f"Command failed: {str(e)}")
            return False
class VideoStreamClient:
    def __init__(self):
        self.client_socket = None
        self.connected = False
        self.lock = threading.Lock()
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        self.frame_queue = queue.Queue(maxsize=30)
        self.stop_event = threading.Event()
        self.reconnect_count = 0
        self.max_reconnects = 3
        
        # Initialize grid manager
        try:
            self.grid_manager = GridOverlayManager()
        except Exception as e:
            st.error(f"Error initializing grid overlay: {str(e)}")
            self.grid_manager = None

        
    def connect(self, host, port):
        try:
            if self.client_socket:
                self.client_socket.close()
                
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(5)  # Connection timeout
            st.info(f"Attempting to connect to {host}:{port}...")
            self.client_socket.connect((host, port))
            
            # Configure socket
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            self.client_socket.settimeout(1.0)  # Operation timeout
            
            self.connected = True
            self.frame_count = 0
            self.start_time = time.time()
            self.stop_event.clear()
            self.reconnect_count = 0
            st.success("Connected successfully")
            return True
        except Exception as e:
            st.error(f"Connection failed: {str(e)}")
            return False
            
    def disconnect(self):
        self.stop_event.set()
        self.connected = False
        if self.client_socket:
            try:
                self.client_socket.shutdown(socket.SHUT_RDWR)
            except:
                pass
            self.client_socket.close()
        with self.lock:
            self.current_frame = None
        self.clear_queue()
        
    def clear_queue(self):
        """Clear the frame queue"""
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
                
    def receive_frame(self):
        """Receive a single frame with improved error handling"""
        try:
            # Get message size
            data = self._receive_exactly(struct.calcsize("L"))
            if not data:
                return None
                
            msg_size = struct.unpack("L", data)[0]
            
            # Get frame data
            frame_data = self._receive_exactly(msg_size)
            if not frame_data:
                return None
                
            # Decode frame
            encoded_frame = pickle.loads(frame_data)
            frame = cv2.imdecode(np.frombuffer(encoded_frame, np.uint8), cv2.IMREAD_COLOR)
            
            if frame is None:
                raise ValueError("Failed to decode frame")
                
            # Update FPS
            self._update_fps()
            
            return frame
            
        except (socket.timeout, TimeoutError) as e:
            print(f"Timeout receiving frame: {str(e)}")
            self._handle_timeout()
            return None
        except Exception as e:
            print(f"Stream error: {str(e)}")
            self.disconnect()
            return None
            
    def _receive_exactly(self, n):
        """Receive exactly n bytes with timeout handling"""
        data = bytearray()
        start_time = time.time()
        
        while len(data) < n and time.time() - start_time < 2.0:  # 2 second total timeout
            try:
                packet = self.client_socket.recv(min(n - len(data), 4096))
                if not packet:
                    return None
                data.extend(packet)
            except socket.timeout:
                continue
                
        if len(data) < n:
            raise TimeoutError("Incomplete frame received")
            
        return bytes(data)
        
    def _update_fps(self):
        """Update FPS calculation"""
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 1:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = time.time()
        self.last_frame_time = time.time()
        
    def _handle_timeout(self):
        """Handle timeout events with potential reconnection"""
        if self.reconnect_count < self.max_reconnects:
            print(f"Attempting reconnect {self.reconnect_count + 1}/{self.max_reconnects}")
            self.reconnect_count += 1
            self.disconnect()
            time.sleep(1)  # Wait before reconnecting
        else:
            print("Max reconnection attempts reached")
            self.disconnect()
            
    def update_frame(self):
        """Main frame update loop - now without overlay processing"""
        while not self.stop_event.is_set() and self.connected:
            try:
                frame = self.receive_frame()
                if frame is not None:
                    # Just queue the original frame
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.frame_queue.put_nowait(frame)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Error in update_frame: {str(e)}")
                self.disconnect()
                break

def main():
    st.set_page_config(page_title="Robot Control Center", layout="wide")
    st.title("Robot Control Center")
    
    # Initialize session state
    if 'video_client' not in st.session_state:
        st.session_state['video_client'] = VideoStreamClient()
    if 'stream_thread' not in st.session_state:
        st.session_state['stream_thread'] = None
    if 'robot_client' not in st.session_state:
        st.session_state['robot_client'] = None
    if 'overlay_enabled' not in st.session_state:
        st.session_state['overlay_enabled'] = False
    if 'current_state' not in st.session_state:
        st.session_state['current_state'] = GridState.TRANSPARENT
    
    # Layout: Split into two columns - video stream and controls
    col1, col2 = st.columns([2, 1])
    
    # Control Panel (right column)
    with col2:
        st.header("Control Panel")
        
        # Video Connection
        st.subheader("Video Stream")
        video_host = st.text_input("Video Host", value="localhost")
        video_port = st.number_input("Video Port", value=8000)
        
        if not st.session_state['video_client'].connected:
            if st.button("Connect Video"):
                if st.session_state['video_client'].connect(video_host, video_port):
                    st.session_state['stream_thread'] = threading.Thread(
                        target=st.session_state['video_client'].update_frame
                    )
                    st.session_state['stream_thread'].daemon = True
                    st.session_state['stream_thread'].start()
                    st.rerun()
        else:
            if st.button("Disconnect Video"):
                st.session_state['video_client'].disconnect()
                st.session_state['stream_thread'] = None
                st.rerun()
                
        # Robot Connection
        st.subheader("Robot Control")
        robot_host = st.text_input("Robot Host", value="10.1.59.194")
        robot_port = st.number_input("Robot Port", value=8001)
        
        if not st.session_state.get('robot_client'):
            if st.button("Connect Robot"):
                st.session_state['robot_client'] = RobotClient(robot_host, robot_port)
                if st.session_state['robot_client'].connect():
                    st.rerun()
        else:
            if st.button("Disconnect Robot"):
                if st.session_state['robot_client']:
                    st.session_state['robot_client'].disconnect()
                st.session_state['robot_client'] = None
                st.rerun()
        
        # Robot Controls
        if st.session_state.get('robot_client') and st.session_state['robot_client'].connected:
            st.subheader("Robot Controls")
            
            # Direction pad layout
            c1, c2, c3 = st.columns(3)
            with c2:
                if st.button("↑"):
                    st.session_state['robot_client'].send_command('F')
                    
            c1, c2, c3 = st.columns(3)
            with c1:
                if st.button("←"):
                    st.session_state['robot_client'].send_command('L')
            with c2:
                if st.button("■"):
                    st.session_state['robot_client'].send_command('S')
            with c3:
                if st.button("→"):
                    st.session_state['robot_client'].send_command('R')
                    
            c1, c2, c3 = st.columns(3)
            with c2:
                if st.button("↓"):
                    st.session_state['robot_client'].send_command('B')
        
        # Grid Overlay Controls
        if st.session_state['video_client'].grid_manager:
            st.subheader("Grid Overlay")
            overlay_enabled = st.checkbox(
                "Enable Grid Overlay",
                value=st.session_state['overlay_enabled']
            )
            
            if overlay_enabled:
                st.write("Grid Type:")
                col1, col2, col3 = st.columns(3)
                with col1:
                    if st.button("Transparent"):
                        st.session_state['current_state'] = GridState.TRANSPARENT
                        st.session_state['video_client'].grid_manager.set_state(GridState.TRANSPARENT)
                with col2:
                    if st.button("Reflectance"):
                        st.session_state['current_state'] = GridState.REFLECTANCE
                        st.session_state['video_client'].grid_manager.set_state(GridState.REFLECTANCE)
                with col3:
                    if st.button("Stressed"):
                        st.session_state['current_state'] = GridState.STRESSED
                        st.session_state['video_client'].grid_manager.set_state(GridState.STRESSED)
                
                transparency = st.slider(
                    "Grid Transparency",
                    0.0, 1.0, 0.8, 0.1
                )
                st.session_state['video_client'].grid_manager.transparency_factor = transparency
            
            if overlay_enabled != st.session_state['overlay_enabled']:
                st.session_state['overlay_enabled'] = overlay_enabled
                st.rerun()
    
    # Video Display (left column)
    with col1:
        if st.session_state['video_client'].connected:
            video_container = st.empty()
            
            while st.session_state['video_client'].connected:
                try:
                    frame = st.session_state['video_client'].frame_queue.get(timeout=0.5)
                    if frame is not None:
                        # Apply grid overlay if enabled
                        if st.session_state['overlay_enabled'] and st.session_state['video_client'].grid_manager:
                            frame = st.session_state['video_client'].grid_manager.overlay_grid(frame)
                        
                        # Convert and display frame
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        video_container.image(frame_rgb, channels="RGB", use_container_width=True)
                except queue.Empty:
                    continue
                except Exception as e:
                    st.error(f"Error displaying frame: {str(e)}")
                    break
                
                time.sleep(0.01)
        else:
            st.info("Connect to video stream to begin")

if __name__ == "__main__":
    main()