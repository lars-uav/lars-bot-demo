import streamlit as st
import cv2
import socket
import pickle
import struct
import numpy as np
import threading
from PIL import Image
import time
import queue
from enum import Enum
import os

class GridState(Enum):
    TRANSPARENT = 1
    REFLECTANCE = 2
    STRESSED = 3
    END = 4

class GridOverlayManager:
    def __init__(self):
        # Remove timing configuration since we're doing manual control
        # Grid paths - using current directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.grid_paths = {
            'transparent': "/Users/tanmay/Documents/GitHub/lars-bot-demo/camera-stuff/transparent_grid.png",
            'reflectance': "/Users/tanmay/Documents/GitHub/lars-bot-demo/camera-stuff/reflectance_grid.png",
            'stressed': "/Users/tanmay/Documents/GitHub/lars-bot-demo/camera-stuff/reflectance_grid_stressed.png"
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

    def get_current_duration(self):
        """Get the duration for the current state"""
        if self.state == GridState.TRANSPARENT:
            return self.TRANSPARENT_DURATION
        elif self.state == GridState.REFLECTANCE:
            return self.REFLECTANCE_DURATION
        elif self.state == GridState.STRESSED:
            return self.STRESSED_DURATION
        return 0  # Default for END state or unknown states
    
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
        
        # Calculate grid size (80% of smaller dimension)
        grid_size = int(min(height, width) * 0.8)
        
        # Calculate center position
        x_offset = (width - grid_size) // 2
        y_offset = (height - grid_size) // 2
        
        current_grid = self.get_current_grid()
        if current_grid is None:
            return result
            
        st.info(f"Applying grid: {self.state.name}")
        
        # Resize overlay
        overlay_resized = cv2.resize(current_grid, (grid_size, grid_size))
        y1, y2 = y_offset, y_offset + grid_size
        x1, x2 = x_offset, x_offset + grid_size
        
        try:
            if self.has_alpha:
                overlay_rgb = overlay_resized[:, :, :3]
                overlay_alpha = (overlay_resized[:, :, 3] / 255.0) * self.transparency_factor
                alpha_3d = np.stack([overlay_alpha] * 3, axis=2)
                result[y1:y2, x1:x2] = (
                    overlay_rgb * alpha_3d + 
                    result[y1:y2, x1:x2] * (1 - alpha_3d)
                ).astype(np.uint8)
            else:
                result[y1:y2, x1:x2] = cv2.addWeighted(
                    overlay_resized, self.transparency_factor,
                    result[y1:y2, x1:x2], 1 - self.transparency_factor, 0
                )
            
            # Add status message
            if self.state in self.status_messages:
                message = self.status_messages[self.state]
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.2
                thickness = 2
                text_size = cv2.getTextSize(message, font, font_scale, thickness)[0]
                text_x = (width - text_size[0]) // 2
                text_y = 70
                
                # Add text with outline
                cv2.putText(result, message, (text_x, text_y), font, font_scale, (0, 0, 0), thickness + 2)
                cv2.putText(result, message, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)
        except Exception as e:
            st.error(f"Error applying overlay: {str(e)}")
            return frame
            
        return result

    def update_state(self):
        """Update grid state based on elapsed time"""
        elapsed = time.time() - self.start_time
        
        if self.state == GridState.TRANSPARENT and elapsed >= self.TRANSPARENT_DURATION:
            self.state = GridState.REFLECTANCE
            self.start_time = time.time()
        elif self.state == GridState.REFLECTANCE and elapsed >= self.REFLECTANCE_DURATION:
            self.state = GridState.STRESSED
            self.start_time = time.time()
        elif self.state == GridState.STRESSED and elapsed >= self.STRESSED_DURATION:
            self.state = GridState.END

    def set_state(self, new_state):
        """Manually set the grid state"""
        if new_state in GridState:
            self.state = new_state
            st.info(f"Grid state set to: {self.state.name}")

    def get_current_grid(self):
        """Get current grid based on state"""
        if self.state == GridState.TRANSPARENT:
            return self.grids.get('transparent')
        elif self.state == GridState.REFLECTANCE:
            return self.grids.get('reflectance')
        elif self.state == GridState.STRESSED:
            return self.grids.get('stressed')
        return None

    def reset(self):
        """Reset grid state"""
        self.state = GridState.TRANSPARENT
        self.start_time = time.time()

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
    st.set_page_config(page_title="Video Stream with Grid Overlay", layout="wide")
    st.title("Video Stream with Grid Overlay")
    
    # Initialize session state
    if 'client' not in st.session_state:
        st.session_state['client'] = VideoStreamClient()
        st.session_state['stream_thread'] = None
        st.session_state['overlay_enabled'] = False
        st.session_state['current_state'] = GridState.TRANSPARENT
    
    # Sidebar controls
    with st.sidebar:
        st.header("Connection Settings")
        host = st.text_input("Host", value="localhost")
        port = st.number_input("Port", value=8000, min_value=1, max_value=65535)
        
        # Connection controls
        col1, col2 = st.columns(2)
        with col1:
            if not st.session_state['client'].connected:
                if st.button("Connect", use_container_width=True):
                    if st.session_state['client'].connect(host, port):
                        st.session_state['stream_thread'] = threading.Thread(
                            target=st.session_state['client'].update_frame
                        )
                        st.session_state['stream_thread'].daemon = True
                        st.session_state['stream_thread'].start()
                        st.rerun()
            else:
                if st.button("Disconnect", use_container_width=True):
                    st.session_state['client'].disconnect()
                    st.session_state['stream_thread'] = None
                    st.rerun()
        
        # Grid overlay controls
        st.header("Grid Overlay Controls")
        if st.session_state['client'].grid_manager and st.session_state['client'].grid_manager.grids:
            overlay_enabled = st.checkbox(
                "Enable Grid Overlay",
                value=st.session_state.get('overlay_enabled', False),
                key='overlay_checkbox'
            )
            
            if overlay_enabled:
                st.write("Select Grid Type:")
                col1, col2, col3 = st.columns(3)
                
                with col1:
                    if st.button("Transparent (1)", key="btn_transparent"):
                        st.session_state['current_state'] = GridState.TRANSPARENT
                        st.session_state['client'].grid_manager.set_state(GridState.TRANSPARENT)
                        st.info("Set to Transparent Grid")
                
                with col2:
                    if st.button("Reflectance (2)", key="btn_reflectance"):
                        st.session_state['current_state'] = GridState.REFLECTANCE
                        st.session_state['client'].grid_manager.set_state(GridState.REFLECTANCE)
                        st.info("Set to Reflectance Grid")
                
                with col3:
                    if st.button("Stressed (3)", key="btn_stressed"):
                        st.session_state['current_state'] = GridState.STRESSED
                        st.session_state['client'].grid_manager.set_state(GridState.STRESSED)
                        st.info("Set to Stressed Grid")
                
                # Display current state
                st.info(f"Current Grid: {st.session_state['current_state'].name}")
                
                # Transparency control
                transparency = st.slider("Grid Transparency", 0.0, 1.0, 0.8, 0.1)
                st.session_state['client'].grid_manager.transparency_factor = transparency
                
            if overlay_enabled != st.session_state.get('overlay_enabled'):
                st.session_state['overlay_enabled'] = overlay_enabled
                if not overlay_enabled:
                    st.session_state['current_state'] = GridState.TRANSPARENT
                st.rerun()
        else:
            st.warning("Grid overlay not available - missing grid files")
            if st.button("Check Grid Files"):
                grid_files = {
                    "Transparent Grid": "transparent_grid.png",
                    "Reflectance Grid": "reflectance_grid.png",
                    "Stressed Grid": "reflectance_grid_stressed.png"
                }
                missing_files = []
                for name, file in grid_files.items():
                    if not os.path.exists(file):
                        missing_files.append(name)
                if missing_files:
                    st.error(f"Missing files: {', '.join(missing_files)}")
                else:
                    st.success("All grid files found!")
        
        # Status information
        st.header("Stream Status")
        if st.session_state['client'].connected:
            st.success("Connected")
            
            col1, col2, col3 = st.columns(3)
            with col1:
                st.metric("FPS", f"{st.session_state['client'].fps:.1f}")
            with col2:
                st.metric("Buffer", st.session_state['client'].frame_queue.qsize())
            with col3:
                st.metric("Reconnects", st.session_state['client'].reconnect_count)
        else:
            st.warning("Not Connected")
    
    # Main video display
    if st.session_state['client'].connected:
        video_container = st.empty()
        status_container = st.empty()
        
        # Help text
        st.markdown("""
        ### Keyboard Shortcuts
        - Press the corresponding number or use the buttons to change grids:
            1. Transparent Grid
            2. Reflectance Grid
            3. Stressed Grid
        - Use the slider to adjust grid transparency
        """)
        
        while st.session_state['client'].connected:
            try:
                frame = st.session_state['client'].frame_queue.get(timeout=0.5)
                if frame is not None:
                    # Apply overlay in main thread if enabled
                    if st.session_state.get('overlay_enabled', False) and st.session_state['client'].grid_manager:
                        frame = st.session_state['client'].grid_manager.overlay_grid(frame)
                    
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    if st.session_state.get('overlay_enabled', False) and st.session_state['client'].grid_manager:
                        current_state = st.session_state['current_state']
                        if current_state in st.session_state['client'].grid_manager.status_messages:
                            status_container.info(st.session_state['client'].grid_manager.status_messages[current_state])
                    else:
                        status_container.empty()
                    
                    pil_image = Image.fromarray(frame_rgb)
                    video_container.image(pil_image, channels="RGB", use_container_width=True)
            except queue.Empty:
                continue
            except Exception as e:
                st.error(f"Error displaying frame: {str(e)}")
                break
            
            time.sleep(0.01)
            
        video_container.empty()
        status_container.empty()
    else:
        st.info("Connect to the video stream using the sidebar controls")
        
        if st.session_state['client'].grid_manager is None:
            st.warning("Grid overlay system not initialized. Please check grid files.")
            st.write("Required grid files:")
            for name, path in {
                "Transparent Grid": "transparent_grid.png",
                "Reflectance Grid": "reflectance_grid.png",
                "Stressed Grid": "reflectance_grid_stressed.png"
            }.items():
                if os.path.exists(path):
                    st.success(f"✓ {name} found")
                else:
                    st.error(f"✗ {name} missing")

if __name__ == "__main__":
    main()