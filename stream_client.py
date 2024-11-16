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

class VideoStreamClient:
    def __init__(self):
        self.client_socket = None
        self.connected = False
        self.current_frame = None
        self.lock = threading.Lock()
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        self.frame_queue = queue.Queue(maxsize=30)
        self.stop_event = threading.Event()
        self.reconnect_count = 0
        self.max_reconnects = 3
        
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
        """Main frame update loop"""
        while not self.stop_event.is_set() and self.connected:
            try:
                frame = self.receive_frame()
                if frame is not None:
                    # Update frame queue
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()  # Remove oldest frame
                        except queue.Empty:
                            pass
                    self.frame_queue.put_nowait(frame)
                else:
                    time.sleep(0.01)  # Short sleep if no frame
            except Exception as e:
                print(f"Error in update_frame: {str(e)}")
                self.disconnect()
                break

def main():
    st.set_page_config(page_title="Real-time Video Stream", layout="wide")
    st.title("Real-time Video Stream")
    
    # Initialize session state
    if 'client' not in st.session_state:
        st.session_state['client'] = VideoStreamClient()
        st.session_state['stream_thread'] = None
    
    # Connection settings
    with st.sidebar:
        st.header("Connection Settings")
        host = st.text_input("Host", value="localhost")
        port = st.number_input("Port", value=8000, min_value=1, max_value=65535)
        
        if not st.session_state['client'].connected:
            if st.button("Connect"):
                if st.session_state['client'].connect(host, port):
                    st.session_state['stream_thread'] = threading.Thread(
                        target=st.session_state['client'].update_frame
                    )
                    st.session_state['stream_thread'].daemon = True
                    st.session_state['stream_thread'].start()
        else:
            if st.button("Disconnect"):
                st.session_state['client'].disconnect()
                st.session_state['stream_thread'] = None
                st.rerun()
        
        # Status information
        st.header("Stream Status")
        if st.session_state['client'].connected:
            st.success("Connected")
            st.metric("FPS", f"{st.session_state['client'].fps:.1f}")
            st.metric("Buffer Size", f"{st.session_state['client'].frame_queue.qsize()}")
            st.metric("Reconnect Attempts", st.session_state['client'].reconnect_count)
        else:
            st.warning("Not Connected")
    
    # Video display
    if st.session_state['client'].connected:
        video_frame = st.empty()
        
        while st.session_state['client'].connected:
            try:
                frame = st.session_state['client'].frame_queue.get(timeout=0.5)
                if frame is not None:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    pil_image = Image.fromarray(frame_rgb)
                    video_frame.image(pil_image, channels="RGB", use_column_width=True)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error displaying frame: {str(e)}")
                break
            
            time.sleep(0.01)  # Small sleep to prevent UI freezing
    else:
        st.warning("Please connect to start streaming")

if __name__ == "__main__":
    main()