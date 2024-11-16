import time
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum

class GridState(Enum):
    TRANSPARENT = 1  # Seconds to Display Transparent Grid
    REFLECTANCE = 2  # Seconds to Display Reflectance Grid
    STRESSED = 15  # Seconds to Display Stressed Grid
    END = 10

class GridOverlay:
    def __init__(self):
        # Timing configuration (in seconds)
        self.TRANSPARENT_DURATION = 4
        self.REFLECTANCE_DURATION = 8
        self.STRESSED_DURATION = 8
        
        # Display dimensions
        self.DISPLAY_WIDTH = 1920  # Display Width
        self.DISPLAY_HEIGHT = 1080  # Display Height
        
        # Grid paths
        self.grid_paths = {
            'transparent': "./transparent_grid.png",
            'reflectance': "./reflectance_grid.png",
            'stressed': "./reflectance_grid_stressed.png"
        }
        
        # Status messages for each state
        self.status_messages = {
            GridState.TRANSPARENT: "Initializing Scan!",
            GridState.REFLECTANCE: "Capturing Reflectance Values...",
            GridState.STRESSED: "Analyzing Stress Levels..."
        }
        
        # Load grid images
        self.grids = {}
        for name, path in self.grid_paths.items():
            if not os.path.exists(path):
                raise FileNotFoundError(f"File does not exist: {path}")
            grid = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if grid is None:
                raise FileNotFoundError(f"Could not load grid image: {path}")
            self.grids[name] = grid
        
        # Check for alpha channel
        self.has_alpha = all(grid.shape[2] == 4 for grid in self.grids.values())
        print("Alpha Channel Present!" if self.has_alpha else "Alpha Channel Absent!")
        
        # Initialize state
        self.state = GridState.TRANSPARENT
        self.start_time = time.time()
        self.transparency_factor = 0.8

    def resize_frame(self, frame):
        """Resize the input frame to desired display dimensions"""
        return cv2.resize(frame, (self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT))

    def overlay_grid(self, frame, grid):
        """Overlay grid on frame with proper alpha blending"""
        # Create a copy of the frame to avoid modifying the original
        result = frame.copy()
        
        # Calculate grid size and position to maintain aspect ratio
        grid_height = int(self.DISPLAY_HEIGHT * 0.8)  # Grid takes 80% of display height
        grid_width = grid_height  # Maintain square aspect ratio
        
        # Calculate center position for grid
        x_offset = (self.DISPLAY_WIDTH - grid_width) // 2
        y_offset = (self.DISPLAY_HEIGHT - grid_height) // 2
        
        # Resize overlay image to fit on frame
        overlay_resized = cv2.resize(grid, (grid_width, grid_height))
        y1, y2 = y_offset, y_offset + overlay_resized.shape[0]
        x1, x2 = x_offset, x_offset + overlay_resized.shape[1]
        
        if self.has_alpha:
            overlay_rgb = overlay_resized[:, :, :3]
            overlay_alpha = (overlay_resized[:, :, 3] / 255.0) * self.transparency_factor
            
            # Vectorized alpha blending
            alpha_3d = np.stack([overlay_alpha] * 3, axis=2)
            result[y1:y2, x1:x2] = (
                overlay_rgb * alpha_3d + 
                result[y1:y2, x1:x2] * (1 - alpha_3d)
            ).astype(np.uint8)
        else:
            # Simple overlay without transparency
            result[y1:y2, x1:x2] = cv2.addWeighted(
                overlay_resized, self.transparency_factor,
                result[y1:y2, x1:x2], 1 - self.transparency_factor, 0
            )
        
        return result

    def update_state(self):
        """Update the current state based on elapsed time"""
        elapsed = time.time() - self.start_time
        
        if self.state == GridState.TRANSPARENT and elapsed >= self.TRANSPARENT_DURATION:
            self.state = GridState.REFLECTANCE
            self.start_time = time.time()
            print("Switching to Reflectance Grid")
            
        elif self.state == GridState.REFLECTANCE and elapsed >= self.REFLECTANCE_DURATION:
            self.state = GridState.STRESSED
            self.start_time = time.time()
            print("Switching to Stressed Grid")
            
        elif self.state == GridState.STRESSED and elapsed >= self.STRESSED_DURATION:
            self.state = GridState.END
            print("Sequence Complete")

    def get_current_grid(self):
        """Get the current grid based on state"""
        if self.state == GridState.TRANSPARENT:
            return self.grids['transparent']
        elif self.state == GridState.REFLECTANCE:
            return self.grids['reflectance']
        elif self.state == GridState.STRESSED:
            return self.grids['stressed']
        return None

    def add_status_text(self, frame):
        """Add status message to frame"""
        if self.state in self.status_messages:
            message = self.status_messages[self.state]
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.2
            thickness = 2
            text_size = cv2.getTextSize(message, font, font_scale, thickness)[0]
            text_x = (frame.shape[1] - text_size[0]) // 2
            text_y = 70

            # Add black outline for better visibility
            outline_thickness = thickness + 2
            cv2.putText(frame, message, (text_x, text_y), font, font_scale, (0, 0, 0), outline_thickness)
            # Add white text
            cv2.putText(frame, message, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)

    def run(self):
        """Main execution loop"""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Could not open video capture device")

        # Set capture resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.DISPLAY_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.DISPLAY_HEIGHT)

        # Create named window and set it to fullscreen
        cv2.namedWindow("Video Feed with Overlay", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("Video Feed with Overlay", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame")
                break

            frame = self.resize_frame(frame)
            self.update_state()
            
            if self.state == GridState.END:
                break

            current_grid = self.get_current_grid()
            if current_grid is not None:
                frame = self.overlay_grid(frame, current_grid)

            self.add_status_text(frame)
            cv2.imshow("Video Feed with Overlay", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
