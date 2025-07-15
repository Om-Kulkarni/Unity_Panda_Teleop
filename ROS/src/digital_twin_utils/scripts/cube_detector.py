#!/usr/bin/env python3

"""
Cube Detection Script for Digital Twin System

This script uses a webcam to detect a cube and determine its (x,y) position 
and yaw orientation assuming the webcam is mounted in a top-down configuration.

Pure Python implementation without ROS dependencies.

Author: Your Name
Date: July 2025
"""

import cv2
import numpy as np
import argparse
import time
import json
import os


class CubeDetector:
    def __init__(self, camera_index=0, debug=False, output_file=None):
        """
        Initialize the cube detector
        
        Args:
            camera_index (int): Camera device index (usually 0 for default webcam)
            debug (bool): Enable debug visualization
            output_file (str): Optional file path to save detection results
        """
        self.camera_index = camera_index
        self.debug = debug
        self.output_file = output_file
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {camera_index}")
        
        # Set camera resolution (adjust as needed)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Cube detection parameters (adjust these based on your cube color)
        # Default HSV range for detecting a blue cube
        self.lower_hsv = np.array([100, 50, 50])    # Lower HSV threshold for blue
        self.upper_hsv = np.array([130, 255, 255])  # Upper HSV threshold for blue
        
        # Second range (blue doesn't wrap around in HSV)
        self.lower_hsv2 = np.array([100, 50, 50])   # Same as primary range
        self.upper_hsv2 = np.array([130, 255, 255]) # Same as primary range
        
        # Minimum contour area to filter noise (increased to filter small objects)
        self.min_contour_area = 1000
        
        # Create trackbars if in debug mode
        if debug:
            cv2.namedWindow('Color Adjustments')
            cv2.createTrackbar('Hue Min 1', 'Color Adjustments', self.lower_hsv[0], 180, lambda x: None)
            cv2.createTrackbar('Hue Max 1', 'Color Adjustments', self.upper_hsv[0], 180, lambda x: None)
            cv2.createTrackbar('Hue Min 2', 'Color Adjustments', self.lower_hsv2[0], 180, lambda x: None)
            cv2.createTrackbar('Hue Max 2', 'Color Adjustments', self.upper_hsv2[0], 180, lambda x: None)
            cv2.createTrackbar('Sat Min', 'Color Adjustments', self.lower_hsv[1], 255, lambda x: None)
            cv2.createTrackbar('Val Min', 'Color Adjustments', self.lower_hsv[2], 255, lambda x: None)
            cv2.createTrackbar('Area Min', 'Color Adjustments', self.min_contour_area, 5000, lambda x: None)
        
        # Camera calibration parameters (in pixels)
        # You may need to calibrate these based on your setup
        self.pixels_per_meter = 1000  # Approximate conversion factor
        
        # Detection results storage
        self.latest_detection = None
        self.detection_history = []
        self.mask = None  # Store mask as instance variable
        
        print("Cube detector initialized")
    
    def detect_cube(self, frame):
        """
        Detect cube in the given frame
        
        Args:
            frame: OpenCV image frame
            
        Returns:
            tuple: (x, y, yaw) position and orientation, or (None, None, None) if not found
        """
        # Convert BGR to HSV for better color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        if self.debug:
            # Update parameters from trackbars
            h_min1 = cv2.getTrackbarPos('Hue Min 1', 'Color Adjustments')
            h_max1 = cv2.getTrackbarPos('Hue Max 1', 'Color Adjustments')
            h_min2 = cv2.getTrackbarPos('Hue Min 2', 'Color Adjustments')
            h_max2 = cv2.getTrackbarPos('Hue Max 2', 'Color Adjustments')
            s_min = cv2.getTrackbarPos('Sat Min', 'Color Adjustments')
            v_min = cv2.getTrackbarPos('Val Min', 'Color Adjustments')
            self.min_contour_area = cv2.getTrackbarPos('Area Min', 'Color Adjustments')
            
            # Update HSV ranges
            self.lower_hsv = np.array([h_min1, s_min, v_min])
            self.upper_hsv = np.array([h_max1, 255, 255])
            self.lower_hsv2 = np.array([h_min2, s_min, v_min])
            self.upper_hsv2 = np.array([h_max2, 255, 255])
        
        # Create mask for cube color
        mask1 = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        mask2 = cv2.inRange(hsv, self.lower_hsv2, self.upper_hsv2)
        self.mask = cv2.bitwise_or(mask1, mask2)  # Store in instance variable
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, None
        
        # Find the largest contour (assuming it's our cube)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Filter out small contours (noise)
        if cv2.contourArea(largest_contour) < self.min_contour_area:
            return None, None, None
        
        # Get the minimum area rectangle (oriented bounding box)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.array(box, dtype=np.int32)  # Use explicit int32 dtype
        
        # Extract position and orientation
        center_x, center_y = rect[0]
        angle = rect[2]  # Angle in degrees
        
        # Convert angle to yaw (0-360 degrees, with 0 pointing up)
        yaw = -angle if angle < 0 else 90 - angle
        yaw = yaw % 360
        
        # Convert pixel coordinates to world coordinates (meters)
        # Assuming camera is centered and top-down
        frame_height, frame_width = frame.shape[:2]
        world_x = (center_x - frame_width / 2) / self.pixels_per_meter
        world_y = (frame_height / 2 - center_y) / self.pixels_per_meter  # Flip Y axis
        
        if self.debug:
            # Draw detection results
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
            
            # Draw coordinate system
            cv2.line(frame, (int(frame_width/2), 0), (int(frame_width/2), frame_height), (255, 255, 255), 1)
            cv2.line(frame, (0, int(frame_height/2)), (frame_width, int(frame_height/2)), (255, 255, 255), 1)
            
            # Add text with position and orientation
            cv2.putText(frame, f"X: {world_x:.3f}m, Y: {world_y:.3f}m", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Yaw: {yaw:.1f}deg", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show mask in debug mode
            cv2.imshow('Cube Mask', self.mask)
        
        return world_x, world_y, yaw
    
    def save_detection(self, x, y, yaw, timestamp):
        """
        Save detection result to file
        
        Args:
            x (float): X position in meters
            y (float): Y position in meters
            yaw (float): Yaw orientation in degrees
            timestamp (float): Timestamp of detection
        """
        detection_data = {
            'timestamp': timestamp,
            'position': {'x': x, 'y': y, 'z': 0.0},
            'orientation': {'yaw': yaw}
        }
        
        self.latest_detection = detection_data
        self.detection_history.append(detection_data)
        
        if self.output_file:
            try:
                with open(self.output_file, 'w') as f:
                    json.dump({
                        'latest': detection_data,
                        'history': self.detection_history[-100:]  # Keep last 100 detections
                    }, f, indent=2)
            except Exception as e:
                print(f"Error saving to file: {e}")
    
    def run(self):
        """
        Main detection loop
        """
        print("Starting cube detection...")
        print("Press 'q' to quit, 'r' to reset detection history")
        
        frame_count = 0
        last_print_time = time.time()
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame from camera")
                    continue
                
                frame_count += 1
                current_time = time.time()
                
                # Detect cube
                x, y, yaw = self.detect_cube(frame)
                
                if x is not None and y is not None and yaw is not None:
                    # Save detection result
                    self.save_detection(x, y, yaw, current_time)
                    
                    # Print detection info every second
                    if current_time - last_print_time >= 1.0:
                        print(f"Cube detected - Position: ({x:.3f}, {y:.3f}), Yaw: {yaw:.1f}°")
                        last_print_time = current_time
                
                # Show debug windows and handle keyboard input
                if self.debug:
                    cv2.imshow('Cube Detection', frame)
                    cv2.imshow('Cube Mask', self.mask)
                    
                    # Handle keyboard input (only when windows are shown)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("Quitting...")
                        break
                    elif key == ord('r'):
                        self.detection_history.clear()
                        print("Detection history cleared")
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nCube detection stopped by user")
        finally:
            self.cleanup()
    
    def get_latest_detection(self):
        """
        Get the latest detection result
        
        Returns:
            dict: Latest detection data or None if no detection
        """
        return self.latest_detection
    
    def cleanup(self):
        """
        Clean up resources
        """
        self.cap.release()
        if self.debug:
            cv2.destroyWindow('Cube Detection')
            cv2.destroyWindow('Cube Mask')
            cv2.destroyWindow('Color Adjustments')
            cv2.destroyAllWindows()  # Catch any remaining windows
            cv2.waitKey(1)  # Give time for windows to close
        print("Cube detector cleaned up")


def main():
    """
    Main function with argument parsing
    """
    parser = argparse.ArgumentParser(description='Cube Detection for Digital Twin')
    parser.add_argument('--camera', type=int, default=0, 
                       help='Camera index (default: 0)')
    parser.add_argument('--debug', action='store_true', 
                       help='Enable debug visualization')
    parser.add_argument('--color', choices=['red', 'blue', 'green'], default='blue',
                       help='Cube color to detect (default: blue)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file path to save detection results (JSON format)')
    parser.add_argument('--calibration', type=float, default=1000.0,
                       help='Pixels per meter calibration factor (default: 1000)')
    
    args = parser.parse_args()
    
    try:
        detector = CubeDetector(
            camera_index=args.camera, 
            debug=args.debug, 
            output_file=args.output
        )
        
        # Set calibration factor
        detector.pixels_per_meter = args.calibration
        
        # Set color detection parameters based on argument
        if args.color == 'blue':
            detector.lower_hsv = np.array([100, 50, 50])
            detector.upper_hsv = np.array([130, 255, 255])
            detector.lower_hsv2 = np.array([100, 50, 50])  # No wrap-around for blue
            detector.upper_hsv2 = np.array([130, 255, 255])
        elif args.color == 'green':
            detector.lower_hsv = np.array([40, 50, 50])
            detector.upper_hsv = np.array([80, 255, 255])
            detector.lower_hsv2 = np.array([40, 50, 50])   # No wrap-around for green
            detector.upper_hsv2 = np.array([80, 255, 255])
        
        print(f"Starting cube detection with:")
        print(f"  Camera: {args.camera}")
        print(f"  Color: {args.color}")
        print(f"  Debug mode: {args.debug}")
        print(f"  Output file: {args.output}")
        print(f"  Calibration: {args.calibration} pixels/meter")
        
        detector.run()
        
    except Exception as e:
        print(f"Error in cube detection: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    main()
