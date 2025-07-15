#!/usr/bin/env python3

"""
Shape-based Cube Detection Script for Digital Twin System

This script uses a webcam to detect a cube based on its shape characteristics
and determine its (x,y) position and yaw orientation assuming the webcam is 
mounted in a top-down configuration.

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


class ShapeDetector:
    def __init__(self, camera_index=0, debug=False, output_file=None):
        """
        Initialize the shape-based cube detector
        
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
        
        # Detection parameters
        self.min_contour_area = 1000  # Minimum area to filter small objects
        self.approx_poly_epsilon = 0.02  # Parameter for polygon approximation
        self.aspect_ratio_tolerance = 0.3  # Maximum allowed deviation from square aspect ratio
        self.min_corners = 4  # Minimum number of corners for cube detection
        
        # Create trackbars if in debug mode
        if debug:
            cv2.namedWindow('Shape Adjustments')
            cv2.createTrackbar('Threshold', 'Shape Adjustments', 128, 255, lambda x: None)
            cv2.createTrackbar('Min Area', 'Shape Adjustments', self.min_contour_area, 5000, lambda x: None)
            cv2.createTrackbar('Epsilon*100', 'Shape Adjustments', int(self.approx_poly_epsilon*100), 10, lambda x: None)
            
        # Camera calibration parameters (in pixels)
        self.pixels_per_meter = 1000  # Approximate conversion factor
        
        # Detection results storage
        self.latest_detection = None
        self.detection_history = []
        self.edges = None  # Store edge detection result
        self.debug_image = None  # Store debug visualization
        
        print("Shape detector initialized")
    
    def preprocess_frame(self, frame):
        """
        Preprocess frame for shape detection
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Get threshold value from trackbar in debug mode
        threshold = cv2.getTrackbarPos('Threshold', 'Shape Adjustments') if self.debug else 128
        
        # Apply adaptive thresholding
        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Edge detection
        self.edges = cv2.Canny(binary, threshold/2, threshold)
        
        # Dilate edges to connect gaps
        kernel = np.ones((3,3), np.uint8)
        dilated = cv2.dilate(self.edges, kernel, iterations=1)
        
        return dilated
    
    def detect_cube(self, frame):
        """
        Detect cube in the given frame using shape-based approach
        
        Args:
            frame: OpenCV image frame
            
        Returns:
            tuple: (x, y, yaw) position and orientation, or (None, None, None) if not found
        """
        # Create debug image copy
        self.debug_image = frame.copy()
        
        # Preprocess frame
        processed = self.preprocess_frame(frame)
        
        # Find contours
        contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, None
            
        # Update parameters from trackbars in debug mode
        if self.debug:
            self.min_contour_area = cv2.getTrackbarPos('Min Area', 'Shape Adjustments')
            self.approx_poly_epsilon = cv2.getTrackbarPos('Epsilon*100', 'Shape Adjustments') / 100.0
        
        # Analyze each contour
        best_candidate = None
        best_score = 0
        
        for contour in contours:
            # Filter by area
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue
                
            # Approximate the contour to a polygon
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.approx_poly_epsilon * perimeter, True)
            
            # Check number of corners (should be close to 4 for a square)
            if len(approx) < self.min_corners:
                continue
            
            # Get the minimum area rectangle
            rect = cv2.minAreaRect(contour)
            width, height = rect[1]
            
            # Check aspect ratio (should be close to 1 for a square)
            if width == 0 or height == 0:
                continue
                
            aspect_ratio = min(width, height) / max(width, height)
            if abs(1 - aspect_ratio) > self.aspect_ratio_tolerance:
                continue
            
            # Calculate a score based on how well it matches our criteria
            score = (1 - abs(1 - aspect_ratio)) * area
            
            if score > best_score:
                best_score = score
                best_candidate = (contour, rect, approx)
        
        if best_candidate is None:
            return None, None, None
            
        # Extract information from best candidate
        contour, rect, approx = best_candidate
        center_x, center_y = rect[0]
        angle = rect[2]  # Angle in degrees
        
        # Convert angle to yaw (0-360 degrees, with 0 pointing up)
        yaw = -angle if angle < 0 else 90 - angle
        yaw = yaw % 360
        
        # Convert pixel coordinates to world coordinates (meters)
        frame_height, frame_width = frame.shape[:2]
        world_x = (center_x - frame_width / 2) / self.pixels_per_meter
        world_y = (frame_height / 2 - center_y) / self.pixels_per_meter  # Flip Y axis
        
        if self.debug:
            # Draw the contour and its approximation
            cv2.drawContours(self.debug_image, [contour], 0, (0, 255, 0), 2)
            cv2.drawContours(self.debug_image, [approx], 0, (0, 0, 255), 2)
            
            # Draw center point
            cv2.circle(self.debug_image, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)
            
            # Draw coordinate system
            cv2.line(self.debug_image, (int(frame_width/2), 0), 
                    (int(frame_width/2), frame_height), (255, 255, 255), 1)
            cv2.line(self.debug_image, (0, int(frame_height/2)), 
                    (frame_width, int(frame_height/2)), (255, 255, 255), 1)
            
            # Add text with position and orientation
            cv2.putText(self.debug_image, f"X: {world_x:.3f}m, Y: {world_y:.3f}m", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(self.debug_image, f"Yaw: {yaw:.1f}deg", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show debug images
            cv2.imshow('Edge Detection', self.edges)
            cv2.imshow('Shape Detection', self.debug_image)
        
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
        print("Starting shape-based cube detection...")
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
                
                # Handle keyboard input (only when windows are shown)
                if self.debug:
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
            cv2.destroyWindow('Shape Detection')
            cv2.destroyWindow('Edge Detection')
            cv2.destroyWindow('Shape Adjustments')
            cv2.destroyAllWindows()  # Catch any remaining windows
            cv2.waitKey(1)  # Give time for windows to close
        print("Shape detector cleaned up")


def main():
    """
    Main function with argument parsing
    """
    parser = argparse.ArgumentParser(description='Shape-based Cube Detection for Digital Twin')
    parser.add_argument('--camera', type=int, default=0, 
                       help='Camera index (default: 0)')
    parser.add_argument('--debug', action='store_true', 
                       help='Enable debug visualization')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file path to save detection results (JSON format)')
    parser.add_argument('--calibration', type=float, default=1000.0,
                       help='Pixels per meter calibration factor (default: 1000)')
    
    args = parser.parse_args()
    
    try:
        detector = ShapeDetector(
            camera_index=args.camera, 
            debug=args.debug, 
            output_file=args.output
        )
        
        # Set calibration factor
        detector.pixels_per_meter = args.calibration
        
        print(f"Starting shape-based cube detection with:")
        print(f"  Camera: {args.camera}")
        print(f"  Debug mode: {args.debug}")
        print(f"  Output file: {args.output}")
        print(f"  Calibration: {args.calibration} pixels/meter")
        
        detector.run()
        
    except Exception as e:
        print(f"Error in shape detection: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    main()
