#!/usr/bin/env python3
"""
Script to check available camera indices for OpenCV
"""

import cv2
import time

def check_camera_indices(max_index=10):
    """
    Check which camera indices are available for OpenCV
    
    Args:
        max_index (int): Maximum camera index to check (default: 10)
    
    Returns:
        list: List of available camera indices
    """
    available_cameras = []
    
    print(f"Checking camera indices from 0 to {max_index-1}...")
    print("-" * 50)
    
    for i in range(max_index):
        print(f"Testing camera index {i}...", end=" ")
        
        # Try to open the camera
        cap = cv2.VideoCapture(i)
        
        if cap.isOpened():
            # Try to read a frame to confirm it's working
            ret, frame = cap.read()
            if ret:
                # Get camera properties
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                print(f"✓ AVAILABLE - Resolution: {width}x{height}, FPS: {fps:.1f}")
                available_cameras.append({
                    'index': i,
                    'resolution': (width, height),
                    'fps': fps
                })
            else:
                print("✗ Opened but can't read frames")
            cap.release()
        else:
            print("✗ Not available")
    
    print("-" * 50)
    print(f"Found {len(available_cameras)} available camera(s):")
    
    for cam in available_cameras:
        print(f"  Camera {cam['index']}: {cam['resolution'][0]}x{cam['resolution'][1]} @ {cam['fps']:.1f} FPS")
    
    return available_cameras

def test_camera_stream(camera_index, duration=5):
    """
    Test streaming from a specific camera index
    
    Args:
        camera_index (int): Camera index to test
        duration (int): Duration to stream in seconds
    """
    print(f"\nTesting camera stream for index {camera_index} for {duration} seconds...")
    
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Failed to open camera {camera_index}")
        return
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if ret:
            cv2.imshow(f'Camera {camera_index}', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Failed to read frame")
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"Camera {camera_index} test completed")

if __name__ == "__main__":
    # Check available cameras
    available_cameras = check_camera_indices(max_index=10)
    
    # If cameras are found, offer to test them
    if available_cameras:
        print("\nWould you like to test any camera streams? (y/n): ", end="")
        try:
            response = input().lower().strip()
            if response == 'y':
                for cam in available_cameras:
                    test_camera_stream(cam['index'], duration=3)
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
    else:
        print("No cameras found. Make sure your cameras are properly connected.") 