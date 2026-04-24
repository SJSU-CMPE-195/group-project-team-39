import cv2
import time
import glob

# ==============================================================================
# APALIS / LINUX DOCKER VERSION (Commented out for Mac Testing)
# ==============================================================================

def open_camera():
    devices = sorted(glob.glob('/dev/video*'))
    print(f"Found video devices: {devices}", flush=True)
    for dev in devices:
        # Use V4L2 backend specifically for Linux/Docker environments
        c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if c.isOpened():
            ret, _ = c.read()
            if ret:
                print(f"Camera found at {dev}", flush=True)
                return c
            c.release()
    raise RuntimeError(f"No working camera found. Tried: {devices}")

def main():
    print("Opening the camera automatically checking dev/video nodes...", flush=True)
    try:
        cap = open_camera()
    except RuntimeError as e:
        print(f"Error: {e}", flush=True)
        return
    
    # FIX 1: Force MJPG compression first! Uncompressed raw video over the 
    # Apalis USB bus completely saturates its bandwidth, artificially throttling it to 30fps.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    
    # =========================================================
    # Option A: 720p HD Mode (Should now hit 60fps with MJPG)
    # =========================================================
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # cap.set(cv2.CAP_PROP_FPS, 60)
    
    # =========================================================
    # Option B: WVGA Low-Res Mode (Natively goes up to 100 fps!)
    # =========================================================
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
    cap.set(cv2.CAP_PROP_FPS, 100)
        
    # Read the camera resolution that was automatically selected
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened successfully at {w}x{h}", flush=True)
    
    frame_count = 0
    frames_in_interval = 0
    start_time = time.time()
    last_print_time = start_time
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("\nFailed to grab frame.", flush=True)
                break
                
            frame_count += 1
            frames_in_interval += 1
            now = time.time()
            
            # Print FPS exactly every 2 seconds
            if now - last_print_time >= 2.0:
                # Calculate FPS over the last 2 seconds
                fps = frames_in_interval / (now - last_print_time)
                print(f"Live Camera FPS: {fps:.1f}", flush=True)
                
                # Reset counters for the next 2-second block
                last_print_time = now
                frames_in_interval = 0

    except KeyboardInterrupt:
        # Gracefully handle the user pressing Ctrl+C in the terminal
        pass

    total_time = time.time() - start_time
    print(f"\n\nAverage FPS: {frame_count / total_time:.2f} over {total_time:.1f} seconds", flush=True)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


# ==============================================================================
# MACOS LOCAL VERSION (Active)
# ==============================================================================

# def main():
#     print("Opening the default camera using OpenCV for macOS...", flush=True)
#     cap = cv2.VideoCapture(0)
    
#     # We must explicitly force the WVGA high-speed mode on macOS
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)
#     cap.set(cv2.CAP_PROP_FPS, 100)
    
#     if not cap.isOpened():
#         print("Error: Could not open camera.", flush=True)
#         return
        
#     # Read the camera resolution that was automatically selected
#     w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#     h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#     print(f"Camera opened successfully at {w}x{h}", flush=True)
    
#     frame_count = 0
#     frames_in_interval = 0
#     start_time = time.time()
#     last_print_time = start_time
    
#     try:
#         while True:
#             ret, frame = cap.read()
#             if not ret:
#                 print("\nFailed to grab frame.", flush=True)
#                 break
                
#             frame_count += 1
#             frames_in_interval += 1
#             now = time.time()
            
#             # Print FPS exactly every 2 seconds
#             if now - last_print_time >= 2.0:
#                 fps = frames_in_interval / (now - last_print_time)
#                 print(f"Live Camera FPS: {fps:.1f}", flush=True)
#                 last_print_time = now
#                 frames_in_interval = 0
                
#             # Show the video window
#             cv2.imshow("Mac Camera Stream (100 FPS Mode)", frame)
            
#             # Press 'q' or 'esc' to exit gracefully
#             if cv2.waitKey(1) in (27, ord('q')):
#                 break

#     except KeyboardInterrupt:
#         pass

#     total_time = time.time() - start_time
#     print(f"\n\nAverage FPS: {frame_count / total_time:.2f} over {total_time:.1f} seconds", flush=True)

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()
