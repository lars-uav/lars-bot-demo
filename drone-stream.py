import cv2

def start_video_stream():
    # Open the video capture from the webcam
    cap = cv2.VideoCapture(0)  # 0 is usually the default camera
    
    # Check if the webcam opened successfully
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Set up GStreamer pipeline to stream video over RTSP
    gst_pipeline = (
        "appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency "
        "bitrate=500 ! rtph264pay config-interval=1 pt=96 ! "
        "udpsink host=<10.1.57.57> port=5000"
    )
    
    out = cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, 0, 20.0, (640, 480))
    
    if not out.isOpened():
        print("Error: Could not open video stream for output.")
        return

    print("Starting video stream...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Write frame to the GStreamer pipeline
            out.write(frame)

            # Display the frame locally (optional)
            cv2.imshow("Drone Camera", frame)
            
            # Press 'q' to quit the streaming
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Streaming stopped by user.")
    
    # Release resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print("Stream closed.")

start_video_stream()