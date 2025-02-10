import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Load the YOLO model
model = YOLO('runs/detect/train/weights/best.pt')  # Replace with the path to your model file

# Access the webcam
cap = cv2.VideoCapture(0)  # 0 usually refers to the default webcam

if not cap.isOpened():
    print("Error opening webcam.")
    exit()

plt.ion()  # Turn on interactive mode for matplotlib

while True:
    # Capture frame from webcam
    ret, frame = cap.read()

    if not ret:
        print("Error capturing frame.")
        break

    # Perform object detection on the current frame
    results = model(frame)

    # Plot detection results on the image (draw bounding boxes and labels)
    annotated_frame = results[0].plot()

    # Convert BGR (OpenCV format) to RGB (matplotlib format)
    annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)

    # Display the processed frame using matplotlib
    plt.imshow(annotated_frame)
    plt.axis('off')  # Hide axes
    plt.draw()  # Update the figure
    plt.pause(0.001)  # Pause to allow the figure to update

    # Break the loop when 'q' is pressed
    if plt.waitforbuttonpress(0.001):  # This handles keyboard events in a non-blocking way
        break

# Release the camera and close the figure
cap.release()
plt.close()
