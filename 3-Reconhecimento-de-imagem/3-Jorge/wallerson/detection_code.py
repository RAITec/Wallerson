import cv2
import matplotlib.pyplot as plt
from ultralytics import YOLO
from detected_object import DetectedObject

model = YOLO('runs/detect/train/weights/best.pt')
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error opening webcam.")
    exit()

plt.ion()

while True:

    ret, frame = cap.read()

    if not ret:
        print("Error capturing frame.")
        break

    results = model(frame)
    detected_objects = []

    for result in results:
        boxes = result.boxes
        for box in boxes:
            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()

            # classe e confiança
            class_id = int(box.cls)
            class_label = model.names[class_id]
            confidence = float(box.conf)

            obj = DetectedObject(class_label, confidence, x_min, y_min, x_max, y_max)
            detected_objects.append(obj)
            print(obj)

    annotated_frame = results[0].plot()
    annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)

    plt.imshow(annotated_frame)
    plt.axis('off')  # Hide axes
    plt.draw()  # Update the figure
    plt.pause(0.001)  # Pause to allow the figure to update

    if plt.waitforbuttonpress(0.001):
        break

# Release the camera and close the figure
cap.release()
plt.close()