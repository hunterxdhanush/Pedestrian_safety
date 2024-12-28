import cv2
import numpy as np
import os
import time
import serial  # Import the serial library

# Define paths
weights_path = "./yolov4.weights"
config_path = "./yolov4.cfg"
names_path = "./coco.names"

# Check if files exist
if not os.path.exists(weights_path):
    print("Weights file not found:", weights_path)
if not os.path.exists(config_path):
    print("Config file not found:", config_path)
if not os.path.exists(names_path):
    print("Names file not found:", names_path)

# Load YOLO
net = cv2.dnn.readNet(weights_path, config_path)
layer_names = net.getLayerNames()
unconnected_out_layers = net.getUnconnectedOutLayers()

# Handle both 1D and 2D outputs
if len(unconnected_out_layers.shape) == 1:
    output_layers = [layer_names[i - 1] for i in unconnected_out_layers]  # Convert 1-based to 0-based
else:
    output_layers = [layer_names[i[0] - 1] for i in unconnected_out_layers]  # Convert 1-based to 0-based

# Load class names
with open(names_path, "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Print class names for debugging
print("Classes loaded:", classes)

# Open video capture from webcam
cap = cv2.VideoCapture(0)  # Try different indices if 0 doesn't work

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Set lower resolution for better performance
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width to 320
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height to 240

# Optional: Delay to allow camera to initialize
time.sleep(1)

# Initialize serial communication with Arduino
arduino = serial.Serial(port='COM7', baudrate=9600)  # Replace 'COM_PORT' with your Arduino port
time.sleep(2)  # Wait for the connection to establish

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Resize frame for faster processing
    frame = cv2.resize(frame, (640, 480))

    # Detect objects
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Process detections
    class_ids = []
    confidences = []
    boxes = []
    person_detected = False  # Flag to track if a person is detected

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and classes[class_id] in ['person','cat','dog','laptop']:
                center_x = int(detection[0] * frame.shape[1])
                center_y = int(detection[1] * frame.shape[0])
                w = int(detection[2] * frame.shape[1])
                h = int(detection[3] * frame.shape[0])
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                person_detected = True  # Set flag to True if a person is detected

    # Apply non-max suppression
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    # Draw bounding boxes
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Send signal to Arduino based on detection
    if person_detected:
        arduino.write(b'1')  # Send '1' to turn on the light
    else:
        arduino.write(b'0')  # Send '0' to turn off the light

    # Display the resulting frame
    cv2.imshow('Webcam', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()  # Close the serial connection when done
