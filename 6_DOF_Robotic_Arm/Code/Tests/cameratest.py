import cv2

# Load the pre-trained Haar Cascade classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Open a connection to the front-facing camera using the AVFoundation backend
cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)

# Check if the camera is opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Create a new window for the camera display
cv2.namedWindow("Face Tracking", cv2.WINDOW_NORMAL)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was read successfully
    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert the frame to grayscale for face detection
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Perform face detection
    faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.3, minNeighbors=5, minSize=(30, 30))

    # Draw a square around each detected face
    for (x, y, w, h) in faces:
        # Calculate the center of the square
        center_x = x + w // 2
        center_y = y + h // 2

        # Calculate the length of the sides (assuming the square is a rectangle)
        side_length = max(w, h)

        # Draw the square on the frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display the center coordinates and side length
        text = f"Center: ({center_x}, {center_y}), Side Length: {side_length}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Display the frame with the square around the face
    cv2.imshow("Face Tracking", frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
