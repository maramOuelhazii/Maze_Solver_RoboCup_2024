import cv2
import pytesseract

# Set the path to the Tesseract executable
pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'

# Open the default camera (camera index 0)
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Resize the frame to a smaller size
    resized_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

    # Convert the resized frame to grayscale (optional)
    gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

    # Perform OCR on the resized or grayscale frame
    text = pytesseract.image_to_string(gray_frame, config='--psm 6')

    # Display the result on the original frame
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Camera OCR', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
