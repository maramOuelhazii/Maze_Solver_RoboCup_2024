import cv2
import pytesseract
import numpy as np


pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'

cap = cv2.VideoCapture(0)

frame_number = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_number += 1
    if frame_number % 2 != 0:
        continue

    resized_frame = cv2.resize(frame, (0, 0), fx=0.75, fy=0.75)

    # Convert to grayscale
    gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

    # Adaptive thresholding
    thresholded_frame = cv2.adaptiveThreshold(gray_frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # Morphological closing operation
    kernel = np.ones((3, 3), np.uint8)
    thresholded_frame = cv2.morphologyEx(thresholded_frame, cv2.MORPH_CLOSE, kernel)

    # Color detection
    hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

    # Define color ranges
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 250, 250])#55 55 

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 265])

    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 100])

    mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
    mask_yellow = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
    mask_black = cv2.inRange(hsv_frame, lower_black, upper_black)

    for i in range(5):
        mask_black = cv2.erode(mask_black, kernel)
    for i in range(9):
        mask_black = cv2.dilate(mask_black, kernel)

    color_result = cv2.bitwise_or(cv2.bitwise_or(cv2.bitwise_and(resized_frame, resized_frame, mask=mask_red), cv2.bitwise_and(resized_frame, resized_frame, mask=mask_yellow)), cv2.bitwise_and(resized_frame, resized_frame, mask=mask_green))

    text_color = pytesseract.image_to_string(mask_black, config='--psm 10 --oem 3 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')


    color = ""  

    if text_color.strip():
        print("Text Detected", text_color)
    else:
        if np.sum(mask_red) > np.sum(mask_yellow) and np.sum(mask_red) > np.sum(mask_green):
            color = "Red"
        elif np.sum(mask_yellow) > np.sum(mask_red) and np.sum(mask_yellow) > np.sum(mask_green):
            color = "Yellow"
        else:
            color = "Green"
        
    cv2.putText(resized_frame, f"Color Detected: {color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
                cv2.LINE_AA)
    
    print(hsv_frame[0, 0, :])
    cv2.imshow('Letter Detection RESULT', mask_black)
    cv2.imshow('Color Detection RESULT', resized_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
