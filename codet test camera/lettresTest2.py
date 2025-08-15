import cv2
import pytesseract
import imutils
import numpy as np

pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'

# Load the EAST text detector
net = cv2.dnn.readNet("C:\\Users\\ASUS\\Desktop\\maaaaze masr\\EAST-Detector-for-text-detection-using-OpenCV-master\\frozen_east_text_detection.pb")

# Open the default camera (camera index 0)
cap = cv2.VideoCapture(0)

def detect_text(frame):
    # Resize the frame to have a width of 320 pixels (adjust as needed)
    frame = imutils.resize(frame, width=320)

    # Get height and width of the frame
    (H, W) = (320,320)

    # Construct a blob from the frame, perform a forward pass with the EAST model
    blob = cv2.dnn.blobFromImage(frame, 1.0, (W, H), (123.68, 116.78, 103.94), swapRB=True, crop=False)
    net.setInput(blob)
    
    (scores, geometry) = net.forward(["feature_fusion/Conv_7/Sigmoid", "feature_fusion/concat_3"])

    # Apply non-maximum suppression
    boxes = get_bounding_boxes(scores, geometry)
    confidences = scores.flatten()

    if len(boxes) > 0:
        indices = cv2.dnn.NMSBoxesRotated(
            boxes,
            confidences,
            score_threshold=0.5,
            nms_threshold=0.4
        )

        # Iterate over the selected indices and draw the bounding boxes
        for i in indices:
            i = i[0]  # Extract the index from the list
            box = boxes[i]
            confidence = confidences[i]

            # Draw the rotated bounding box on the frame
            draw_rotated_box(frame, box, confidence)

            # Extract the region of interest (ROI) containing the detected letter
            (startX, startY), (endX, endY) = box
            letter_roi = frame[startY:endY, startX:endX]

            # Perform OCR on the letter ROI
            letter_text, letter_confidence = perform_ocr(letter_roi)

            # Display the detected letter and its confidence on the frame
            display_text = f'{letter_text} ({letter_confidence:.2%})'
            cv2.putText(frame, display_text, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)

    return frame

def get_bounding_boxes(scores, geometry):
    (numRows, numCols) = scores.shape[2:4]
    rects = []
    confidences = []

    for y in range(0, numRows):
        scoresData = scores[0, 0, y]
        xData0 = geometry[0, 0, y]
        xData1 = geometry[0, 1, y]
        xData2 = geometry[0, 2, y]
        xData3 = geometry[0, 3, y]
        anglesData = geometry[0, 4, y]

        for x in range(0, numCols):
            if scoresData[x] < 0.5:
                continue

            (offsetX, offsetY) = (x * 4.0, y * 4.0)

            angle = anglesData[x]
            cos = np.cos(angle)
            sin = np.sin(angle)

            h = xData0[x] + xData2[x]
            w = xData1[x] + xData3[x]

            endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
            endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
            startX = int(endX - w)
            startY = int(endY - h)

            rects.append(((startX, startY), (endX, endY)))
            confidences.append(scoresData[x])

    return rects

def draw_rotated_box(frame, box, confidence):
    # Unpack the box information
    (startX, startY), (endX, endY) = box

    # Create a rotated rectangle using OpenCV's RotatedRect
    rect = ((startX + endX) / 2, (startY + endY) / 2, endX - startX, endY - startY, 0)
    box_points = cv2.boxPoints(rect)
    box_points = np.int0(box_points)

    # Draw the rotated bounding box on the frame
    cv2.drawContours(frame, [box_points], 0, (0, 255, 0), 2)

def perform_ocr(letter_roi):
    # Convert the letter ROI to grayscale
    gray_letter = cv2.cvtColor(letter_roi, cv2.COLOR_BGR2GRAY)

    # Threshold the letter ROI
    _, thresh = cv2.threshold(gray_letter, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    # Convert back to RGB
    img = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)

    # Add denoising
    img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)

    # Perform OCR on the letter ROI
    letter_text = pytesseract.image_to_string(img, config='--psm 6')
    letter_confidence = pytesseract.image_to_boxes(img).splitlines()[0].split(' ')[-1]

    return letter_text, float(letter_confidence)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply text detection and OCR
    detected_frame = detect_text(frame)

    cv2.imshow('RESULT', detected_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture object and close the window
cap.release()
cv2.destroyAllWindows()
