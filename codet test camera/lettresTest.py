import cv2
import pytesseract
import imutils

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

    # Loop over the rectangles and draw them on the frame without non-maximum suppression
    for y in range(0, scores.shape[2]):
        for x in range(0, scores.shape[3]):
            if scores[0, 0, y, x] > 0.5:
                startX = int(x * 4.0)
                startY = int(y * 4.0)
                endX = int(W if x + 1 == scores.shape[3] else (x + 1) * 4.0)
                endY = int(H if y + 1 == scores.shape[2] else (y + 1) * 4.0)

                rect = ((startX, startY), (endX, endY))
                cv2.rectangle(frame, rect[0], rect[1], (0, 255, 0), 2)

    return frame

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Apply text detection
    detected_frame = detect_text(frame)

    # Resize the frame back to the original size for OCR
    resized_frame = imutils.resize(detected_frame, width=frame.shape[1])

    # Convert to grayscale
    gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

    # Add thresholding
    _, thresh = cv2.threshold(gray_frame, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    img = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)

    # Add denoising
    img = cv2.fastNlMeansDenoisingColored(img, None, 10, 10, 7, 21)

    # Modify Tesseract configuration
    text = pytesseract.image_to_string(img, config='--psm 6')
    print(text)

    cv2.imshow('RESULT', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.waitKey(0)
cv2.destroyAllWindows()
