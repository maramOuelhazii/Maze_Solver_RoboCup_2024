import cv2
import pytesseract

pytesseract.pytesseract.tesseract_cmd='C:\\Program Files\\Tesseract-OCR\\tesseract.exe'
# Open the default camera (camera index 0)
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    resized_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
    img = cv2.cvtColor(gray_frame, cv2.COLOR_BGR2RGB)
    print(pytesseract.image_to_string(img, config='--psm 10'))
    cv2.imshow('RESULT',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

cv2.waitKey(0)
cv2.destroyAllWindows()

    