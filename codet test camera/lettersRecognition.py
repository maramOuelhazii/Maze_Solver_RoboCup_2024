import cv2
import pytesseract

pytesseract.pytesseract.tesseract_cmd='C:\\Program Files\\Tesseract-OCR\\tesseract.exe'
img = cv2.imread('hello.png')
img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
print(pytesseract.image_to_string(img, config='--psm 10'))
cv2.imshow('RESULT',img)
cv2.waitKey(0)
    
    
    
   