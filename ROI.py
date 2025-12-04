import cv2

# Open video
cap = cv2.VideoCapture(0)  # Change to your video file if needed

# Define ROI (x, y, width, height)
x, y, w, h = 100, 50, 200, 150  # Modify as needed

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Crop ROI
    roi = frame[y:y+h, x:x+w]

    cv2.imshow("Original", frame)
    cv2.imshow("ROI", roi)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
