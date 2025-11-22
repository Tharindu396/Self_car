import cv2
import pupil_apriltags as apriltag

def main():
    # Initialize camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Initialize the AprilTag detector
    detector = apriltag.Detector()

    print("AprilTag Scanner Started - Press 'q' to exit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert to grayscale (AprilTag works on grayscale)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        results = detector.detect(gray)

        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            ptA, ptB, ptC, ptD = (tuple(map(int, ptA)),
                                 tuple(map(int, ptB)),
                                 tuple(map(int, ptC)),
                                 tuple(map(int, ptD)))

            # Draw bounding box
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

            # Draw center
            cx, cy = r.center
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)

            # Display Tag ID
            tag_id = r.tag_id
            cv2.putText(frame, f"ID: {tag_id}", (int(cx), int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            print(f"Detected Tag ID: {tag_id}")

        cv2.imshow("AprilTag Scanner", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
