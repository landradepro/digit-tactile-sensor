import argparse
import cv2
import numpy as np

BLUR_KERNEL = (9, 9)
MIN_CONTACT_AREA = 400  # pixels; raise/lower depending on noise vs sensitivity
THRESHOLD_START = 25

reference_gray = None
threshold_value = THRESHOLD_START


def to_gray_blurred(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, BLUR_KERNEL, 0)


def main():
    global reference_gray, threshold_value

    parser = argparse.ArgumentParser(description="DIGIT contact detection")
    parser.add_argument('--device-index', type=int, default=0,
                         help='Camera device index for the DIGIT sensor (default: 0)')
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        raise RuntimeError("Could not open DIGIT sensor")

    print("Keys: 'r' = capture reference frame (do this with nothing touching the gel)")
    print("      '+' / '-' = adjust sensitivity threshold")
    print("      'q' = quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame")
            break

        gray = to_gray_blurred(frame)

        display = frame.copy()
        status_lines = [f"threshold: {threshold_value}"]

        if reference_gray is None:
            status_lines.append("no reference yet - press 'r'")
        else:
            diff = cv2.absdiff(gray, reference_gray)
            _, mask = cv2.threshold(diff, threshold_value, 255, cv2.THRESH_BINARY)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contact_contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTACT_AREA]

            if contact_contours:
                largest = max(contact_contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                x, y, w, h = cv2.boundingRect(largest)
                M = cv2.moments(largest)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.drawContours(display, [largest], -1, (0, 255, 0), 2)
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 1)
                cv2.circle(display, (cx, cy), 4, (0, 0, 255), -1)

                status_lines.append(f"TOUCHING  area={int(area)}px  center=({cx},{cy})")
            else:
                status_lines.append("no contact")

            cv2.imshow("diff mask", mask)

        for i, line in enumerate(status_lines):
            cv2.putText(display, line, (10, 25 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("DIGIT contact detection", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('r'):
            reference_gray = gray.copy()
            print("Reference frame captured")
        elif key == ord('+'):
            threshold_value += 1
        elif key == ord('-'):
            threshold_value = max(1, threshold_value - 1)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
