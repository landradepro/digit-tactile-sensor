import argparse
import cv2
import numpy as np
import os
import json

BLUR_KERNEL = (9, 9)
DIFF_THRESHOLD = 25
MIN_CONTACT_AREA = 200

DATA_DIR = "calibration_data"
SAMPLES_DIR = os.path.join(DATA_DIR, "samples")
REFERENCE_PATH = os.path.join(DATA_DIR, "reference.png")
SCALE_PATH = os.path.join(DATA_DIR, "scale.json")
MANIFEST_PATH = os.path.join(DATA_DIR, "samples.json")

reference = None
scale_point_a = None  # (cx, cy) in pixels, first translation-calibration point


def to_gray_blurred(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, BLUR_KERNEL, 0)


def find_contact_circle(frame, reference_gray):
    gray = to_gray_blurred(frame)
    diff = cv2.absdiff(gray, reference_gray)
    _, mask = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contact_contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTACT_AREA]
    if not contact_contours:
        return None

    largest = max(contact_contours, key=cv2.contourArea)
    (cx, cy), radius_px = cv2.minEnclosingCircle(largest)
    return cx, cy, radius_px


def load_scale():
    if os.path.exists(SCALE_PATH):
        with open(SCALE_PATH) as f:
            return json.load(f).get("px_per_mm")
    return None


def save_scale(px_per_mm):
    with open(SCALE_PATH, "w") as f:
        json.dump({"px_per_mm": px_per_mm}, f)


def load_manifest():
    if os.path.exists(MANIFEST_PATH):
        with open(MANIFEST_PATH) as f:
            return json.load(f)
    return []


def save_manifest(manifest):
    with open(MANIFEST_PATH, "w") as f:
        json.dump(manifest, f, indent=2)


def main():
    global reference, scale_point_a

    parser = argparse.ArgumentParser(description="DIGIT calibration data capture")
    parser.add_argument('--device-index', type=int, default=0,
                         help='Camera device index for the DIGIT sensor (default: 0)')
    args = parser.parse_args()

    os.makedirs(SAMPLES_DIR, exist_ok=True)

    if os.path.exists(REFERENCE_PATH):
        reference = cv2.imread(REFERENCE_PATH)
        print("Loaded existing reference frame")

    px_per_mm = load_scale()
    if px_per_mm:
        print(f"Loaded existing scale: {px_per_mm:.2f} px/mm")

    manifest = load_manifest()
    print(f"Loaded {len(manifest)} existing calibration samples")

    cap = cv2.VideoCapture(args.device_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise RuntimeError("Could not open DIGIT sensor")

    print("\nKeys:")
    print("  r     - capture reference frame (nothing touching the gel)")
    print("  1     - record scale point A (place ball, hold still)")
    print("  2     - record scale point B (after moving ball a known distance)")
    print("  space - save a calibration sample (ball touching the gel)")
    print("  q     - quit\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame")
            break

        display = frame.copy()
        info = []

        if reference is None:
            info.append("no reference yet - press 'r'")
            circle = None
        else:
            reference_gray = to_gray_blurred(reference)
            circle = find_contact_circle(frame, reference_gray)
            if circle:
                cx, cy, radius_px = circle
                cv2.circle(display, (int(cx), int(cy)), int(radius_px), (0, 255, 0), 2)
                cv2.circle(display, (int(cx), int(cy)), 3, (0, 0, 255), -1)
                info.append(f"contact at ({cx:.0f},{cy:.0f}) r={radius_px:.1f}px")
            else:
                info.append("no contact detected")

        if px_per_mm:
            info.append(f"scale: {px_per_mm:.2f} px/mm")
        else:
            info.append("scale not set - use keys 1 / 2")

        info.append(f"samples collected: {len(manifest)}")

        for i, line in enumerate(info):
            cv2.putText(display, line, (10, 25 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("calibration capture", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('r'):
            reference = frame.copy()
            cv2.imwrite(REFERENCE_PATH, reference)
            print("Reference frame captured and saved")

        elif key == ord('1'):
            if circle is None:
                print("No contact detected - press the ball on the gel first")
            else:
                scale_point_a = (circle[0], circle[1])
                print(f"Point A recorded at {scale_point_a}")

        elif key == ord('2'):
            if scale_point_a is None:
                print("Record point A first (key '1')")
            elif circle is None:
                print("No contact detected - press the ball on the gel first")
            else:
                point_b = (circle[0], circle[1])
                pixel_dist = np.hypot(point_b[0] - scale_point_a[0], point_b[1] - scale_point_a[1])
                try:
                    real_mm = float(input(f"Pixel distance = {pixel_dist:.2f}px. "
                                          f"Enter real distance moved in mm: "))
                    px_per_mm = pixel_dist / real_mm
                    save_scale(px_per_mm)
                    print(f"Saved scale: {px_per_mm:.2f} px/mm")
                except ValueError:
                    print("Invalid number, try again")
                scale_point_a = None

        elif key == ord(' '):
            if reference is None:
                print("Capture a reference frame first (key 'r')")
            elif circle is None:
                print("No contact detected - press the ball on the gel first")
            else:
                cx, cy, radius_px = circle
                idx = len(manifest)
                filename = f"sample_{idx:04d}.png"
                cv2.imwrite(os.path.join(SAMPLES_DIR, filename), frame)
                manifest.append({"file": filename, "cx": cx, "cy": cy, "radius_px": radius_px})
                save_manifest(manifest)
                print(f"Saved sample {idx}: center=({cx:.0f},{cy:.0f}) radius={radius_px:.1f}px")

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nDone. {len(manifest)} calibration samples saved in {SAMPLES_DIR}/")


if __name__ == "__main__":
    main()
