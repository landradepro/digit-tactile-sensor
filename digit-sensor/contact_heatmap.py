import argparse
import cv2
import numpy as np

BLUR_KERNEL = (7, 7)

reference = None


def main():
    global reference

    parser = argparse.ArgumentParser(description="DIGIT contact intensity heatmap")
    parser.add_argument('--device-index', type=int, default=0,
                         help='Camera device index for the DIGIT sensor (default: 0)')
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        raise RuntimeError("Could not open DIGIT sensor")

    print("Press 'r' with nothing touching the gel to set the reference frame")
    print("Press 'q' to quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame")
            break

        blurred = cv2.GaussianBlur(frame, BLUR_KERNEL, 0).astype(np.float32)

        if reference is None:
            cv2.putText(frame, "no reference yet - press 'r'", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow("DIGIT live", frame)
        else:
            diff = blurred - reference  # signed, per-channel (B, G, R)
            magnitude = np.sqrt(np.sum(diff ** 2, axis=2))

            normalized = np.clip(magnitude / magnitude.max() * 255, 0, 255).astype(np.uint8) \
                if magnitude.max() > 0 else magnitude.astype(np.uint8)
            heatmap = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

            cv2.imshow("DIGIT live", frame)
            cv2.imshow("contact intensity heatmap", heatmap)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            reference = cv2.GaussianBlur(frame, BLUR_KERNEL, 0).astype(np.float32)
            print("Reference frame captured")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
