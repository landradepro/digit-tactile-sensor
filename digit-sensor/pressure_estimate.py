import argparse
import cv2
import numpy as np
from collections import deque

BLUR_KERNEL = (9, 9)
DIFF_THRESHOLD = 25
MIN_CONTACT_AREA = 200

GRAPH_WIDTH = 640
GRAPH_HEIGHT = 200
GRAPH_HISTORY = 200

reference_gray = None
history = deque(maxlen=GRAPH_HISTORY)


def to_gray_blurred(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, BLUR_KERNEL, 0)


def draw_graph(values, running_max):
    graph = np.zeros((GRAPH_HEIGHT, GRAPH_WIDTH, 3), dtype=np.uint8)
    if running_max <= 0 or len(values) < 2:
        return graph

    points = list(values)
    step = GRAPH_WIDTH / GRAPH_HISTORY
    for i in range(1, len(points)):
        x1 = int((i - 1) * step)
        x2 = int(i * step)
        y1 = GRAPH_HEIGHT - int(points[i - 1] / running_max * (GRAPH_HEIGHT - 10))
        y2 = GRAPH_HEIGHT - int(points[i] / running_max * (GRAPH_HEIGHT - 10))
        cv2.line(graph, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.putText(graph, f"max: {running_max:.0f}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (0, 255, 255), 1, cv2.LINE_AA)
    return graph


def main():
    global reference_gray

    parser = argparse.ArgumentParser(description="DIGIT pressure/force proxy estimator")
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

    running_max = 1.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame")
            break

        display = frame.copy()
        info = []

        if reference_gray is None:
            info.append("no reference yet - press 'r'")
            history.append(0)
        else:
            gray = to_gray_blurred(frame)
            diff = cv2.absdiff(gray, reference_gray)
            _, mask = cv2.threshold(diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contact_contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTACT_AREA]

            if contact_contours:
                contact_mask = np.zeros_like(mask)
                cv2.drawContours(contact_mask, contact_contours, -1, 255, -1)

                area_px = int(np.count_nonzero(contact_mask))
                mean_intensity = float(diff[contact_mask == 255].mean())
                pressure_proxy = area_px * mean_intensity / 1000.0  # scaled down for readability

                history.append(pressure_proxy)
                running_max = max(running_max * 0.995, pressure_proxy)  # slow decay, quick rise

                largest = max(contact_contours, key=cv2.contourArea)
                cv2.drawContours(display, [largest], -1, (0, 255, 0), 2)

                info.append(f"contact area: {area_px}px")
                info.append(f"mean intensity: {mean_intensity:.1f}")
                info.append(f"pressure proxy: {pressure_proxy:.1f}")
            else:
                history.append(0)
                info.append("no contact")

        for i, line in enumerate(info):
            cv2.putText(display, line, (10, 25 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 1, cv2.LINE_AA)

        graph = draw_graph(history, running_max)

        cv2.imshow("DIGIT live", display)
        cv2.imshow("pressure proxy over time", graph)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            reference_gray = to_gray_blurred(frame)
            print("Reference frame captured")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
