import argparse
import cv2
import numpy as np
import json
import os
from scipy.spatial import cKDTree

DATA_DIR = "calibration_data"
REFERENCE_PATH = os.path.join(DATA_DIR, "reference.png")
COLORS_PATH = os.path.join(DATA_DIR, "lut_colors.npy")
GRADIENTS_PATH = os.path.join(DATA_DIR, "lut_gradients.npy")
META_PATH = os.path.join(DATA_DIR, "lut_meta.json")

COLOR_BLUR_KERNEL = (5, 5)
GRAY_BLUR_KERNEL = (9, 9)
DIFF_THRESHOLD = 25
MIN_CONTACT_AREA = 200


def color_blur(frame):
    return cv2.GaussianBlur(frame, COLOR_BLUR_KERNEL, 0).astype(np.float32)


def gray_blur(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, GRAY_BLUR_KERNEL, 0)


def integrate_gradients(gx, gy, pixel_spacing_mm):
    h, w = gx.shape
    wx = np.fft.fftfreq(w, d=pixel_spacing_mm) * 2 * np.pi
    wy = np.fft.fftfreq(h, d=pixel_spacing_mm) * 2 * np.pi
    wx_grid, wy_grid = np.meshgrid(wx, wy)

    fx = np.fft.fft2(gx)
    fy = np.fft.fft2(gy)

    denom = wx_grid ** 2 + wy_grid ** 2
    denom[0, 0] = 1.0

    z_hat = (-1j * wx_grid * fx - 1j * wy_grid * fy) / denom
    z_hat[0, 0] = 0.0

    return np.real(np.fft.ifft2(z_hat))


def main():
    parser = argparse.ArgumentParser(description="DIGIT calibrated depth reconstruction")
    parser.add_argument('--device-index', type=int, default=0,
                         help='Camera device index for the DIGIT sensor (default: 0)')
    args = parser.parse_args()

    with open(META_PATH) as f:
        meta = json.load(f)
    px_per_mm = meta["px_per_mm"]
    pixel_spacing_mm = 1.0 / px_per_mm

    colors = np.load(COLORS_PATH)
    gradients = np.load(GRADIENTS_PATH)
    print(f"Loaded lookup table with {len(colors)} entries")
    tree = cKDTree(colors)

    reference_raw = cv2.imread(REFERENCE_PATH)
    reference_color = color_blur(reference_raw)
    reference_gray = gray_blur(reference_raw)

    cap = cv2.VideoCapture(args.device_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise RuntimeError("Could not open DIGIT sensor")

    print("Press 'q' to quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame")
            break

        gray = gray_blur(frame)
        gray_diff = cv2.absdiff(gray, reference_gray)
        _, mask = cv2.threshold(gray_diff, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contact_contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTACT_AREA]

        gx = np.zeros(gray.shape, dtype=np.float32)
        gy = np.zeros(gray.shape, dtype=np.float32)
        peak_depth_mm = 0.0

        if contact_contours:
            contact_mask = np.zeros_like(mask)
            cv2.drawContours(contact_mask, contact_contours, -1, 255, -1)

            ys, xs = np.where(contact_mask == 255)
            color_diff = color_blur(frame) - reference_color
            query_colors = color_diff[ys, xs]

            _, idx = tree.query(query_colors, k=1)
            matched_gradients = gradients[idx]

            gx[ys, xs] = matched_gradients[:, 0]
            gy[ys, xs] = matched_gradients[:, 1]

            height_map = integrate_gradients(gx, gy, pixel_spacing_mm)
            peak_depth_mm = -height_map.min()
        else:
            height_map = np.zeros(gray.shape, dtype=np.float64)

        display_range = max(np.abs(height_map).max(), 1e-6)
        normalized = np.clip((height_map / display_range) * 0.5 * 255 + 127, 0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

        cv2.putText(colored, f"peak depth: {peak_depth_mm:.2f}mm", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("DIGIT live", frame)
        cv2.imshow("reconstructed depth", colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
