import cv2
import numpy as np
import json
import os

DATA_DIR = "calibration_data"
SAMPLES_DIR = os.path.join(DATA_DIR, "samples")
REFERENCE_PATH = os.path.join(DATA_DIR, "reference.png")
SCALE_PATH = os.path.join(DATA_DIR, "scale.json")
MANIFEST_PATH = os.path.join(DATA_DIR, "samples.json")

OUTPUT_COLORS_PATH = os.path.join(DATA_DIR, "lut_colors.npy")
OUTPUT_GRADIENTS_PATH = os.path.join(DATA_DIR, "lut_gradients.npy")
OUTPUT_META_PATH = os.path.join(DATA_DIR, "lut_meta.json")

BALL_RADIUS_MM = 5.0  # 1cm diameter 3D printed ball
BLUR_KERNEL = (5, 5)


def load_scale():
    if not os.path.exists(SCALE_PATH):
        raise RuntimeError(
            "No scale.json found. Run calibrate_capture.py and use keys '1'/'2' "
            "to set pixels-per-mm before building the lookup table."
        )
    with open(SCALE_PATH) as f:
        return json.load(f)["px_per_mm"]


def blur(frame):
    return cv2.GaussianBlur(frame, BLUR_KERNEL, 0).astype(np.float32)


def main():
    px_per_mm = load_scale()
    print(f"Using scale: {px_per_mm:.2f} px/mm")

    with open(MANIFEST_PATH) as f:
        manifest = json.load(f)
    print(f"Processing {len(manifest)} calibration samples...")

    reference = blur(cv2.imread(REFERENCE_PATH))

    all_colors = []
    all_gradients = []

    for entry in manifest:
        frame = cv2.imread(os.path.join(SAMPLES_DIR, entry["file"]))
        diff = blur(frame) - reference  # (H, W, 3), signed, BGR order

        cx, cy, radius_px = entry["cx"], entry["cy"], entry["radius_px"]
        radius_mm = radius_px / px_per_mm
        if radius_mm >= BALL_RADIUS_MM:
            radius_mm = BALL_RADIUS_MM * 0.98  # clip: can't exceed the ball's own radius

        r_px_int = int(np.ceil(radius_px)) + 1
        x0, x1 = max(0, int(cx) - r_px_int), min(frame.shape[1], int(cx) + r_px_int)
        y0, y1 = max(0, int(cy) - r_px_int), min(frame.shape[0], int(cy) + r_px_int)

        ys, xs = np.meshgrid(np.arange(y0, y1), np.arange(x0, x1), indexing="ij")
        dx_px = xs - cx
        dy_px = ys - cy
        r_px = np.sqrt(dx_px ** 2 + dy_px ** 2)
        r_mm = r_px / px_per_mm

        valid = (r_px <= radius_px) & (r_mm < BALL_RADIUS_MM) & (r_px > 1e-6)
        if not np.any(valid):
            continue

        r_mm_valid = r_mm[valid]
        grad_mag = r_mm_valid / np.sqrt(BALL_RADIUS_MM ** 2 - r_mm_valid ** 2)

        dir_x = dx_px[valid] / r_px[valid]
        dir_y = dy_px[valid] / r_px[valid]

        gx = grad_mag * dir_x
        gy = grad_mag * dir_y

        colors = diff[ys[valid], xs[valid]]

        all_colors.append(colors)
        all_gradients.append(np.stack([gx, gy], axis=1))

    colors = np.concatenate(all_colors, axis=0)
    gradients = np.concatenate(all_gradients, axis=0)
    print(f"Collected {len(colors)} (color -> gradient) training pairs")

    np.save(OUTPUT_COLORS_PATH, colors.astype(np.float32))
    np.save(OUTPUT_GRADIENTS_PATH, gradients.astype(np.float32))
    with open(OUTPUT_META_PATH, "w") as f:
        json.dump({"px_per_mm": px_per_mm, "ball_radius_mm": BALL_RADIUS_MM}, f)

    print(f"Saved lookup table to {OUTPUT_COLORS_PATH} and {OUTPUT_GRADIENTS_PATH}")


if __name__ == "__main__":
    main()
