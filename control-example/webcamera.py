#!/usr/bin/env python3
"""
Webcam wrapper supporting one or two cameras.

Usage as a library:
    from webcamera import WebCamera
    cam = WebCamera(camera_indices=[0, 1])
    img0, img1 = cam.get_images()
    cam.close()

    # or with context manager:
    with WebCamera() as cam:
        frame = cam.get_image()          # single camera (index 0)
        left, right = cam.get_images()   # both cameras

Controls (when running standalone):
    q  -  quit
    s  -  save snapshots to disk
"""

import sys
import cv2
import numpy as np


class WebCamera:
    def __init__(
        self,
        camera_indices: list[int] | int = 0,
        width: int = 1280,
        height: int = 720,
    ):
        if isinstance(camera_indices, int):
            camera_indices = [camera_indices]
        self._indices = camera_indices
        self._caps: list[cv2.VideoCapture] = []

        for idx in camera_indices:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.close()
                raise RuntimeError(
                    f"Could not open camera at index {idx}. "
                    "Check `ls /dev/video*`."
                )
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._caps.append(cap)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_image(self, camera: int = 0) -> np.ndarray:
        """Return a single BGR frame from the camera at the given list position."""
        if camera >= len(self._caps):
            raise IndexError(
                f"Camera position {camera} requested but only "
                f"{len(self._caps)} camera(s) opened."
            )
        ok, frame = self._caps[camera].read()
        if not ok or frame is None:
            raise RuntimeError(f"Failed to grab frame from camera position {camera}.")
        return frame

    def get_images(self) -> list[np.ndarray]:
        """Return one BGR frame from every opened camera."""
        return [self.get_image(i) for i in range(len(self._caps))]

    def close(self) -> None:
        for cap in self._caps:
            cap.release()
        self._caps.clear()

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


# ----------------------------------------------------------------------
# Standalone viewer
# ----------------------------------------------------------------------

def main():
    indices = [0, 2]

    with WebCamera(camera_indices=indices) as cam:
        n = len(cam._caps)
        w = int(cam._caps[0].get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cam._caps[0].get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Opened {n} camera(s) at {w}x{h}. Press 'q' to quit, 's' to save.")

        snapshot_idx = 0
        while True:
            frames = cam.get_images()

            for i, f in enumerate(frames):
                cv2.imshow(f"Webcam {i}", f)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                for i, f in enumerate(frames):
                    name = f"snapshot_{snapshot_idx:03d}_cam{i}.png"
                    cv2.imwrite(name, f)
                    print(f"Saved {name}")
                snapshot_idx += 1

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
