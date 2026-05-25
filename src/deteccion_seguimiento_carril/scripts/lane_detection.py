import cv2
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class LaneConfig:
    canny_low:        int   = 50
    canny_high:       int   = 150
    hough_rho:        int   = 2
    hough_threshold:  int   = 50
    hough_min_len:    int   = 40
    hough_max_gap:    int   = 100
    min_slope:        float = 0.4
    smoothing:        int   = 8
    horizon:          float = 0.58
    center_threshold: float = 0.20


@dataclass
class LaneState:
    zone:          str           = "UNKNOWN"
    lateral:       float         = 0.5
    offset_px:     int           = 0
    lane_width_px: Optional[int] = None
    left_detected: bool          = False
    right_detected: bool         = False


class LaneDetector:

    def __init__(self, cfg: LaneConfig = None):
        self._cfg        = cfg or LaneConfig()
        self._left_hist  = deque(maxlen=self._cfg.smoothing)
        self._right_hist = deque(maxlen=self._cfg.smoothing)

    def _detect(self, frame, h, w):
        cfg = self._cfg

        hls         = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        white_mask  = cv2.inRange(hls, (0, 190, 0),  (180, 255, 255))
        yellow_mask = cv2.inRange(hls, (15, 30, 80), (40, 220, 255))
        color_mask  = cv2.bitwise_or(white_mask, yellow_mask)

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked  = cv2.bitwise_and(gray, color_mask)
        blurred = cv2.GaussianBlur(masked, (7, 7), 0)
        edges   = cv2.Canny(blurred, cfg.canny_low, cfg.canny_high)

        roi = self._roi(edges, h, w)

        lines = cv2.HoughLinesP(
            roi, cfg.hough_rho, np.pi / 180, cfg.hough_threshold,
            minLineLength=cfg.hough_min_len, maxLineGap=cfg.hough_max_gap,
        )

        lx, ly, rx, ry = [], [], [], []
        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < cfg.min_slope:
                    continue
                if slope < 0:
                    lx += [x1, x2]; ly += [y1, y2]
                else:
                    rx += [x1, x2]; ry += [y1, y2]

        min_y, max_y = int(h * cfg.horizon), h

        def fit(xs, ys):
            if len(xs) < 2:
                return None
            p = np.poly1d(np.polyfit(ys, xs, 1))
            return (int(p(max_y)), max_y, int(p(min_y)), min_y)

        left  = fit(lx, ly)
        right = fit(rx, ry)

        if left:  self._left_hist.append(left)
        if right: self._right_hist.append(right)

        def smooth(hist):
            if not hist: return None
            return tuple(int(v) for v in np.mean(hist, axis=0))

        return smooth(self._left_hist), smooth(self._right_hist)

    def _roi(self, img, h, w):
        horizon = self._cfg.horizon
        cx = w // 2
        verts = np.array([[
            (int(cx - w*0.475), h),
            (int(cx - w*0.10),  int(h*horizon)),
            (int(cx + w*0.10),  int(h*horizon)),
            (int(cx + w*0.475), h),
        ]], dtype=np.int32)
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, verts, 255)
        return cv2.bitwise_and(img, mask)

    def _analyze(self, w, left, right) -> LaneState:
        cx = w // 2
        lx = left[0]  if left  else None
        rx = right[0] if right else None

        state = LaneState(left_detected=left is not None,
                          right_detected=right is not None)

        if lx is None and rx is None:
            state.zone = "UNKNOWN"
            return state

        if lx is not None and rx is not None:
            lane_w = rx - lx
            state.lane_width_px = lane_w
            state.lateral   = (cx - lx) / lane_w if lane_w > 0 else 0.5
            state.offset_px = cx - (lx + rx) // 2
        elif lx is not None:
            state.lateral   = 0.0
            state.offset_px = cx - lx
        else:
            state.lateral   = 1.0
            state.offset_px = cx - rx

        lat = state.lateral
        thr = self._cfg.center_threshold
        if lat < thr:
            state.zone = "LEFT"
        elif lat > 1.0 - thr:
            state.zone = "RIGHT"
        else:
            state.zone = "CENTER"

        return state