"""
╔══════════════════════════════════════════════════════════════╗
║         LANE EVENTS  —  Módulo de salidas del detector       ║
║         Importa esto desde tu script de control              ║
╚══════════════════════════════════════════════════════════════╝

USO BÁSICO (en tu control):

    from lane_events import LaneDetector, LaneEvent

    def mi_callback(event):
        if event.type == LaneEvent.CROSSED_LEFT:
            # girar derecha para volver al carril
            pass

    detector = LaneDetector(source=0, on_event=mi_callback)
    detector.start()          # arranca en hilo separado
    ...
    detector.stop()

USO AVANZADO (polling desde tu loop):

    detector = LaneDetector(source=0)
    detector.start()

    while True:
        state  = detector.get_state()    # estado continuo actual
        events = detector.get_events()   # lista de eventos nuevos (vacía si no hay)
        ...
"""

import cv2
import numpy as np
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional, List


# ══════════════════════════════════════════════
# EVENTOS DISCRETOS
# ══════════════════════════════════════════════

class LaneEvent:
    """Tipos de evento que emite el detector."""
    CROSSED_LEFT   = "CROSSED_LEFT"    # Cruzó la línea izquierda (salió por la izq)
    CROSSED_RIGHT  = "CROSSED_RIGHT"   # Cruzó la línea derecha   (salió por la dcha)
    ZONE_CHANGED   = "ZONE_CHANGED"    # Cambió de zona: centro↔izq o centro↔dcha
    LANE_LOST      = "LANE_LOST"       # Se perdió detección de carril
    LANE_FOUND     = "LANE_FOUND"      # Carril recuperado tras pérdida


@dataclass
class Event:
    """Un evento con tipo, zona anterior/nueva y timestamp."""
    type:      str
    timestamp: float         = field(default_factory=time.time)
    from_zone: Optional[str] = None    # zona de origen  (ZONE_CHANGED)
    to_zone:   Optional[str] = None    # zona de destino (ZONE_CHANGED)
    lateral:   float         = 0.5     # posición 0.0–1.0 cuando ocurrió

    def __str__(self):
        t = time.strftime("%H:%M:%S", time.localtime(self.timestamp))
        if self.type == LaneEvent.ZONE_CHANGED:
            return f"[{t}] ZONE_CHANGED  {self.from_zone} → {self.to_zone}  (lat={self.lateral:.2f})"
        return f"[{t}] {self.type}  (lat={self.lateral:.2f})"


# ══════════════════════════════════════════════
# ESTADO CONTINUO
# ══════════════════════════════════════════════

@dataclass
class LaneState:
    """
    Estado completo del detector en cada frame.
    Tu control puede leerlo con detector.get_state()
    """
    zone:          str   = "UNKNOWN"   # "LEFT" | "CENTER" | "RIGHT" | "UNKNOWN"
    lateral:       float = 0.5         # 0.0=línea izq … 0.5=centro … 1.0=línea dcha
    offset_px:     int   = 0           # px que el centro del frame está desplazado del carril
    lane_width_px: Optional[int] = None
    left_detected: bool  = False
    right_detected: bool = False
    fps:           float = 0.0


# ══════════════════════════════════════════════
# PARÁMETROS INTERNOS
# ══════════════════════════════════════════════
_CANNY_LOW        = 50
_CANNY_HIGH       = 150
_HOUGH_RHO        = 2
_HOUGH_THETA      = np.pi / 180
_HOUGH_THRESHOLD  = 50
_HOUGH_MIN_LEN    = 40
_HOUGH_MAX_GAP    = 100
_MIN_SLOPE        = 0.4
_SMOOTHING        = 8
_HORIZON          = 0.58

CENTER_THRESHOLD  = 0.20   # ±20 % del carril → zona CENTER
CROSS_THRESHOLD   = 0.05   # más allá de este margen → cruce de línea
CROSS_COOLDOWN_F  = 30     # frames de enfriamiento entre cruces


# ══════════════════════════════════════════════
# DETECTOR PRINCIPAL
# ══════════════════════════════════════════════

class LaneDetector:
    """
    Detector de carril con salida de eventos para sistemas de control.

    Parámetros
    ----------
    source : int | str
        Índice de cámara (0, 1, …) o ruta a fichero de vídeo.
    on_event : callable, opcional
        Función que se llama INMEDIATAMENTE al generarse un evento.
        Firma: on_event(event: Event)
        Se ejecuta en el hilo del detector — mantenla rápida o
        usa la cola (get_events) si necesitas hacer trabajo pesado.
    show_window : bool
        Muestra ventana de visualización (útil para depuración).
    """

    def __init__(
        self,
        source: int | str = 0,
        on_event: Optional[Callable[[Event], None]] = None,
        show_window: bool = True,
    ):
        self._source      = source
        self._on_event    = on_event
        self._show_window = show_window

        self._state       = LaneState()
        self._event_queue: deque[Event] = deque(maxlen=100)
        self._lock        = threading.Lock()
        self._running     = False
        self._thread      = None

        # Suavizadores de línea
        self._left_hist  = deque(maxlen=_SMOOTHING)
        self._right_hist = deque(maxlen=_SMOOTHING)

        # Control de transiciones
        self._prev_zone      = "UNKNOWN"
        self._lane_was_lost  = False
        self._cross_cooldown = 0

    # ── API pública ────────────────────────────

    def start(self):
        """Arranca la captura y detección en un hilo de fondo."""
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Para el detector y libera la cámara."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def get_state(self) -> LaneState:
        """Devuelve una copia del estado actual (thread-safe)."""
        with self._lock:
            return LaneState(**self._state.__dict__)

    def get_events(self) -> List[Event]:
        """
        Devuelve todos los eventos nuevos desde la última llamada y vacía la cola.
        Llama esto desde tu loop de control.
        """
        with self._lock:
            events = list(self._event_queue)
            self._event_queue.clear()
        return events

    # ── Hilo interno ───────────────────────────

    def _loop(self):
        try:
            src = int(self._source)
        except (ValueError, TypeError):
            src = self._source

        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print(f"[LaneDetector] ERROR: no se puede abrir '{src}'")
            return

        fps_deque = deque(maxlen=30)
        prev_time = time.time()

        while self._running:
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            h, w = frame.shape[:2]

            # FPS
            now = time.time()
            fps_deque.append(1.0 / max(now - prev_time, 1e-6))
            prev_time = now
            fps = float(np.mean(fps_deque))

            # Detección
            left, right = self._detect(frame, h, w)

            # Análisis y emisión de eventos
            state = self._analyze(w, left, right, fps)

            with self._lock:
                self._state = state

            # Visualización
            if self._show_window:
                vis = self._render(frame, left, right, state, h, w)
                cv2.imshow("Lane Detector", vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    self._running = False

        cap.release()
        if self._show_window:
            cv2.destroyAllWindows()

    # ── Detección de líneas ────────────────────

    def _detect(self, frame, h, w):
        hls         = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        white_mask  = cv2.inRange(hls, (0, 190, 0),  (180, 255, 255))
        yellow_mask = cv2.inRange(hls, (15, 30, 80), (40, 220, 255))
        color_mask  = cv2.bitwise_or(white_mask, yellow_mask)

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        masked  = cv2.bitwise_and(gray, color_mask)
        blurred = cv2.GaussianBlur(masked, (7, 7), 0)
        edges   = cv2.Canny(blurred, _CANNY_LOW, _CANNY_HIGH)

        roi = self._roi(edges, h, w)

        lines = cv2.HoughLinesP(
            roi, _HOUGH_RHO, _HOUGH_THETA, _HOUGH_THRESHOLD,
            minLineLength=_HOUGH_MIN_LEN, maxLineGap=_HOUGH_MAX_GAP,
        )

        lx, ly, rx, ry = [], [], [], []
        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < _MIN_SLOPE:
                    continue
                if slope < 0:
                    lx += [x1, x2]; ly += [y1, y2]
                else:
                    rx += [x1, x2]; ry += [y1, y2]

        min_y, max_y = int(h * _HORIZON), h

        def fit(xs, ys):
            if len(xs) < 2:
                return None
            p = np.poly1d(np.polyfit(ys, xs, 1))
            return (int(p(max_y)), max_y, int(p(min_y)), min_y)

        left  = fit(lx, ly)
        right = fit(rx, ry)

        # Suavizado
        if left:  self._left_hist.append(left)
        if right: self._right_hist.append(right)

        def smooth(hist):
            if not hist: return None
            return tuple(int(v) for v in np.mean(hist, axis=0))

        return smooth(self._left_hist), smooth(self._right_hist)

    def _roi(self, img, h, w):
        cx = w // 2
        verts = np.array([[
            (int(cx - w*0.475), h),
            (int(cx - w*0.10),  int(h*_HORIZON)),
            (int(cx + w*0.10),  int(h*_HORIZON)),
            (int(cx + w*0.475), h),
        ]], dtype=np.int32)
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, verts, 255)
        return cv2.bitwise_and(img, mask)

    # ── Análisis y emisión de eventos ──────────

    def _analyze(self, w, left, right, fps) -> LaneState:
        cx     = w // 2
        lx     = left[0]  if left  else None
        rx     = right[0] if right else None

        state  = LaneState(fps=fps,
                           left_detected=left is not None,
                           right_detected=right is not None)

        # ── Sin carril ──
        if lx is None and rx is None:
            state.zone = "UNKNOWN"
            if not self._lane_was_lost:
                self._emit(Event(LaneEvent.LANE_LOST, lateral=0.5))
                self._lane_was_lost = True
            return state

        # Carril recuperado
        if self._lane_was_lost:
            self._emit(Event(LaneEvent.LANE_FOUND, lateral=0.5))
            self._lane_was_lost = False

        # ── Calcular lateral y offset ──
        if lx is not None and rx is not None:
            lane_w        = rx - lx
            state.lane_width_px = lane_w
            state.lateral = (cx - lx) / lane_w if lane_w > 0 else 0.5
            state.offset_px = cx - (lx + rx) // 2
        elif lx is not None:
            state.lateral   = 0.0
            state.offset_px = cx - lx
        else:
            state.lateral   = 1.0
            state.offset_px = cx - rx

        lat = state.lateral

        # ── Zona actual ──
        if lat < CENTER_THRESHOLD:
            zone = "LEFT"
        elif lat > 1.0 - CENTER_THRESHOLD:
            zone = "RIGHT"
        else:
            zone = "CENTER"
        state.zone = zone

        # ── Evento ZONE_CHANGED ──
        if zone != self._prev_zone and self._prev_zone != "UNKNOWN":
            self._emit(Event(
                LaneEvent.ZONE_CHANGED,
                from_zone=self._prev_zone,
                to_zone=zone,
                lateral=lat,
            ))
        self._prev_zone = zone

        # ── Evento CROSSED (salió del carril) ──
        if self._cross_cooldown > 0:
            self._cross_cooldown -= 1
        else:
            if lat <= CROSS_THRESHOLD:
                self._emit(Event(LaneEvent.CROSSED_LEFT, lateral=lat))
                self._cross_cooldown = CROSS_COOLDOWN_F
            elif lat >= 1.0 - CROSS_THRESHOLD:
                self._emit(Event(LaneEvent.CROSSED_RIGHT, lateral=lat))
                self._cross_cooldown = CROSS_COOLDOWN_F

        return state

    def _emit(self, event: Event):
        with self._lock:
            self._event_queue.append(event)
        if self._on_event:
            self._on_event(event)

    # ── Renderizado ────────────────────────────

    def _render(self, frame, left, right, state: LaneState, h, w):
        out = frame.copy()

        # Polígono carril
        if left and right:
            pts = np.array([
                [left[0],  left[1]],  [left[2],  left[3]],
                [right[2], right[3]], [right[0], right[1]],
            ], np.int32)
            ov = out.copy()
            cv2.fillPoly(ov, [pts], (0, 180, 0))
            cv2.addWeighted(ov, 0.25, out, 0.75, 0, out)

        if left:
            cv2.line(out, (left[0], left[1]),   (left[2], left[3]),   (255,100,0), 4, cv2.LINE_AA)
        if right:
            cv2.line(out, (right[0], right[1]), (right[2], right[3]), (0,200,255), 4, cv2.LINE_AA)

        # HUD
        zone_color = {"CENTER": (0,230,0), "LEFT": (60,100,255),
                      "RIGHT": (255,180,0), "UNKNOWN": (180,180,180)}
        color = zone_color.get(state.zone, (255,255,255))

        pane = out.copy()
        cv2.rectangle(pane, (8,8), (300, 110), (0,0,0), -1)
        cv2.addWeighted(pane, 0.5, out, 0.5, 0, out)

        cv2.putText(out, f"FPS: {state.fps:.0f}",
                    (15,32),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(out, f"Zona: {state.zone}",
                    (15,60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(out, f"Lateral: {state.lateral:.2f}  Offset: {state.offset_px:+d}px",
                    (15,88),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(out, f"L:{int(state.left_detected)} R:{int(state.right_detected)}",
                    (15,106), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150,150,150), 1)

        return out


# ══════════════════════════════════════════════
# DEMO STANDALONE
# ══════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0")
    args = parser.parse_args()

    try:
        src = int(args.source)
    except ValueError:
        src = args.source

    print("╔══════════════════════════════════════╗")
    print("║  LANE EVENTS — demo de salidas       ║")
    print("╚══════════════════════════════════════╝")
    print("Eventos que se emitirán:")
    print(f"  {LaneEvent.CROSSED_LEFT:<16} → cruzó línea izquierda")
    print(f"  {LaneEvent.CROSSED_RIGHT:<16} → cruzó línea derecha")
    print(f"  {LaneEvent.ZONE_CHANGED:<16} → cambió de zona")
    print(f"  {LaneEvent.LANE_LOST:<16} → perdió el carril")
    print(f"  {LaneEvent.LANE_FOUND:<16} → recuperó el carril")
    print("\nQ / ESC para salir\n")

    def on_event(ev: Event):
        # Aquí recibirías el evento en tu sistema de control
        print(f"  ▶ {ev}")

    det = LaneDetector(source=src, on_event=on_event, show_window=True)
    det.start()

    try:
        while det._running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        det.stop()
        print("\n[Detector detenido]")