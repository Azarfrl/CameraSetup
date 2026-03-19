import cv2 as cv
import threading
import time
import numpy as np
from picamera2 import Picamera2

class CamManage:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        self.frame = None
        self.isRunning = False
        self.RCP = threading.Lock()
        self.fps_start_time = time.time()
        self.fps_counter = 0
        self.fps_value = 0.0
        self.src_points = np.float32([
            [100, 180],
            [540, 180],
            [620, 450],
            [20,  450]
        ])
        self.dst_points = np.float32([
            [0, 0],
            [320, 0],     
            [320, 240],
            [0, 240]
        ])
        self.M = cv.getPerspectiveTransform(self.src_points, self.dst_points)

    def start(self):
        self.isRunning = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        while self.isRunning:
            array = self.picam2.capture_array()
            frame = cv.cvtColor(array, cv.COLOR_RGB2BGR)
            frame = cv.flip(frame, 1)
            with self.RCP:
                self.frame = frame
            self.fps_counter += 1
            if time.time() - self.fps_start_time >= 1.0:
                self.fps_value = self.fps_counter
                self.fps_counter = 0
                self.fps_start_time = time.time()

    def read(self):
        with self.RCP:
            return self.frame.copy() if self.frame is not None else None
    def stop(self):
        self.isRunning = False
        if hasattr(self, 'thread'):
            self.thread.join()
        self.picam2.stop()
        self.picam2.close()

manager = CamManage()
manager.start()

show_birds_eye = True
print("Line Detection + Bird's Eye View (optimized)")
print("Press 'b' to toggle Birds Eye | 'q' or 'l' to quit")

while True:
    frame = manager.read()
    if frame is None:
        continue
      
    birds_eye = cv.warpPerspective(frame, manager.M, (320, 240))
    gray = cv.cvtColor(birds_eye, cv.COLOR_BGR2GRAY)
    _, binary = cv.threshold(gray, 100, 255, cv.THRESH_BINARY_INV)
    binary = cv.dilate(binary, None, iterations=2)
    contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    line_center_x = 160   
    error = 0
    
    if contours:
        largest = max(contours, key=cv.contourArea)
        if cv.contourArea(largest) > 300:
            x, y, w, h = cv.boundingRect(largest)
            line_center_x = x + w // 2
            cv.rectangle(birds_eye, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.circle(birds_eye, (line_center_x, y + h//2), 8, (0, 255, 0), -1)

    error = line_center_x - 160
    cv.putText(frame, f"Error: {error}", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    cv.putText(frame, f"FPS: {manager.fps_value:.1f}", (10, 80), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    pts = manager.src_points.astype(np.int32)
    cv.polylines(frame, [pts.reshape((-1,1,2))], True, (255, 0, 0), 3)
    cv.imshow('Raw + Trapezoid', frame)
    if show_birds_eye:
        cv.imshow('Birds Eye View', birds_eye)
        cv.imshow('Binary', binary)
    key = cv.waitKey(1) & 0xFF
    if key == ord('q') or key == ord('l'):
        break
    elif key == ord('b'):
        show_birds_eye = not show_birds_eye

manager.stop()
cv.destroyAllWindows()
