# # #!/usr/bin/env python3
# # """
# # Pillow/numpy vision pipeline -- replaces the old cv2-based vision.py.
# # cv2 is heavy and often not installable on ev3dev's ARM image, so this does
# # the same job (find the black tray, crop to it, classify each of the 12
# # cells by color) with PIL + numpy only.

# # Same pipeline as the original cv2 version:
# #     1. threshold to find the black tray frame     (cv2.inRange)
# #     2. find its bounding box                       (cv2.findContours +
# #                                                      boundingRect -> here:
# #                                                      mask -> PIL Image ->
# #                                                      getbbox())
# #     3. crop to that box                            (roi)
# #     4. split into a 4x3 grid
# #     5. classify each cell by counting matching-color pixels
# #        (cv2 used per-color contours + moments -> centroid -> cell; here
# #        it's simple per-cell pixel counting instead, since there's no
# #        lightweight moments/centroid equivalent without cv2 -- plenty
# #        robust for a fixed, known 4x3 layout)

# # -----------------------------------------------------------------------------
# # CALIBRATION
# # -----------------------------------------------------------------------------
# # BLACK_HSV_RANGE and COLOR_RANGES below are transplanted from the working
# # cv2 script's calibrated numbers (cv2 Hue is 0-179, PIL Hue is 0-255, so
# # Hue bounds are rescaled by 255/179; Saturation/Value are already 0-255 in
# # both, so those carry over unchanged).

# # If detection looks wrong on real frames, use sample_swatch.py (separate
# # file) to pull real HSV stats from your own photos and update the ranges
# # below, then check with `python3 vision.py <photo.jpg> --debug`, which
# # saves an annotated overlay so you can visually confirm the bounding box
# # and per-cell labels are actually right instead of just trusting the
# # printed grid.
# # -----------------------------------------------------------------------------
# # """

# # import io
# # import sys
# # import threading

# # import numpy as np
# # from PIL import Image, ImageFilter

# # from worker import StoppableThread

# # WHITE = 0
# # GREEN = 8
# # YELLOW = 7
# # BLUE = 6
# # UNKNOWN = 99

# # COLOR_NAMES = {WHITE: "WHITE", GREEN: "GREEN", YELLOW: "YELLOW",
# #                BLUE: "BLUE", UNKNOWN: "UNKNOWN"}

# # GRID_ROWS = 4
# # GRID_COLS = 3

# # # Resize every frame down to this before processing -- keeps per-cell pixel
# # # counting fast enough for a tight thread loop on ev3dev's CPU.
# # PROC_SIZE = (160, 120)

# # # Transplanted from the working cv2 script (Hue rescaled 0-179 -> 0-255).
# # BLACK_HSV_RANGE = (np.array([78, 75, 10]), np.array([135, 225, 100]))

# # COLOR_RANGES = [
# #     (GREEN,  np.array([87, 138,  61]), np.array([107, 196, 109])),
# #     (YELLOW, np.array([58, 164, 149]), np.array([64,  204, 205])),
# #     (BLUE,   np.array([127, 136,  60]), np.array([148, 213, 113])),
# #     (WHITE,  np.array([107,  77, 151]), np.array([114, 135, 197])),
# # ]

# # MIN_CELL_PIXELS = 15      # below this count, a cell is classified UNKNOWN
# # MAX_BBOX_FRACTION = 0.9   # reject a bbox covering this much of the frame or
# #                           # more -- almost certainly noise/background, not
# #                           # an actual tray


# # # =============================================================================
# # # Core detection
# # # =============================================================================

# # def _find_tray_bbox(hsv):
# #     """
# #     Black-frame detection using the full 3-channel HSV range, same idea
# #     as cv2.inRange + findContours + boundingRect, done with numpy + PIL.
# #     Returns (left, top, right, bottom), or None if nothing usable found.
# #     """
# #     lower, upper = BLACK_HSV_RANGE
# #     black_mask = np.all((hsv >= lower) & (hsv <= upper), axis=2).astype(np.uint8) * 255
# #     mask_img = Image.fromarray(black_mask).filter(ImageFilter.MedianFilter(3))
# #     bbox = mask_img.getbbox()

# #     if bbox is None:
# #         return None

# #     left, top, right, bottom = bbox
# #     frame_area = hsv.shape[0] * hsv.shape[1]
# #     bbox_area = (right - left) * (bottom - top)
# #     if bbox_area >= MAX_BBOX_FRACTION * frame_area:
# #         return None  # almost the whole frame matched -- not a real tray

# #     return bbox


# # def _count_pixels(hsv_roi, lower, upper):
# #     mask = np.all((hsv_roi >= lower) & (hsv_roi <= upper), axis=2)
# #     return int(np.count_nonzero(mask))


# # def _classify_cell(hsv_cell):
# #     best_tag = UNKNOWN
# #     best_count = 0
# #     for tag, lower, upper in COLOR_RANGES:
# #         n = _count_pixels(hsv_cell, lower, upper)
# #         if n > best_count:
# #             best_count = n
# #             best_tag = tag
# #     return best_tag if best_count >= MIN_CELL_PIXELS else UNKNOWN


# # def detect_grid_from_image(image):
# #     """image: a PIL.Image (any mode). Returns a GRID_ROWS x GRID_COLS grid
# #     of color tags."""
# #     grid = [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]

# #     img = image.convert("RGB").resize(PROC_SIZE)
# #     hsv = np.array(img.convert("HSV")).astype(int)

# #     bbox = _find_tray_bbox(hsv)
# #     if bbox is None:
# #         return grid

# #     left, top, right, bottom = bbox
# #     if right - left < 10 or bottom - top < 10:
# #         return grid  # bbox too small to be a real tray -- probably noise

# #     roi = hsv[top:bottom, left:right]
# #     h, w = roi.shape[0], roi.shape[1]
# #     cell_h = max(h // GRID_ROWS, 1)
# #     cell_w = max(w // GRID_COLS, 1)

# #     for r in range(GRID_ROWS):
# #         for c in range(GRID_COLS):
# #             y1, y2 = r * cell_h, (r + 1) * cell_h
# #             x1, x2 = c * cell_w, (c + 1) * cell_w
# #             cell = roi[y1:y2, x1:x2]

# #             # ignore outer 20% of each cell to dodge frame/edge bleed
# #             ch, cw = cell.shape[0], cell.shape[1]
# #             cell = cell[ch // 5: 4 * ch // 5, cw // 5: 4 * cw // 5]

# #             if cell.size == 0:
# #                 continue
# #             grid[r][c] = _classify_cell(cell)

# #     return grid


# # def detect_grid_from_jpeg_bytes(jpeg_bytes):
# #     try:
# #         image = Image.open(io.BytesIO(jpeg_bytes))
# #     except Exception:
# #         return [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
# #     return detect_grid_from_image(image)


# # # =============================================================================
# # # Threaded version -- drop-in replacement for the old cv2 Vision thread
# # # =============================================================================

# # class Vision(StoppableThread):
# #     """Give it a `camera` object with get_image() returning JPEG bytes (or
# #     None); keeps a thread-safe color grid updated in the background."""

# #     poll_interval = 0.01

# #     def __init__(self, camera):
# #         super().__init__()
# #         self.camera = camera
# #         self.lock = threading.Lock()
# #         self.color_grid = [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
# #         self._last_image = None

# #     def loop(self):
# #         jpeg = self.camera.get_image()
# #         if jpeg is None or jpeg == self._last_image:
# #             return
# #         self._last_image = jpeg
# #         grid = detect_grid_from_jpeg_bytes(jpeg)
# #         with self.lock:
# #             self.color_grid = grid

# #     def get_grid(self):
# #         with self.lock:
# #             return [row[:] for row in self.color_grid]


# # # =============================================================================
# # # CLI: offline test/calibration against a saved image -- run on your PC,
# # # not on the EV3.
# # # =============================================================================

# # def _diagnose(path):
# #     img = Image.open(path).convert("RGB").resize(PROC_SIZE)
# #     hsv = np.array(img.convert("HSV")).astype(int)

# #     bbox = _find_tray_bbox(hsv)
# #     lower, upper = BLACK_HSV_RANGE
# #     black_count = np.count_nonzero(np.all((hsv >= lower) & (hsv <= upper), axis=2))
# #     print("BLACK_HSV_RANGE matched pixels: {}  bbox: {}".format(black_count, bbox))

# #     if bbox:
# #         left, top, right, bottom = bbox
# #         roi = hsv[top:bottom, left:right]
# #         print("bbox size: {}x{}".format(right - left, bottom - top))
# #         for tag, lo, up in COLOR_RANGES:
# #             n = _count_pixels(roi, lo, up)
# #             print("  {:8s} matched pixels in bbox: {}".format(COLOR_NAMES[tag], n))
# #     else:
# #         print("No usable bbox -- either nothing matched BLACK_HSV_RANGE, or "
# #               "the match covered almost the whole frame (rejected as noise).")


# # def _debug_draw(path, out_path="debug_overlay.png"):
# #     from PIL import ImageDraw

# #     orig = Image.open(path).convert("RGB")
# #     proc = orig.resize(PROC_SIZE)
# #     hsv = np.array(proc.convert("HSV")).astype(int)

# #     bbox = _find_tray_bbox(hsv)
# #     grid = detect_grid_from_image(orig)

# #     scale = 4
# #     dbg = proc.resize((PROC_SIZE[0] * scale, PROC_SIZE[1] * scale), Image.NEAREST)
# #     draw = ImageDraw.Draw(dbg)

# #     if bbox:
# #         left, top, right, bottom = [c * scale for c in bbox]
# #         draw.rectangle([left, top, right, bottom], outline=(255, 0, 0), width=3)

# #         w, h = right - left, bottom - top
# #         cell_w, cell_h = w / GRID_COLS, h / GRID_ROWS
# #         for r in range(GRID_ROWS):
# #             for c in range(GRID_COLS):
# #                 x1 = left + c * cell_w
# #                 y1 = top + r * cell_h
# #                 x2 = left + (c + 1) * cell_w
# #                 y2 = top + (r + 1) * cell_h
# #                 draw.rectangle([x1, y1, x2, y2], outline=(255, 255, 0), width=1)
# #                 draw.text((x1 + 4, y1 + 4), COLOR_NAMES[grid[r][c]], fill=(255, 0, 0))

# #     dbg.save(out_path)
# #     print("Saved debug overlay to:", out_path)
# #     print()
# #     for row in grid:
# #         print([COLOR_NAMES[t] for t in row])


# # if __name__ == "__main__":
# #     if len(sys.argv) < 2:
# #         print("Usage:")
# #         print("  python3 vision.py <image.jpg>              # print grid")
# #         print("  python3 vision.py <image.jpg> --diagnose   # threshold stats")
# #         print("  python3 vision.py <image.jpg> --debug      # save annotated overlay image")
# #         sys.exit(1)

# #     path = sys.argv[1]
# #     if "--diagnose" in sys.argv:
# #         _diagnose(path)
# #         sys.exit(0)
# #     if "--debug" in sys.argv:
# #         _debug_draw(path)
# #         sys.exit(0)

# #     image = Image.open(path)
# #     grid = detect_grid_from_image(image)
# #     print()
# #     for row in grid:
# #         print([COLOR_NAMES[t] for t in row])
# #!/usr/bin/env python3

# import io
# import threading
# import numpy as np
# from PIL import Image, ImageFilter
# from worker import StoppableThread

# # Global Constants
# WHITE = 0
# GREEN = 8
# YELLOW = 7
# BLUE = 6
# UNKNOWN = 99

# COLOR_NAMES = {WHITE: "WHITE", GREEN: "GREEN", YELLOW: "YELLOW", BLUE: "BLUE", UNKNOWN: "UNKNOWN"}
# GRID_ROWS = 4
# GRID_COLS = 3
# PROC_SIZE = (160, 120)

# BLACK_HSV_RANGE = (np.array([78, 75, 10]), np.array([135, 225, 100]))
# COLOR_RANGES = [
#     (GREEN,  np.array([87, 138,  61]), np.array([107, 196, 109])),
#     (YELLOW, np.array([58, 164, 149]), np.array([64,  204, 205])),
#     (BLUE,   np.array([127, 136,  60]), np.array([148, 213, 113])),
#     (WHITE,  np.array([107,  77, 151]), np.array([114, 135, 197])),
# ]

# MIN_CELL_PIXELS = 15
# MAX_BBOX_FRACTION = 0.9


# class TrayVision(StoppableThread):
#     """
#     All-in-one vision pipeline class that safely handles background polling
#     and automatically logs images to disk for structural validation.
#     """
#     # FIX: Overriding poll_time to match StoppableThread's internal looping interval variable name
#     poll_time = 0.05  # Increased to 50ms to prevent heavy image loops from choking the EV3 CPU

#     def __init__(self, camera, debug_save_path="/home/robot/claude/debug_capture.jpg"):
#         super(TrayVision, self).__init__()  # Python 3.5 compatible explicit parent assignment pass
#         self.camera = camera
#         self.lock = threading.Lock()
#         self.color_grid = [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
#         self._last_image = None
#         self.debug_save_path = debug_save_path

#     def loop(self):
#         """Internal worker method running in the background thread."""
#         jpeg = self.camera.get_image()
#         if jpeg is None or jpeg == self._last_image:
#             return
#         self._last_image = jpeg
        
#         # --- VISUAL VERIFICATION CODE ---
#         try:
#             with open(self.debug_save_path, "wb") as f:
#                 f.write(jpeg)
#         except IOError as e:
#             # FIX: Replaced f-string with Python 3.5 safe format structure
#             print("Vision Thread Warning: Could not write debug image: {}".format(e))
#         # ------------------------------------

#         # Core image array classification processing
#         grid = self._detect_grid_from_jpeg_bytes(jpeg)
        
#         with self.lock:
#             self.color_grid = grid

#     def get_grid(self):
#         """Returns a snapshot deep-copy of the latest 4x3 color grid layout."""
#         with self.lock:
#             return [row[:] for row in self.color_grid]

#     def _detect_grid_from_jpeg_bytes(self, jpeg_bytes):
#         try:
#             image = Image.open(io.BytesIO(jpeg_bytes))
#             return self._detect_grid_from_image(image)
#         except Exception:
#             return [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]

#     def _detect_grid_from_image(self, image):
#         grid = [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
#         img = image.convert("RGB").resize(PROC_SIZE)
#         hsv = np.array(img.convert("HSV")).astype(int)

#         bbox = self._find_tray_bbox(hsv)
#         if bbox is None:
#             return grid

#         left, top, right, bottom = bbox
#         if right - left < 10 or bottom - top < 10:
#             return grid

#         roi = hsv[top:bottom, left:right]
#         h, w = roi.shape[0], roi.shape[1]
#         cell_h, cell_w = max(h // GRID_ROWS, 1), max(w // GRID_COLS, 1)

#         for r in range(GRID_ROWS):
#             for c in range(GRID_COLS):
#                 y1, y2 = r * cell_h, (r + 1) * cell_h
#                 x1, x2 = c * cell_w, (c + 1) * cell_w
#                 cell = roi[y1:y2, x1:x2]

#                 ch, cw = cell.shape[0], cell.shape[1]
#                 cell = cell[ch // 5: 4 * ch // 5, cw // 5: 4 * cw // 5]
#                 if cell.size == 0:
#                     continue
#                 grid[r][c] = self._classify_cell(cell)
#         return grid

#     def _find_tray_bbox(self, hsv):
#         lower, upper = BLACK_HSV_RANGE
#         black_mask = np.all((hsv >= lower) & (hsv <= upper), axis=2).astype(np.uint8) * 255
#         mask_img = Image.fromarray(black_mask).filter(ImageFilter.MedianFilter(3))
#         bbox = mask_img.getbbox()
#         if bbox is None:
#             return None

#         left, top, right, bottom = bbox
#         frame_area = hsv.shape[0] * hsv.shape[1]
#         bbox_area = (right - left) * (bottom - top)
#         if bbox_area >= MAX_BBOX_FRACTION * frame_area:
#             return None
#         return bbox

#     def _classify_cell(self, hsv_cell):
#         best_tag, best_count = UNKNOWN, 0
#         for tag, lower, upper in COLOR_RANGES:
#             mask = np.all((hsv_cell >= lower) & (hsv_cell <= upper), axis=2)
#             n = int(np.count_nonzero(mask))
#             if n > best_count:
#                 best_count, best_tag = n, tag
#         return best_tag if best_count >= MIN_CELL_PIXELS else UNKNOWN
    
#     def process_snapshot(self,robot_instance=None):
#         """
#         Captures a single image from the camera, processes the grid once,
#         updates internal memory, and returns the grid layout immediately.
#         """
#         jpeg = self.camera.get_image()
#         if jpeg is None:
#             return [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
#         if robot_instance:
#             robot_instance.image_captured_raw = True
#         # Optional visual verification debug dump save
#         try:
#             with open(self.debug_save_path, "wb") as f:
#                 f.write(jpeg)
#         except IOError as e:
#             print("Vision Warning: Could not write debug image: {}".format(e))

#         # Run detection once on this single frame
#         grid = self._detect_grid_from_jpeg_bytes(jpeg)
        
#         with self.lock:
#             self.color_grid = grid
#         print("[VISION STATUS] Matrix cell color extraction logic completed.")   
#         return grid



#!/usr/bin/env python3
import io
import threading
import numpy as np
from PIL import Image, ImageFilter
from worker import StoppableThread

# Color Definitions
WHITE = 0
GREEN = 8
YELLOW = 7
BLUE = 6
UNKNOWN = 99

COLOR_NAMES = {WHITE: "WHITE", GREEN: "GREEN", YELLOW: "YELLOW", BLUE: "BLUE", UNKNOWN: "UNKNOWN"}
GRID_ROWS = 4
GRID_COLS = 3
PROC_SIZE = (160, 120)

# Black Frame Detection (PIL HSV: H 0-255, S 0-255, V 0-255)
BLACK_HSV_RANGE = (np.array([0, 0, 0]), np.array([255, 255, 75]))

# Calibrated Ranges for PIL HSV Scale (0-255)
COLOR_RANGES = [
    # (Color_ID, Lower [H, S, V], Upper [H, S, V])
    (GREEN,  np.array([45,  70,  40]), np.array([115, 255, 200])),
    (YELLOW, np.array([22, 110, 120]), np.array([48,  255, 255])),
    (BLUE,   np.array([130,  80,  40]), np.array([180, 255, 220])),
]

MIN_CELL_PIXELS = 10
MAX_BBOX_FRACTION = 0.95

class TrayVision(StoppableThread):
    poll_time = 0.05

    def __init__(self, camera, debug_save_path="/home/robot/claude/debug_capture.jpg"):
        super(TrayVision, self).__init__()
        self.camera = camera
        self.lock = threading.Lock()
        self.color_grid = [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
        self._last_image = None
        self.debug_save_path = debug_save_path

    def loop(self):
        jpeg = self.camera.get_image()
        if jpeg is None or jpeg == self._last_image:
            return
        self._last_image = jpeg
        
        grid = self._detect_grid_from_jpeg_bytes(jpeg)
        with self.lock:
            self.color_grid = grid

    def get_grid(self):
        with self.lock:
            return [row[:] for row in self.color_grid]

    def _detect_grid_from_jpeg_bytes(self, jpeg_bytes):
        try:
            image = Image.open(io.BytesIO(jpeg_bytes))
            return self._detect_grid_from_image(image)
        except Exception:
            return [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]

    def _detect_grid_from_image(self, image):
        grid = [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
        img = image.convert("RGB").resize(PROC_SIZE)
        hsv = np.array(img.convert("HSV")).astype(int)

        bbox = self._find_tray_bbox(hsv)
        if bbox is None:
            return grid

        left, top, right, bottom = bbox
        width = right - left
        height = bottom - top

        if width < 15 or height < 15:
            return grid

        roi = hsv[top:bottom, left:right]
        
        # Perspective-adjusted row ratios (top rows are smaller due to angle)
        row_ratios = [0.0, 0.22, 0.46, 0.71, 1.0]

        for r in range(GRID_ROWS):
            y1 = int(height * row_ratios[r])
            y2 = int(height * row_ratios[r+1])
            
            for c in range(GRID_COLS):
                cell_w = width // GRID_COLS
                x1 = c * cell_w
                x2 = (c + 1) * cell_w
                
                cell = roi[y1:y2, x1:x2]

                # Center crop 60% to ignore cell borders
                ch, cw = cell.shape[0], cell.shape[1]
                if ch < 3 or cw < 3:
                    continue
                cell_cropped = cell[ch // 5: 4 * ch // 5, cw // 5: 4 * cw // 5]

                grid[r][c] = self._classify_cell(cell_cropped)
                
        return grid

    def _find_tray_bbox(self, hsv):
        lower, upper = BLACK_HSV_RANGE
        black_mask = np.all((hsv >= lower) & (hsv <= upper), axis=2).astype(np.uint8) * 255
        mask_img = Image.fromarray(black_mask).filter(ImageFilter.MedianFilter(3))
        bbox = mask_img.getbbox()
        
        if bbox is None:
            return None

        left, top, right, bottom = bbox
        frame_area = hsv.shape[0] * hsv.shape[1]
        bbox_area = (right - left) * (bottom - top)
        
        if bbox_area >= MAX_BBOX_FRACTION * frame_area:
            return None
        return bbox

    def _classify_cell(self, hsv_cell):
        if hsv_cell.size == 0:
            return UNKNOWN

        # Check for WHITE tile (High brightness, low saturation)
        mean_s = np.mean(hsv_cell[:, :, 1])
        mean_v = np.mean(hsv_cell[:, :, 2])
        if mean_s < 55 and mean_v > 130:
            return WHITE

        best_tag, best_count = UNKNOWN, 0
        for tag, lower, upper in COLOR_RANGES:
            mask = np.all((hsv_cell >= lower) & (hsv_cell <= upper), axis=2)
            n = int(np.count_nonzero(mask))
            if n > best_count:
                best_count, best_tag = n, tag
                
        return best_tag if best_count >= MIN_CELL_PIXELS else UNKNOWN
    
    def process_snapshot(self, robot_instance=None):
        jpeg = self.camera.get_image()
        if jpeg is None:
            return [[UNKNOWN] * GRID_COLS for _ in range(GRID_ROWS)]
            
        if robot_instance:
            robot_instance.image_captured_raw = True

        try:
            with open(self.debug_save_path, "wb") as f:
                f.write(jpeg)
        except IOError as e:
            print("Vision Warning: Could not write debug image: {}".format(e))

        grid = self._detect_grid_from_jpeg_bytes(jpeg)
        with self.lock:
            self.color_grid = grid
        return grid