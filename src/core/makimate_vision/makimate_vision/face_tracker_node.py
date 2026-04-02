"""
face_tracker_node.py — Advanced Multi-Face Tracking & Attention Engine
=======================================================================
Replaces the original Haar Cascade detector with a full pipeline:

  1. MediaPipe FaceMesh   — robust detection + 468 landmarks (including lips)
  2. IoU Centroid Tracker — assigns persistent IDs across frames (no extra deps)
  3. Attention Scorer     — weighted score: Centerness + Size + Lifetime + Lips
  4. face_recognition     — optional identity matching against a local JSON db
  5. Calibration Service  — /maki/calibrate_face saves a new identity to the db

Subscriptions  (unchanged from original):
  /camera/image_raw   sensor_msgs/Image
  /maki/awake         std_msgs/Bool

Publications:
  /camera/face_image      sensor_msgs/Image        — annotated debug image
  /maki/largest_face_bbox std_msgs/Int32MultiArray  — highest-score face [x,y,w,h]
                                                      (kept for face_to_maki.py compat)
  /maki/face_tracks       makimate_interfaces/FaceTrackArray — ALL faces + scores

Service:
  /maki/calibrate_face    makimate_interfaces/srv/CalibrateFace
"""

import os
import json
import math
import collections
import time
from typing import Dict, List, Optional, Tuple

import urllib.request

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python.vision import FaceLandmarker, FaceLandmarkerOptions, RunningMode
import face_recognition

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Bool, String
from cv_bridge import CvBridge

from makimate_interfaces.msg import FaceTrack, FaceTrackArray
from makimate_interfaces.srv import CalibrateFace


# ---------------------------------------------------------------------------
# MediaPipe landmark indices for lip gap measurement
# Inner upper lip top and inner lower lip bottom (in FaceMesh 468-point model)
# ---------------------------------------------------------------------------
_LIP_UPPER_IDX = 13   # inner upper lip
_LIP_LOWER_IDX = 14   # inner lower lip


# ---------------------------------------------------------------------------
# TrackState — all mutable state for one tracked face
# ---------------------------------------------------------------------------
class TrackState:
    """Holds per-face tracking state between frames."""

    def __init__(self, track_id: int, bbox: Tuple[int, int, int, int]):
        self.track_id: int = track_id
        self.bbox: Tuple[int, int, int, int] = bbox           # (x, y, w, h)
        self.centroid: Tuple[int, int] = _centroid(bbox)
        self.frames_alive: int = 1                            # frames seen so far
        self.frames_missed: int = 0                           # consecutive missed frames
        self.name: str = "Unknown"
        # Rolling window of lip-gap pixel distances for lip-movement detection
        self.lip_dist_history: collections.deque = collections.deque(maxlen=10)
        # Cached score from last frame
        self.score: float = 0.0
        # Last known MediaPipe landmarks (List[NormalizedLandmark] or None)
        self.landmarks: Optional[object] = None
        # Smoothed centroid velocity (pixels/frame) for exit-direction detection
        self.velocity: Tuple[float, float] = (0.0, 0.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _centroid(bbox: Tuple[int, int, int, int]) -> Tuple[int, int]:
    x, y, w, h = bbox
    return (x + w // 2, y + h // 2)


def _iou(a: Tuple[int, int, int, int], b: Tuple[int, int, int, int]) -> float:
    """Intersection-over-Union of two (x, y, w, h) boxes."""
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix = max(ax, bx)
    iy = max(ay, by)
    ix2 = min(ax + aw, bx + bw)
    iy2 = min(ay + ah, by + bh)
    if ix2 <= ix or iy2 <= iy:
        return 0.0
    inter = (ix2 - ix) * (iy2 - iy)
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


# ---------------------------------------------------------------------------
# FaceTracker Node
# ---------------------------------------------------------------------------

class FaceTracker(Node):
    """
    Subscribes to a camera image topic, detects all faces via MediaPipe FaceMesh,
    tracks them with persistent IDs, scores each face, and publishes:
      - FaceTrackArray on /maki/face_tracks
      - Int32MultiArray on /maki/largest_face_bbox (best-score face, for face_to_maki)
      - Annotated Image on /camera/face_image
    """

    def __init__(self):
        super().__init__('face_tracker')

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('input_image_topic',        '/camera/image_raw')
        self.declare_parameter('output_image_topic',       '/camera/face_image')
        self.declare_parameter('largest_face_topic',       '/maki/largest_face_bbox')
        self.declare_parameter('face_tracks_topic',        '/maki/face_tracks')

        # Face recognition database path (stored in home dir so it survives builds)
        self.declare_parameter('face_db_path',
                               os.path.expanduser('~/.maki_faces/face_db.json'))

        # Run face_recognition every N frames (CPU-intensive on Pi)
        self.declare_parameter('recognition_interval',     10)

        # Attention score weights (must sum to 1.0 for interpretability)
        self.declare_parameter('weight_centerness',        0.35)
        self.declare_parameter('weight_size',              0.25)
        self.declare_parameter('weight_lifetime',          0.20)
        self.declare_parameter('weight_lips',              0.20)

        # Lifetime cap for L score normalisation
        self.declare_parameter('max_lifetime_frames',      150)

        # Lip variance parameters
        self.declare_parameter('lip_var_max',              50.0)    # px^2 -> M = 1.0
        self.declare_parameter('speaking_variance_threshold', 10.0) # px^2 -> is_speaking

        # Tracker parameters
        self.declare_parameter('iou_threshold',            0.3)
        self.declare_parameter('max_missed_frames',        10)

        # face_recognition match distance threshold (lower = stricter)
        self.declare_parameter('recognition_threshold',    0.55)

        # Debug window (kept from original; disabled by default on robot)
        self.declare_parameter('show_debug_window',        False)

        # Cache parameter values
        self.input_topic          = self.get_parameter('input_image_topic').value
        self.output_topic         = self.get_parameter('output_image_topic').value
        self.largest_face_topic   = self.get_parameter('largest_face_topic').value
        self.face_tracks_topic    = self.get_parameter('face_tracks_topic').value
        self.face_db_path         = self.get_parameter('face_db_path').value
        self.recognition_interval = int(self.get_parameter('recognition_interval').value)
        self.w_c                  = float(self.get_parameter('weight_centerness').value)
        self.w_s                  = float(self.get_parameter('weight_size').value)
        self.w_l                  = float(self.get_parameter('weight_lifetime').value)
        self.w_m                  = float(self.get_parameter('weight_lips').value)
        self.max_lifetime         = int(self.get_parameter('max_lifetime_frames').value)
        self.lip_var_max          = float(self.get_parameter('lip_var_max').value)
        self.speaking_threshold   = float(self.get_parameter('speaking_variance_threshold').value)
        self.iou_threshold        = float(self.get_parameter('iou_threshold').value)
        self.max_missed           = int(self.get_parameter('max_missed_frames').value)
        self.recog_threshold      = float(self.get_parameter('recognition_threshold').value)
        self.show_debug           = bool(self.get_parameter('show_debug_window').value)

        # ------------------------------------------------------------------ #
        # Awake flag — same behaviour as original node
        # ------------------------------------------------------------------ #
        self.awake = False
        _latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/maki/awake', self._on_awake, _latched_qos)
        self.get_logger().info("FaceTracker: listening to /maki/awake")

        # ------------------------------------------------------------------ #
        # MediaPipe FaceLandmarker (Tasks API — compatible with 0.10.x)
        # The model file is downloaded once to ~/.maki_faces/ if not present.
        # ------------------------------------------------------------------ #
        model_dir  = os.path.dirname(self.face_db_path)
        model_path = os.path.join(model_dir, 'face_landmarker.task')
        os.makedirs(model_dir, exist_ok=True)
        if not os.path.exists(model_path):
            self.get_logger().info(
                "FaceTracker: Downloading face_landmarker.task model (~30 MB)..."
            )
            url = (
                "https://storage.googleapis.com/mediapipe-models/"
                "face_landmarker/face_landmarker/float16/1/face_landmarker.task"
            )
            urllib.request.urlretrieve(url, model_path)
            self.get_logger().info(f"FaceTracker: Model saved to {model_path}")

        options = FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            num_faces=10,
            running_mode=RunningMode.IMAGE,
        )
        self.face_landmarker = FaceLandmarker.create_from_options(options)
        self.get_logger().info("FaceTracker: MediaPipe FaceLandmarker initialised.")

        # ------------------------------------------------------------------ #
        # Face recognition database
        # db format: { "Name": [[128-dim emb list], ...], ... }
        # ------------------------------------------------------------------ #
        self.face_db: Dict[str, List[np.ndarray]] = {}
        self._load_face_db()

        # ------------------------------------------------------------------ #
        # Tracker state
        # ------------------------------------------------------------------ #
        self.tracks: Dict[int, TrackState] = {}  # track_id -> TrackState
        self._next_id: int = 0
        self.frame_count: int = 0
        self._last_frame: Optional[np.ndarray] = None  # needed by calibration service
        self._fps: float = 0.0
        self._fps_last_time: float = time.monotonic()
        self._session_faces: Dict[str, List[np.ndarray]] = {}  # session-only, cleared on sleep
        self._last_detect_time: float = 0.0
        self._last_recognition_time: float = 0.0
        self._speaking_states: Dict[int, bool] = {}  # track_id -> last logged speaking state

        # ------------------------------------------------------------------ #
        # cv_bridge
        # ------------------------------------------------------------------ #
        self.bridge = CvBridge()

        # ------------------------------------------------------------------ #
        # QoS — BEST_EFFORT to avoid frame backlog (same as original)
        # ------------------------------------------------------------------ #
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------------------------------------------------ #
        # Publishers & Subscribers
        # ------------------------------------------------------------------ #
        self.sub = self.create_subscription(
            Image, self.input_topic, self._image_callback, qos
        )
        self.pub_image  = self.create_publisher(Image,            self.output_topic,        10)
        self.pub_bbox   = self.create_publisher(Int32MultiArray,  self.largest_face_topic,  10)
        self.pub_tracks = self.create_publisher(FaceTrackArray,   self.face_tracks_topic,   10)
        self._debug_pub = self.create_publisher(Image,            '/maki/debug_image',      10)

        # ------------------------------------------------------------------ #
        # Calibration service
        # ------------------------------------------------------------------ #
        self.create_service(
            CalibrateFace,
            '/maki/calibrate_face',
            self._calibrate_face_callback,
        )
        self.create_subscription(
            String,
            '/maki/calibration_name',
            self._on_calibration_name,
            10,
        )

        self.get_logger().info(
            f"FaceTracker initialised. "
            f"Input: {self.input_topic} | "
            f"Tracks: {self.face_tracks_topic} | "
            f"DB: {self.face_db_path}"
        )

    # ====================================================================== #
    # Awake callback
    # ====================================================================== #

    def _on_awake(self, msg: Bool):
        self.awake = bool(msg.data)
        self.get_logger().info(f"FaceTracker: awake = {self.awake}")
        if not self.awake:
            self._session_faces = {}
            self.get_logger().info('[Session] Cleared session faces on sleep.')

    # ====================================================================== #
    # Main image callback
    # ====================================================================== #

    def _image_callback(self, msg: Image):
        # Convert ROS image -> OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        self._last_frame = frame
        self.frame_count += 1
        h_img, w_img = frame.shape[:2]

        # Update FPS estimate (exponential moving average)
        now = time.monotonic()
        dt = now - self._fps_last_time
        self._fps_last_time = now
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)

        # ------------------------------------------------------------------
        # When asleep: publish "no face" on bbox topic and return.
        # Identical behaviour to the original node.
        # ------------------------------------------------------------------
        if not self.awake:
            bbox_msg = Int32MultiArray()
            bbox_msg.data = [-1, -1, -1, -1]
            self.pub_bbox.publish(bbox_msg)
            return

        # ------------------------------------------------------------------
        # Step 1: Detect faces + landmarks with MediaPipe FaceMesh
        # Throttle to ~10 fps — full MediaPipe inference at 30 fps pegs the
        # Pi CPU and starves other processes (camera feed, dashboard).
        # ------------------------------------------------------------------
        if now - self._last_detect_time < 0.067:
            return
        self._last_detect_time = now

        t0 = time.monotonic()
        detections = self._detect_faces(frame, w_img, h_img)
        t1 = time.monotonic()
        if t1 - t0 > 0.05:
            self.get_logger().warn(f'[TIMING] detect_faces: {(t1-t0)*1000:.0f}ms')

        # ------------------------------------------------------------------
        # Step 2: Update IoU tracker
        # ------------------------------------------------------------------
        self._update_tracker(detections, w_img, h_img)

        # ------------------------------------------------------------------
        # Step 3: Run face_recognition at most once every 3 seconds.
        # ------------------------------------------------------------------
        if now - self._last_recognition_time >= 3.0 and (self.face_db or self._session_faces):
            self._last_recognition_time = now
            t0 = time.monotonic()
            self._run_recognition(frame)
            t1 = time.monotonic()
            self.get_logger().warn(f'[TIMING] recognition: {(t1-t0)*1000:.0f}ms')

        # ------------------------------------------------------------------
        # Step 4: Score every active track
        # ------------------------------------------------------------------
        frame_cx  = w_img / 2.0
        frame_cy  = h_img / 2.0
        max_dist  = math.sqrt(frame_cx ** 2 + frame_cy ** 2)
        frame_area = float(w_img * h_img)

        scored_tracks: List[TrackState] = []
        for track in self.tracks.values():
            track.score = self._attention_score(
                track, frame_cx, frame_cy, max_dist, frame_area
            )
            scored_tracks.append(track)

        # Sort descending — best face is index 0
        scored_tracks.sort(key=lambda t: t.score, reverse=True)

        # ------------------------------------------------------------------
        # Active speaker: log speaking state changes and elevate the
        # confirmed-speaking face to index 0 so face_to_maki automatically
        # gazes toward the person who is talking.
        # "Confirmed speaking" = at least 5 lip-history samples + variance
        # above threshold (avoids reacting to a single noisy frame).
        # ------------------------------------------------------------------
        for track in scored_tracks:
            speaking_now = self._is_speaking(track)
            was_speaking = self._speaking_states.get(track.track_id, False)
            if speaking_now != was_speaking:
                self._speaking_states[track.track_id] = speaking_now
                self.get_logger().info(
                    f"[ActiveSpeaker] Track {track.track_id} ({track.name}) "
                    f"{'STARTED' if speaking_now else 'STOPPED'} speaking"
                )

        speaking_candidates = [
            t for t in scored_tracks
            if self._is_speaking(t) and len(t.lip_dist_history) >= 5
        ]
        if speaking_candidates and speaking_candidates[0] is not scored_tracks[0]:
            best_speaker = max(speaking_candidates, key=lambda t: t.score)
            scored_tracks.remove(best_speaker)
            scored_tracks.insert(0, best_speaker)
            self.get_logger().info(
                f"[ActiveSpeaker] Gaze → track {best_speaker.track_id} "
                f"({best_speaker.name}) — speaking"
            )

        # ------------------------------------------------------------------
        # Step 5: Publish FaceTrackArray (all faces)
        # ------------------------------------------------------------------
        track_array = FaceTrackArray()
        # Use the incoming image stamp if set; fall back to node clock.
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            track_array.header.stamp = self.get_clock().now().to_msg()
        else:
            track_array.header.stamp = msg.header.stamp
        track_array.header.frame_id = msg.header.frame_id

        for track in scored_tracks:
            ft = FaceTrack()
            ft.id               = track.track_id
            ft.name             = track.name
            ft.bbox             = list(track.bbox)   # [x, y, w, h]
            ft.confidence_score = float(track.score)
            ft.is_speaking      = self._is_speaking(track)
            track_array.faces.append(ft)

        self.pub_tracks.publish(track_array)

        # ------------------------------------------------------------------
        # Step 6: Publish best face on /maki/largest_face_bbox
        # Keeps face_to_maki.py working without any changes.
        # ------------------------------------------------------------------
        bbox_msg = Int32MultiArray()
        if scored_tracks:
            bbox_msg.data = list(scored_tracks[0].bbox)
        else:
            bbox_msg.data = [-1, -1, -1, -1]
        self.pub_bbox.publish(bbox_msg)

        # ------------------------------------------------------------------
        # Step 7: Annotate and publish debug image (only when debug is on)
        # Skipped by default — publishing two annotated 640x480 frames at
        # 30 fps floods DDS loopback (~54 MB/s) and starves other subscribers.
        # ------------------------------------------------------------------
        if self.show_debug:
            annotated = self._draw_annotations(frame, scored_tracks, w_img, h_img)
            try:
                cv2.imshow("MakiMate Debug", annotated)
                cv2.waitKey(1)
            except cv2.error:
                self.get_logger().warn(
                    "WARNING: Cannot open display. Set show_debug_window:=false.",
                    throttle_duration_sec=10,
                )
            try:
                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                out_msg.header = msg.header
                self.pub_image.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f"cv2_to_imgmsg failed: {e}")
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                self._debug_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"cv2_to_imgmsg (debug) failed: {e}")

    # ====================================================================== #
    # MediaPipe detection
    # ====================================================================== #

    def _detect_faces(
        self,
        frame: np.ndarray,
        w_img: int,
        h_img: int,
    ) -> List[Tuple[Tuple[int, int, int, int], object]]:
        """
        Run MediaPipe FaceMesh on the frame.

        Returns a list of (bbox, face_landmarks) where:
          bbox           = (x, y, w, h) in pixels
          face_landmarks = list of NormalizedLandmark for this face (468 points)
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        results  = self.face_landmarker.detect(mp_image)

        detections = []
        if not results.face_landmarks:
            return detections

        # results.face_landmarks is List[List[NormalizedLandmark]]
        for face_lm_list in results.face_landmarks:
            # Derive bounding box from the extents of all 468 landmarks
            xs = [lm.x * w_img for lm in face_lm_list]
            ys = [lm.y * h_img for lm in face_lm_list]
            x_min = int(min(xs))
            y_min = int(min(ys))
            x_max = int(max(xs))
            y_max = int(max(ys))
            w = x_max - x_min
            h = y_max - y_min
            if w > 0 and h > 0:
                detections.append(((x_min, y_min, w, h), face_lm_list))

        return detections

    # ====================================================================== #
    # IoU Tracker
    # ====================================================================== #

    def _update_tracker(
        self,
        detections: List[Tuple[Tuple, object]],
        w_img: int,
        h_img: int,
    ):
        """
        Match new detections to existing tracks by IoU (greedy, highest first).

        Matched tracks  → update bbox, centroid, frames_alive, lip history
        Unmatched tracks → increment frames_missed; delete if > max_missed_frames
        Unmatched dets  → create new TrackState with a fresh ID
        """
        det_bboxes    = [d[0] for d in detections]
        det_landmarks = [d[1] for d in detections]

        track_ids = list(self.tracks.keys())
        matched_track_indices = set()
        matched_det_indices   = set()

        # Build IoU matrix: rows = existing tracks, cols = new detections
        iou_matrix = [
            [_iou(self.tracks[tid].bbox, db) for db in det_bboxes]
            for tid in track_ids
        ]

        # Greedy matching: repeatedly pick the highest IoU pair
        while True:
            best_val = self.iou_threshold  # minimum acceptable IoU
            best_ti = best_di = -1
            for ti in range(len(track_ids)):
                if ti in matched_track_indices:
                    continue
                for di in range(len(det_bboxes)):
                    if di in matched_det_indices:
                        continue
                    if iou_matrix[ti][di] > best_val:
                        best_val = iou_matrix[ti][di]
                        best_ti  = ti
                        best_di  = di
            if best_ti == -1:
                break

            tid = track_ids[best_ti]
            matched_track_indices.add(best_ti)
            matched_det_indices.add(best_di)

            # Update matched track
            track = self.tracks[tid]
            old_centroid = track.centroid
            track.bbox          = det_bboxes[best_di]
            track.centroid      = _centroid(track.bbox)
            track.frames_alive += 1
            track.frames_missed = 0
            track.landmarks     = det_landmarks[best_di]

            # Update smoothed centroid velocity (pixels/frame)
            dx = float(track.centroid[0] - old_centroid[0])
            dy = float(track.centroid[1] - old_centroid[1])
            alpha_v = 0.35
            track.velocity = (
                alpha_v * dx + (1 - alpha_v) * track.velocity[0],
                alpha_v * dy + (1 - alpha_v) * track.velocity[1],
            )

            # Update lip distance history with this frame's measurement
            lip_d = self._lip_distance(det_landmarks[best_di], w_img, h_img)
            if lip_d is not None:
                track.lip_dist_history.append(lip_d)

        # Age unmatched existing tracks; remove stale ones
        dead_ids = []
        for ti, tid in enumerate(track_ids):
            if ti not in matched_track_indices:
                self.tracks[tid].frames_missed += 1
                if self.tracks[tid].frames_missed > self.max_missed:
                    dead_ids.append(tid)
        for tid in dead_ids:
            del self.tracks[tid]
            self._speaking_states.pop(tid, None)

        # Register new tracks for unmatched detections
        for di in range(len(detections)):
            if di not in matched_det_indices:
                new_track = TrackState(self._next_id, det_bboxes[di])
                new_track.landmarks = det_landmarks[di]
                lip_d = self._lip_distance(det_landmarks[di], w_img, h_img)
                if lip_d is not None:
                    new_track.lip_dist_history.append(lip_d)
                self.tracks[self._next_id] = new_track
                self._next_id += 1

    # ====================================================================== #
    # Lip gap measurement
    # ====================================================================== #

    def _lip_distance(
        self,
        face_landmarks,
        w_img: int,
        h_img: int,
    ) -> Optional[float]:
        """
        Pixel distance between inner upper lip (#13) and inner lower lip (#14).
        Returns None if landmarks are unavailable.
        """
        try:
            upper = face_landmarks[_LIP_UPPER_IDX]
            lower = face_landmarks[_LIP_LOWER_IDX]
            ux, uy = upper.x * w_img, upper.y * h_img
            lx, ly = lower.x * w_img, lower.y * h_img
            return math.sqrt((ux - lx) ** 2 + (uy - ly) ** 2)
        except (IndexError, AttributeError):
            return None

    def _is_speaking(self, track: TrackState) -> bool:
        """True when variance of the lip-gap rolling window exceeds the threshold."""
        if len(track.lip_dist_history) < 2:
            return False
        return float(np.var(list(track.lip_dist_history))) > self.speaking_threshold

    # ====================================================================== #
    # Attention scoring
    # ====================================================================== #

    def _attention_score(
        self,
        track: TrackState,
        frame_cx: float,
        frame_cy: float,
        max_dist: float,
        frame_area: float,
    ) -> float:
        """
        Weighted attention score clamped to [0.0, 1.0]:

          Score = W_c*C + W_s*S + W_l*L + W_m*M

        C — Centerness : 1 - (dist_to_frame_center / max_possible_dist)
        S — Size       : bbox_area / frame_area  (clamped [0,1])
        L — Lifetime   : min(frames_alive, max_lifetime) / max_lifetime
        M — Lip motion : normalised variance of lip-gap rolling window
        """
        x, y, w, h = track.bbox
        cx = x + w / 2.0
        cy = y + h / 2.0

        # C — centerness
        dist = math.sqrt((cx - frame_cx) ** 2 + (cy - frame_cy) ** 2)
        C = 1.0 - (dist / max_dist) if max_dist > 0 else 0.0
        C = max(0.0, min(1.0, C))

        # S — relative size
        S = (w * h) / frame_area if frame_area > 0 else 0.0
        S = max(0.0, min(1.0, S))

        # L — lifetime (how long this face has been consistently tracked)
        L = min(track.frames_alive, self.max_lifetime) / float(self.max_lifetime)

        # M — lip movement (speaking indicator)
        if len(track.lip_dist_history) >= 2:
            var = float(np.var(list(track.lip_dist_history)))
            M = min(var / self.lip_var_max, 1.0) if self.lip_var_max > 0 else 0.0
        else:
            M = 0.0

        score = self.w_c * C + self.w_s * S + self.w_l * L + self.w_m * M
        return float(max(0.0, min(1.0, score)))

    # ====================================================================== #
    # Face recognition
    # ====================================================================== #

    def _load_face_db(self):
        """Load face embeddings from JSON file into self.face_db."""
        if not os.path.exists(self.face_db_path):
            self.get_logger().info(
                f"FaceTracker: No face DB at {self.face_db_path}. "
                "Starting with empty database."
            )
            self.face_db = {}
            return
        try:
            with open(self.face_db_path, 'r') as f:
                raw = json.load(f)
            self.face_db = {
                name: [np.array(emb, dtype=np.float64) for emb in embs]
                for name, embs in raw.items()
            }
            self.get_logger().info(
                f"FaceTracker: Loaded {len(self.face_db)} identities from DB."
            )
        except Exception as e:
            self.get_logger().error(f"FaceTracker: Failed to load face DB: {e}")
            self.face_db = {}

    def _save_face_db(self):
        """Persist self.face_db to JSON (creates parent directory if needed)."""
        os.makedirs(os.path.dirname(self.face_db_path), exist_ok=True)
        try:
            raw = {
                name: [emb.tolist() for emb in embs]
                for name, embs in self.face_db.items()
            }
            with open(self.face_db_path, 'w') as f:
                json.dump(raw, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"FaceTracker: Failed to save face DB: {e}")

    def _run_recognition(self, frame: np.ndarray):
        """
        Run face_recognition on all active tracks and update their .name field.
        Called every recognition_interval frames to manage CPU load.
        """
        self.get_logger().info(f'[Recog] Running, session={list(self._session_faces.keys())}')
        if not self.tracks:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Build (top, right, bottom, left) list for face_recognition
        locations = []
        ordered_ids = []
        for tid, track in self.tracks.items():
            x, y, w, h = track.bbox
            locations.append((y, x + w, y + h, x))  # top, right, bottom, left
            ordered_ids.append(tid)

        try:
            encodings = face_recognition.face_encodings(rgb, locations)
        except Exception as e:
            self.get_logger().warning(f"face_recognition encoding failed: {e}")
            return

        # Flatten known faces: session-only entries first (take priority), then DB
        known_names: List[str] = []
        known_encs:  List[np.ndarray] = []
        for sname, sencs in self._session_faces.items():
            for enc in sencs:
                known_names.append(sname)
                known_encs.append(enc)
        for name, emb_list in self.face_db.items():
            for emb in emb_list:
                known_names.append(name)
                known_encs.append(emb)

        for tid, enc in zip(ordered_ids, encodings):
            if enc is None or len(enc) == 0:
                continue
            if not known_encs:
                self.tracks[tid].name = "Unknown"
                continue
            distances = face_recognition.face_distance(known_encs, enc)
            self.get_logger().info(f'[Recognition] Session faces: {list(self._session_faces.keys())}')
            self.get_logger().info(f'[Recognition] known_names count: {len(known_names)}, known_encs count: {len(known_encs)}')
            self.get_logger().info(f'[Recognition] distances: {distances}')
            best_idx  = int(np.argmin(distances))
            if distances[best_idx] < self.recog_threshold:
                self.tracks[tid].name = known_names[best_idx]
            else:
                self.tracks[tid].name = "Unknown"

    # ====================================================================== #
    # Calibration name topic (session-only)
    # ====================================================================== #

    def _on_calibration_name(self, msg: String) -> None:
        name = msg.data.strip().capitalize()
        if not name:
            return
        if not self.tracks:
            self.get_logger().warn('[CalibName] No tracked faces; cannot calibrate.')
            return
        if self._last_frame is None:
            self.get_logger().warn('[CalibName] No frame available yet.')
            return

        best = max(self.tracks.values(), key=lambda t: t.score)
        x, y, w, h = best.bbox
        rgb = cv2.cvtColor(self._last_frame, cv2.COLOR_BGR2RGB)
        locs = [(y, x + w, y + h, x)]
        encs = face_recognition.face_encodings(rgb, known_face_locations=locs, num_jitters=1)
        if not encs:
            self.get_logger().warn(f'[CalibName] No encoding for "{name}". Face may be unclear.')
            return

        if name not in self._session_faces:
            self._session_faces[name] = []
        self._session_faces[name].append(encs[0])
        self.get_logger().info(
            f'Calibrated face for {name} (session only, '
            f'{len(self._session_faces[name])} encoding(s))'
        )

    # ====================================================================== #
    # Calibration service
    # ====================================================================== #

    def _calibrate_face_callback(
        self,
        request: CalibrateFace.Request,
        response: CalibrateFace.Response,
    ) -> CalibrateFace.Response:
        """
        ROS 2 service handler for /maki/calibrate_face.

        Extracts the face_recognition encoding of the highest-scoring tracked
        face and saves it under request.name in the JSON database.
        """
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = "Name must not be empty."
            return response

        if not self.tracks:
            response.success = False
            response.message = "No faces currently tracked."
            return response

        if self._last_frame is None:
            response.success = False
            response.message = "No frame received yet."
            return response

        # Select the highest-scoring track (score was set in the last frame)
        best_track = max(self.tracks.values(), key=lambda t: t.score)
        self.get_logger().info(
            f"Calibrating '{name}' from track {best_track.track_id} "
            f"(score={best_track.score:.2f})"
        )

        rgb = cv2.cvtColor(self._last_frame, cv2.COLOR_BGR2RGB)
        x, y, w, h = best_track.bbox
        location = [(y, x + w, y + h, x)]  # top, right, bottom, left

        try:
            encs = face_recognition.face_encodings(rgb, location)
        except Exception as e:
            response.success = False
            response.message = f"Encoding failed: {e}"
            return response

        if not encs or len(encs[0]) == 0:
            response.success = False
            response.message = "Could not extract face encoding from best track."
            return response

        # Append embedding to database and persist
        if name not in self.face_db:
            self.face_db[name] = []
        self.face_db[name].append(encs[0])
        self._save_face_db()

        total = sum(len(v) for v in self.face_db.values())
        response.success = True
        response.message = (
            f"Saved embedding for '{name}'. "
            f"DB now has {total} total embeddings across {len(self.face_db)} identities."
        )
        self.get_logger().info(response.message)
        return response

    # ====================================================================== #
    # Debug annotation
    # ====================================================================== #

    def _draw_annotations(
        self,
        frame: np.ndarray,
        scored_tracks: List[TrackState],
        w_img: int,
        h_img: int,
    ) -> np.ndarray:
        """
        Draw bounding boxes, landmarks, per-face labels, and HUD overlays
        onto a copy of the frame when show_debug_window is True.
        """
        out = frame.copy()

        # ---- Per-face overlays ------------------------------------------- #
        for track in scored_tracks:
            x, y, w, h = track.bbox
            speaking = self._is_speaking(track)

            # All 468 MediaPipe landmark dots in light blue (radius 1px)
            if track.landmarks is not None:
                for lm in track.landmarks:
                    px = int(lm.x * w_img)
                    py = int(lm.y * h_img)
                    cv2.circle(out, (px, py), 1, (255, 200, 100), -1)

            # Green bounding box (thickness 2px)
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # White text above box: "ID: 0  Unknown"
            id_label = f"ID: {track.track_id}  {track.name}"
            label_y = max(y - 6, 16)
            cv2.putText(out, id_label, (x, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, id_label, (x, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # Yellow text: Score
            score_label = f"Score: {track.score:.2f}"
            cv2.putText(out, score_label, (x, label_y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, score_label, (x, label_y + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

            # Red text: SPEAKING (next to the face, when speaking)
            if speaking:
                cv2.putText(out, "SPEAKING", (x + w + 5, y + h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(out, "SPEAKING", (x + w + 5, y + h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        # ---- HUD — top-left ---------------------------------------------- #
        hud_lines = [
            f"FPS: {self._fps:.1f}",
            f"Frame: {self.frame_count}",
            f"Faces: {len(scored_tracks)}",
        ]
        for li, text in enumerate(hud_lines):
            cv2.putText(out, text, (8, 20 + li * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, text, (8, 20 + li * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # ---- HUD — top-right (awake status) ------------------------------ #
        status_text = "AWAKE" if self.awake else "SLEEPING"
        status_colour = (0, 200, 0) if self.awake else (0, 0, 200)
        (sw, _), _ = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.putText(out, status_text, (w_img - sw - 8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, status_text, (w_img - sw - 8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_colour, 1, cv2.LINE_AA)

        return out


# ============================================================================ #
# Entry point
# ============================================================================ #

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_debug:
            cv2.destroyAllWindows()
        node.face_landmarker.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
