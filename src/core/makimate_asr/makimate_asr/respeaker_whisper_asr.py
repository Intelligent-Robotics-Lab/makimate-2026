#!/usr/bin/env python3
"""
ReSpeaker Whisper ASR node with VAD-based utterance segmentation.

Improvements over the old fixed-2s-polling node:
  - webrtcvad gates audio: only sends actual speech to Whisper
  - Flushes on silence gap → natural utterance boundaries, no cut words
  - condition_on_previous_text=False → no hallucination loops
  - no_speech_prob filter → discards segments Whisper is uncertain about
  - Server offload: point server_url at makimate-asr-server for large models
  - sounddevice replaces pyaudio (consistent with vosk node)
"""

import io
import re
import threading
import time
import wave
from collections import Counter, deque
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sounddevice as sd
import webrtcvad

# ---- Audio constants --------------------------------------------------------
SAMPLE_RATE    = 16000
FRAME_MS       = 20                           # webrtcvad works on 10/20/30ms frames
FRAME_SAMPLES  = SAMPLE_RATE * FRAME_MS // 1000   # 320 samples per frame
FRAME_BYTES    = FRAME_SAMPLES * 2            # 640 bytes (int16 = 2 bytes/sample)

# ---- Noise filter -----------------------------------------------------------
NOISE_WORDS = {
    'huh', 'h', 'uh', 'um', 'hmm', 'mm', 'ah', 'eh', 'oh',
    'a', 'the', 'and', 'you', 'thank', 'thanks',
}

# Known Whisper hallucination phrases (startswith match on lowercased text).
# These are phrases Whisper emits when given near-silence or ambient noise.
HALLUCINATION_PREFIXES = (
    "i'm going to",
    "i am going to",
    "thank you for watching",
    "thanks for watching",
    "please subscribe",
    "subscribe to",
    "transcribed by",
    "all rights reserved",
    "visit our website",
    "for more information",
    "in this video",
    "in the next video",
    "check out our",
    "don't forget to",
    "like and subscribe",
)

_CLAUSE_SPLIT = re.compile(r'[,\.!?]+\s*')


# ---- VAD chunker ------------------------------------------------------------

class VadChunker:
    """
    Wraps webrtcvad with a state machine to segment an audio stream into
    individual utterances.

    Feed 20ms PCM frames via process(). When a complete utterance is detected
    (speech followed by enough silence) it returns a float32 numpy array.
    """

    def __init__(self, aggressiveness: int = 2, silence_ms: int = 400,
                 min_ms: int = 250, max_s: float = 8.0):
        self.vad            = webrtcvad.Vad(aggressiveness)
        self.silence_frames = max(1, silence_ms // FRAME_MS)
        self.min_frames     = max(1, min_ms // FRAME_MS)
        self.max_frames     = int(max_s * 1000 / FRAME_MS)

        self._buf:           list = []   # accumulated PCM frame bytes
        self._silence_count: int  = 0
        self._in_speech:     bool = False

    def process(self, frame_bytes: bytes) -> Optional[np.ndarray]:
        """
        Feed one 20ms frame (640 bytes of int16 PCM at 16kHz).
        Returns float32 numpy array when an utterance is complete, else None.
        """
        is_speech = self.vad.is_speech(frame_bytes, SAMPLE_RATE)

        if is_speech:
            self._buf.append(frame_bytes)
            self._silence_count = 0
            self._in_speech = True
        elif self._in_speech:
            self._buf.append(frame_bytes)
            self._silence_count += 1

            flush = (self._silence_count >= self.silence_frames or
                     len(self._buf) >= self.max_frames)
            if flush:
                return self._flush()

        return None

    def _flush(self) -> Optional[np.ndarray]:
        # Trim trailing silence frames
        trim = self._silence_count
        speech_frames = self._buf[:-trim] if trim else self._buf

        self._buf           = []
        self._silence_count = 0
        self._in_speech     = False

        if len(speech_frames) < self.min_frames:
            return None  # too short — likely noise click

        pcm   = b''.join(speech_frames)
        audio = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        return audio

    def reset(self):
        """Discard any buffered audio (call when ASR is disabled)."""
        self._buf           = []
        self._silence_count = 0
        self._in_speech     = False


# ---- ROS node ---------------------------------------------------------------

class ReSpeakerWhisperASR(Node):

    def __init__(self):
        super().__init__('respeaker_whisper_asr')

        # ---- Parameters ----
        self.declare_parameter('device',               '0')   # int index or ALSA name
        self.declare_parameter('model_size',           'base')
        self.declare_parameter('asr_topic',            '/asr/text')
        self.declare_parameter('enable_topic',         '/asr/enable')
        self.declare_parameter('server_url',           '')     # '' = local inference
        self.declare_parameter('no_speech_threshold',  0.6)
        self.declare_parameter('vad_aggressiveness',   2)      # 0-3, 3 = most aggressive
        self.declare_parameter('silence_ms',           400)    # ms of silence → utterance end
        self.declare_parameter('channels',             1)      # kept for API compat; UAC1.0 = 1
        self.declare_parameter('save_debug_audio',     False)  # write each VAD chunk to /tmp/maki_heard/
        self.declare_parameter('rms_threshold',        0.0)    # 0=disabled; ~0.02 rejects distant voices

        self._save_debug_audio = bool(self.get_parameter('save_debug_audio').value)
        self._debug_audio_counter = 0
        self._rms_threshold = float(self.get_parameter('rms_threshold').value)

        _dev_raw = str(self.get_parameter('device').value).strip()
        try:
            device = int(_dev_raw)   # numeric index
        except ValueError:
            device = _dev_raw        # ALSA name e.g. 'respeaker_shared'
        model_size         = self.get_parameter('model_size').value
        asr_topic          = self.get_parameter('asr_topic').value
        enable_topic       = self.get_parameter('enable_topic').value
        self.server_url    = self.get_parameter('server_url').value.strip()
        self.no_speech_thr = float(self.get_parameter('no_speech_threshold').value)
        vad_aggressiveness = int(self.get_parameter('vad_aggressiveness').value)
        silence_ms         = int(self.get_parameter('silence_ms').value)

        # ---- Publishers / subscribers ----
        self.text_pub = self.create_publisher(String, asr_topic, 10)
        self.create_subscription(Bool, enable_topic, self._on_asr_enable, 10)

        # ---- State ----
        self.enabled      = True
        self._lock        = threading.Lock()
        self._pending     = deque(maxlen=8)   # cap: don't pile up on slow Pi

        # ---- VAD chunker ----
        self.chunker = VadChunker(
            aggressiveness=vad_aggressiveness,
            silence_ms=silence_ms,
        )

        # ---- Whisper model (local only) ----
        self.model = None
        if not self.server_url:
            self.get_logger().info(
                f"Loading faster-whisper '{model_size}' on CPU (int8) — "
                "this may take 10–30s on first run while model downloads..."
            )
            from faster_whisper import WhisperModel
            self.model = WhisperModel(model_size, device='cpu', compute_type='int8')
            self.get_logger().info("Whisper model ready.")
        else:
            self.get_logger().info(f"Whisper server mode: {self.server_url}")

        # ---- Audio stream ----
        # int -1 → default; int ≥0 → index; str → ALSA device name
        device_arg = (None if isinstance(device, int) and device < 0 else device)
        self.stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='int16',
            blocksize=FRAME_SAMPLES,
            device=device_arg,
            callback=self._audio_callback,
        )
        self.stream.start()
        self.get_logger().info(
            f"Audio stream started — device={device_arg}, "
            f"VAD aggressiveness={vad_aggressiveness}, silence_ms={silence_ms}"
        )

        # ---- Inference thread ----
        self._inf_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inf_thread.start()
        self.get_logger().info("ReSpeakerWhisperASR node started.")

    # ------------------------------------------------------------------ #
    # Audio callback (runs in sounddevice thread — must be fast)
    # ------------------------------------------------------------------ #
    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}", throttle_duration_sec=2.0)
        if not self.enabled:
            return
        frame_bytes = indata[:, 0].tobytes()   # shape (FRAME_SAMPLES, 1) → bytes
        chunk = self.chunker.process(frame_bytes)
        if chunk is not None:
            with self._lock:
                self._pending.append(chunk)

    # ------------------------------------------------------------------ #
    # Enable / disable (called by TTS to mute mic while speaking)
    # ------------------------------------------------------------------ #
    def _on_asr_enable(self, msg: Bool):
        enabled = bool(msg.data)
        if enabled == self.enabled:
            return
        self.enabled = enabled
        if not enabled:
            self.chunker.reset()
            with self._lock:
                self._pending.clear()
        self.get_logger().info(f"ASR {'enabled' if enabled else 'disabled'}")

    # ------------------------------------------------------------------ #
    # Inference loop (runs in dedicated thread)
    # ------------------------------------------------------------------ #
    def _inference_loop(self):
        while rclpy.ok():
            chunk = None
            with self._lock:
                if self._pending:
                    chunk = self._pending.popleft()

            if chunk is None:
                time.sleep(0.02)
                continue

            if self._rms_threshold > 0.0:
                rms = float(np.sqrt(np.mean(chunk ** 2)))
                if rms < self._rms_threshold:
                    self.get_logger().debug(
                        f"[RMS] Discarded chunk rms={rms:.4f} < threshold={self._rms_threshold:.4f}"
                    )
                    continue

            if self._save_debug_audio:
                self._write_debug_wav(chunk)

            text = self._transcribe(chunk)
            if text:
                self.get_logger().info(f"ASR text: {text!r}")
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Debug audio saving
    # ------------------------------------------------------------------ #
    def _write_debug_wav(self, audio: np.ndarray):
        """Save this VAD chunk to /tmp/maki_heard/ so you can aplay it."""
        import os
        out_dir = '/tmp/maki_heard'
        os.makedirs(out_dir, exist_ok=True)
        self._debug_audio_counter = (self._debug_audio_counter + 1) % 20
        path = os.path.join(out_dir, f'{self._debug_audio_counter:02d}.wav')
        try:
            with wave.open(path, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes((audio * 32768).astype(np.int16).tobytes())
            # symlink latest.wav → most recent chunk for easy access
            latest = os.path.join(out_dir, 'latest.wav')
            if os.path.lexists(latest):
                os.remove(latest)
            os.symlink(path, latest)
        except Exception as e:
            self.get_logger().warn(f'[DebugAudio] Could not write {path}: {e}')

    # ------------------------------------------------------------------ #
    # Transcription (local or server)
    # ------------------------------------------------------------------ #
    def _transcribe(self, audio: np.ndarray) -> Optional[str]:
        try:
            if self.server_url:
                return self._transcribe_server(audio)
            return self._transcribe_local(audio)
        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None

    def _transcribe_local(self, audio: np.ndarray) -> Optional[str]:
        segments, info = self.model.transcribe(
            audio,
            language='en',
            beam_size=5,
            temperature=0,
            condition_on_previous_text=False,
            vad_filter=True,
            vad_parameters=dict(min_silence_duration_ms=200),
            no_speech_threshold=self.no_speech_thr,
        )
        parts = [seg.text.strip() for seg in segments]
        return self._filter(' '.join(p for p in parts if p))

    def _transcribe_server(self, audio: np.ndarray) -> Optional[str]:
        import requests
        buf = io.BytesIO()
        with wave.open(buf, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(SAMPLE_RATE)
            wf.writeframes((audio * 32768).astype(np.int16).tobytes())
        buf.seek(0)
        resp = requests.post(
            f'{self.server_url}/transcribe',
            files={'audio': ('audio.wav', buf, 'audio/wav')},
            timeout=30,
        )
        resp.raise_for_status()
        data = resp.json()
        if data.get('no_speech_prob', 0.0) > self.no_speech_thr:
            self.get_logger().debug(
                f"Discarded (no_speech_prob={data['no_speech_prob']:.2f}): "
                f"{data.get('text', '')!r}"
            )
            return None
        return self._filter(data.get('text', '').strip())

    # ------------------------------------------------------------------ #
    # Noise filter
    # ------------------------------------------------------------------ #
    def _filter(self, text: str) -> Optional[str]:
        if not text:
            return None

        # Single noise word
        words = text.split()
        if len(words) == 1 and words[0].lower().strip('.,!?') in NOISE_WORDS:
            self.get_logger().debug(f"Filtered noise: {text!r}")
            return None

        # Known Whisper hallucination phrases
        lower = text.lower().strip()
        for prefix in HALLUCINATION_PREFIXES:
            if lower.startswith(prefix):
                self.get_logger().debug(f"Filtered hallucination: {text!r:.80}")
                return None

        # Repetition loop: same clause appears 3+ times → hallucination
        clauses = [c.strip() for c in _CLAUSE_SPLIT.split(lower) if len(c.strip()) > 8]
        if len(clauses) >= 3:
            counts = Counter(clauses)
            if counts.most_common(1)[0][1] >= 3:
                self.get_logger().debug(f"Filtered repetition loop: {text!r:.80}")
                return None

        return text

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self.get_logger().info("Shutting down ReSpeakerWhisperASR...")
        if self.stream:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReSpeakerWhisperASR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
