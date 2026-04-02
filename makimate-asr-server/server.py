#!/usr/bin/env python3
"""
MakiMate Whisper ASR Server
============================
FastAPI server that runs faster-whisper and exposes a /transcribe endpoint.
Run this on your Mac or school GPU server for better accuracy than the Pi can
provide with tiny/base locally.

Usage:
    pip install -r requirements.txt
    python server.py                        # default: base model, port 8001
    python server.py --model large-v3       # best accuracy (needs GPU or patience)
    python server.py --model distil-large-v3 --port 8001

Pi side: set the respeaker_whisper_asr parameter:
    server_url: "http://<this-machine-ip>:8001"

Switch model live from the MakiMate dashboard (Whisper Model dropdown → Apply).
"""

import argparse
import io
import threading
import wave

import numpy as np
import uvicorn
from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel

SAMPLE_RATE = 16000

# ---------------------------------------------------------------------------
# Global model state — updated by /set_model
# ---------------------------------------------------------------------------
_model       = None
_model_name  = "base"
_model_lock  = threading.Lock()


def _load_model(name: str):
    """Load (or reload) faster-whisper model. Thread-safe."""
    global _model, _model_name
    from faster_whisper import WhisperModel
    try:
        import torch
        device       = "cuda" if torch.cuda.is_available() else "cpu"
    except ImportError:
        device       = "cpu"
    compute_type = "float16" if device == "cuda" else "int8"
    print(f"[ASR server] Loading '{name}' on {device} ({compute_type})...")
    new_model = WhisperModel(name, device=device, compute_type=compute_type)
    with _model_lock:
        _model      = new_model
        _model_name = name
    print(f"[ASR server] Model '{name}' ready.")


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------
app = FastAPI(title="MakiMate ASR Server")

# Allow the dashboard (served from the Pi) to call this server directly
# from the browser — without this, every browser fetch is blocked by CORS.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def _startup():
    import asyncio
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, _load_model, _model_name)


@app.get("/health")
def health():
    return {"status": "ok", "model": _model_name}


class SetModelRequest(BaseModel):
    model: str


@app.post("/set_model")
async def set_model(req: SetModelRequest):
    """
    Hot-swap the Whisper model. Called by the MakiMate dashboard when
    the user changes the Whisper Model dropdown.
    Accepted values: tiny, base, small, medium, large-v3, distil-large-v3
    """
    import asyncio
    valid = {"tiny", "base", "small", "medium", "large-v3", "distil-large-v3",
             "tiny.en", "base.en", "small.en", "medium.en"}
    if req.model not in valid:
        raise HTTPException(400, f"Unknown model '{req.model}'. Valid: {sorted(valid)}")
    if req.model == _model_name:
        return {"status": "already_loaded", "model": _model_name}
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, _load_model, req.model)
    return {"status": "ok", "model": _model_name}


@app.post("/transcribe")
async def transcribe(audio: UploadFile = File(...)):
    """
    Accept a mono 16 kHz WAV file, return:
        {text: str, no_speech_prob: float, language: str}
    """
    with _model_lock:
        m = _model
    if m is None:
        raise HTTPException(503, "Model not loaded yet — try again in a moment")

    raw      = await audio.read()
    audio_np = _wav_to_float32(raw)
    if audio_np is None:
        raise HTTPException(400, "Could not decode audio — send mono 16 kHz WAV")

    segments, info = m.transcribe(
        audio_np,
        language="en",
        beam_size=5,
        temperature=0,
        condition_on_previous_text=False,
        vad_filter=True,
        vad_parameters=dict(min_silence_duration_ms=200),
    )

    parts = [seg.text.strip() for seg in segments]
    text  = " ".join(p for p in parts if p)

    # language_probability ≈ 1 when clearly speech; invert for no_speech_prob
    no_speech_prob = round(max(0.0, 1.0 - info.language_probability), 3)

    return {"text": text, "no_speech_prob": no_speech_prob, "language": info.language}


# ---------------------------------------------------------------------------
# WAV decoding helper
# ---------------------------------------------------------------------------

def _wav_to_float32(data: bytes):
    try:
        buf = io.BytesIO(data)
        with wave.open(buf) as wf:
            assert wf.getnchannels() == 1,    "expected mono WAV"
            assert wf.getframerate() == SAMPLE_RATE, f"expected {SAMPLE_RATE} Hz"
            pcm = wf.readframes(wf.getnframes())
        return np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
    except Exception as e:
        print(f"[WARN] wav decode failed: {e}")
        return None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MakiMate Whisper ASR server")
    parser.add_argument("--model",  default="base",
                        help="faster-whisper model to load at startup (default: base)")
    parser.add_argument("--host",   default="0.0.0.0")
    parser.add_argument("--port",   default=8001, type=int)
    args = parser.parse_args()

    _model_name = args.model   # picked up by startup handler
    print(f"Starting MakiMate ASR server — model={_model_name}  {args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
