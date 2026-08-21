#!/usr/bin/env python3
"""TTS + audio playback for the exhibition tour.

- edge-tts (Microsoft online neural voices) for Chinese narration synthesis
- ffplay for playback to the local sound card
- Audio files are cached under <web>/audio/intro_<id>.mp3
"""

import asyncio
import hashlib
import logging
import os
import subprocess

import edge_tts

log = logging.getLogger("go2web.tts")

AUDIO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "audio")
VOICE = "zh-CN-XiaoxiaoNeural"  # natural female Mandarin voice
_RATE = "+0%"                    # speaking rate (edge-tts syntax: +0% normal)

DEFAULT_INTRO = "欢迎参观，这里是本展区。"


def audio_path(wid, text_hash):
    return os.path.join(AUDIO_DIR, f"intro_{wid}_{text_hash}.mp3")


def _text_hash(text):
    return hashlib.md5(text.encode("utf-8")).hexdigest()[:8]


async def synthesize(text, out_path):
    """Synthesize text to mp3 using edge-tts (network required)."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    communicate = edge_tts.Communicate(text, VOICE, rate=_RATE)
    await communicate.save(out_path)
    log.info("TTS synthesized -> %s", out_path)
    return out_path


async def ensure_audio(wp):
    """Return an audio file path for a waypoint, synthesizing if needed.

    Priority: wp['audio'] (custom file) > cached intro_<id>_<hash>.mp3
    (hash of the intro text, so edits re-synthesize) > synthesize intro text.
    """
    # custom audio file (audio/ dir)
    custom = wp.get("audio", "")
    if custom:
        p = os.path.join(AUDIO_DIR, custom)
        if os.path.isfile(p):
            return p
        p = custom if os.path.isfile(custom) else None
        if p:
            return p

    # cached synthesized audio (hash-bound to the intro text)
    text = wp.get("intro", "").strip() or DEFAULT_INTRO
    cached = audio_path(wp.get("id", 0), _text_hash(text))
    if os.path.isfile(cached):
        return cached

    # synthesize
    return await synthesize(text, cached)


def duration_sec(path):
    """Get audio duration in seconds via ffprobe."""
    try:
        out = subprocess.run(
            ["ffprobe", "-v", "quiet", "-show_entries", "format=duration",
             "-of", "csv=p=0", path],
            capture_output=True, text=True, timeout=10,
        ).stdout.strip()
        return float(out) if out else 0.0
    except Exception:
        return 0.0


async def play(path, on_done=None):
    """Play an audio file to the local sound card; call on_done when finished.

    Returns the ffplay process; if playback fails, calls on_done immediately.
    """
    if not os.path.isfile(path):
        log.warning("audio file missing: %s", path)
        if on_done:
            on_done()
        return None
    try:
        # Force ALSA output via SDL so playback goes to the default analog card
        # (the Nvidia HDMI sink can otherwise be picked by mistake).
        env = dict(os.environ)
        env["SDL_AUDIODRIVER"] = "alsa"
        proc = await asyncio.create_subprocess_exec(
            "ffplay", "-nodisp", "-autoexit", "-loglevel", "error", path,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
            env=env,
        )
        await proc.wait()
        log.info("playback finished: %s", path)
    except Exception as e:
        log.warning("playback failed: %s", e)
    if on_done:
        on_done()
    return None
