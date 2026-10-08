#!/usr/bin/env python3
"""TTS + audio playback for the exhibition tour.

- edge-tts (Microsoft online neural voices) for Chinese narration synthesis
- ffplay for playback to the local sound card
- Narration is streamed: edge-tts audio chunks are piped straight into ffplay
  stdin, so playback starts as soon as the first chunk arrives instead of
  waiting for the whole clip. The complete mp3 is still cached under
  <web>/audio/intro_<id>.mp3 so later plays are instant and offline.
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


def target_audio(wp):
    """Return the audio file to use for a waypoint's narration.

    An existing file (custom wp['audio'], or a cached synthesis bound to the
    current intro text) is returned as-is; otherwise this returns the cache
    path that speak() should synthesize into.
    """
    custom = wp.get("audio", "")
    if custom:
        p = os.path.join(AUDIO_DIR, custom)
        if os.path.isfile(p):
            return p
        if os.path.isfile(custom):
            return custom
    text = wp.get("intro", "").strip() or DEFAULT_INTRO
    return audio_path(wp.get("id", 0), _text_hash(text))


def _ffplay_env():
    # Force ALSA output via SDL so playback goes to the default analog card
    # (the Nvidia HDMI sink can otherwise be picked by mistake).
    env = dict(os.environ)
    env["SDL_AUDIODRIVER"] = "alsa"
    return env


async def synthesize(text, out_path):
    """Synthesize text to a complete mp3 file using edge-tts (network required)."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    communicate = edge_tts.Communicate(text, VOICE, rate=_RATE)
    await communicate.save(out_path)
    log.info("TTS synthesized -> %s", out_path)
    return out_path


async def _stream_play(communicate, out_path=None):
    """Pipe edge-tts audio chunks into ffplay stdin as they arrive.

    Playback starts on the first chunk; the full stream is also written to
    out_path (if given) so the next play can hit the cache. A partial file is
    removed if synthesis/playback is interrupted, so it is never mistaken for
    a valid cache entry.
    """
    proc = await asyncio.create_subprocess_exec(
        "ffplay", "-nodisp", "-autoexit", "-loglevel", "error", "-i", "pipe:0",
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL,
        env=_ffplay_env(),
    )
    f = None
    if out_path:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        f = open(out_path, "wb")
    complete = False
    cancelled = False
    try:
        async for chunk in communicate.stream():
            if chunk["type"] != "audio":
                continue
            data = chunk.get("data")
            if not data:
                continue
            if f:
                f.write(data)
            if proc.stdin is not None:
                proc.stdin.write(data)
                await proc.stdin.drain()
        complete = True
    except asyncio.CancelledError:
        cancelled = True
        raise
    except Exception as e:
        log.warning("stream playback error: %s", e)
    finally:
        if f:
            f.close()
            f = None
        # never leave a truncated file behind as a "cache hit"
        if not complete and out_path and os.path.isfile(out_path):
            try:
                os.remove(out_path)
            except OSError:
                pass
        if cancelled and proc.returncode is None:
            proc.kill()  # stop buffered audio immediately on skip/stop
        try:
            if proc.stdin is not None and not proc.stdin.is_closing():
                proc.stdin.close()
        except Exception:
            pass
        try:
            await asyncio.wait_for(proc.wait(), timeout=15)
        except Exception:
            if proc.returncode is None:
                proc.kill()
    log.info("stream playback finished: %s", out_path)


async def speak(text, out_path=None):
    """Play narration for text, streaming from edge-tts.

    If out_path already exists it is played directly (instant, offline);
    otherwise the text is synthesized and streamed chunk-by-chunk, and cached
    to out_path for next time.
    """
    if out_path and os.path.isfile(out_path):
        return await play(out_path)
    communicate = edge_tts.Communicate(text, VOICE, rate=_RATE)
    await _stream_play(communicate, out_path)
    return out_path


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

    Returns None. If playback fails, calls on_done immediately. If the
    surrounding task is cancelled (e.g. tour stopped), ffplay is killed so
    playback stops immediately.
    """
    if not os.path.isfile(path):
        log.warning("audio file missing: %s", path)
        if on_done:
            on_done()
        return None
    proc = None
    try:
        proc = await asyncio.create_subprocess_exec(
            "ffplay", "-nodisp", "-autoexit", "-loglevel", "error", path,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
            env=_ffplay_env(),
        )
        await proc.wait()
        log.info("playback finished: %s", path)
    except asyncio.CancelledError:
        if proc is not None and proc.returncode is None:
            proc.kill()
        log.info("playback cancelled: %s", path)
        raise
    except Exception as e:
        log.warning("playback failed: %s", e)
    if on_done:
        on_done()
    return None
