#!/usr/bin/env python3
"""离线语音：机器人麦克风 -> sherpa-onnx ASR -> Ollama -> sherpa-onnx TTS -> 扬声器。

麦克风走 DDS `/audiosender`（unitree_go/AudioData，50Hz、20ms Opus 单声道
16kHz），用系统自带的 libopus 解码，不依赖容器里的 ALSA 采集设备——容器
的 /dev/snd 是启动时快照，麦克风节点经常不存在。

识别/合成都用 sherpa-onnx 本地模型（models/ 目录），TTS 不可用时回退现有
edge-tts（需要联网，仅作降级）。
"""

import asyncio
import ctypes
import ctypes.util
import hashlib
import logging
import os
import re
import time
import wave

import numpy as np

import tts

log = logging.getLogger("go2web.voice")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(BASE_DIR, "models")
AUDIO_DIR = os.path.join(BASE_DIR, "audio")

ASR_DEFAULT_DIR = "sherpa-onnx-paraformer-zh-2023-09-14"
TTS_DEFAULT_DIR = "vits-melo-tts-zh_en"

SAMPLE_RATE = 16000
MIC_SILENCE_RMS = 700      # int16 RMS 阈值，低于它视为静音
MIC_MIN_SPEECH_S = 0.35    # 至少这么长才算一句话


def _rms16(buf):
    if not buf:
        return 0.0
    a = np.frombuffer(buf, dtype=np.int16)
    if a.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(a.astype(np.float64) ** 2)))


_SENT_SPLIT = re.compile(r"(?<=[。！？!?；;\n])\s*")


def split_sentences(text, max_len=20):
    """按句号/问号/感叹号/分号切句；过长再按逗号切，便于流式合成播放。

    切得短一点，首句合成完就能出声；后面的句子边播边合成。
    """
    parts = [p.strip() for p in _SENT_SPLIT.split((text or "").strip())
             if p.strip()]
    out = []
    for p in parts:
        while len(p) > max_len:
            idx = max(p.rfind("，", 0, max_len),
                      p.rfind(",", 0, max_len),
                      p.rfind("、", 0, max_len))
            if idx <= 0:
                break
            out.append(p[:idx + 1])
            p = p[idx + 1:].strip()
        if p:
            out.append(p)
    return out


# ---------------------------------------------------------------- Opus 解码
class OpusDecoder:
    """把 /audiosender 的 20ms Opus 包解成 16k 单声道 int16 PCM。"""

    def __init__(self, sample_rate=SAMPLE_RATE):
        libname = ctypes.util.find_library("opus") or "libopus.so.0"
        self._lib = ctypes.CDLL(libname)
        self._lib.opus_decoder_create.restype = ctypes.c_void_p
        self._lib.opus_decoder_create.argtypes = [
            ctypes.c_int32, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self._lib.opus_decode.restype = ctypes.c_int
        self._lib.opus_decode.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int32,
            ctypes.POINTER(ctypes.c_int16), ctypes.c_int, ctypes.c_int]
        self._lib.opus_decoder_destroy.argtypes = [ctypes.c_void_p]
        err = ctypes.c_int(0)
        self._dec = self._lib.opus_decoder_create(sample_rate, 1, ctypes.byref(err))
        if not self._dec or err.value != 0:
            raise RuntimeError("opus_decoder_create failed: %d" % err.value)
        self._pcm = (ctypes.c_int16 * 5760)()

    def decode(self, packet):
        n = self._lib.opus_decode(
            self._dec, ctypes.c_char_p(packet), len(packet),
            self._pcm, 5760, 0)
        if n < 0:
            return b""
        return ctypes.string_at(ctypes.addressof(self._pcm), n * 2)

    def close(self):
        if getattr(self, "_dec", None):
            self._lib.opus_decoder_destroy(self._dec)
            self._dec = None


# ---------------------------------------------------------------- ASR
class AsrEngine:
    """sherpa-onnx 离线中文识别（Paraformer，兼容 SenseVoice）。"""

    def __init__(self, model_dir, asr_type="paraformer", num_threads=2):
        import sherpa_onnx
        d = model_dir
        if not os.path.isdir(d):
            raise RuntimeError("ASR 模型目录不存在: %s" % d)

        def pick(*names):
            for n in names:
                p = os.path.join(d, n)
                if os.path.isfile(p):
                    return p
            return None

        model = pick("model.int8.onnx", "model.onnx")
        tokens = pick("tokens.txt")
        if not model or not tokens:
            raise RuntimeError("ASR 模型文件缺失: %s" % d)
        if asr_type == "sense_voice":
            self._rec = sherpa_onnx.OfflineRecognizer.from_sense_voice(
                model=model, tokens=tokens, num_threads=num_threads,
                use_itn=True, debug=False)
        else:
            self._rec = sherpa_onnx.OfflineRecognizer.from_paraformer(
                paraformer=model, tokens=tokens, num_threads=num_threads,
                sample_rate=SAMPLE_RATE, feature_dim=80,
                decoding_method="greedy_search", debug=False)

    def recognize(self, pcm_bytes):
        samples = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        if samples.size == 0:
            return ""
        s = self._rec.create_stream()
        s.accept_waveform(SAMPLE_RATE, samples)
        self._rec.decode_stream(s)
        return (s.result.text or "").strip()


# ---------------------------------------------------------------- TTS
class TtsEngine:
    """sherpa-onnx 离线中文合成（VITS / Melo）。"""

    def __init__(self, model_dir, num_threads=2):
        import sherpa_onnx
        d = model_dir
        if not os.path.isdir(d):
            raise RuntimeError("TTS 模型目录不存在: %s" % d)

        def pick(*names):
            for n in names:
                p = os.path.join(d, n)
                if os.path.isfile(p):
                    return p
            return None

        model = pick("model.onnx", "model.int8.onnx")
        tokens = pick("tokens.txt")
        if not model or not tokens:
            raise RuntimeError("TTS 模型文件缺失: %s" % d)
        lexicon = pick("lexicon.txt")
        data_dir = pick("espeak-ng-data")
        dict_dir = os.path.join(d, "dict")
        if not os.path.isdir(dict_dir):
            dict_dir = ""

        vits = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=model,
            lexicon=lexicon or "",
            tokens=tokens,
            data_dir=data_dir or "",
            dict_dir=dict_dir or "",
        )
        cfg = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=vits, num_threads=num_threads, debug=False),
            max_num_sentences=1,
        )
        self._tts = sherpa_onnx.OfflineTts(cfg)
        self.sample_rate = self._tts.sample_rate

    def synthesize_wav(self, text, out_path):
        audio = self._tts.generate(text, sid=0, speed=1.0)
        samples = np.asarray(audio.samples, dtype=np.float32)
        if samples.size == 0:
            return None
        pcm = np.clip(samples, -1.0, 1.0)
        # 模型输出峰值只有 0.14-0.39（不同句子差异很大），归一化到满量程，
        # 保证每句话都一样响；扬声器混音器已 100%，这是最干净的“最大音量”。
        peak = float(np.abs(pcm).max())
        if peak > 1e-4:
            pcm = pcm * (0.95 / peak)
        pcm16 = (pcm * 32767.0).astype(np.int16)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        tmp = out_path + ".tmp"
        with wave.open(tmp, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(int(audio.sample_rate))
            w.writeframes(pcm16.tobytes())
        os.replace(tmp, out_path)
        return out_path


# ---------------------------------------------------------------- 服务
class VoiceService:
    """一次只允许一个语音会话；listen 与 speak 都从这里进出。"""

    def __init__(self, app):
        self.app = app
        self.bridge = app.bridge
        self.cfg = app.cfg
        self._decoder = None
        self._asr = None
        self._tts = None
        self._stop = None
        self._busy = False
        self._last_error = ""

    # ---- 配置 ----
    @property
    def vcfg(self):
        return (self.cfg.get("ai") or {}).get("voice") or {}

    def enabled(self):
        return bool((self.cfg.get("ai") or {}).get("enabled", True)) \
            and bool(self.vcfg.get("enabled", True))

    def _model_path(self, key, default):
        name = self.vcfg.get(key) or default
        return name if os.path.isabs(name) else os.path.join(MODEL_DIR, name)

    def asr_ready(self):
        return self._asr is not None

    def tts_ready(self):
        return self._tts is not None

    def _model_present(self, key, default):
        d = self._model_path(key, default)
        if not os.path.isdir(d):
            return False
        return any(os.path.isfile(os.path.join(d, n))
                   for n in ("model.int8.onnx", "model.onnx"))

    def status(self):
        return {
            "enabled": self.enabled(),
            "mic_alive": self.bridge.audio_alive(2.0),
            "asr_ready": self.asr_ready(),
            "tts_ready": self.tts_ready(),
            "asr_available": self._model_present("asr_model", ASR_DEFAULT_DIR),
            "tts_available": self._model_present("tts_model", TTS_DEFAULT_DIR),
            "asr_model": os.path.basename(self._model_path("asr_model", ASR_DEFAULT_DIR)),
            "tts_model": os.path.basename(self._model_path("tts_model", TTS_DEFAULT_DIR)),
            "backend": self.vcfg.get("tts_backend", "sherpa"),
            "busy": self._busy,
            "error": self._last_error,
        }

    # ---- 引擎懒加载 ----
    def ensure_asr(self):
        if self._asr is not None:
            return self._asr
        self._asr = AsrEngine(
            self._model_path("asr_model", ASR_DEFAULT_DIR),
            asr_type=self.vcfg.get("asr_type", "paraformer"),
            num_threads=int(self.vcfg.get("asr_threads", 2)))
        return self._asr

    def ensure_tts(self):
        if self._tts is not None:
            return self._tts
        if self.vcfg.get("tts_backend", "sherpa") != "sherpa":
            return None
        self._tts = TtsEngine(
            self._model_path("tts_model", TTS_DEFAULT_DIR),
            num_threads=int(self.vcfg.get("tts_threads", 2)))
        return self._tts

    def ensure_decoder(self):
        if self._decoder is None:
            self._decoder = OpusDecoder()
        return self._decoder

    def release(self):
        """AI 总开关关闭时释放模型内存。"""
        try:
            if self._decoder:
                self._decoder.close()
        except Exception:
            pass
        self._decoder = None
        self._asr = None
        self._tts = None
        self._stop = None
        self._busy = False

    def cancel(self):
        if self._stop is not None:
            self._stop.set()

    # ---- 录音 ----
    async def listen(self, push=None):
        """录一句话并返回文本。push 用于向前端推电平/状态。"""
        dec = self.ensure_decoder()
        if not self.bridge.audio_alive(2.0):
            raise RuntimeError("麦克风没有数据（/audiosender 无更新）")

        max_s = float(self.vcfg.get("max_utterance_s", 15))
        silence_s = float(self.vcfg.get("silence_ms", 800)) / 1000.0
        threshold = float(self.vcfg.get("vad_rms", MIC_SILENCE_RMS))
        self._stop = asyncio.Event()

        cursor = self.bridge.audio_cursor()
        pcm = bytearray()
        start = time.monotonic()
        last_loud = start
        peak = 0.0
        noise = []                 # 前 0.4s 的底噪样本，用来抬高门限
        threshold_eff = threshold
        last_level_push = 0.0
        try:
            while True:
                if self._stop.is_set():
                    break
                pkts, cursor = self.bridge.audio_packets(cursor)
                for p in pkts:
                    frame = dec.decode(p)
                    if not frame:
                        continue
                    pcm.extend(frame)
                    rms = _rms16(frame)
                    if rms > peak:
                        peak = rms
                    elapsed = time.monotonic() - start
                    if elapsed < 0.4:
                        noise.append(rms)
                        if len(noise) >= 6:
                            # 底噪 80 分位 * 1.4：Go2 内置麦克风底噪约 400，
                            # 固件增益固定，只能靠相对电平判断
                            base = sorted(noise)[int(len(noise) * 0.8)]
                            threshold_eff = max(threshold, base * 1.4)
                    if rms >= threshold_eff:
                        last_loud = time.monotonic()
                now = time.monotonic()
                if push and now - last_level_push > 0.1:
                    last_level_push = now
                    push({"op": "level", "rms": round(peak, 1),
                          "thr": round(threshold_eff, 1),
                          "seconds": round(now - start, 1)})
                if now - start > max_s:
                    break
                if now - start > MIC_MIN_SPEECH_S and (now - last_loud) > silence_s:
                    break
                await asyncio.sleep(0.05)
        finally:
            self._stop = None

        if peak < max(700.0, threshold_eff):
            return ""
        asr = self.ensure_asr()
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, asr.recognize, bytes(pcm))

    # ---- 合成 + 播放 ----
    async def synthesize_path(self, text):
        """离线合成到 wav 并返回路径；模型不可用返回 None。

        同时作为 tts.speak 的离线兜底（导览解说在 edge-tts 403/断网时用）。
        """
        text = (text or "").strip()
        if not text:
            return None
        try:
            engine = self.ensure_tts()
        except Exception as e:
            self._last_error = "TTS 模型不可用: %s" % e
            return None
        if engine is None:
            return None
        h = hashlib.md5(text.encode("utf-8")).hexdigest()[:12]
        path = os.path.join(AUDIO_DIR, "tts_%s.wav" % h)
        if not os.path.isfile(path):
            loop = asyncio.get_event_loop()
            try:
                await loop.run_in_executor(
                    None, engine.synthesize_wav, text, path)
            except Exception as e:
                self._last_error = "离线 TTS 失败: %s" % e
                return None
        return path if os.path.isfile(path) else None

    def _synth_sentence_sync(self, text):
        """合成一句到缓存 wav（同步，供线程池调用）。"""
        engine = self._tts
        if engine is None:
            return None
        h = hashlib.md5(text.encode("utf-8")).hexdigest()[:12]
        path = os.path.join(AUDIO_DIR, "tts_s_%s.wav" % h)
        if not os.path.isfile(path):
            try:
                engine.synthesize_wav(text, path)
            except Exception as e:
                self._last_error = "离线 TTS 失败: %s" % e
                return None
        return path if os.path.isfile(path) else None

    async def speak_stream(self, text):
        """按句流式播放：首句合成完就出声，后面边播边合成。

        展厅解说通常有 3-5 句，整段合成要 4-6 秒才出声；按句流式第一句
        约 1 秒就能听到，句子之间靠预取掩盖合成耗时。
        """
        sents = split_sentences(text)
        if not sents:
            return False
        try:
            engine = self.ensure_tts()
        except Exception as e:
            self._last_error = "TTS 模型不可用: %s" % e
            return False
        if engine is None:
            return False
        loop = asyncio.get_event_loop()
        sem = asyncio.Semaphore(2)

        async def synth(s):
            async with sem:
                return await loop.run_in_executor(
                    None, self._synth_sentence_sync, s)

        tasks = [asyncio.ensure_future(synth(s)) for s in sents]
        played = False
        try:
            for t in tasks:
                path = await t
                if path:
                    await tts.play(path)
                    played = True
        finally:
            for t in tasks:
                if not t.done():
                    t.cancel()
        return played

    async def warm_text(self, text):
        """后台预合成（不播放），让试听/导览直接命中缓存。返回合成句数。"""
        sents = split_sentences(text)
        if not sents:
            return 0
        try:
            engine = self.ensure_tts()
        except Exception as e:
            self._last_error = "TTS 模型不可用: %s" % e
            return 0
        if engine is None:
            return 0
        loop = asyncio.get_event_loop()
        sem = asyncio.Semaphore(2)
        done = [0]

        async def one(s):
            async with sem:
                p = await loop.run_in_executor(
                    None, self._synth_sentence_sync, s)
                if p:
                    done[0] += 1
        await asyncio.gather(*(one(s) for s in sents))
        log.info("warm_text: %d/%d sentences cached", done[0], len(sents))
        return done[0]

    async def speak(self, text):
        text = (text or "").strip()
        if not text:
            return False
        path = await self.synthesize_path(text)
        if path:
            try:
                await tts.play(path)
                return True
            except Exception as e:
                self._last_error = "播放失败: %s" % e
        # 降级：现有 edge-tts（联网）
        try:
            return bool(await tts.speak(text))
        except Exception as e:
            self._last_error = "TTS 全部失败: %s" % e
            return False

    # ---- 一次完整语音对话 ----
    async def listen_and_chat(self, push):
        if self._busy:
            push({"op": "error", "error": "上一轮语音还在进行"})
            return
        self._busy = True
        try:
            push({"op": "listening"})
            text = await self.listen(push)
            if not text:
                push({"op": "asr", "text": "", "empty": True})
                return
            push({"op": "asr", "text": text})
            reply = await self.app.voice_chat(text, push)
            if reply:
                push({"op": "reply", "text": reply})
                await self.speak(reply)
                push({"op": "done"})
        except Exception as e:
            self._last_error = str(e)
            push({"op": "error", "error": str(e)})
        finally:
            self._busy = False
