#!/bin/bash
# 安装离线语音依赖（sherpa-onnx + 中文 ASR/TTS 模型）。
# 在机器人主机上运行：./install_ai.sh
# 模型从 hf-mirror 下载（GitHub Releases 资产在本网络下会超时）。
set -u

WEB_DIR="$(cd "$(dirname "$0")" && pwd)"
MODELS="$WEB_DIR/models"
HF="${HF_ENDPOINT:-https://hf-mirror.com}"
CONTAINER="${GO2_CONTAINER:-go2_humble}"

echo "== 1/3 安装 sherpa-onnx（容器 $CONTAINER 内）"
if docker exec "$CONTAINER" python3 -c "import sherpa_onnx" 2>/dev/null; then
  echo "   sherpa-onnx 已安装，跳过"
else
  docker exec "$CONTAINER" pip3 install --no-cache-dir sherpa-onnx || {
    echo "!! pip 安装失败。可改用主机 py3.8 版：pip3 install sherpa-onnx==1.10.46"; exit 1; }
fi

fetch() {  # fetch <repo> <file> <dest_dir>
  local repo="$1" file="$2" dest="$3"
  local out="$dest/$file"
  mkdir -p "$(dirname "$out")"
  if [ -s "$out" ]; then
    echo "   ✓ 已存在 $file ($(du -h "$out" | cut -f1))"
    return 0
  fi
  echo "   ↓ $repo/$file"
  curl -fL --retry 3 --retry-delay 2 -C - --max-time 900 \
       -o "$out.part" "$HF/$repo/resolve/main/$file" || {
    echo "!! 下载失败: $file"; rm -f "$out.part"; return 1; }
  mv "$out.part" "$out"
}

echo "== 2/3 下载中文 ASR 模型（Paraformer int8）"
ASR_REPO="csukuangfj/sherpa-onnx-paraformer-zh-2023-09-14"
ASR_DIR="$MODELS/sherpa-onnx-paraformer-zh-2023-09-14"
fetch "$ASR_REPO" model.int8.onnx "$ASR_DIR"
fetch "$ASR_REPO" tokens.txt "$ASR_DIR"

echo "== 3/3 下载中文 TTS 模型（VITS Melo zh_en int8）"
TTS_REPO="csukuangfj/vits-melo-tts-zh_en"
TTS_DIR="$MODELS/vits-melo-tts-zh_en"
fetch "$TTS_REPO" model.int8.onnx "$TTS_DIR"
fetch "$TTS_REPO" tokens.txt "$TTS_DIR"
fetch "$TTS_REPO" lexicon.txt "$TTS_DIR"
for f in dict/hmm_model.utf8 dict/idf.utf8 dict/jieba.dict.utf8 \
         dict/pos_dict/char_state_tab.utf8 dict/pos_dict/prob_emit.utf8 \
         dict/pos_dict/prob_start.utf8 dict/pos_dict/prob_trans.utf8 \
         dict/stop_words.utf8 dict/user.dict.utf8 \
         date.fst number.fst phone.fst new_heteronym.fst; do
  fetch "$TTS_REPO" "$f" "$TTS_DIR"
done

echo
echo "== 检查"
ls -lh "$ASR_DIR"/model.int8.onnx "$ASR_DIR"/tokens.txt 2>/dev/null
ls -lh "$TTS_DIR"/model.int8.onnx "$TTS_DIR"/tokens.txt "$TTS_DIR"/lexicon.txt 2>/dev/null
echo
echo "完成。重启 Web 服务生效：sudo systemctl restart go2-web"
