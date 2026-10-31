#!/bin/bash
# ap_watchdog.sh —— AP 热点看门狗（安全网）
#
# 背景：开 AP 那一刻机器人断开当前 WiFi → 原 IP 不可达 → SSH 进不去。
# 若手机一直没连上来，或连过之后断开走人，机器人会卡在 AP 模式无人能进。
#
# 本脚本状态机（两个超时）：
#   1. 首连超时（TIMEOUT，默认 300s=5 分钟）：
#      AP 起来后若这段时间内一直无任何设备连过 → 自动 teardown 回 WiFi。
#   2. 空闲超时（IDLE_TIMEOUT，默认 180s=3 分钟）：
#      一旦有设备连过，进入"已连接"态；之后所有设备全断开 → 重新开始空闲倒计时，
#      超过 IDLE_TIMEOUT 仍无设备 → 自动 teardown 回 WiFi。
#
# 用法（由 setup_ap.sh 后台调用）：
#   sudo bash ap_watchdog.sh <网卡> [首连超时秒] [原WiFi连接名] [空闲超时秒]
#
# 检测手段（双保险）：
#   1. iw dev <iface> station dump —— hostapd 软 AP 下已关联的 STA
#   2. /tmp/dnsmasq_ap.leases       —— DHCP 已发租约（拿到 IP 才算）
#
# ⚠️ 与 G1 版本的关键差异：do_teardown 里不再 pkill app.py（Go2 上那是 web
#    面板，杀了会连带面板重启），也不再 pkill -x dnsmasq（会误杀系统 dnsmasq）。

set +e
IFACE="${1:-wlan0}"
TIMEOUT="${2:-${AP_WATCHDOG_TIMEOUT:-300}}"
PREV_WIFI="$3"
IDLE_TIMEOUT="${4:-${AP_IDLE_TIMEOUT:-180}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$EUID" -ne 0 ]; then
  echo "[watchdog] ❌ 需要 root。由 sudo 调用方继承，勿直接跑" >&2
  exit 1
fi

# 计算当前已连接设备数（STA 关联数 + DHCP 租约数）
count_clients() {
  local sta=0 lease=0
  if command -v iw >/dev/null 2>&1; then
    sta=$(iw dev "$IFACE" station dump 2>/dev/null | grep -c '^Station')
  fi
  lease=$(grep -c . /tmp/dnsmasq_ap.leases 2>/dev/null)
  sta=${sta:-0}; lease=${lease:-0}
  echo "$((sta + lease))"
}

# 关热点 + 回 WiFi。首连超时 / 空闲超时 都走这里。
do_teardown() {
  local reason="$1"
  echo "[watchdog] ⏰ $reason → 自动 teardown 回 WiFi"
  pkill -f "hostapd.*/tmp/hostapd_ap.conf" 2>/dev/null
  pkill -f "dnsmasq.*/tmp/dnsmasq_ap.conf" 2>/dev/null
  sleep 1
  bash "$SCRIPT_DIR/teardown_ap.sh" "$IFACE" 2>&1 | sed 's/^/[watchdog] /'
  if [ -n "$PREV_WIFI" ]; then
    echo "[watchdog] 尝试重连 $PREV_WIFI ..."
    nmcli connection up "$PREV_WIFI" 2>&1 | sed 's/^/[watchdog] /' || true
  fi
  echo "[watchdog] ✅ 已回 WiFi。可重新 SSH 到原 IP"
  exit 0
}

echo "[watchdog] 启动: 网卡=$IFACE 首连超时=${TIMEOUT}s 空闲超时=${IDLE_TIMEOUT}s ${PREV_WIFI:+原WiFi=$PREV_WIFI}"
echo "[watchdog] 1) ${TIMEOUT}s 内无设备连过 → 回 WiFi"
echo "[watchdog] 2) 设备连过又全断 → ${IDLE_TIMEOUT}s 后回 WiFi"

# ===== 阶段 1: 等首连 =====
START=$(date +%s)
while :; do
  sleep 15
  N=$(count_clients)
  if [ "$N" -gt 0 ]; then
    echo "[watchdog] ✅ 首次检测到设备连接 (clients=$N), 进入空闲监控"
    break
  fi
  ELAPSED=$(( $(date +%s) - START ))
  if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
    do_teardown "首连超时 ${TIMEOUT}s 内无设备连接"
  fi
  echo "[watchdog] 等待首连... ${ELAPSED}/${TIMEOUT}s 仍无设备"
done

# ===== 阶段 2: 空闲监控 =====
IDLE_START=0
while :; do
  sleep 15
  N=$(count_clients)
  if [ "$N" -gt 0 ]; then
    if [ "$IDLE_START" -ne 0 ]; then
      echo "[watchdog] 设备重新连接 (clients=$N), 空闲计时取消"
      IDLE_START=0
    fi
  else
    if [ "$IDLE_START" -eq 0 ]; then
      IDLE_START=$(date +%s)
      echo "[watchdog] 所有设备已断开, 开始空闲倒计时 ${IDLE_TIMEOUT}s"
    else
      IDLE_ELAPSED=$(( $(date +%s) - IDLE_START ))
      if [ "$IDLE_ELAPSED" -ge "$IDLE_TIMEOUT" ]; then
        do_teardown "空闲超时 设备全断 ${IDLE_TIMEOUT}s"
      fi
      echo "[watchdog] 空闲中... ${IDLE_ELAPSED}/${IDLE_TIMEOUT}s 仍无设备"
    fi
  fi
done
