#!/bin/bash
# teardown_ap.sh —— 关闭 AP 热点，还原网卡（让机器人能重新连 WiFi 联网）
# 用法: sudo bash teardown_ap.sh [网卡名]   默认 wlan0

set +e
IFACE="${1:-wlan0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREV_FILE="/tmp/go2_ap_prev_wifi"

if [ "$EUID" -ne 0 ]; then
  echo "❌ 需要 root。请用: sudo bash $0 $@"
  exit 1
fi

echo ">> 停止 hostapd / AP dnsmasq ..."
# 只杀本脚本用 /tmp 配置起的那两个，别误杀系统 dnsmasq（127.0.0.1:53）
pkill -f "hostapd.*/tmp/hostapd_ap.conf" 2>/dev/null
pkill -f "hostapd.*$SCRIPT_DIR/hostapd.conf" 2>/dev/null
pkill -f "dnsmasq.*/tmp/dnsmasq_ap.conf" 2>/dev/null
sleep 1

echo ">> 还原 $IFACE ..."
ip link set "$IFACE" down 2>/dev/null
ip addr flush dev "$IFACE" 2>/dev/null
ip link set "$IFACE" up 2>/dev/null

echo ">> 清理 NAT 规则 ..."
iptables -t nat -F POSTROUTING 2>/dev/null
iptables -F FORWARD 2>/dev/null

echo ">> 交还给 NetworkManager ..."
# 只还 wlan0；NetworkManager 本身一直在跑（eth0 是机器人内网，不能停）
nmcli device set "$IFACE" managed yes 2>/dev/null
sleep 2

# 显式重连原来的 WiFi（NM 通常会自动重连，这里兜底一次）
PREV_WIFI=$(cat "$PREV_FILE" 2>/dev/null)
if [ -n "$PREV_WIFI" ]; then
  echo ">> 重连原 WiFi: $PREV_WIFI"
  nmcli connection up "$PREV_WIFI" 2>&1 | sed 's/^/   /'
fi

echo ""
echo "✅ 已关热点。网卡 $IFACE 已还原。"
echo "   若没自动联网，用 nmtui 或 nmcli 连回你的 WiFi。"
