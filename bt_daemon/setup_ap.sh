#!/bin/bash
# setup_ap.sh —— 在 Go2 机载电脑上开 WiFi 热点（由 go2_net_btctl 调用）
# 用法: sudo bash setup_ap.sh [网卡名] [网关IP]
#   默认 网卡=wlan0  网关=192.168.124.1
# 结果: 手机连 SSID `Unitree-Go2`（密码 12345678）后访问 http://192.168.124.1:8080
#
# ⚠️ 与 G1 版本的关键差异：这里只把 wlan0 从 NetworkManager 摘出来，
#    不停 NetworkManager 本身。Go2 的 eth0（192.168.123.18）是机器人内网
#    控制链路，整体停 NM 有把运控链路一起断掉的风险。
# ⚠️ 网关用 124.1 不用 123.1：eth0 是 192.168.123.x，同段会路由冲突。

set -e

IFACE="${1:-${AP_IFACE:-wlan0}}"
GATEWAY="${2:-${AP_GATEWAY:-192.168.124.1}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOSTAPD_CONF="$SCRIPT_DIR/hostapd.conf"
DNSMASQ_CONF="$SCRIPT_DIR/dnsmasq.conf"
DNSMASQ_RUN="/tmp/dnsmasq_ap.conf"
HOSTAPD_RUN="/tmp/hostapd_ap.conf"
PREV_FILE="/tmp/go2_ap_prev_wifi"
WATCHDOG_TIMEOUT="${AP_WATCHDOG_TIMEOUT:-300}"  # 首连超时: AP 起 N 秒无设备连过 → 回 WiFi
IDLE_TIMEOUT="${AP_IDLE_TIMEOUT:-180}"          # 空闲超时: 设备连过又全断 N 秒 → 回 WiFi

echo "================================================"
echo "  Go2 AP 直连热点启动"
echo "  网卡: $IFACE"
echo "  网关: $GATEWAY"
echo "  SSID: Unitree-Go2  密码: 12345678"
echo "  手机访问: http://$GATEWAY:8080"
echo "================================================"

# ===== 0. root 检查 =====
if [ "$EUID" -ne 0 ]; then
  echo "❌ 需要 root。请用: sudo bash $0 $@"
  exit 1
fi

# ===== 1. 依赖检查 =====
for cmd in hostapd dnsmasq ip iptables; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "❌ 缺少命令: $cmd"
    exit 1
  fi
done
[ -f "$HOSTAPD_CONF" ] || { echo "❌ 找不到 $HOSTAPD_CONF"; exit 1; }
[ -f "$DNSMASQ_CONF" ] || { echo "❌ 找不到 $DNSMASQ_CONF"; exit 1; }

# ===== 2. 确认网卡存在 =====
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "❌ 网卡 $IFACE 不存在。当前网卡:"
  ip -br link
  exit 1
fi

# ===== 3. 只摘 wlan0：记下当前 WiFi，交还/停用 wpa_supplicant =====
PREV_WIFI=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | grep ":$IFACE" | cut -d: -f1 | head -1)
echo "$PREV_WIFI" > "$PREV_FILE"
echo ">> 释放 $IFACE（AP 前连着: ${PREV_WIFI:-无}）..."
nmcli device set "$IFACE" managed no 2>/dev/null || true
pkill -f "wpa_supplicant.*-i $IFACE" 2>/dev/null || true
sleep 1

# ===== 4. 清理可能残留的 AP 进程（只杀本目录配置起的）=====
pkill -f "dnsmasq.*$DNSMASQ_RUN" 2>/dev/null || true
pkill -f "hostapd.*$HOSTAPD_RUN" 2>/dev/null || true
sleep 1

# ===== 5. 设网卡固定 IP（网关）=====
echo ">> 设置 $IFACE 静态 IP = $GATEWAY ..."
ip link set "$IFACE" down
ip addr flush dev "$IFACE"
ip addr add "$GATEWAY/24" dev "$IFACE"
ip link set "$IFACE" up

# ===== 6. 开启转发 + NAT（可选；纯直连其实不需要外网）=====
echo ">> 开启 IP 转发 + NAT ..."
sysctl -w net.ipv4.ip_forward=1 >/dev/null
iptables -F FORWARD 2>/dev/null || true
UPLINK=$(ip route show default 2>/dev/null | awk '{print $5; exit}')
if [ -n "$UPLINK" ] && [ "$UPLINK" != "$IFACE" ]; then
  echo "   上联网卡: $UPLINK (做 masquerade)"
  iptables -t nat -F POSTROUTING 2>/dev/null || true
  iptables -t nat -A POSTROUTING -o "$UPLINK" -j MASQUERADE 2>/dev/null || true
  iptables -A FORWARD -i "$IFACE" -o "$UPLINK" -j ACCEPT 2>/dev/null || true
  iptables -A FORWARD -i "$UPLINK" -o "$IFACE" -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
else
  echo "   无上联网卡（纯 AP 直连模式），跳过 NAT"
fi

# ===== 7. 生成 dnsmasq 配置（把模板里的网关/网卡替换掉）=====
sed "s|192.168.124.1|$GATEWAY|g; s|interface=wlan0|interface=$IFACE|g" "$DNSMASQ_CONF" > "$DNSMASQ_RUN"
echo ">> dnsmasq 配置已生成: $DNSMASQ_RUN"

# ===== 8. 生成 hostapd 配置（同步网卡名）=====
sed "s|^interface=.*|interface=$IFACE|" "$HOSTAPD_CONF" > "$HOSTAPD_RUN"

# ===== 9. 启动 dnsmasq（DHCP+DNS，只监听 AP 网段，不和系统 dnsmasq 抢 127.0.0.1:53）=====
echo ">> 启动 dnsmasq ..."
dnsmasq --conf-file="$DNSMASQ_RUN"
echo "   dnsmasq 已启动"

# ===== 10. 启动 hostapd（前台阻塞）=====
echo ">> 启动 hostapd（前台运行，退出会停掉热点）..."
nohup bash "$SCRIPT_DIR/ap_watchdog.sh" "$IFACE" "$WATCHDOG_TIMEOUT" "$PREV_WIFI" "$IDLE_TIMEOUT" \
  > /tmp/ap_watchdog.log 2>&1 &
WATCHDOG_PID=$!
echo "   看门狗 PID=$WATCHDOG_PID（首连 ${WATCHDOG_TIMEOUT}s / 空闲 ${IDLE_TIMEOUT}s 自动回 WiFi，日志 /tmp/ap_watchdog.log）"
hostapd "$HOSTAPD_RUN"

# hostapd 退出后自动清理
kill "$WATCHDOG_PID" 2>/dev/null || true
pkill -f "ap_watchdog.sh.*$IFACE" 2>/dev/null || true
echo ">> hostapd 已退出，清理中..."
bash "$SCRIPT_DIR/teardown_ap.sh" "$IFACE" || true
