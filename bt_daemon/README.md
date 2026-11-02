# bt_daemon — Go2 蓝牙配网守护进程

从 G1 的 `/home/unitree/huang/bt_daemon` 迁移过来。手机通过经典蓝牙 SPP 连上这台
机器，用一行 JSON 命令让它扫 WiFi、连 WiFi、切 AP 热点，并回报自己的 IP。

这样手机在机器人还没接入任何 WiFi 时也能把它配置上网。

## 协议

一行 JSON 请求 → 一行 JSON 响应（`\n` 分隔）。

| 命令 | 说明 |
| --- | --- |
| `{"cmd":"status"}` | 返回 `wlan_mode` / `ssid` / `host_ip` / `app_py_running` |
| `{"cmd":"scan"}` | 扫描周围 WiFi，返回 `aps[]`（ssid / signal / security） |
| `{"cmd":"connect","ssid":"x","psk":"y"}` | 连指定 WiFi；失败自动回滚到原连接 |
| `{"cmd":"ap"}` | 开热点 `Unitree-Go2`（密码 `12345678`，网关 `192.168.124.1`） |
| `{"cmd":"wifi"}` | 关热点、回 WiFi |
| `{"cmd":"reboot_app"}` | 重启 web 面板（systemd 约 2s 自动拉起） |
| `{"cmd":"handshake"}` | G1 的机械臂动作；Go2 没有机械臂，返回明确的不支持 |

## 安装

```bash
sudo cp go2-net-btctl.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now go2-net-btctl
systemctl status go2-net-btctl
journalctl -u go2-net-btctl -f
```

启动日志里应能看到：

```
适配器已设为 Unitree-Go2（可被发现/可配对）
已注册 SPP profile (UUID=00001101-..., channel=1)
已注册无头自动配对 agent (KeyboardDisplay)
```

手机蓝牙里搜 **`Unitree-Go2`**（本机适配器 `9C:C7:D3:F6:A9:20`）。注意别和
机器人主板的 `Go2_63329`（`14:0A:02:F8:BC:1B`）搞混——那是宇树 App 用的 BLE，
不提供 SPP。

## 与 G1 版本的差异

| 点 | G1 | Go2（本目录） |
| --- | --- | --- |
| AP 实现 | `setup_ap.sh` 里 `systemctl stop NetworkManager` | 只 `nmcli device set wlan0 managed no`，**不停 NM**（eth0 是机器人内网控制链路，停 NM 有断链风险） |
| SSID | `Unitree-G1` | `Unitree-Go2` |
| 适配器名 | 写在 `/etc/bluetooth/main.conf` 的 `Name = Unitree-G1` | 运行时用 D-Bus 设 `Alias`，不改 main.conf、不重启 bluetoothd |
| `handshake` | 调 app.py `/api/execute` 做机械臂握手 | 返回不支持（Go2 无机械臂） |
| `reboot_app` | `pkill python app.py`（:5000） | `pkill -f web/app.py`（:8080，systemd 拉起） |
| `scan` | 直接读 nmcli 缓存 | 先 `nmcli dev wifi rescan`，否则只回当前 AP |
| 参数传递 | 拼 shell 字符串 | argv 列表，SSID/密码带引号不会注入 |
| watchdog 清理 | `pkill -x dnsmasq`、`pkill python app.py` | 只杀 `/tmp/dnsmasq_ap.conf` 起的那个；**不动**系统 dnsmasq（127.0.0.1:53）和 web 面板 |

## 注意

- **开 AP 会切断 wlan0**，如果面板正是通过这块网卡访问的，页面会失联，只能连热点
  `Unitree-Go2` 后访问 `http://192.168.124.1:8080`。
- 看门狗兜底：热点起来 300s 无设备连接、或设备全断 180s，会自动 teardown 回 WiFi。
  可用 `AP_WATCHDOG_TIMEOUT` / `AP_IDLE_TIMEOUT` 覆盖。
- 测试时别在只靠 wlan0 的远程会话里直接 `{"cmd":"ap"}`。
- 只支持经典蓝牙 SPP。BLE-only 的设备连不上。
