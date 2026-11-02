# go2-netd — 网络/蓝牙配置守护进程

「连接机器人」向导的后端。跑在 **宿主机**（Jetson）、以 root 运行、只监听 `127.0.0.1:8099`。

## 为什么单独一个进程

web 后端 `app.py` 跑在 `go2_humble` 容器里。容器虽然是 host 网络，但镜像是精简的
`ros:humble-base`：没有 `nmcli`、没有 `bluetoothctl`，也没挂 D-Bus，拿不到
NetworkManager 和 BlueZ。所以特权网络操作都放在宿主这一侧，容器通过
`/api/net/*` 代理过来（见 `app.py` 里的 `_netd_proxy`）。

## 接口

全部 GET，查询串传参，返回 JSON。`ok=false` 时 `msg` 是给人看的失败原因。

| 路径 | 说明 |
| --- | --- |
| `/status` | 当前网络状态：`wlan_mode` / `host_ip` / `gateway` / `ssid` / `app_py_running` |
| `/wifi/scan` | 让 wlan0 重新扫描，返回去重后的 AP 列表（SSID/信号/加密） |
| `/wifi/connect?ssid=&psk=` | 连指定 WiFi；失败自动回滚到原连接。`dry_run=1` 只回显命令 |
| `/wifi/last` | 上一次 WiFi 切换的结果（页面失联后回来可以查） |
| `/ap/on?ssid=&psk=` | 把 wlan0 切成热点（`ipv4.method shared`，自带 DHCP），默认 `Unitree-Go2` / `12345678` / `192.168.124.1`。`dry_run=1` 只回显命令 |
| `/ap/off` | 关热点并恢复之前的 WiFi |
| `/bt/scan?timeout=` | 蓝牙发现（经典 + BLE），返回 `devices[]`，标出已配对；附带 `adapter` 状态，失败原因放 `msg`（不抛 500） |
| `/bt/info` | 本机蓝牙适配器：名称/地址/是否开启/是否可被发现/是否可配对 + 已配对数量 |
| `/bt/paired` | 已配对设备 |
| `/bt/connect?mac=` | 配对（未配对时）+ 建立 RFCOMM SPP 会话，成功后先发一次 `status` 做数据层校验 |
| `/bt/cmd?cmd=...` | 在 SPP 会话上发一行 JSON、读一行 JSON 响应 |
| `/bt/disconnect` | 断开 SPP 会话 |
| `/health` | 存活探测 |

`/bt/cmd` 的协议与 G1 的 `bt_daemon` 对齐：`{"cmd":"status"}`、`{"cmd":"scan"}`、
`{"cmd":"connect","ssid":..,"psk":..}`、`{"cmd":"ap"}`、`{"cmd":"wifi"}`。
前端因此可以不关心当前走的是蓝牙还是本机后端。

## 安装

```bash
sudo cp go2-netd.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now go2-netd
systemctl status go2-netd
curl -s http://127.0.0.1:8099/status
```

## 注意

- **`/wifi/connect` 会切换 wlan0**，如果面板正是通过这块网卡访问的，页面会失联。
  后端在拿不到 IP 时会自动回滚到原连接，结果记在 `/run/go2-netd-state.json`。
- **`/ap/on` 同样会断开当前 WiFi**，之后只能连热点访问。恢复用 `/ap/off`
  （或向导里的「关闭热点」）。
- 测试时用 `dry_run=1` 先看命令，别在远程会话里直接切网卡。
- 蓝牙 SPP 默认走 channel 1（与 G1 一致）。对端没有 `bt_daemon` 时会报
  `SPP 连接失败: Host is down` 或「对端没回 status」，属正常。
