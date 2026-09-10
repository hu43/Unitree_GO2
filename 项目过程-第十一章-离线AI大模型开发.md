# 第十一章 离线AI大模型开发 — 项目过程记录

> 课程：Yahboom Jetson Orin Nano SUPER（https://www.yahboom.com/build?id=14774&cid=694 起）
> 硬件：Yahboom Orin Nano SUPER 机器狗（Go2 生态），L4T R35.3.1 / JetPack 5.1.1，RAM 7.2G，NVMe 790G
> 时间：2026-09-02 刷机引导 → 09-07 Ollama 部署 → 09-08/09 多模态应用全部跑通
> 本文记录实际执行过程、遇到的问题与最终解法，与官方教程的差异点全部标注。

## 0. 课程章节与实际完成情况

| 课程章节 | 主题 | 实际情况 |
|---|---|---|
| 1. AI大模型环境部署 | Ollama 安装 | ✅ 改用手动部署（官方脚本被墙） |
| 2. 中文输入法切换 | — | 未涉及 |
| 3. 大模型对话平台安装 | — | 未涉及 |
| 4-8. Llama3.2 / Qwen3 / Phi-4 / DeepSeek-R1 / Qwen2.5vl | 文本模型 | 未做（按需） |
| 9. 谷歌 Gemma3 | 视觉多模态模型 | ✅ v0.33.3 + gemma3:4b |
| 10-11. Llava / MiniCPM-V | 备选视觉模型 | 未做（gemma3:4b 够用） |
| 12. 多模态视觉理解 | seewhat 工具 | ✅ 改用 Go2 相机（免 USB 相机） |
| 13. 多模态文生图 | — | 未做 |
| 14. 多模态视频分析 | analyze_video | ✅ |
| 15. 多模态视觉定位 | visual_positioning | ✅ |
| 16. 多模态表格扫描 | scan_table | ✅ |
| 17. 多模态自主代理 | agent_call | 未做（多轮已验证可用） |

## 1. 环境部署：Ollama（与教程差异最大的一章）

### 1.1 教程方法失效

教程命令 `sudo curl -fsSL https://ollama.com/install.sh | sh` 在机器人网络下失败：

- `ollama.com` 时通时断（GFW），首次尝试 `curl: (52) Empty reply from server`
- 更致命的是 install.sh 下载 tarball 走 `github.com`，机器人直连 GitHub 完全不通（`000`）

### 1.2 下载通道实测

| 通道 | 结果 |
|---|---|
| 机器人 → github.com | ❌ 不通 |
| 机器人 → ollama.com | ⚠️ 不稳定 |
| 机器人 → ghfast.top | ❌ 大文件卡死（0 字节挂起） |
| 机器人 → gh-proxy.com | ✅ ~1.4MB/s |
| 本机 → github.com 直连 | ✅ 2MB/s（会被限速，并行可绕） |

**最终方案：本机 4/6 路并行 range 下载 → scp 到机器人**（LAN ~5.4MB/s），比机器人直接拉快数倍。

### 1.3 JetPack5 需要两个包（教程没提的关键点）

- 基础包 `ollama-linux-arm64.tar.zst`（1.55G）：主程序
- 插件包 `ollama-linux-arm64-jetpack5.tar.zst`（297M）：**只含** `lib/ollama/cuda_jetpack5/` CUDA 库，必须叠装

两个包都解压到 `/usr`。只装插件包会报 `/usr/bin/ollama: not found`。

### 1.4 systemd 服务与 GPU

- 创建 `ollama` 服务用户；**必须 `usermod -aG video ollama`**——Jetson 的 `/dev/nvmap` 等 GPU 设备节点属 `video` 组，不加组则 GPU 检测不到（只走 CPU）
- `OLLAMA_MODELS=/home/unitree/hu/ai/ollama_models`（文件集中在 hu/ai）
- `OLLAMA_CONTEXT_LENGTH`：模型默认 16384 在 7.2G 机器上必 OOM，最终定 **8192**（4096 会限制多轮+图像；另加 4G `/swapfile` 兜底）

## 2. Gemma3 部署（绕过模型拉取劫持）

`ollama pull gemma3:4b` 失败：blob 重定向到 Cloudflare R2 后 TLS 证书被网络劫持
（`x509: certificate is not valid for any names ... r2.cloudflarestorage.com`）。

**方案：本机走 registry.ollama.ai 手动下载 → 校验 → scp 放置**

1. `GET https://registry.ollama.ai/v2/library/gemma3/manifests/4b` 取 manifest（1 大 4 小共 5 个 blob）
2. 主 blob 3.34GB 用 6 路并行 range 下载（单连接会被限速到 80KB/s；分片后合计 ~5-18MB/s）
3. `sha256sum` 逐个校验（曾因后台任务被杀导致分片错位，靠校验发现并重下）
4. 放置：
   - blobs → `ollama_models/blobs/sha256-<hex>`
   - manifest → `ollama_models/manifests/registry.ollama.ai/library/gemma3/4b`
5. `ollama list` 识别 → `ollama run gemma3:4b` 视觉实测通过（能读图中中文）

注意：`ollama_models` 子目录属主是服务用户 `ollama`，手动放置需 `chown`/`chmod g+w`。

## 3. largemodel 多模态应用（ch12/14/15/16）

### 3.1 配置（教程步骤，均完成）

- `config/yahboom.yaml`：`llm_platform: 'ollama'`
- `config/large_model_interface.yaml`：`ollama_model: "gemma3:4b"`（教程示例 llava，换更强的 gemma3）

### 3.2 测试结果

| 章节 | 指令 | 耗时 | 结果 |
|---|---|---|---|
| 16 表格扫描 | "分析一下表格" | ~19s | Markdown 表格写入 `scan_table/test_table_table.md` |
| 15 视觉定位 | "分析一下图片中齿轮的位置" | ~34s | 7 个齿轮 `[cx,cy,w,h]` 坐标写入 md |
| 14 视频分析 | "分析一下这个视频" | ~47s | 正确描述测试视频（红球动画+文字） |
| 12 视觉理解 | "你看到了什么" | ~26s | 正确描述 Go2 相机实时画面 |

### 3.3 三个根因修复（"昨天能跑今天不行"复盘）

**① `format='json'` 严格模式让 gemma3 退化（最核心）**

`large_model_interface.py` 的工具规划调用带 `format='json'`。消融实验（curl 直连 ollama 重放真实 prompt）证明：
- `format=json` → 确定性输出空 `{}` 或拒绝填 `tools`（模型认为需要向用户要图片）
- 去掉约束（自由模式）→ 模型自然输出 ```json 包裹的工具调用，`image_path: ""` 正好触发系统自动默认文件

修复：规划调用改 `options={'temperature': 0.6}`、去掉 `format='json'`，依赖代码里现成的 markdown JSON 提取器。

**② 对话历史污染的放大效应**

- 发布端一次发 10 条重复消息 → 每条都进对话历史 → 错误回复成为 few-shot 自我强化
- 重置 `conversation_message.json` 文件**不会清内存历史**，必须重启服务
- 修复：`pub_cmd.py` 改为等待订阅者后**单发**；上下文升到 8192 抗多轮+图像 token

**③ DDS 环境（段错误与 bad_alloc）**

- 主机重启后 Foxy 自带 FastRTPS/旧 rmw 出现竞态：`model_service` 原生段错误、`ros2` CLI 随机崩、`text_chat` 陷入 `bad_alloc caught: std::bad_alloc` 循环（FastRTPS 共享内存传输的著名问题）
- 修复：统一改用 `~/cyclonedds_ws` 自编译的**新版 rmw_cyclonedds**；largemodel 用独立配置**只绑 wlan0**（`cyclonedds_lm.xml`），不吃 Go2 内网 eth0 的 DDS 洪水；狗侧 DDS 完全不动

### 3.4 相机方案（ch12 免 USB 相机）

机器人无 `/dev/video*`（且缺相机时 `model_service` 相机初始化会段错误，已加跳过守卫）。改走 Unitree 官方 DDS 服务：

```
videohub 服务(API 1001 GetImageSample, DDS over eth0)
  → camera_daemon.py 常驻抓帧(1.5s/帧, 断线自动重连)
  → ~/hu/ai/camera_latest.jpg
  → largemodel capture_frame 回退补丁读取
  → gemma3:4b 视觉推理 → 自然语言描述
```

依赖：`pylibs/` 内自编译 `cyclonedds==0.10.2` Python 绑定（`CYCLONEDDS_HOME` 指向 `~/cyclonedds_ws/install/cyclonedds`）+ `unitree_sdk2_python`。

## 4. 踩坑速查表

| 症状 | 根因 | 解法 |
|---|---|---|
| install.sh `curl: (52)` | GFW | 本机并行下载 + scp |
| `/usr/bin/ollama: not found` | jetpack5 包只是 CUDA 插件 | 基础包+插件包都装 |
| GPU 检测不到（CPU 推理） | ollama 用户不在 video 组 | `usermod -aG video ollama` |
| 视觉推理 500 / runner killed | 16K 上下文 + 8G prompt cache OOM | ctx=8192 + 4G swap |
| 规划返回 `{}` | `format='json'` + gemma3 退化 | 改自由模式 temp 0.6 |
| 连续 0 tools | 历史污染（内存不清） | 重置文件 + 重启服务 |
| `text_chat` bad_alloc 循环 | FastRTPS 共享内存 | 用 `text_chat.sh`（cyclonedds + 正确配置） |
| `ros2 topic pub` 随机段错误 | 旧 rmw 竞态 | 换新版 rmw 覆盖层 |
| 拉模型 TLS x509 报错 | R2 被劫持 | 本机下载 blob 手动放置 |
| `pkill -f` 自杀 (exit 144/255) | 模式匹配到自己命令行 | `[x]` 方括号技巧 |

## 5. 日常使用

```bash
bash ~/hu/ai/restart_largemodel.sh        # 起服务
bash ~/hu/ai/text_chat.sh                 # 交互对话
python3 ~/hu/ai/pub_cmd.py "你看到了什么"   # 或脚本单发
tail -f ~/hu/ai/largemodel.log            # 看结果
```
