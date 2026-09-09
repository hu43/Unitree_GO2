# go2_AI

Yahboom Jetson Orin Nano SUPER 机器狗 + Unitree Go2 生态上的**离线大模型与多模态应用**实践（教程第 11 章，Yahboom `largemodel` ROS2 工程）。

硬件/环境：L4T R35.3.1 (JetPack 5.1.1)、ROS2 Foxy、Ollama v0.33.3、`gemma3:4b`（视觉多模态）、Go2 前置相机（DDS `videohub` 服务）。

## 目录

| 文件 | 作用 |
|---|---|
| `camera_daemon.py` | 通过 Unitree DDS `videohub` 常驻抓取 Go2 前置相机帧 → `~/hu/ai/camera_latest.jpg`（断线自动重连） |
| `pub_cmd.py` | 向 largemodel 的 `/asr` 话题发指令（等待订阅者后**单发**，避免污染对话历史） |
| `text_chat.sh` | 教程官方 `text_chat` 交互界面的一键包装（自动配置正确的 DDS/RMW 环境） |
| `restart_largemodel.sh` | 重启 `model_service`，自动生成 wlan0 绑定的 DDS 配置 |
| `cyclonedds_lm.xml(.tpl)` | largemodel 专用 CycloneDDS 配置（只绑 wlan0，隔离 Go2 内网 eth0 的 DDS 洪水；狗侧 DDS 不受影响） |
| `lm_params.yaml` | `model_service` 参数示例（人工调试用） |
| `test_image.jpg` / `test_table.jpg` / `test_video.mp4` | 视觉定位 / 表格扫描 / 视频分析的测试素材 |
| `多模态章节完成报告.md` | 四章（视觉理解/视频分析/视觉定位/表格扫描）完成记录与根因分析 |
| `patches/` | largemodel 源码补丁 + 问题排查用的消融脚本 |

## 快速上手

```bash
# 1. 起服务（Ollama + model_service 需已就绪，见下）
bash ~/hu/ai/restart_largemodel.sh

# 2. 交互对话
bash ~/hu/ai/text_chat.sh
# > 你看到了什么        （视觉理解，读 Go2 前置相机）
# > 分析一下表格        （表格扫描）
# > 分析一下这个视频    （视频分析）
# > 分析一下图片中齿轮的位置  （视觉定位）

# 或脚本单发
python3 ~/hu/ai/pub_cmd.py "你看到了什么"
# 查看结果
tail -f ~/hu/ai/largemodel.log
```

前置依赖：相机需 `camera_daemon.py` 常驻；Ollama 用 `gemma3:4b`（视觉多模态）本地推理。

## 踩坑记录（关键结论）

1. **Ollama 上下文**：`OLLAMA_CONTEXT_LENGTH=8192`（16K + 8G prompt cache 在 7.2G Jetson 上会 OOM；已加 4G `/swapfile` 兜底）。
2. **工具规划退化**：`large_model_interface.py` 里 `format='json'` 严格模式会让 gemma3 确定性输出空 `{}` 或拒绝填 tools；改为**自由模式**（temp 0.6）+ 保留原有 ```json markdown 提取器（见 `patches/patch_freeform.py`）。
3. **对话历史污染**：重置 `resources_file/conversation_message.json` 后必须**重启服务**（内存历史不会自动清）。
4. **RMW 环境**：用 `~/cyclonedds_ws` 里编译的新版 `rmw_cyclonedds`（Foxy 自带 FastRTPS 在高负载下会疯狂 `bad_alloc`——`text_chat` 报错根因）。
5. **无 USB 相机**：`model_service` 相机初始化在无 `/dev/video*` 时会段错误，加跳过守卫（`patches/patch_camskip.py`）；画面改走 Go2 DDS 相机回退（`patches/patch_capture.py`）。

详细内容见 `多模态章节完成报告.md`。