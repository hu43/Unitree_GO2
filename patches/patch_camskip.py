p = "/home/unitree/hu/ai/yahboom_ws/src/largemodel/largemodel/model_service.py"
s = open(p, encoding="utf-8").read()
old = """        try:
            self.get_logger().info("Attempting to initialize USB camera with auto-detected device")"""
new = """        # Skip entirely when no V4L device exists (prevents OpenCV/GStreamer segfault on this box).
        # Go2 front camera via DDS fallback (tools_manager.capture_frame) provides frames instead.
        import glob as _glob
        if not _glob.glob("/dev/video*"):
            self.get_logger().info("No /dev/video* device found; skipping USB camera init (Go2 DDS camera fallback will be used).")
            return False
        try:
            self.get_logger().info("Attempting to initialize USB camera with auto-detected device")"""
if old in s:
    open(p, "w", encoding="utf-8").write(s.replace(old, new, 1))
    print("PATCH-OK")
elif "Go2 DDS camera fallback will be used" in s:
    print("ALREADY-PATCHED")
else:
    print("PATTERN-NOT-FOUND")
