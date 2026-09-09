import io

p = "/home/unitree/hu/ai/yahboom_ws/src/largemodel/utils/tools_manager.py"
s = open(p, encoding="utf-8").read()

old = '''        if not self.node.camera_initialized or self.node.cap is None:
            if self.node.language == 'zh':
                error_msg = "摄像头未初始化或未就绪，无法捕获画面。"
            else:
                error_msg = "Camera not initialized or not ready, unable to capture frame."
            self.node.get_logger().error(error_msg)
            return None'''

new = '''        if not self.node.camera_initialized or self.node.cap is None:
            # Fallback for Go2 front camera: read latest frame saved by camera_daemon (DDS videohub).
            fallback_img = os.path.expanduser("~/hu/ai/camera_latest.jpg")
            if os.path.exists(fallback_img):
                frame = cv2.imread(fallback_img)
                if frame is not None:
                    self.node.last_frame = frame
                    resources_dir = os.path.join(self.node.pkg_path, "resources_file")
                    os.makedirs(resources_dir, exist_ok=True)
                    import time
                    timestamp = int(time.time() * 1000)
                    temp_image_path = os.path.join(resources_dir, "captured_frame_{}.jpg".format(timestamp))
                    success = cv2.imwrite(temp_image_path, frame)
                    if success:
                        self.node.get_logger().info("Captured frame from Go2 front camera (DDS fallback)")
                        return temp_image_path
            if self.node.language == 'zh':
                error_msg = "摄像头未初始化或未就绪，无法捕获画面。"
            else:
                error_msg = "Camera not initialized or not ready, unable to capture frame."
            self.node.get_logger().error(error_msg)
            return None'''

if old in s:
    open(p, "w", encoding="utf-8").write(s.replace(old, new, 1))
    print("PATCH-OK: capture_frame fallback added")
elif "Go2 front camera" in s:
    print("ALREADY-PATCHED")
else:
    print("PATTERN-NOT-FOUND")