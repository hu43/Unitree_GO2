import sys
import os
import time

AI = os.path.expanduser("~/hu/ai")
sys.path.insert(0, os.path.join(AI, "pylibs"))
sys.path.insert(0, os.path.expanduser("~/hu/unitree_sdk2_python"))

iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
out = os.path.join(AI, "camera_latest.jpg")


def init_client():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    ChannelFactoryInitialize(0, iface)
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    return client


client = None
while client is None:
    try:
        client = init_client()
        print(f"[camera_daemon] DDS ready iface={iface}", flush=True)
    except Exception as e:
        print(f"[camera_daemon] init retry: {e}", flush=True)
        time.sleep(5)

print(f"[camera_daemon] target={out}", flush=True)
ok = False
while True:
    try:
        code, data = client.GetImageSample()
        if code == 0 and data:
            tmp = out + ".tmp"
            with open(tmp, "wb") as f:
                f.write(bytes(data))
            os.replace(tmp, out)
            if not ok:
                print("[camera_daemon] first frame saved", flush=True)
                ok = True
        elif not ok:
            print(f"[camera_daemon] GetImageSample ret={code}", flush=True)
    except Exception as e:
        if not ok:
            print(f"[camera_daemon] error: {e}", flush=True)
    time.sleep(1.5)