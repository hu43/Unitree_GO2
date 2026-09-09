import json, subprocess
req = json.load(open("/home/unitree/hu/ai/last_ollama_request.json"))
sysm, greet = req[0], req[1]
user = {"role": "user", "content": "分析一下表格"}
def call(msgs, use_format, opts):
    payload = {"model": "gemma3:4b", "stream": False, "messages": msgs}
    if use_format:
        payload["format"] = "json"
    if opts:
        payload["options"] = opts
    open("/tmp/abl3.json", "w").write(json.dumps(payload, ensure_ascii=False))
    out = subprocess.run(["curl", "-s", "http://localhost:11434/api/chat", "-d", "@/tmp/abl3.json"], capture_output=True, text=True).stdout
    try:
        return json.loads(out)["message"]["content"][:220]
    except Exception as e:
        return "ERR " + str(e)
print("F1_free_default:", call([sysm, greet, user], False, None))
print("F2_free_temp06:", call([sysm, greet, user], False, {"temperature": 0.6}))
