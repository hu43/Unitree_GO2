import json, subprocess
req = json.load(open("/home/unitree/hu/ai/last_ollama_request.json"))
sysm = req[0]
greet = req[1]
user = {"role": "user", "content": "分析一下表格"}
def call(msgs, npred):
    opts = {"temperature": 0.1, "num_ctx": 4096}
    if npred:
        opts["num_predict"] = npred
    payload = {"model": "gemma3:4b", "stream": False, "format": "json", "messages": msgs, "options": opts}
    open("/tmp/abl2.json", "w").write(json.dumps(payload, ensure_ascii=False))
    out = subprocess.run(["curl", "-s", "http://localhost:11434/api/chat", "-d", "@/tmp/abl2.json"], capture_output=True, text=True).stdout
    try:
        return json.loads(out)["message"]["content"][:160]
    except Exception as e:
        return "ERR " + str(e)
r1 = call([sysm, greet, user], 512)
print("R1_service_shape:", r1)
r2 = call([sysm, greet, user], None)
print("R2_no_num_predict:", r2)
r3 = call([sysm, greet, user], 512)
print("R3_repeat:", r3)
