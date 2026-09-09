import json, subprocess
req = json.load(open("/home/unitree/hu/ai/last_ollama_request.json"))
sysm = req[0]
cur = [m for m in req if m["role"] == "user"][-1]
def call(msgs, numctx):
    payload = {"model": "gemma3:4b", "stream": False, "format": "json", "messages": msgs, "options": {"temperature": 0.1, "num_ctx": numctx}}
    open("/tmp/abl.json", "w").write(json.dumps(payload, ensure_ascii=False))
    out = subprocess.run(["curl", "-s", "http://localhost:11434/api/chat", "-d", "@/tmp/abl.json"], capture_output=True, text=True).stdout
    try:
        return json.loads(out)["message"]["content"][:200]
    except Exception as e:
        return "ERR " + str(e) + " " + out[:100]
print("A_exact_ctx4096:", call(req, 4096))
print("B_clean_ctx4096:", call([sysm, cur], 4096))
print("C_clean_ctx16384:", call([sysm, cur], 16384))
print("D_exact_ctx16384:", call(req, 16384))
