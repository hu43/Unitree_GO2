for p in ["/home/unitree/hu/ai/yahboom_ws/src/largemodel/utils/large_model_interface.py", "/home/unitree/hu/ai/yahboom_ws/build/largemodel/utils/large_model_interface.py"]:
    s = open(p, encoding="utf-8").read()
    if "last_ollama_request" in s:
        print("ALREADY", p)
        continue
    key = "def ollama_infer(self, messages, image_path=None, video_path=None):"
    if key not in s:
        print("NOANCHOR", p)
        continue
    snippet = "\n".join([
        "        try:",
        "            import json as _json",
        "            with open(\"/home/unitree/hu/ai/last_ollama_request.json\", \"w\", encoding=\"utf-8\") as _f:",
        "                _json.dump(messages, _f, ensure_ascii=False, indent=1)",
        "        except Exception:",
        "            pass",
    ])
    open(p, "w", encoding="utf-8").write(s.replace(key, key + "\n" + snippet, 1))
    print("PATCHED", p)
