for p in ["/home/unitree/hu/ai/yahboom_ws/src/largemodel/utils/large_model_interface.py", "/home/unitree/hu/ai/yahboom_ws/build/largemodel/utils/large_model_interface.py"]:
    lines = open(p, encoding="utf-8").read().split("\n")
    out = []
    changed = 0
    for ln in lines:
        if ("options={" in ln) and ("num_predict" in ln):
            out.append("                    options={\"temperature\": 0.6}")
            changed += 1
            continue
        if ("format=" + chr(39) + "json") in ln:
            changed += 1
            continue
        out.append(ln)
    open(p, "w", encoding="utf-8").write("\n".join(out))
    print("DONE", p, "changes:", changed)
