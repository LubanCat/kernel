#!/usr/bin/env python3
import os
import re

UENV_DIR = "arch/arm64/boot/dts/rockchip/uEnv"
OVERLAY_DIR = "arch/arm64/boot/dts/rockchip/overlay"

dtoverlay_re = re.compile(r'dtoverlay\s*=\s*([^\s#]+\.dtbo)')
gsdt_plugin_re = re.compile(r'gsdt_plugin\d+\s*=\s*([^\s#]+\.dtbo)')

missing = []

for name in os.listdir(UENV_DIR):
    if not name.endswith(".txt"):
        continue

    # 排除 uEnv.txt
    if name == "uEnv.txt":
        continue

    path = os.path.join(UENV_DIR, name)

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for lineno, line in enumerate(f, 1):
            # 1. dtoverlay
            m = dtoverlay_re.search(line)
            if m:
                dtbo_path = m.group(1)
                dtbo_name = os.path.basename(dtbo_path)
                dtbo_file = os.path.join(OVERLAY_DIR, dtbo_name)

                if not os.path.isfile(dtbo_file):
                    missing.append((name, lineno, dtbo_name))
                continue

            # 2. gsdt_plugin
            m = gsdt_plugin_re.search(line)
            if m:
                dtbo_path = m.group(1)
                dtbo_name = os.path.basename(dtbo_path)
                dtbo_file = os.path.join(OVERLAY_DIR, dtbo_name)

                if not os.path.isfile(dtbo_file):
                    missing.append((name, lineno, dtbo_name))
                continue

if missing:
    print("❌ Missing dtbo files:")
    for txt, line, dtbo in missing:
        print(f"  {txt}:{line} -> {dtbo}")
else:
    print("✅ All dtoverlay / gsdt_plugin dtbo files exist.")
