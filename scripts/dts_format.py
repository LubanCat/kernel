#!/usr/bin/env python3
import os
import re

ROOT = "arch/arm64/boot/dts/rockchip"
SPACES_PER_TAB = 4

def convert_indent(line):
    # 去掉行尾空白（保留换行）
    has_nl = line.endswith('\n')
    line = line.rstrip('\n').rstrip()
    line = line + ('\n' if has_nl else '')

    # 仅处理行首空格缩进
    m = re.match(r'^( +)', line)
    if not m:
        return line

    spaces = m.group(1)
    tabs = '\t' * (len(spaces) // SPACES_PER_TAB)
    rest = spaces[len(tabs) * SPACES_PER_TAB:]

    return tabs + rest + line[len(spaces):]

for root, dirs, files in os.walk(ROOT):
    for name in files:
        # 只处理包含 lubancat 的 dts / dtsi
        if "lubancat" not in name:
            continue
        if not name.endswith((".dts", ".dtsi")):
            continue

        path = os.path.join(root, name)
        print("Formatting:", path)

        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()

        # 1. 缩进 + 行尾空白处理
        new_lines = [convert_indent(line) for line in lines]

        # 2. 删除文件末尾多余空行
        while new_lines and new_lines[-1].strip() == "":
            new_lines.pop()

        # 3. 确保最后一行以 \n 结尾（只补，不多加）
        if not new_lines or not new_lines[-1].endswith("\n"):
            new_lines.append("\n")
        else:
            new_lines[-1] = new_lines[-1].rstrip('\n') + '\n'

        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.writelines(new_lines)

print("Done.")
