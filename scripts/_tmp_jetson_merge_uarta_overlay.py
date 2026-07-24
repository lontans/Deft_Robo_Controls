#!/usr/bin/env python3
"""Ensure JetsonIO boot loads stock hdr40 (uarta) + our GPIO custom overlay.

create_dtbo drops default SFIO pins, so uarta never landed in the custom dtbo.
Stock tegra234-*-hdr40.dtbo carries the header UART pinmux; chain both overlays.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys


def sudo_run(args: list[str]) -> subprocess.CompletedProcess:
    pw = os.environ.get("JETSON_PASS", "4565")
    return subprocess.run(
        ["sudo", "-S", "-p", "", *args],
        input=pw + "\n",
        text=True,
        capture_output=True,
        check=False,
    )


def main() -> int:
    if os.geteuid() != 0:
        # re-exec under sudo
        pw = os.environ.get("JETSON_PASS", "4565")
        r = subprocess.run(
            ["sudo", "-S", "-p", "", "env", f"JETSON_PASS={pw}", "python3", __file__],
            input=pw + "\n",
            text=True,
        )
        return r.returncode

    stock_candidates = [
        "/boot/tegra234-p3737-0000+p3701-0000-hdr40.dtbo",
        "/boot/tegra234-p3767-0000+p3509-a02-hdr40.dtbo",
    ]
    stock = next((p for p in stock_candidates if os.path.isfile(p)), None)
    custom = "/boot/jetson-io-hdr40-user-custom.dtbo"
    extlinux = "/boot/extlinux/extlinux.conf"

    if stock is None:
        print("FAIL: no stock hdr40.dtbo found", file=sys.stderr)
        return 2
    if not os.path.isfile(custom):
        print("FAIL: missing custom dtbo", custom, file=sys.stderr)
        return 2

    print("stock", stock)
    print("custom", custom)

    # Show uart-related strings in stock
    st = subprocess.check_output(["strings", stock], text=True, errors="replace")
    for needle in ("uarta", "hdr40-pin8", "hdr40-pin10", "uart1", "UART1"):
        print(f"stock has {needle!r}: {needle in st}")

    with open(extlinux, encoding="utf-8", errors="replace") as f:
        conf = f.read()

    # Desired overlays line: stock first, then custom GPIO
    overlays = f"{stock},{custom}"
    new_line = f"\tOVERLAYS {overlays}\n"

    if "LABEL JetsonIO" not in conf:
        print("FAIL: JetsonIO label missing", file=sys.stderr)
        return 3

    # Replace OVERLAYS inside JetsonIO block only
    parts = re.split(r"(LABEL JetsonIO\n)", conf, maxsplit=1)
    if len(parts) < 3:
        print("FAIL: cannot find JetsonIO block", file=sys.stderr)
        return 3
    head, label, rest = parts[0], parts[1], parts[2]
    # rest until next LABEL or EOF
    m = re.match(r"(.*?)(\nLABEL |\Z)", rest, flags=re.S)
    if not m:
        print("FAIL: parse JetsonIO body", file=sys.stderr)
        return 3
    body, tail_start = m.group(1), m.group(2)
    tail = rest[len(body) :]

    if re.search(r"^\tOVERLAYS .+$", body, flags=re.M):
        body2 = re.sub(r"^\tOVERLAYS .+$", new_line.rstrip("\n"), body, count=1, flags=re.M)
    else:
        body2 = body.rstrip() + "\n" + new_line

    # Ensure DEFAULT JetsonIO
    conf2 = head
    if re.search(r"^DEFAULT .+$", conf2, flags=re.M):
        conf2 = re.sub(r"^DEFAULT .+$", "DEFAULT JetsonIO", conf2, count=1, flags=re.M)
    else:
        conf2 = "DEFAULT JetsonIO\n" + conf2
    conf2 = conf2 + label + body2 + tail

    bak = extlinux + ".bak-uarta-merge"
    with open(bak, "w", encoding="utf-8") as f:
        f.write(conf)
    with open(extlinux, "w", encoding="utf-8") as f:
        f.write(conf2)

    print("wrote", extlinux, "(backup", bak + ")")
    print("--- JetsonIO block ---")
    # print block
    m2 = re.search(r"LABEL JetsonIO\n(?:.*\n)*?(?=\nLABEL |\Z)", conf2)
    print(m2.group(0) if m2 else conf2)

    # Also re-run GPIO custom generator so pin18 stays GPIO (idempotent)
    gen = "/home/deft-robotics/controls_pcb/scripts/_tmp_jetson_io_gpio_config.py"
    if os.path.isfile(gen):
        print("re-running GPIO custom generator for fresh custom dtbo...")
        subprocess.check_call([sys.executable, gen])

    # Re-apply overlays line again (generator may rewrite OVERLAYS to custom-only)
    with open(extlinux, encoding="utf-8", errors="replace") as f:
        conf3 = f.read()
    conf3 = re.sub(
        r"(LABEL JetsonIO\n(?:.*\n)*?)^\tOVERLAYS .+$",
        lambda m: re.sub(
            r"^\tOVERLAYS .+$",
            new_line.rstrip("\n"),
            m.group(0),
            count=1,
            flags=re.M,
        ),
        conf3,
        count=1,
        flags=re.M,
    )
    conf3 = re.sub(r"^DEFAULT .+$", "DEFAULT JetsonIO", conf3, count=1, flags=re.M)
    with open(extlinux, "w", encoding="utf-8") as f:
        f.write(conf3)

    print("final OVERLAYS:")
    subprocess.call(["grep", "-E", "DEFAULT|OVERLAYS|LABEL JetsonIO", extlinux])
    print("Done. Reboot Jetson.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
