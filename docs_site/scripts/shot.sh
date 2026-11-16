#!/usr/bin/env bash
# Screenshot a built page with Windows Chrome (WSL): bash scripts/shot.sh ko/getting_started/overview.html out.png [width] [height]
P=$(wslpath -w "$(dirname "$0")/../_build/html/$1")
"/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" --headless=new --disable-gpu --hide-scrollbars \
  --window-size=${3:-1400},${4:-2400} --screenshot="$(wslpath -w "$2")" "file:///$P" >/dev/null 2>&1
