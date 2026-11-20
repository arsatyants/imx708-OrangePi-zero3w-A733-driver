#!/bin/bash
# After reboot: is the IMX708 detected, what does the VIN pipeline offer, and grab a test frame.
# Output goes to ./capture/ (frame.nv12 and frame.jpg).
cd "$(dirname "$0")"
echo "== overlay / modules"
tr -d '\0' < /proc/device-tree/soc@3000000/vind@5800800/sensor@5812000/sensor0_mname; echo
lsmod | grep -E '^(imx708|vin_v4l2|vin_io) ' || echo "modules not loaded (try: sudo modprobe vin_v4l2)"
echo "== kernel log"
journalctl -k -b --no-pager 2>/dev/null | grep -iE 'imx708|chip id|vin|sensor' | tail -25
echo "== video nodes"
ls /dev/video* 2>/dev/null || { echo "no /dev/video*: sensor not found (cable, slot, power)"; exit 1; }
DEV=${DEV:-/dev/video0}
v4l2-ctl -d $DEV --info --list-formats-ext 2>&1 | head -40
W=${W:-1920}; H=${H:-1080}
mkdir -p capture
echo "== capture ${W}x${H} NV12 from $DEV (ISP running: AE/AWB)"
# The 3A loop lives in userspace (libAWIspApi): plain v4l2-ctl leaves the sensor at its start exposure and the
# image dark. isp_capture starts the ISP, logs AE convergence and writes the last frame without buffer padding.
[ -x isp_capture ] || make isp_capture >/dev/null || exit 1
./isp_capture -d $DEV -w $W -h $H -n 90 -o capture/frame.nv12 2>&1 | grep -E '^(format|isp tuning|frame)'
[ -s capture/frame.nv12 ] || exit 1
if command -v ffmpeg >/dev/null; then
    ffmpeg -loglevel error -y -f rawvideo -pix_fmt nv12 -s ${W}x${H} -i capture/frame.nv12 capture/frame.jpg
else
    python3 - "$W" "$H" <<'PY'
import sys
from PIL import Image
W, H = int(sys.argv[1]), int(sys.argv[2])
f = open('capture/frame.nv12', 'rb').read()
y = Image.frombytes('L', (W, H), f[:W*H]); uv = f[W*H:W*H*3//2]
u = Image.frombytes('L', (W//2, H//2), uv[0::2]).resize((W, H))
v = Image.frombytes('L', (W//2, H//2), uv[1::2]).resize((W, H))
Image.merge('YCbCr', (y, u, v)).convert('RGB').save('capture/frame.jpg')
PY
fi && echo "saved capture/frame.jpg"
