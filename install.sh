#!/bin/bash
# Install the IMX708 (Camera Module 3) driver on Orange Pi Zero 3W / A733. Run as root, then reboot.
#   sudo ./install.sh            install module + overlay + autoload
#   sudo ./install.sh --remove   undo everything (back to the stock imx219 config)
set -euo pipefail
cd "$(dirname "$0")"
KREL=$(uname -r)
ENV=/boot/orangepiEnv.txt
[ "$(id -u)" = 0 ] || { echo "run as root (sudo $0)"; exit 1; }

if [ "${1:-}" = "--remove" ]; then
    rm -f /lib/modules/$KREL/extra/imx708.ko /boot/overlay-user/imx708.dtbo \
          /etc/modules-load.d/imx708.conf /etc/modprobe.d/imx708.conf
    depmod -a "$KREL"
    if grep -q '^user_overlays=' $ENV; then
        sed -i -E 's/^(user_overlays=.*)\bimx708\b ?/\1/; s/ +$//; /^user_overlays=$/d' $ENV
    fi
    echo "removed; reboot to return to the stock device tree"
    exit 0
fi

[ -f imx708.ko ] && [ -f imx708.dtbo ] || { echo "build first: make && dtc (see README)"; exit 1; }
modinfo -F vermagic imx708.ko | grep -q "^$KREL " || { echo "imx708.ko not built for $KREL"; exit 1; }

install -D -m 644 imx708.ko /lib/modules/$KREL/extra/imx708.ko
depmod -a "$KREL"
install -D -m 644 imx708.dtbo /boot/overlay-user/imx708.dtbo

cp -n $ENV $ENV.before-imx708 || true
if grep -q '^user_overlays=' $ENV; then
    grep -qE '^user_overlays=.*\bimx708\b' $ENV || sed -i -E 's/^(user_overlays=.*)$/\1 imx708/' $ENV
else
    echo 'user_overlays=imx708' >> $ENV
fi

# vin_io provides the helpers imx708.ko uses (no symbol-version deps in an out-of-tree build)
cat > /etc/modprobe.d/imx708.conf <<'C'
softdep imx708 pre: vin_io
# options imx708 flip=3        # rotate 180 if the image is upside down
C
printf 'vin_io\nvin_v4l2\n' > /etc/modules-load.d/imx708.conf

echo "installed. $ENV now:"; grep -E 'overlay' $ENV
echo "reboot, then run ./check_camera.sh"
