# IMX708 (Raspberry Pi Camera Module 3) driver for the Orange Pi Zero 3W (Allwinner A733)

Out-of-tree port of the Raspberry Pi `imx708` sensor driver to the **Allwinner VIN
framework** of the Orange Pi BSP kernel `6.6.98-sun60iw2` (the SoC is otherwise
called `sun60iw2` / T527-class A733). The BSP ships only `imx219` / `ov13850` /
`gc0xx` sensor drivers; this repo makes the 12 MP Raspberry Pi Camera Module 3
(IMX708) work on the Zero 3W's two MIPI-CSI camera slots.

## What the board gives you without this driver

Nothing for the IMX708: there is no `CONFIG_VIDEO_IMX708` in the BSP kernel and
no `imx708.c` anywhere in `drivers/media/i2c` or the Allwinner VIN sensor
directory. There is also no mainline Linux support for the A733 (nothing to
borrow from upstream), and Orange Pi's official cameras for Allwinner boards use
other sensors. So a port is the only option — this is that port.

## Files

| File | Purpose |
|---|---|
| `imx708.c` | VIN sensor driver (modelled on the BSP `imx219.c`): power, chip-ID detect, modes, exposure/gain controls for the ISP's AE loop |
| `imx708_regs.h` | register tables taken verbatim from `raspberrypi/linux` `rpi-6.6.y` `drivers/media/i2c/imx708.c` |
| `imx708.dts` | device-tree overlay: `sensor0_mname = "imx708"`, `sensor2_mname = "imx708_2"`, I2C address 0x1a (encoded as 0x34) |
| `install.sh` | installs the module and overlay (`user_overlays=imx708` in `/boot/orangepiEnv.txt`), autoload of `vin_io`/`vin_v4l2`; `--remove` undoes it. `make install` / `make uninstall` |
| `isp_capture.c` | userspace capture tool that runs the ISP (AE/AWB); the A733 camera only produces usable frames through it — see below |
| `opi_cam_vid.py` | `rpicam-vid`-compatible wrapper (`isp_capture` + ffmpeg) so Raspberry-Pi benchmark scripts keep working on this board |
| `verify_camera.sh` | full PASS/FAIL check: driver loaded, ISP AE settles, pipe to ffmpeg H.264, clean shutdown (`--help` for options) |
| `check_camera.sh` | after a reboot: kernel log, `/dev/video*`, formats, captures a 1080p NV12 frame via `isp_capture` to `capture/frame.jpg` |

## Sensor modes (2 lanes, ~900 Mbit/s/lane, RAW10 into the A733 ISP)

| Sensor mode | fps | Notes |
|---|---|---|
| 2304x1296 (2x2 binned) | 30, 56 | default |
| 1536x864 (binned + crop) | 30, 60, 120 | |
| 4608x2592 (full) | 14 | may exceed the ISP line width |

The VIN/ISP scaler downsizes for you: request e.g. `-w 1920 -h 1080` and the
VIN delivers 1080p regardless of sensor mode. Not done: autofocus (DW9817 VCM —
lens stays at rest position), HDR, PDAF, proper imx708-specific ISP tuning.
Module parameters: `flip=0..3` (3 = rotate 180°), `test_pattern=0..4`
(2 = colour bars — useful for checking the MIPI path without optics).

## Important: the 3A loop lives in userspace, not in the kernel

On the A733, ISP AE/AWB is performed by `libAWIspApi`/`libisp`
(package `libawispapi-isp-602-arm64`), loaded into the **capture process**. A
plain V4L2 client (`ffmpeg`, GStreamer, `v4l2-ctl`) may get frames but nothing
adjusts exposure: the sensor sits at the driver's start exposure, the image
stays dark and the black level is not subtracted (Y min ≈ 34). Also, on the
6.6.98-sun60iw2 kernel, opening the capture node *without* the ISP setup makes
the VIN pipeline walker fail (`vidioc_s_fmt` → `vin_pipeline_try_format`
crash / `scaler get_selection error`); treat that as a signal to use the ISP
path, not to keep poking V4L2 directly. **Do not mix direct V4L2 access with
the ISP process on the same vin pipeline** — the ISP keeps per-stream bayer
state and errors out with `isp0 the bayer format is changed when d3d is open`
until the board is rebooted or the VIN modules are re-probed cleanly.

A capture process must, after `S_INPUT`/`S_FMT`/`REQBUFS` and before
`STREAMON`, call `CreateAWIspApi()->ispApiInit()`, `ispGetIspId(video_id)`,
`ispStart(isp_id)` (and `ispStop`/`ispWaitToExit` on the way out).
`isp_capture.c` does all of this; AE settles in ~20 frames, and exposure/gain
are written to the sensor through `VIDIOC_VIN_SENSOR_EXP_GAIN`.

### ISP tuning

`libisp` picks its config from a built-in table by (sensor name, width, height,
fps). There is no imx708 entry and its fallback is `ov13850` (strong magenta
cast). `isp_capture` interposes `parser_ini_info()` (linked with `-rdynamic`)
and hands libisp the **imx219** tuning instead (Sony, RGGB, black level 64):
correct black level, near-neutral colours. Override with
`ISP_TUNING=sensor:w:h:fps` (table entries: `imx219:3444:1766:30`,
`ov13850_mipi:4224:3136:30`, `gc05a2_mipi:2592:1944:30`, ...) or
`ISP_TUNING=off`. The same trick works in any program linked with
`libAWIspApi`: define `parser_ini_info` and export it.

```
./isp_capture -w 1920 -h 1080 -n 120 -o out.nv12 -k 60   # capture 120 frames, keep the last 60
```

Pipe mode (`-o -`) sends every frame after the first `-s` (AE settle) to stdout
as raw NV12, endless unless `-n` is given. It stops cleanly (ISP stopped,
stream off) on SIGINT/SIGTERM or when the reader exits; libisp logs to stderr.

```
./isp_capture -o - -s 30 | ffmpeg -f rawvideo -pix_fmt nv12 -s 1920x1080 -framerate 30 -i - -c:v libx264 out.mp4
```

On exit libisp may log `isp_ctx_save_exit ... Permission denied` — it cannot
save its AE state under `/mnt` unless run as root. Harmless. Note the VIN pads
1080p buffers to 1920x1088 (`sizeimage` 3133440): the NV12 picture is the first
W*H*3/2 bytes.

## `opi_cam_vid.py`: drop-in `rpicam-vid` stand-in

Accepts the `rpicam-vid` options used by the related benchmark suite
(`-t --codec h264|libav|mjpeg|yuv420 --width --height --framerate --bitrate
--intra --inline --profile -o udp://... --save-pts --metadata --shutter --gain
--buffer-count`; `--mode --flush --denoise` accepted and ignored) and runs
`isp_capture -p YU12 | ffmpeg` (libx264 ultrafast/zerolatency, or mjpeg/raw).

```
./opi_cam_vid.py -t 10000 --codec h264 --width 1280 --height 720 --framerate 60 --bitrate 8000000 -o udp://HOST:5000
```

- The VIN only downscales; sizes above the active sensor mode (e.g. 1080p60,
  whose 60 fps mode is 1536x864) are upscaled by ffmpeg, so the output is the
  requested size, like on the Pi.
- `--save-pts` / `--metadata` (SensorTimestamp in CLOCK_BOOTTIME ns,
  ExposureTime in us, ...) come from `isp_capture -T` and are written when the
  recording ends.
- Software encoding: 1080p60 H.264 needs ~5 cores (fps sags when the board is
  busy); MJPEG at high resolutions is slow.

Details used by the wrapper (`isp_capture`):

- **Fixed frame rate:** AE (with 50 Hz anti-flicker it likes 10 ms steps) may
  ask for more exposure than one frame; the driver would then stretch the frame
  (720p60 dropped to 33 fps in a dim room). `isp_capture` interposes `ioctl()`
  and caps the exposure libisp sends (`VIDIOC_VIN_SENSOR_EXP_GAIN`) at the
  mode's `intg_max`, as libcamera does by default; AE makes up with gain.
  `-L` disables the cap.
- **Manual exposure:** `-e us` sets VIN `auto_exposure`=manual +
  `exposure_time_absolute`; `-g gain` goes through the ISO menu
  (ISO 100 = 1x, powers of two up to 64x) because the ISP resets its `gain`
  control.
- `-T file` logs per frame: sequence, V4L2 timestamp (CLOCK_MONOTONIC),
  exposure actually sent to the sensor, gain.
- `-p YU12` selects I420 output, `-b N` the number of V4L2 buffers.

## Build and install (on the board)

```
# 1. kernel tree, prepared with the running config
sudo apt install -y bc bison flex libssl-dev
git clone --depth 1 -b orange-pi-6.6-sun60iw2 \
  https://github.com/orangepi-xunlong/linux-orangepi.git ~/code/a733-kernel/linux
cd ~/code/a733-kernel/linux
cp /boot/config-$(uname -r) .config && make olddefconfig
make -j8 modules_prepare LOCALVERSION=-sun60iw2

# 2. this repo
cd ~/code
git clone https://github.com/<you>/imx708-OrangePi-zero3w-A733-driver.git
cd imx708-OrangePi-zero3w-A733-driver
make            # imx708.ko + imx708.dtbo + isp_capture
make install    # sudo ./install.sh
sudo reboot     # required for the DT overlay to take effect

# 3. check
./check_camera.sh
./verify_camera.sh
```

Userspace deps for `isp_capture`: the BSP packages `libawispapi-isp-602-arm64`
(provides `libAWIspApi.so`, `libisp.so` and `libisp_algo.so` in `/usr/lib`) and
`ffmpeg`. `opi_cam_vid.py` additionally needs `python3`.

The capture tool is meant to run as your normal user; running it under `sudo`
lets libisp persist its AE context to `/mnt` between runs.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `sensor_s_stream` never logged, no chip ID | camera cable orientation (Zero 3W CSI is 90° to the sensor), and check the overlay actually applied: `dmesg \| grep imx708` |
| image dark, Y min ≈ 34 | you bypassed the ISP — capture via `isp_capture`, not ffmpeg/v4l2-ctl |
| `isp0 the bayer format is changed when d3d is open`, `isp0 width error` | another client touched the vin pipeline with a different input/format; close everything ISP-related (`pkill -f isp_capture`), or `sudo modprobe -r vin_v4l2 imx708` + re-probe, or reboot |
| `VIN` pipeline errors on plain `S_FMT` (`vin_pipeline_try_format` crash) | expected on this kernel without ISP setup; use the ISP path |
| magenta cast | ISP tuning fell back to `ov13850`; check that the `parser_ini_info` interpose is active (built with `-rdynamic`) |
| 4608x2592 fails to stream | sensor full-res mode exceeds the ISP line width on 2 lanes; use 2304x1296 or 1536x864 |

## Status / tested hardware

- Board: Orange Pi Zero 3W, 12 GB LPDDR5, kernel `6.6.98-sun60iw2` (vendor image, Debian 12 bookworm)
- Camera: Raspberry Pi Camera Module 3 (IMX708, wide), 2-lane CSI
- Driver: loaded from `/lib/modules`, AE via userspace ISP works; H.264 via
  `opi_cam_vid.py` + libx264 verified
- Not working yet: autofocus, sensor-native HDR/PDAF, imx708-specific ISP tuning

## Credits and licensing

`imx708.c` is adapted from the Orange Pi BSP `imx219.c` VIN driver.
`imx708_regs.h` is a verbatim copy of the register tables from
[raspberrypi/linux](https://github.com/raspberrypi/linux) (`rpi-6.6.y`,
`drivers/media/i2c/imx708.c`); the register data originates from Sony and is
published by Raspberry Pi under GPLv2. The userspace tooling depends on
Allwinner's `libAWIspApi`/`libisp` binaries shipped with the BSP.

The code in this repository is GPL-2.0-only, consistent with both upstream
sources.
