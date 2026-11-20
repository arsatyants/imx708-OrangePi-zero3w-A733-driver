#!/usr/bin/env python3
"""rpicam-vid stand-in for the Orange Pi (Allwinner A733 + IMX708): isp_capture (VIN + ISP with AE/AWB) | ffmpeg.

Takes the rpicam-vid options the h.264-video-test scripts use, so they run unchanged on either board:
  -n  -t MS  --codec h264|libav|mjpeg|yuv420  --libav-format FMT  --width W --height H --framerate F
  --bitrate BPS  --intra N  --inline  --profile P  --level L  --quality Q  -o OUT (file, udp://..., -, /dev/null)
  --save-pts FILE  --metadata FILE  --shutter US  --gain G  --buffer-count N
Accepted and ignored (no equivalent here): --mode --flush --denoise --nopreview -v and anything unknown (warned).

Differences from rpicam-vid:
  * H.264/MJPEG are encoded in software (libx264 ultrafast/zerolatency, ffmpeg mjpeg); there is no hardware encoder
    in this ffmpeg. 1080p60 needs ~3 big cores.
  * Sizes the VIN cannot deliver (it only downscales; above 56 fps the largest sensor mode is 1536x864) are
    upscaled by ffmpeg to the requested size; a line on stderr says so.
  * --gain goes through the ISP's ISO menu: powers of two (1, 2, 4, ... 64).
  * --save-pts and --metadata are written when recording stops (rpicam-vid writes them as it goes). Metadata has
    SensorTimestamp (ns, CLOCK_BOOTTIME like libcamera), ExposureTime (us), AnalogueGain, FrameDuration (us).
Stops after -t ms (0 = until SIGINT/SIGTERM), like rpicam-vid. ISP_CAPTURE=path overrides the isp_capture binary.
"""
import argparse, json, os, signal, subprocess, sys, tempfile, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
ISP = os.environ.get("ISP_CAPTURE", os.path.join(HERE, "isp_capture"))


def log(msg):
    print(f"opi-cam-vid: {msg}", file=sys.stderr, flush=True)


def fail(msg):
    print(f"ERROR: {msg}", file=sys.stderr, flush=True)    # the test scripts grep stderr for 'ERROR'
    sys.exit(1)


ap = argparse.ArgumentParser(prog="opi_cam_vid.py", description="rpicam-vid subset for Orange Pi + IMX708")
ap.add_argument("-n", "--nopreview", action="store_true")
ap.add_argument("-t", "--timeout", type=int, default=5000, help="ms, 0 = run until signalled")
ap.add_argument("--codec", default="h264", choices=["h264", "libav", "mjpeg", "yuv420"])
ap.add_argument("--libav-format", default=None)
ap.add_argument("--width", type=int, default=640)
ap.add_argument("--height", type=int, default=480)
ap.add_argument("--framerate", type=float, default=30)
ap.add_argument("--bitrate", type=int, default=0)
ap.add_argument("--intra", type=int, default=0)
ap.add_argument("--inline", action="store_true")
ap.add_argument("--profile", default=None)
ap.add_argument("--level", default=None)
ap.add_argument("--quality", "-q", type=int, default=50)
ap.add_argument("-o", "--output", default=None)
ap.add_argument("--save-pts", default=None)
ap.add_argument("--metadata", default=None)
ap.add_argument("--shutter", type=int, default=0, help="us")
ap.add_argument("--gain", type=float, default=0)
ap.add_argument("--buffer-count", type=int, default=0)
for ignored in ("--mode", "--denoise", "--awb", "--metering", "--ev"):
    ap.add_argument(ignored, default=None)
ap.add_argument("--flush", action="store_true")
ap.add_argument("-v", "--verbose", nargs="?", const=1, default=0)
args, unknown = ap.parse_known_args()
if unknown:
    log(f"ignoring unsupported options: {' '.join(unknown)}")
if not os.access(ISP, os.X_OK):
    fail(f"{ISP} not found: run 'make isp_capture' in {HERE}")

W, H, FPS = args.width & ~1, args.height & ~1, args.framerate
tmp = tempfile.mkdtemp(prefix="opi_cam_vid_")
ts_file = os.path.join(tmp, "ts.txt")

# --- capture: isp_capture writes I420 frames to stdout, its log (and libisp's) to stderr
isp_cmd = [ISP, "-o", "-", "-w", str(W), "-h", str(H), "-f", str(round(FPS)), "-p", "YU12", "-T", ts_file]
if args.buffer_count:
    isp_cmd += ["-b", str(max(2, args.buffer_count))]
if args.shutter:
    isp_cmd += ["-e", str(args.shutter)]
if args.gain:
    isp_cmd += ["-g", str(args.gain)]
isp = subprocess.Popen(isp_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)

got_format = threading.Event()
real = {}
isp_log = []


def isp_stderr():
    for raw in isp.stderr:
        line = raw.decode(errors="replace").rstrip()
        isp_log.append(line)
        if line.startswith("format "):              # "format 1536x864 YU12": what the VIN really delivers
            real["w"], real["h"] = map(int, line.split()[1].split("x"))
            got_format.set()
        if args.verbose or line.startswith(("sensor mode", "manual", "isp tuning")):
            log(line)


threading.Thread(target=isp_stderr, daemon=True).start()
while not got_format.wait(0.1):
    if isp.poll() is not None:
        errs = [l for l in isp_log if not l.startswith("[ISP")] or isp_log
        fail("isp_capture exited: " + " | ".join(errs[-3:]))
aw, ah = real["w"], real["h"]

# --- encode / mux with ffmpeg
ff = ["ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "error",
      "-f", "rawvideo", "-pix_fmt", "yuv420p", "-s", f"{aw}x{ah}", "-framerate", f"{FPS:g}", "-i", "pipe:0"]
if (aw, ah) != (W, H):
    log(f"VIN delivers {aw}x{ah} at {FPS:g} fps, upscaling to {W}x{H} in ffmpeg")
    ff += ["-vf", f"scale={W}:{H}:flags=fast_bilinear"]
if args.codec in ("h264", "libav"):
    ff += ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p",
           "-g", str(args.intra or 60)]
    if args.bitrate:
        b = str(args.bitrate)
        ff += ["-b:v", b, "-maxrate", b, "-bufsize", str(args.bitrate // 2)]
    if args.profile:
        ff += ["-profile:v", args.profile]
    if args.level:
        ff += ["-level", args.level]
    if args.inline:
        ff += ["-x264-params", "repeat-headers=1"]
    fmt = "h264" if args.codec == "h264" else (args.libav_format or "mpegts")
elif args.codec == "mjpeg":
    ff += ["-c:v", "mjpeg", "-pix_fmt", "yuvj420p", "-q:v", str(round(2 + (100 - args.quality) * 29 / 100))]
    fmt = "mjpeg"
else:
    ff += ["-c:v", "rawvideo"]
    fmt = "rawvideo"
out = args.output or "/dev/null"
if out == "-":
    out = "pipe:1"
ff += ["-fps_mode", "passthrough", "-flush_packets", "1", "-f", fmt, "-y", out]
enc = subprocess.Popen(ff, stdin=isp.stdout, stderr=subprocess.PIPE, start_new_session=True)
isp.stdout.close()                                  # ffmpeg owns the pipe now: EOF reaches it when isp_capture stops

# --- run for -t ms or until signalled
stop = threading.Event()
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda *_: stop.set())
deadline = time.monotonic() + args.timeout / 1000 if args.timeout > 0 else None
while not stop.is_set() and isp.poll() is None and enc.poll() is None:
    if deadline and time.monotonic() >= deadline:
        break
    stop.wait(0.05)
if isp.poll() is None:
    isp.send_signal(signal.SIGINT)                  # isp_capture stops the ISP and closes the pipe
try:
    isp.wait(timeout=10)
except subprocess.TimeoutExpired:
    isp.kill(); isp.wait()
try:
    enc_err = enc.communicate(timeout=15)[1].decode(errors="replace")
except subprocess.TimeoutExpired:
    enc.kill(); enc_err = enc.communicate()[1].decode(errors="replace")
if enc.returncode not in (0, None) and not stop.is_set():
    print(enc_err, file=sys.stderr)
    fail(f"ffmpeg exited with {enc.returncode}")

# --- per-frame side files in rpicam-vid's formats
frames = []
if os.path.exists(ts_file):
    for line in open(ts_file):
        p = line.split()
        if len(p) >= 5:
            frames.append((int(p[1]), int(p[2]), float(p[3]), p[4]))
if frames and frames[0][3] == "mono":               # libcamera's SensorTimestamp is CLOCK_BOOTTIME
    off = time.clock_gettime_ns(time.CLOCK_BOOTTIME) - time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    frames = [(t + off, e, g, "boot") for t, e, g, _ in frames]
if args.save_pts:
    with open(args.save_pts, "w") as f:
        f.write("# timecode format v2\n")
        for t, *_ in frames:
            f.write(f"{(t - frames[0][0]) / 1e6:.3f}\n")
if args.metadata:
    md = []
    for i, (t, e, g, _) in enumerate(frames):
        md.append({"SensorTimestamp": t, "ExposureTime": e, "AnalogueGain": g, "DigitalGain": 1.0,
                   "FrameDuration": round((t - frames[i - 1][0]) / 1000) if i else round(1e6 / FPS)})
    with open(args.metadata, "w") as f:
        json.dump(md, f)
try:
    os.remove(ts_file); os.rmdir(tmp)
except OSError:
    pass
n = len(frames)
if n > 1:
    log(f"{n} frames, {(n - 1) * 1e9 / (frames[-1][0] - frames[0][0]):.1f} fps")
