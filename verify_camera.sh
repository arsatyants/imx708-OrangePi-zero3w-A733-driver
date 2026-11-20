#!/bin/bash
# Full check of the IMX708 camera chain: driver -> VIN -> ISP (AE/AWB) -> pipe -> ffmpeg H.264.
# Prints PASS/FAIL per step and a summary; results land in ./capture/verify/.
#   ./verify_camera.sh [-d /dev/video0] [-t seconds] [-w 1920] [-h 1080] [-f 30]
#   ./verify_camera.sh --help
cd "$(dirname "$0")"

DEV=/dev/video0; SECS=5; W=1920; H=1080; FPS=30
usage() { sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        -d) DEV=$2; shift 2 ;;
        -t) SECS=$2; shift 2 ;;
        -w) W=$2; shift 2 ;;
        -h) H=$2; shift 2 ;;
        -f) FPS=$2; shift 2 ;;
        --help) usage 0 ;;
        *) echo "unknown option: $1"; usage 1 ;;
    esac
done

OUT=capture/verify; mkdir -p $OUT; rm -f $OUT/*
if [ -t 1 ]; then G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'; else G=; R=; Y=; N=; fi
PASS=0; FAIL=0; WARN=0
pass() { echo "  ${G}PASS${N} $*"; PASS=$((PASS+1)); }
fail() { echo "  ${R}FAIL${N} $*"; FAIL=$((FAIL+1)); }
warn() { echo "  ${Y}WARN${N} $*"; WARN=$((WARN+1)); }
step() { echo; echo "== $*"; }
summary() {
    echo; echo "== summary: ${G}$PASS passed${N}, ${R}$FAIL failed${N}, ${Y}$WARN warnings${N}  (files: $OUT/)"
    [ $FAIL = 0 ]; exit $?
}

step "1. driver"
if lsmod | grep -q '^imx708 '; then pass "imx708 module loaded"; else fail "imx708 module not loaded (sudo modprobe vin_v4l2; see install.sh)"; fi
for m in vin_io vin_v4l2; do lsmod | grep -q "^$m " || fail "$m not loaded"; done
if journalctl -k -b --no-pager 2>/dev/null | grep -q 'find the sony IMX708'; then
    pass "sensor answered on I2C (chip id 0x0708)"
else
    fail "no 'find the sony IMX708' in this boot's kernel log: cable, slot or overlay"
fi
if [ -e "$DEV" ]; then pass "$DEV exists"; else fail "$DEV missing"; summary; fi
if pgrep -x isp_capture >/dev/null; then fail "isp_capture already running (pid $(pgrep -x isp_capture)): camera busy"; summary; fi

step "2. tools"
[ -f /usr/lib/aarch64-linux-gnu/libAWIspApi.so ] && pass "libAWIspApi present" || fail "libAWIspApi missing (package libawispapi-isp-602-arm64)"
if [ ! -x isp_capture ] || [ isp_capture.c -nt isp_capture ]; then
    make -s isp_capture >$OUT/build.log 2>&1 && pass "isp_capture built" || { fail "isp_capture build failed: $OUT/build.log"; summary; }
else
    pass "isp_capture up to date"
fi
HAVE_FFMPEG=0
command -v ffmpeg >/dev/null && command -v ffprobe >/dev/null && HAVE_FFMPEG=1
[ $HAVE_FFMPEG = 1 ] && pass "ffmpeg $(ffmpeg -version | awk 'NR==1{print $3}')" || fail "ffmpeg not installed (sudo apt install ffmpeg)"

step "3. ISP auto exposure (90 frames)"
./isp_capture -d "$DEV" -w $W -h $H -f $FPS -n 90 -o $OUT/frame.nv12 >$OUT/ae.log 2>&1
RC=$?
grep -E '^(isp tuning|frame)' $OUT/ae.log | sed 's/^/     /'
if [ $RC != 0 ] || ! grep -q '^frame' $OUT/ae.log; then
    fail "capture failed (rc=$RC), see $OUT/ae.log"
else
    grep -q 'isp tuning: imx708' $OUT/ae.log && pass "ISP uses the imx219 tuning substitute" \
        || warn "tuning substitution not applied (ov13850 fallback: colour cast)"
    # columns: frame N  Y v  exposure E lines  gain Gx
    # last Y, Y two log lines earlier, exposure min/max over the run, last exposure and gain
    awk 'BEGIN{n=0} /^frame/{y[n]=$4; e=$6+0; if(n==0||e<emin)emin=e; if(e>emax)emax=e; el=e; g=$9; n++}
         END{ printf "%s %s %d %d %d %s\n", y[n-1], y[n-3], emin, emax, el, g }' $OUT/ae.log > $OUT/ae.sum
    read YLAST YPREV EMIN EMAX ELAST GLAST < $OUT/ae.sum
    awk -v y=$YLAST 'BEGIN{exit !(y>=60 && y<=190)}' \
        && pass "brightness OK: mean Y $YLAST (target 60..190; was ~45 without ISP)" \
        || fail "brightness out of range: mean Y $YLAST"
    awk -v a=$YLAST -v b=$YPREV 'BEGIN{d=a-b; if(d<0)d=-d; exit !(d<8)}' \
        && pass "AE converged (Y $YPREV -> $YLAST over the last frames)" \
        || warn "Y still changing ($YPREV -> $YLAST): scene moving or AE not settled"
    [ "$EMIN" != "$EMAX" ] || [ "$GLAST" != "1.00x" ] \
        && pass "AE drives the sensor (exposure $EMIN..$EMAX lines during the run, now $ELAST, gain $GLAST)" \
        || warn "exposure/gain did not change (fine if the first guess was already right)"
    MIN=$(python3 -c "d=open('$OUT/frame.nv12','rb').read()[:$W*$H:5]; print(min(d))" 2>/dev/null)
    [ -n "$MIN" ] && { [ "$MIN" -le 24 ] && pass "black level subtracted (min Y $MIN)" || warn "min Y $MIN: black level high or scene has no dark areas"; }
fi

step "4. pipe -> ffmpeg H.264 (${SECS}s)"
if [ $HAVE_FFMPEG = 1 ]; then
    NFR=$((SECS*FPS)); T0=$(date +%s.%N)
    ./isp_capture -d "$DEV" -w $W -h $H -f $FPS -o - -s 30 -n $((NFR+30)) 2>$OUT/pipe.log \
        | ffmpeg -loglevel error -y -f rawvideo -pix_fmt nv12 -s ${W}x${H} -framerate $FPS -i - \
                 -c:v libx264 -preset ultrafast -tune zerolatency $OUT/video.mp4 2>$OUT/ffmpeg.log
    RCS=("${PIPESTATUS[@]}"); T1=$(date +%s.%N)
    [ "${RCS[0]}" = 0 ] && [ "${RCS[1]}" = 0 ] && pass "pipe finished (isp_capture rc=${RCS[0]}, ffmpeg rc=${RCS[1]})" \
        || fail "pipe rc: isp_capture=${RCS[0]} ffmpeg=${RCS[1]} (see $OUT/pipe.log, $OUT/ffmpeg.log)"
    GOT=$(ffprobe -v error -count_frames -select_streams v -show_entries stream=nb_read_frames -of csv=p=0 $OUT/video.mp4 2>/dev/null)
    RES=$(ffprobe -v error -select_streams v -show_entries stream=width,height -of csv=s=x:p=0 $OUT/video.mp4 2>/dev/null)
    [ "$GOT" = "$NFR" ] && pass "video.mp4: $GOT frames, $RES (stream not corrupted by ISP logs)" \
        || fail "video.mp4: ${GOT:-0} frames, expected $NFR"
    REAL=$(awk -v n=$((NFR+30)) -v a=$T0 -v b=$T1 'BEGIN{printf "%.1f", n/(b-a)}')
    awk -v r=$REAL -v f=$FPS 'BEGIN{exit !(r>=f*0.85)}' && pass "throughput ~$REAL fps (sensor $FPS, incl. ISP start)" \
        || warn "throughput $REAL fps < $FPS: encoder too slow or frames dropped"
    ffmpeg -loglevel error -y -sseof -0.2 -i $OUT/video.mp4 -frames:v 1 -update 1 $OUT/video_last.jpg 2>/dev/null
else
    warn "skipped: no ffmpeg"
fi

step "5. clean shutdown on Ctrl-C"
./isp_capture -d "$DEV" -o - >/dev/null 2>$OUT/sigint.log & P=$!
sleep 3; kill -INT $P 2>/dev/null
for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null; then
    fail "isp_capture still running 5s after SIGINT"; kill -9 $P
else
    wait $P; RC=$?
    [ $RC = 0 ] && pass "stopped on SIGINT (rc 0), ISP released" || fail "rc $RC after SIGINT (see $OUT/sigint.log)"
fi

step "6. snapshot"
if [ -s $OUT/frame.nv12 ]; then
    if [ $HAVE_FFMPEG = 1 ]; then
        ffmpeg -loglevel error -y -f rawvideo -pix_fmt nv12 -s ${W}x${H} -i $OUT/frame.nv12 $OUT/frame.jpg
    else
        python3 -c "
from PIL import Image
W,H=$W,$H; f=open('$OUT/frame.nv12','rb').read()
y=Image.frombytes('L',(W,H),f[:W*H]); uv=f[W*H:W*H*3//2]
u=Image.frombytes('L',(W//2,H//2),uv[0::2]).resize((W,H)); v=Image.frombytes('L',(W//2,H//2),uv[1::2]).resize((W,H))
Image.merge('YCbCr',(y,u,v)).convert('RGB').save('$OUT/frame.jpg')"
    fi
    [ -s $OUT/frame.jpg ] && pass "look at $OUT/frame.jpg$([ -s $OUT/video_last.jpg ] && echo " and $OUT/video_last.jpg"), play $OUT/video.mp4" \
        || fail "jpg conversion failed"
fi

summary
