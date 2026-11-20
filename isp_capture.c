/*
 * Capture from the Allwinner VIN with the userspace ISP (libAWIspApi) running, so AE/AWB work.
 * Plain v4l2-ctl does not start the ISP: the sensor then stays at the driver's start exposure/gain.
 *
 *   isp_capture [-d /dev/video0] [-w 1920] [-h 1080] [-f 30] [-n 120] [-o out.yuv|-] [-k keep_last] [-s skip]
 *               [-p NV12|YU12] [-b buffers] [-e exposure_us] [-g gain] [-T timestamps.txt] [-L]
 *
 * File mode: writes the last -k frames (default 1), packed (W*H*3/2 per frame, padding stripped).
 * Pipe mode (-o -): writes every frame after the first -s to stdout, endless unless -n is given; stops cleanly
 * on Ctrl-C/SIGTERM or when the reader closes the pipe. libisp's own printf output is moved to stderr. E.g.
 *   ./isp_capture -o - -s 30 | ffmpeg -f rawvideo -pix_fmt nv12 -s 1920x1080 -framerate 30 -i - out.mp4
 * -p YU12 gives planar I420 (ffmpeg yuv420p) instead of NV12.
 * -e/-g switch AE off: fixed exposure in microseconds and gain (1.0 = none); either may be given. The ISP ignores
 *    writes to its gain control, so -g goes through manual ISO (ISO 100 = 1x): powers of two, 1x..64x.
 * -T writes one line per frame written to the output: "seq timestamp_ns exposure_us gain clock", timestamp from
 *    the V4L2 buffer (clock: mono or boot), exposure as last written to the sensor, gain as the ISP reports it.
 * Exposure is capped at the frame period so the frame rate stays as requested, like libcamera's default (the driver
 * would otherwise stretch the frame, and AE with 50 Hz anti-flicker happily picks 30 ms at 60 fps). -L lifts the
 * cap: long exposures then lower the frame rate.
 * Logs mean luma and exposure/gain to stderr, plus the sensor mode (from SENSOR_SUBDEV, default /dev/v4l-subdev0).
 * Build: make isp_capture
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <AWIspApi.h>
#include <sunxi_camera_v2.h>

#include <dlfcn.h>

/*
 * libisp picks the tuning by (sensor name, width, height, fps) from a table in libisp_ini.so. It has no imx708
 * entry and falls back to the first one (ov13850), which gives a strong colour cast. libisp.so calls
 * parser_ini_info() through its PLT, so this executable (linked with -rdynamic) can interpose it and hand the
 * closest Sony tuning instead: imx219 (RGGB, 10-bit black level 64), whose only entry is 3444x1766@30.
 * ISP_TUNING=sensor:w:h:fps overrides the substitute, ISP_TUNING=off disables the substitution.
 */
int parser_ini_info(void *cfg, char *name, int w, int h, int fps, int wdr, int ir, int sync, int isp_id)
{
	static int (*real)(void *, char *, int, int, int, int, int, int, int);
	char sub[32] = "imx219";
	int sw = 3444, sh = 1766, sfps = 30;
	const char *env = getenv("ISP_TUNING");

	if (!real)
		real = dlsym(RTLD_NEXT, "parser_ini_info");
	if (strncmp(name, "imx708", 6) || (env && !strcmp(env, "off")))
		return real(cfg, name, w, h, fps, wdr, ir, sync, isp_id);
	if (env)
		sscanf(env, "%31[^:]:%d:%d:%d", sub, &sw, &sh, &sfps);
	fprintf(stderr, "isp tuning: %s %dx%d@%d -> %s %dx%d@%d\n", name, w, h, fps, sub, sw, sh, sfps);
	return real(cfg, sub, sw, sh, sfps, wdr, ir, sync, isp_id);
}

/*
 * libisp sets exposure/gain with ioctl(sensor subdev, VIDIOC_VIN_SENSOR_EXP_GAIN). ioctl() is interposed here (same
 * -rdynamic mechanism as above) to cap the exposure at the sensor mode's intg_max (frame length - margin, Q4 lines).
 */
static unsigned exp_cap_q4;		/* 0 = no cap */
static volatile int last_exp_q4;	/* exposure actually sent to the sensor */

int ioctl(int fd, unsigned long req, ...)
{
	static int (*real)(int, unsigned long, ...);
	va_list ap;
	va_start(ap, req);
	void *arg = va_arg(ap, void *);
	va_end(ap);
	if (!real)
		real = dlsym(RTLD_NEXT, "ioctl");
	if (req == VIDIOC_VIN_SENSOR_EXP_GAIN && arg) {
		struct sensor_exp_gain *eg = arg;
		if (exp_cap_q4 && (unsigned)eg->exp_val > exp_cap_q4)
			eg->exp_val = exp_cap_q4;
		last_exp_q4 = eg->exp_val;
	}
	return real(fd, req, arg);
}

static volatile sig_atomic_t stop;
static void on_signal(int sig) { (void)sig; stop = 1; }

/* write() all of buf; returns -1 when the reader went away (EPIPE) */
static int write_all(int fd, const uint8_t *buf, size_t len)
{
	while (len) {
		ssize_t w = write(fd, buf, len);
		if (w < 0) {
			if (errno == EINTR)
				continue;
			return -1;
		}
		buf += w;
		len -= w;
	}
	return 0;
}

static int get_ctrl(int fd, unsigned id)
{
	struct v4l2_control c = { .id = id };
	return fd >= 0 && ioctl(fd, VIDIOC_G_CTRL, &c) == 0 ? c.value : -1;
}

static int set_ctrl(int fd, unsigned id, int val, const char *name)
{
	struct v4l2_control c = { .id = id, .value = val };
	if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
		fprintf(stderr, "set %s=%d: %s\n", name, val, strerror(errno));
		return -1;
	}
	return 0;
}

#define MAXBUF 16
#define CHECK(x) do { if ((x) < 0) { fprintf(stderr, "%s: %s\n", #x, strerror(errno)); exit(1); } } while (0)

int main(int argc, char **argv)
{
	const char *dev = "/dev/video0", *out = NULL, *ts_path = NULL, *pixfmt = "NV12";
	int W = 1920, H = 1080, fps = 30, n = -1, keep = 1, skip = 0, nbuf = 4, opt;
	int exp_us = 0, long_exp = 0;
	double gain = 0;

	while ((opt = getopt(argc, argv, "d:w:h:f:n:o:k:s:p:b:e:g:T:L")) != -1) {
		switch (opt) {
		case 'd': dev = optarg; break;
		case 'w': W = atoi(optarg); break;
		case 'h': H = atoi(optarg); break;
		case 'f': fps = atoi(optarg); break;
		case 'n': n = atoi(optarg); break;
		case 'o': out = optarg; break;
		case 'k': keep = atoi(optarg); break;
		case 's': skip = atoi(optarg); break;
		case 'p': pixfmt = optarg; break;
		case 'b': nbuf = atoi(optarg); break;
		case 'e': exp_us = atoi(optarg); break;
		case 'g': gain = atof(optarg); break;
		case 'T': ts_path = optarg; break;
		case 'L': long_exp = 1; break;
		default:
			fprintf(stderr, "usage: %s [-d dev] [-w W] [-h H] [-f fps] [-n frames] [-o out|-] [-k keep] [-s skip]\n"
				"       [-p NV12|YU12] [-b buffers] [-e exposure_us] [-g gain] [-T timestamps.txt] [-L]\n", argv[0]);
			return 1;
		}
	}
	unsigned fourcc;
	if (!strcmp(pixfmt, "NV12"))
		fourcc = V4L2_PIX_FMT_NV12;
	else if (!strcmp(pixfmt, "YU12") || !strcmp(pixfmt, "I420"))
		fourcc = V4L2_PIX_FMT_YUV420;
	else {
		fprintf(stderr, "unsupported -p %s (NV12 or YU12)\n", pixfmt);
		return 1;
	}
	if (nbuf < 2 || nbuf > MAXBUF) {
		fprintf(stderr, "-b must be 2..%d\n", MAXBUF);
		return 1;
	}
	int pipe_mode = out && !strcmp(out, "-");
	if (n < 0)
		n = pipe_mode ? 0 : 120;	/* 0 = endless */
	int out_fd = -1;
	if (pipe_mode) {
		/* keep the real stdout for video, send everything printed to fd 1 (libisp logs) to stderr */
		out_fd = dup(STDOUT_FILENO);
		CHECK(out_fd);
		CHECK(dup2(STDERR_FILENO, STDOUT_FILENO));
	}
	struct sigaction sa = { .sa_handler = on_signal };	/* no SA_RESTART: DQBUF returns EINTR */
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	int video_id = atoi(dev + strcspn(dev, "0123456789"));
	int fd = open(dev, O_RDWR);
	CHECK(fd);
	const char *sd_dev = getenv("SENSOR_SUBDEV");
	int sd = open(sd_dev ? sd_dev : "/dev/v4l-subdev0", O_RDWR);	/* sensor subdev, for logging only */

	struct v4l2_input inp = { .index = 0 };
	CHECK(ioctl(fd, VIDIOC_S_INPUT, &inp));	/* builds the VIN pipeline */

	struct v4l2_streamparm parm = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = fps;
	CHECK(ioctl(fd, VIDIOC_S_PARM, &parm));

	struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
	fmt.fmt.pix_mp.width = W;
	fmt.fmt.pix_mp.height = H;
	fmt.fmt.pix_mp.pixelformat = fourcc;
	fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
	CHECK(ioctl(fd, VIDIOC_S_FMT, &fmt));
	CHECK(ioctl(fd, VIDIOC_G_FMT, &fmt));
	W = fmt.fmt.pix_mp.width;
	H = fmt.fmt.pix_mp.height;
	if (fmt.fmt.pix_mp.num_planes != 1) {
		fprintf(stderr, "%d planes: only single-plane formats are handled\n", fmt.fmt.pix_mp.num_planes);
		return 1;
	}
	fprintf(stderr, "format %dx%d %s\n", W, H, pixfmt);
	struct sensor_config scfg;
	memset(&scfg, 0, sizeof(scfg));
	double line_us = 0;
	if (sd >= 0 && ioctl(sd, VIDIOC_VIN_SENSOR_CFG_REQ, &scfg) == 0 && scfg.pclk && scfg.hts) {
		line_us = scfg.hts * 1e6 / scfg.pclk;
		if (!long_exp)
			exp_cap_q4 = scfg.intg_max;
		fprintf(stderr, "sensor mode %dx%d@%u, line %.2f us, max exposure %.1f ms%s\n", scfg.width, scfg.height,
			scfg.fps_fixed, line_us, (scfg.intg_max >> 4) * line_us / 1e3,
			long_exp ? " (not capped: -L)" : " (cap)");
	} else if (!long_exp)
		fprintf(stderr, "sensor mode unknown (no %s): exposure not capped\n", sd_dev ? sd_dev : "/dev/v4l-subdev0");

	struct v4l2_requestbuffers req = { .count = nbuf, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
					   .memory = V4L2_MEMORY_MMAP };
	CHECK(ioctl(fd, VIDIOC_REQBUFS, &req));
	void *mem[MAXBUF];
	size_t len[MAXBUF];
	for (unsigned i = 0; i < req.count; i++) {
		struct v4l2_plane pl[VIDEO_MAX_PLANES] = {0};
		struct v4l2_buffer b = { .type = req.type, .memory = V4L2_MEMORY_MMAP, .index = i,
					 .length = VIDEO_MAX_PLANES, .m.planes = pl };
		CHECK(ioctl(fd, VIDIOC_QUERYBUF, &b));
		len[i] = pl[0].length;
		if (len[i] < (size_t)W * H * 3 / 2) {	/* G_FMT's sizeimage can be stale; the buffer length is not */
			fprintf(stderr, "buffer %u is %zu bytes, smaller than a %dx%d frame\n", i, len[i], W, H);
			return 1;
		}
		mem[i] = mmap(NULL, len[i], PROT_READ | PROT_WRITE, MAP_SHARED, fd, pl[0].m.mem_offset);
		if (mem[i] == MAP_FAILED) { perror("mmap"); return 1; }
		CHECK(ioctl(fd, VIDIOC_QBUF, &b));
	}

	AWIspApi *isp = CreateAWIspApi();
	isp->ispApiInit();
	int isp_id = isp->ispGetIspId(video_id);
	fprintf(stderr, "video%d -> isp%d\n", video_id, isp_id);
	if (isp_id < 0 || isp->ispStart(isp_id) < 0) {
		fprintf(stderr, "isp start failed\n");
		return 1;
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	CHECK(ioctl(fd, VIDIOC_STREAMON, &type));

	/* Manual exposure/gain go through the VIN node's controls, which the ISP's AE honours */
	if (exp_us > 0) {
		set_ctrl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "auto_exposure");
		set_ctrl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exp_us, "exposure_time_absolute");
	}
	if (gain > 0) {
		/* iso_sensitivity menu: index i = ISO 100 * 2^i */
		int idx = (int)(log2(gain) + 0.5);
		idx = idx < 0 ? 0 : idx > 6 ? 6 : idx;
		set_ctrl(fd, V4L2_CID_ISO_SENSITIVITY_AUTO, V4L2_ISO_SENSITIVITY_MANUAL, "iso_sensitivity_auto");
		set_ctrl(fd, V4L2_CID_ISO_SENSITIVITY, idx, "iso_sensitivity");
		fprintf(stderr, "manual gain %.2f -> ISO %d (%dx)\n", gain, 100 << idx, 1 << idx);
	}
	if (exp_us > 0)
		fprintf(stderr, "manual exposure %d us\n", exp_us);

	FILE *fo = out && !pipe_mode ? fopen(out, "wb") : NULL;
	if (out && !pipe_mode && !fo) { perror(out); return 1; }
	FILE *fts = ts_path ? fopen(ts_path, "w") : NULL;
	if (ts_path && !fts) { perror(ts_path); return 1; }
	size_t frame = (size_t)W * H * 3 / 2;	/* both formats: Y (W*H) then chroma (W*H/2); the buffer tail is padding */
	int log_every = pipe_mode ? fps : 10;
	for (int i = 0; !stop && (n == 0 || i < n); i++) {
		struct v4l2_plane pl[VIDEO_MAX_PLANES] = {0};
		struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP,
					 .length = VIDEO_MAX_PLANES, .m.planes = pl };
		if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) {
			if (errno == EINTR) {
				i--;
				continue;
			}
			fprintf(stderr, "VIDIOC_DQBUF: %s\n", strerror(errno));
			break;
		}
		const uint8_t *y = mem[b.index];
		int writing = pipe_mode ? i >= skip : (fo && i >= n - keep);
		/* exposure in us and gain (Q4) as the ISP currently has them */
		int e_us = get_ctrl(fd, V4L2_CID_EXPOSURE_ABSOLUTE), g_q4 = get_ctrl(fd, V4L2_CID_GAIN);
		if (line_us > 0 && last_exp_q4 > 0)	/* what the sensor really got (after the cap) */
			e_us = (int)((last_exp_q4 >> 4) * line_us + 0.5);
		if (fts && writing) {
			int boot = (b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
			fprintf(fts, "%u %lld %d %.3f %s\n", b.sequence,
				(long long)b.timestamp.tv_sec * 1000000000LL + b.timestamp.tv_usec * 1000LL,
				e_us, g_q4 / 16.0, boot ? "boot" : "mono");
		}
		if (i % log_every == 0 || i == n - 1) {
			uint64_t sum = 0, cnt = 0;
			for (int r = 0; r < H; r += 8)
				for (int c = 0; c < W; c += 8, cnt++)
					sum += y[(size_t)r * W + c];
			fprintf(stderr, "frame %3d  Y %5.1f  exposure %d lines  gain %.2fx  (%.2f ms)\n", i,
				(double)sum / cnt, get_ctrl(sd, V4L2_CID_EXPOSURE) >> 4, g_q4 / 16.0, e_us / 1000.0);
		}
		if (fo && writing)
			fwrite(y, 1, frame, fo);
		if (pipe_mode && writing && write_all(out_fd, y, frame) < 0) {
			fprintf(stderr, "output closed (%s), stopping\n", strerror(errno));
			stop = 1;
		}
		CHECK(ioctl(fd, VIDIOC_QBUF, &b));
	}
	ioctl(fd, VIDIOC_STREAMOFF, &type);
	isp->ispStop(isp_id);
	isp->ispWaitToExit(isp_id);
	isp->ispApiUnInit();
	DestroyAWIspApi(isp);
	if (fo)
		fclose(fo);
	if (fts)
		fclose(fts);
	if (out_fd >= 0)
		close(out_fd);
	close(fd);
	return 0;
}
