/*
 * qadrc - classic dynamic range compressor for ffmpeg
 *
 * Written by Alexey Tourbin.
 * Based on qaac compressor by nu774.
 * This file is distributed as Public Domain.
 *
 * This implementation is based on "Digital Dynamic Range Compressor Design -
 * A Tutorial and Analysis", JAES2012.  It adds the delay (lookahead) parameter.
 */

#include <stdlib.h>
#include <math.h>
#include "libavutil/avassert.h"
#include "libavutil/opt.h"
#include "libavutil/channel_layout.h"
#include "avfilter.h"
#include "internal.h"

typedef struct QADRCContext {
    const AVClass *class;

    double thresh;
    double ratio;
    double knee;
    double attack;
    double release;
    double delay;

    double slope;
    double Tlo;
    double Thi;
    double knee_factor;
    size_t delay_samples;
    size_t total_samples;

    double alphaA;
    double alphaR;

    double yR;
    double yA;

    AVFrame **frames;
    size_t nframes;
    size_t fpos;
    float lasta;
} QADRCContext;

#if 1
#include "simd_math_prims.h"
static inline float dB_to_scale(float dB)
{
    return expapprox(2.302585093f*0.05f*dB);
}

static inline float scale_to_dB(float x)
{
    if (x < 1e-6f)
	return -120;
    return 20.0f*0.4342944819f*logapprox(x);
}
#else
static inline double dB_to_scale(double dB)
{
    return pow(10, 0.05 * dB);
}

static inline double scale_to_dB(double scale)
{
    return 20 * log10(scale);
}
#endif

/*
 * gain computer, works on log domain
 */
static double computeGain(QADRCContext *s, double x)
{
    if (x < s->Tlo)
	return 0.0;
    else if (x > s->Thi)
	return s->slope * (x - s->thresh);
    else {
	double delta = x - s->Tlo;
	return delta * delta * s->knee_factor;
    }
}

/*
 * smooth, level corrected decoupled peak detector
 * works on log domain
 */
static double smoothAverage(QADRCContext *s, double x)
{
    const double eps = 1e-120;
    s->yR = fmin(x, s->alphaR * s->yR + (1.0 - s->alphaR) * x + eps - eps);
    s->yA = s->alphaA * s->yA + (1.0 - s->alphaA) * s->yR + eps - eps;
    return s->yA;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    QADRCContext *s = ctx->priv;

    s->slope = (1.0 - s->ratio) / s->ratio;
    s->attack = s->attack / 1000.0;
    s->release = s->release / 1000.0;
    s->Tlo = s->thresh - s->knee / 2.0;
    s->Thi = s->thresh + s->knee / 2.0;
    s->knee_factor = s->slope / (s->knee * 2.0);
    s->yR = -10;
    s->yA = -10;

    const double Fs = inlink->sample_rate;
    s->delay_samples = s->delay * Fs / 1000;
    av_log(ctx, AV_LOG_DEBUG, "delay samples = %zu\n", s->delay_samples);

    s->alphaA = s->attack > 0.0 ? exp(-1.0 / (s->attack * Fs)) : 0.0;
    s->alphaR = s->release > 0.0 ? exp(-1.0 / (s->release * Fs)) : 0.0;

    return 0;
}

static void chew(QADRCContext *s, AVFrame *frame, int fmt, unsigned nc, float *a)
{
    float **data = (float **) frame->extended_data;
    const int nsamples = frame->nb_samples;
    // calculate peak level
    switch (fmt) {
    case AV_SAMPLE_FMT_FLT  | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 1 << 8:
	for (size_t i = 0; i < nsamples; i++) {
	    float xL = fabsf(data[0][i]);
	    float xG = scale_to_dB(xL);
	    a[i] = xG;
	}
	break;
    case AV_SAMPLE_FMT_FLTP | 2 << 8:
	for (size_t i = 0; i < nsamples; i++) {
	    float xL = fabsf(data[0][i]);
	    float xM = fabsf(data[1][i]);
	    if (xM > xL)
		xL = xM;
	    float xG = scale_to_dB(xL);
	    a[i] = xG;
	}
	break;
    case AV_SAMPLE_FMT_FLTP:
	for (size_t i = 0; i < nsamples; i++) {
	    float xL = fabsf(data[0][i]);
	    for (int j = 1; j < nc; j++) {
		float xM = fabsf(data[j][i]);
		if (xM > xL)
		    xL = xM;
	    }
	    a[i] = xL;
	}
	break;
    case AV_SAMPLE_FMT_FLT | 2 << 8:
	for (size_t j = 0; j < 2 * nsamples; j += 2) {
	    float xL = fabsf(data[0][j+0]);
	    float xM = fabsf(data[0][j+1]);
	    if (xM > xL)
		xL = xM;
	    a[j/2] = xL;
	}
	break;
    default:
	av_assert0(fmt == AV_SAMPLE_FMT_FLT);
	for (size_t i = 0, j = 0; i < nsamples; i++, j += nc) {
	    float xL = fabsf(data[0][j]);
	    for (size_t k = 1; k < nc; k++) {
		float xM = fabsf(data[0][j+k]);
		if (xM > xL)
		    xL = xM;
	    }
	    a[i] = xL;
	}
    }

    switch (fmt) {
    case AV_SAMPLE_FMT_FLT  | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 2 << 8:
	break;
    default:
	for (size_t i = 0; i < nsamples; i++) {
	    float xL = a[i];
	    float xG = scale_to_dB(xL);
	    a[i] = xG;
	}
    }

    // non-vectorizable
    for (size_t i = 0; i < nsamples; ++i) {
	double xG = a[i];
	double yG = computeGain(s, xG);
	double cG = smoothAverage(s, yG);
	a[i] = cG;
    }
}

static void apply1(float **data, size_t off, int fmt, unsigned nc, float *a, size_t n)
{
    switch (fmt) {
    case AV_SAMPLE_FMT_FLT  | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 1 << 8:
	for (size_t i = 0; i < n; i++) {
	    float cG = a[i];
	    float cL = dB_to_scale(cG);
	    data[0][off+i] *= cL;
	}
	break;
    case AV_SAMPLE_FMT_FLTP | 2 << 8:
	for (size_t i = 0; i < n; i++) {
	    float cG = a[i];
	    float cL = dB_to_scale(cG);
	    data[0][off+i] *= cL;
	    data[1][off+i] *= cL;
#if 0
	    static int cnt;
	    static double sum;
	    sum += cL;
	    if (++cnt == 480) {
		fprintf(stderr, "\ncL=%f\n", sum / 480);
		sum = 0;
		cnt = 0;
	    }
#endif
	}
	break;
    default:
	for (size_t i = 0; i < n; i++) {
	    float cG = a[i];
	    float cL = dB_to_scale(cG);
	    a[i] = cL;
	}
    }

    switch (fmt) {
    case AV_SAMPLE_FMT_FLT  | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 2 << 8:
	break;
    case AV_SAMPLE_FMT_FLTP:
	for (size_t i = 0; i < n; i++) {
	    float cL = a[i];
	    for (int j = 0; j < nc; j++)
	        data[j][off+i] *= cL;
	}
	break;
    case AV_SAMPLE_FMT_FLT | 2 << 8:
	off *= 2;
	for (size_t j = 0; j < 2 * n; j += 2) {
	    float cL = a[j/2];
	    data[0][off+j+0] *= cL;
	    data[0][off+j+1] *= cL;
	}
	break;
    default:
	av_assert0(fmt == AV_SAMPLE_FMT_FLT);
	off *= nc;
	for (size_t i = 0, j = 0; i < n; i++, j += nc) {
	    float cL = a[i];
	    for (size_t k = 1; k < nc; k++)
		data[0][off+j+k] *= cL;
	}
    }
}

static int apply(QADRCContext *s, AVFilterLink *outlink,
	int fmt, unsigned nc, float *a, size_t nsamples)
{
    if (s->total_samples >= s->delay_samples)
	s->total_samples += nsamples;
    else {
	size_t off = s->delay_samples - s->total_samples;
	s->total_samples += nsamples;
	if (s->total_samples <= s->delay_samples)
	    return 0;
	av_assert0(off < nsamples);
	a += off;
	nsamples -= off;
    }

    int ret = 0;
    while (1) {
	AVFrame *f0 = s->frames[0];
	size_t f0samples = f0->nb_samples - s->fpos;
	size_t apply_samples = FFMIN(nsamples, f0samples);
	float **data0 = (float **) f0->extended_data;
	apply1(data0, s->fpos, fmt, nc, a, apply_samples);
	if (f0samples > nsamples) {
	    s->fpos += nsamples;
	    break;
        }
	else {
	    ret |= ff_filter_frame(outlink, f0);
	    s->nframes--;
	    memmove(s->frames, s->frames + 1, s->nframes * sizeof(AVFrame *));
	    s->fpos = 0;
	    nsamples -= apply_samples;
	    if (nsamples == 0)
		break;
	    av_assert0(s->nframes > 0);
	    a += apply_samples;
        }
    }

    return ret;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    QADRCContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    int nsamples = frame->nb_samples;
    unsigned nc = inlink->channels;

    float a[nsamples];
    int fmt = outlink->format | (nc <= 2 ? nc << 8 : 0);

    chew(s, frame, fmt, nc, a);
    s->lasta = a[nsamples - 1];

    s->frames = av_realloc_f(s->frames, s->nframes + 1, sizeof(AVFrame *));
    s->frames[s->nframes++] = frame;

    return apply(s, outlink, fmt, nc, a, nsamples);
}

static int final_flush(AVFilterLink *inlink, AVFilterContext *ctx, QADRCContext *s)
{
    AVFilterLink *outlink = ctx->outputs[0];

    AVFrame *f0 = s->frames[0];
    size_t nsamples = f0->nb_samples - s->fpos;
    unsigned nc = inlink->channels;

    float a[nsamples];
    int fmt = outlink->format | (nc <= 2 ? nc << 8 : 0);

    for (size_t i = 0; i < nsamples; i++)
	a[i] = s->lasta;

    return apply(s, outlink, fmt, nc, a, nsamples);
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    QADRCContext *s = ctx->priv;

    int ret = ff_request_frame(ctx->inputs[0]);
    if (ret == AVERROR_EOF && !ctx->is_disabled && s->nframes)
	ret = final_flush(outlink, ctx, s);

    return ret;
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts;
    int ret;

    layouts = ff_all_channel_layouts();

    if (!layouts)
	return AVERROR(ENOMEM);

    ff_add_format(&formats, AV_SAMPLE_FMT_FLT);
    ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);

    ret = ff_set_common_formats(ctx, formats);
    if (ret < 0)
	return ret;
    ret = ff_set_common_channel_layouts(ctx, layouts);
    if (ret < 0)
	return ret;
    return ff_set_common_samplerates(ctx, ff_all_samplerates());
}

static int config_output(AVFilterLink *outlink)
{
    outlink->flags |= FF_LINK_FLAG_REQUEST_LOOP;
    return 0;
}

#define OFFSET(x) offsetof(QADRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
static const AVOption qadrc_options[] = {
    { "thresh", "threshold", OFFSET(thresh), AV_OPT_TYPE_DOUBLE, {.dbl = -35}, -70, 0, FLAGS },
    { "ratio", "compression ratio", OFFSET(ratio), AV_OPT_TYPE_DOUBLE, {.dbl = 1.5}, 1, 100, FLAGS },
    { "knee", "knee width", OFFSET(knee), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 70, FLAGS },
    { "attack", "attack time", OFFSET(attack), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 1000, FLAGS },
    { "release", "release time", OFFSET(release), AV_OPT_TYPE_DOUBLE, {.dbl = 800}, 0, 9000, FLAGS },
    { "delay", "delay (lookahead) time", OFFSET(delay), AV_OPT_TYPE_DOUBLE, {.dbl = 10}, 0, 1000, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(qadrc);

static const AVFilterPad inputs[] = {
    {
	.name	   = "default",
	.type	   = AVMEDIA_TYPE_AUDIO,
	.filter_frame   = filter_frame,
	.config_props   = config_input,
	.needs_writable = 1,
    },
    { NULL }
};

static const AVFilterPad outputs[] = {
    {
	.name = "default",
	.type = AVMEDIA_TYPE_AUDIO,
	.config_props  = config_output,
	.request_frame = request_frame,
    },
    { NULL }
};

AVFilter ff_af_qadrc = {
    .name	  = "qadrc",
    .description   = NULL_IF_CONFIG_SMALL("qaac dynamic range compressor"),
    .query_formats = query_formats,
    .inputs	= inputs,
    .outputs       = outputs,
    .priv_size     = sizeof(QADRCContext),
    .priv_class    = &qadrc_class,
};
