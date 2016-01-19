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

    double slope;
    double Tlo;
    double Thi;
    double knee_factor;

    double alphaA;
    double alphaR;

    double yR;
    double yA;
} QADRCContext;

/* Workaround a lack of optimization in gcc */
static float exp_cst1 = 2139095040.f;
static float exp_cst2 = 0.f;

/* Relative error bounded by 1e-5 for normalized outputs
   Returns invalid outputs for nan inputs
   Continuous error */
static inline float expapprox(float val)
{
  union { int i; float f; } xu, xu2;
  float val2, val3, val4, b;
  int val4i;
  val2 = 12102203.1615614f*val+1065353216.f;
  val3 = val2 < exp_cst1 ? val2 : exp_cst1;
  val4 = val3 > exp_cst2 ? val3 : exp_cst2;
  val4i = (int) val4;
  xu.i = val4i & 0x7F800000;
  xu2.i = (val4i & 0x7FFFFF) | 0x3F800000;
  b = xu2.f;

  /* Generated in Sollya with:
     > f=remez(1-x*exp(-(x-1)*log(2)),
	       [|1,(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x|],
	       [1,2], exp(-(x-1)*log(2)));
     > plot(exp((x-1)*log(2))/(f+x)-1, [1,2]);
     > f+x;
  */
  return
    xu.f * (0.510397365625862338668154f + b *
	    (0.310670891004095530771135f + b *
	     (0.168143436463395944830000f + b *
	      (-2.88093587581985443087955e-3f + b *
	       1.3671023382430374383648148e-2f))));
}

/* Absolute error bounded by 1e-6 for normalized inputs
   Returns a finite number for +inf input
   Returns -inf for nan and <= 0 inputs.
   Continuous error. */
static inline float logapprox(float val)
{
  union { float f; int i; } valu;
  float exp, addcst, x;
  valu.f = val;
  exp = valu.i >> 23;
  /* 89.970756366f = 127 * log(2) - constant term of polynomial */
  addcst = val > 0 ? -89.970756366f : -(float)INFINITY;
  valu.i = (valu.i & 0x7FFFFF) | 0x3F800000;
  x = valu.f;

  /* Generated in Sollya using :
    > f = remez(log(x)-(x-1)*log(2),
	    [|1,(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x,
	      (x-1)*(x-2)*x*x*x|], [1,2], 1, 1e-8);
    > plot(f+(x-1)*log(2)-log(x), [1,2]);
    > f+(x-1)*log(2)
  */
  return
    x * (3.529304993f + x * (-2.461222105f +
      x * (1.130626167f + x * (-0.288739945f +
	x * 3.110401639e-2f))))
    + (addcst + 0.69314718055995f*exp);
}

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

#if 0
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
    s->yR = 0.0;
    s->yA = 0.0;

    const double Fs = inlink->sample_rate;
    s->alphaA = s->attack > 0.0 ? exp(-1.0 / (s->attack * Fs)) : 0.0;
    s->alphaR = s->release > 0.0 ? exp(-1.0 / (s->release * Fs)) : 0.0;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    QADRCContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    float **data = (float **) frame->extended_data;
    const int nsamples = frame->nb_samples;
    const unsigned nc = inlink->channels;

    float a[nsamples];
    int fmt = outlink->format | (nc <= 2 ? nc << 8 : 0);

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

    // apply gain
    switch (fmt) {
    case AV_SAMPLE_FMT_FLT  | 1 << 8:
    case AV_SAMPLE_FMT_FLTP | 1 << 8:
	for (size_t i = 0; i < nsamples; i++) {
	    float cG = a[i];
	    float cL = dB_to_scale(cG);
	    data[0][i] *= cL;
	}
	break;
    case AV_SAMPLE_FMT_FLTP | 2 << 8:
	for (size_t i = 0; i < nsamples; i++) {
	    float cG = a[i];
	    float cL = dB_to_scale(cG);
	    data[0][i] *= cL;
	    data[1][i] *= cL;
	}
	break;
    default:
	for (size_t i = 0; i < nsamples; i++) {
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
	for (size_t i = 0; i < nsamples; i++) {
	    float cL = a[i];
	    for (int j = 0; j < nc; j++)
	        data[j][i] *= cL;
	}
	break;
    case AV_SAMPLE_FMT_FLT | 2 << 8:
	for (size_t j = 0; j < 2 * nsamples; j += 2) {
	    float cL = a[j/2];
	    data[0][j+0] *= cL;
	    data[0][j+1] *= cL;
	}
	break;
    default:
	av_assert0(fmt == AV_SAMPLE_FMT_FLT);
	for (size_t i = 0, j = 0; i < nsamples; i++, j += nc) {
	    float cL = a[i];
	    for (size_t k = 1; k < nc; k++)
		data[0][j+k] *= cL;
	}
    }

    return ff_filter_frame(outlink, frame);
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

#define OFFSET(x) offsetof(QADRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
static const AVOption qadrc_options[] = {
    { "thresh", "threshold", OFFSET(thresh), AV_OPT_TYPE_DOUBLE, {.dbl = -35}, -70, 0, FLAGS },
    { "ratio", "compression ratio", OFFSET(ratio), AV_OPT_TYPE_DOUBLE, {.dbl = 1.5}, 1, 100, FLAGS },
    { "knee", "knee width", OFFSET(knee), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 70, FLAGS },
    { "attack", "attack time", OFFSET(attack), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 1000, FLAGS },
    { "release", "release time", OFFSET(release), AV_OPT_TYPE_DOUBLE, {.dbl = 800}, 0, 9000, FLAGS },
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
