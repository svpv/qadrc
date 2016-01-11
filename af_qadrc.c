#include <stdlib.h>
#include <math.h>
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

static inline double dB_to_scale(double dB)
{
    return pow(10, 0.05 * dB);
}

static inline double scale_to_dB(double scale)
{
    return 20 * log10(scale);
}

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

    void **data = frame->extended_data;
    const int nsamples = frame->nb_samples;
    const unsigned nc = inlink->channels;

#define FOR_PLANAR(T)					\
    for (j = 0, sample = &((T**)data)[j][i];		\
	 j < nc;					\
	 j++,   sample = &((T**)data)[j][i] )

#define FOR_NOPLAN(T)					\
    for (j = 0, sample = &((T**)data)[0][i*nc+j];	\
	 j < nc;					\
	 j++,   sample = &((T**)data)[0][i*nc+j] )

#define FILTER_LOOP(T, PLAN)				\
    for (size_t i = 0; i < nsamples; ++i) {		\
	int j; T *sample;				\
	double xL = 0;					\
	FOR_##PLAN(T) {					\
		double x = fabs(*sample);		\
		if (x > xL) xL = x;			\
	}						\
	double xG = scale_to_dB(xL);			\
	double yG = computeGain(s, xG);			\
	double cG = smoothAverage(s, yG);		\
	double cL = dB_to_scale(cG);			\
	FOR_##PLAN(T)					\
	    *sample *= cL;				\
    }							\

    switch (outlink->format) {
    case AV_SAMPLE_FMT_FLT: FILTER_LOOP(float, NOPLAN); break;
    case AV_SAMPLE_FMT_FLTP: FILTER_LOOP(float, PLANAR); break;
    case AV_SAMPLE_FMT_DBL: FILTER_LOOP(double, NOPLAN); break;
    case AV_SAMPLE_FMT_DBLP: FILTER_LOOP(double, PLANAR); break;
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
    ff_add_format(&formats, AV_SAMPLE_FMT_DBL);
    ff_add_format(&formats, AV_SAMPLE_FMT_DBLP);

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
