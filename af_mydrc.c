/*
 * Dynamic Audio Normalizer
 * Copyright (c) 2015 LoRd_MuldeR <mulder2@gmx.de>. Some rights reserved.
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Dynamic Audio Normalizer
 */

#include <float.h>
#include <stdbool.h>

#include "libavutil/avassert.h"
#include "libavutil/opt.h"

#define FF_BUFQUEUE_SIZE 302
#include "libavfilter/bufferqueue.h"

#include "audio.h"
#include "avfilter.h"
#include "internal.h"

typedef struct cqueue {
    double *elements;
    int size;
    int nb_elements;
    int first;
} cqueue;

typedef struct MyDRCContext {
    const AVClass *class;

    struct FFBufQueue queue;

    int frame_len;
    int frame_len_msec;
    int filter_size;

    double *prev_amplification_factor;
    double *dc_correction_value;
    double *compress_threshold;
    double *fade_factors[2];
    double *weights;

    int channels;
    int delay;

    cqueue **gain_history_original;
    cqueue **gain_history_smoothed;

    // gain computer
    double thresh;
    double ratio;
    double knee;

    double Tlo;
    double Thi;

    double slope;
    double knee_factor;

    // highpass filter
    double hi_a;
    double hi_x[8];
    double hi_y[8];
    bool hi_once;

    long cnt[8];
    double sum[8];
} MyDRCContext;

#define OFFSET(x) offsetof(MyDRCContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption mydrc_options[] = {
    { "thresh", "threshold", OFFSET(thresh), AV_OPT_TYPE_DOUBLE, {.dbl = -35}, -70, 0, FLAGS },
    { "ratio", "compression ratio", OFFSET(ratio), AV_OPT_TYPE_DOUBLE, {.dbl = 1.5}, 1, 100, FLAGS },
    { "knee", "knee width", OFFSET(knee), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 70, FLAGS },
    { "f", "set the frame length in msec",     OFFSET(frame_len_msec),    AV_OPT_TYPE_INT,    {.i64 = 500},   10,  8000, FLAGS },
    { "g", "set the filter size",              OFFSET(filter_size),       AV_OPT_TYPE_INT,    {.i64 = 31},     3,   301, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(mydrc);

static av_cold int init(AVFilterContext *ctx)
{
    MyDRCContext *s = ctx->priv;

    if (!(s->filter_size & 1)) {
        av_log(ctx, AV_LOG_ERROR, "filter size %d is invalid. Must be an odd value.\n", s->filter_size);
        return AVERROR(EINVAL);
    }

    return 0;
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats;
    AVFilterChannelLayouts *layouts;
    static const enum AVSampleFormat sample_fmts[] = {
        AV_SAMPLE_FMT_DBLP,
        AV_SAMPLE_FMT_NONE
    };
    int ret;

    layouts = ff_all_channel_layouts();
    if (!layouts)
        return AVERROR(ENOMEM);
    ret = ff_set_common_channel_layouts(ctx, layouts);
    if (ret < 0)
        return ret;

    formats = ff_make_format_list(sample_fmts);
    if (!formats)
        return AVERROR(ENOMEM);
    ret = ff_set_common_formats(ctx, formats);
    if (ret < 0)
        return ret;

    formats = ff_all_samplerates();
    if (!formats)
        return AVERROR(ENOMEM);
    return ff_set_common_samplerates(ctx, formats);
}

static inline int frame_size(int sample_rate, int frame_len_msec)
{
    const int frame_size = round((double)sample_rate * (frame_len_msec / 1000.0));
    return frame_size + (frame_size % 2);
}

static void precalculate_fade_factors(double *fade_factors[2], int frame_len)
{
    const double step_size = 1.0 / frame_len;
    int pos;

    for (pos = 0; pos < frame_len; pos++) {
        fade_factors[0][pos] = 1.0 - (step_size * pos);
	av_assert0(fade_factors[0][pos] >= 0.0);
	av_assert0(fade_factors[0][pos] <= 1.0);
        fade_factors[1][pos] = 1.0 - fade_factors[0][pos];
	av_assert0(fade_factors[1][pos] >= 0.0);
	av_assert0(fade_factors[1][pos] <= 1.0);
    }
}

static cqueue *cqueue_create(int size)
{
    cqueue *q;

    q = av_malloc(sizeof(cqueue));
    if (!q)
        return NULL;

    q->size = size;
    q->nb_elements = 0;
    q->first = 0;

    q->elements = av_malloc(sizeof(double) * size);
    if (!q->elements) {
        av_free(q);
        return NULL;
    }

    return q;
}

static void cqueue_free(cqueue *q)
{
    av_free(q->elements);
    av_free(q);
}

static int cqueue_size(cqueue *q)
{
    return q->nb_elements;
}

static int cqueue_empty(cqueue *q)
{
    return !q->nb_elements;
}

static int cqueue_enqueue(cqueue *q, double element)
{
    int i;

    av_assert2(q->nb_elements != q->size);

    i = (q->first + q->nb_elements) % q->size;
    q->elements[i] = element;
    q->nb_elements++;

    return 0;
}

static double cqueue_peek(cqueue *q, int index)
{
    av_assert2(index < q->nb_elements);
    return q->elements[(q->first + index) % q->size];
}

static int cqueue_dequeue(cqueue *q, double *element)
{
    av_assert2(!cqueue_empty(q));

    *element = q->elements[q->first];
    q->first = (q->first + 1) % q->size;
    q->nb_elements--;

    return 0;
}

static int cqueue_pop(cqueue *q)
{
    av_assert2(!cqueue_empty(q));

    q->first = (q->first + 1) % q->size;
    q->nb_elements--;

    return 0;
}

static void init_gaussian_filter(MyDRCContext *s)
{
    double total_weight = 0.0;
    const double sigma = (((s->filter_size / 2.0) - 1.0) / 3.0) + (1.0 / 3.0);
    double adjust;
    int i;

    // Pre-compute constants
    const int offset = s->filter_size / 2;
    const double c1 = 1.0 / (sigma * sqrt(2.0 * M_PI));
    const double c2 = 2.0 * pow(sigma, 2.0);

    // Compute weights
    for (i = 0; i < s->filter_size; i++) {
        const int x = i - offset;

        s->weights[i] = c1 * exp(-(pow(x, 2.0) / c2));
        total_weight += s->weights[i];
    }

    // Adjust weights
    adjust = 1.0 / total_weight;
    for (i = 0; i < s->filter_size; i++) {
        s->weights[i] *= adjust;
    }
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    MyDRCContext *s = ctx->priv;
    int c;

    int hz = 150;
    double RC = 1 / (2 * M_PI * hz);
    s->hi_a = RC / (RC + 1.0 / inlink->sample_rate);

    s->frame_len =
    inlink->min_samples =
    inlink->max_samples =
    inlink->partial_buf_size = frame_size(inlink->sample_rate, s->frame_len_msec);
    av_log(ctx, AV_LOG_DEBUG, "frame len %d\n", s->frame_len);

    s->fade_factors[0] = av_malloc(s->frame_len * sizeof(*s->fade_factors[0]));
    s->fade_factors[1] = av_malloc(s->frame_len * sizeof(*s->fade_factors[1]));

    s->prev_amplification_factor = av_malloc(inlink->channels * sizeof(*s->prev_amplification_factor));
    s->dc_correction_value = av_calloc(inlink->channels, sizeof(*s->dc_correction_value));
    s->compress_threshold = av_calloc(inlink->channels, sizeof(*s->compress_threshold));
    s->gain_history_original = av_calloc(inlink->channels, sizeof(*s->gain_history_original));
    s->gain_history_smoothed = av_calloc(inlink->channels, sizeof(*s->gain_history_smoothed));
    s->weights = av_malloc(s->filter_size * sizeof(*s->weights));
    if (!s->prev_amplification_factor || !s->dc_correction_value ||
        !s->compress_threshold || !s->fade_factors[0] || !s->fade_factors[1] ||
        !s->gain_history_original ||
        !s->gain_history_smoothed || !s->weights)
        return AVERROR(ENOMEM);

    for (c = 0; c < inlink->channels; c++) {
        s->prev_amplification_factor[c] = 1.0;

        s->gain_history_original[c] = cqueue_create(s->filter_size);
        s->gain_history_smoothed[c] = cqueue_create(s->filter_size);

        if (!s->gain_history_original[c] ||
            !s->gain_history_smoothed[c])
            return AVERROR(ENOMEM);
    }

    precalculate_fade_factors(s->fade_factors, s->frame_len);
    init_gaussian_filter(s);

    s->channels = inlink->channels;
    s->delay = s->filter_size;

    return 0;
}

static int config_output(AVFilterLink *outlink)
{
    outlink->flags |= FF_LINK_FLAG_REQUEST_LOOP;
    return 0;
}

static inline double fade(double prev, double next, int pos,
                          double *fade_factors[2])
{
    return fade_factors[0][pos] * prev + fade_factors[1][pos] * next;
}

static inline double bound(const double threshold, const double val)
{
    const double CONST = 0.8862269254527580136490837416705725913987747280611935; //sqrt(PI) / 2.0
    return erf(CONST * (val / threshold)) * threshold;
}

static double rms_sum(MyDRCContext *s, double *data, int c, int ns)
{
    double sum = 0;
    double x0 = s->hi_x[c];
    double y0 = s->hi_y[c];

    for (int i = 0; i < ns; i++) {
	double x1 = data[i];
	double y1 = s->hi_a * (y0 + x1 - x0);
	sum += y1 * y1;
	x0 = x1;
	y0 = y1;
    }

    s->hi_x[c] = x0;
    s->hi_y[c] = y0;

    return sum;
}

static double get_frame_rms_dB(MyDRCContext *s, AVFrame *frame)
{
    int nc = av_frame_get_channels(frame);

    if (!s->hi_once) {
	for (int c = 0; c < nc; c++)
	    s->hi_x[c] = s->hi_y[c] = *(double *) frame->extended_data[c];
	s->hi_once = true;
    }

    double sum = 0;
    for (int c = 0; c < nc; c++)
	sum += rms_sum(s, frame->extended_data[c], c, frame->nb_samples);

    double mean = FFMAX(sum / frame->nb_samples, DBL_EPSILON);
    return 10 * log10(mean);
}

static double gaussian_filter(MyDRCContext *s, cqueue *q)
{
    double result = 0.0;
    int i;

    for (i = 0; i < cqueue_size(q); i++) {
        result += cqueue_peek(q, i) * s->weights[i];
    }

    return result;
}

static void update_gain_history(MyDRCContext *s, int channel,
                                double current_gain_factor)
{
    if (cqueue_empty(s->gain_history_original[channel])) {
        const int pre_fill_size = s->filter_size / 2;

        s->prev_amplification_factor[channel] = current_gain_factor;

        while (cqueue_size(s->gain_history_original[channel]) < pre_fill_size) {
            cqueue_enqueue(s->gain_history_original[channel], current_gain_factor);
        }
    }

    cqueue_enqueue(s->gain_history_original[channel], current_gain_factor);

    while (cqueue_size(s->gain_history_original[channel]) >= s->filter_size) {
        double smoothed;
        av_assert0(cqueue_size(s->gain_history_original[channel]) == s->filter_size);
        smoothed = gaussian_filter(s, s->gain_history_original[channel]);

        cqueue_enqueue(s->gain_history_smoothed[channel], smoothed);

        cqueue_pop(s->gain_history_original[channel]);
    }
}

static inline double update_value(double new, double old, double aggressiveness)
{
    av_assert0((aggressiveness >= 0.0) && (aggressiveness <= 1.0));
    return aggressiveness * new + (1.0 - aggressiveness) * old;
}

static inline double dB_to_scale(double dB)
{
    return pow(10, 0.05 * dB);
}

static inline double scale_to_dB(double scale)
{
    return 20 * log10(scale);
}

static double compute_gain(MyDRCContext *s, double x)
{
    s->slope = (1.0 - s->ratio) / s->ratio;
    s->Tlo = s->thresh - s->knee / 2.0;
    s->Thi = s->thresh + s->knee / 2.0;
    s->knee_factor = s->slope / (s->knee * 2.0);

    if (x < s->Tlo)
        return 0.0;
    else if (x > s->Thi)
        return s->slope * (x - s->thresh);
    else {
        double delta = x - s->Tlo;
        return delta * delta * s->knee_factor;
    }
}

static void analyze_frame(MyDRCContext *s, AVFrame *frame)
{
	const double vol_dB = get_frame_rms_dB(s, frame);
	fprintf(stderr, "vol_dB=%f\n", vol_dB);
	const double gain_dB = compute_gain(s, vol_dB);
	const double gain = dB_to_scale(gain_dB);
	//fprintf(stderr, "gain=%f\n", gain);
        int c;

        for (c = 0; c < s->channels; c++)
            update_gain_history(s, c, gain);
}

static void amplify_frame(MyDRCContext *s, AVFrame *frame)
{
    int c, i;

    for (c = 0; c < s->channels; c++) {
        double *dst_ptr = (double *)frame->extended_data[c];
        double current_amplification_factor;

        cqueue_dequeue(s->gain_history_smoothed[c], &current_amplification_factor);

        for (i = 0; i < frame->nb_samples; i++) {
            const double amplification_factor = fade(s->prev_amplification_factor[c],
                                                     current_amplification_factor, i,
                                                     s->fade_factors);
            dst_ptr[i] *= amplification_factor;

	    s->sum[c] += amplification_factor;
	    if (++s->cnt[c] % 480 == 0) {
	        fprintf(stderr, "cL%d=%f\n", c, s->sum[c] / 480);
	        s->sum[c] = 0;
	    }
        }

        s->prev_amplification_factor[c] = current_amplification_factor;
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    MyDRCContext *s = ctx->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    int ret = 0;

    if (!cqueue_empty(s->gain_history_smoothed[0])) {
        AVFrame *out = ff_bufqueue_get(&s->queue);

        amplify_frame(s, out);
        ret = ff_filter_frame(outlink, out);
    }

    analyze_frame(s, in);
    ff_bufqueue_add(ctx, &s->queue, in);

    return ret;
}

static int flush_buffer(MyDRCContext *s, AVFilterLink *inlink,
                        AVFilterLink *outlink)
{
    AVFrame *out = ff_get_audio_buffer(outlink, s->frame_len);
    int c, i;

    if (!out)
        return AVERROR(ENOMEM);

    for (c = 0; c < s->channels; c++) {
        double *dst_ptr = (double *)out->extended_data[c];

        for (i = 0; i < out->nb_samples; i++) {
            dst_ptr[i] = DBL_EPSILON;
        }
    }

    s->delay--;
    return filter_frame(inlink, out);
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MyDRCContext *s = ctx->priv;
    int ret = 0;

    ret = ff_request_frame(ctx->inputs[0]);

    if (ret == AVERROR_EOF && !ctx->is_disabled && s->delay)
        ret = flush_buffer(s, ctx->inputs[0], outlink);

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    MyDRCContext *s = ctx->priv;
    int c;

    av_freep(&s->prev_amplification_factor);
    av_freep(&s->dc_correction_value);
    av_freep(&s->compress_threshold);
    av_freep(&s->fade_factors[0]);
    av_freep(&s->fade_factors[1]);

    for (c = 0; c < s->channels; c++) {
        cqueue_free(s->gain_history_original[c]);
        cqueue_free(s->gain_history_smoothed[c]);
    }

    av_freep(&s->gain_history_original);
    av_freep(&s->gain_history_smoothed);

    av_freep(&s->weights);

    ff_bufqueue_discard_all(&s->queue);
}

static const AVFilterPad avfilter_af_mydrc_inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_AUDIO,
        .filter_frame   = filter_frame,
        .config_props   = config_input,
        .needs_writable = 1,
    },
    { NULL }
};

static const AVFilterPad avfilter_af_mydrc_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
        .request_frame = request_frame,
    },
    { NULL }
};

AVFilter ff_af_mydrc = {
    .name          = "mydrc",
    .description   = NULL_IF_CONFIG_SMALL("Dynamic Audio Normalizer."),
    .query_formats = query_formats,
    .priv_size     = sizeof(MyDRCContext),
    .init          = init,
    .uninit        = uninit,
    .inputs        = avfilter_af_mydrc_inputs,
    .outputs       = avfilter_af_mydrc_outputs,
    .priv_class    = &mydrc_class,
};
