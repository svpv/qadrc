/*
 * qalimiter - stray spike limiter for ffmpeg
 *
 * Written by Alexey Tourbin.
 * Based on qaac limiter by nu774.
 * This file is distributed as Public Domain.
 *
 * Description by nu774:
 * When limiter is on, qaac will apply smart limiter that only affects
 * portions surrounding peaks beyond (near) 0dBFS. It will search peaks,
 * then applies non-linear filter to half cycle (zero-crossing point
 * to next zero-crossing point) surrounding each peak. The result falls
 * within 0dBFS range and still is smoothly connected to other parts,
 * and has much less audible distortions than simple dumb hard clip.
 */

#include <stddef.h>
#include <math.h>
#include "libavutil/channel_layout.h"
#include "libavutil/avassert.h"
#include "avfilter.h"
#include "audio.h"
#include "internal.h"

#define m_thresh 0.97f

/* fix a single spike between frames[fi1][f1_pos] and frames[fi2][f2_end] */
static void fix_spike1(AVFilterLink *inlink, AVFrame **frames, int ch,
	int fi1, size_t f1_pos, int fi2, size_t f2_end, float peak)
{
    float xpeak = peak;
    peak = fabsf(peak);

    for (int fi = fi1; fi <= fi2; fi++) {
	AVFrame *frame = frames[fi];
	if (!av_frame_is_writable(frame)) {
	    AVFrame *copy = ff_get_audio_buffer(inlink, frame->nb_samples);
	    av_frame_copy_props(copy, frame);
	    av_frame_copy(copy, frame);
	    av_frame_free(&frame);
	    frame = frames[fi] = copy;
        }

	size_t begin = (fi == fi1) ? f1_pos : 0;
	size_t end = (fi == fi2) ? f2_end : frame->nb_samples;
	float *x = (float *) frame->extended_data[ch];

	if (peak < m_thresh * 2.0) {
	    float a = (peak - m_thresh) / (peak * peak);
	    if (xpeak > 0) a = - a;
	    for (size_t i = begin; i < end; ++i)
		x[i] = x[i] + a * x[i] * x[i];
	}
	else {
	    float u = peak, v = m_thresh;
	    float a = (u - 2 * v) / (u * u * u);
	    float b = (3 * v - 2 * u) / (u * u);
	    if (xpeak < 0)
		b *= -1.0;
	    for (size_t i = begin; i < end; ++i)
		x[i] = x[i] + b * x[i] * x[i] + a * x[i] * x[i] * x[i];
	}
    }
}

/* search for a peak (above the threshold) */
static void find_peak(AVFrame **frames, int ch,
	int fi1, size_t f1_pos, int fi2, size_t f2_end,
	int *peak_fi, size_t *peak_pos, float *peak_val)
{
    for (int fi = fi1; fi <= fi2; fi++) {
	AVFrame *frame = frames[fi];

	size_t begin = (fi == fi1) ? f1_pos : 0;
	size_t end = (fi == fi2) ? f2_end : frame->nb_samples;
	float *x = (float *) frame->extended_data[ch];

	size_t i;
	for (i = begin; i < end; i++)
	    if (x[i] > m_thresh || x[i] < -m_thresh)
		break;
	if (i == end)
		continue;

	/* found a peak */
	*peak_fi = fi;
	*peak_pos = i;
	*peak_val = x[i];
	return;
    }
}

/* when a peak is found, search backwards for spike start */
static void find_spike_start(AVFrame **frames, int ch,
	int peak_fi, size_t peak_pos,
	int *start_fi, size_t *start_pos,
	float peak_val)
{
    for (int fi = peak_fi; fi >= 0; fi--) {
	AVFrame *frame = frames[fi];

	ssize_t pos = (fi == peak_fi) ? peak_pos : frame->nb_samples;
	float *x = (float *) frame->extended_data[ch];

	if (peak_val < 0) {
	    if (x[0] < 0)
		while (pos >= 0 && x[pos] < 0)
		    pos--;
	    else
		while (x[pos] < 0)
		    pos--;
	}
	else {
	    if (x[0] > 0)
		while (pos >= 0 && x[pos] > 0)
		    pos--;
	    else
		while (x[pos] > 0)
		    pos--;
	}
	if (pos < 0)
	    continue;
	/* found intersection */
	*start_fi = fi;
	*start_pos = pos + 1;
	return;
    }
    /* assume leftmost */
    *start_fi = 0;
    *start_pos = 0;
}

/* search to the end of the spike and update the peak value */
static void find_spike_end(AVFrame **frames, int ch,
	int peak_fi, size_t peak_pos, int fi2,
	int *end_fi, size_t *end_end,
	float *peak_val)
{
    for (int fi = peak_fi; fi <= fi2; fi++) {
	AVFrame *frame = frames[fi];
	size_t begin = (fi == peak_fi) ? peak_pos : 0;
	size_t end = frame->nb_samples;
	float *x = (float *) frame->extended_data[ch];

	size_t i = begin;
	float p = *peak_val;

	if (p < 0) {
	    /* the spike is below x-axis */
	    if (x[end-1] < 0) {
		for (; i < end && x[i] < 0; i++)
		    if (x[i] < p)
			p = x[i];
	    }
	    else {
		for (; x[i] < 0; i++)
		    if (x[i] < p)
			p = x[i];
	    }
	}
	else {
	    /* the spike is above x-axis */
	    if (x[end-1] > 0) {
		for (; i < end && x[i] > 0; i++)
		    if (x[i] > p)
			p = x[i];
	    }
	    else {
		for (; x[i] > 0; i++)
		    if (x[i] > p)
			p = x[i];
	    }
	}

	*peak_val = p;

	if (i == end)
	    continue;
	/* found intersection */
	*end_fi = fi;
	*end_end = i;
	return;
    }
    /* assume rightmost */
    *end_fi = fi2;
    *end_end = frames[fi2]->nb_samples;
}

static void fix_spikes(AVFilterLink *inlink, AVFrame **frames, int ch,
	int fi1, size_t f1_pos, int fi2, size_t f2_end)
{
    while (1) {
	/* find peak */
	int peak_fi;
	size_t peak_pos;
	float peak_val = 0;
	find_peak(frames, ch, fi1, f1_pos, fi2, f2_end,
		&peak_fi, &peak_pos, &peak_val);
	if (peak_val == 0)
	    return;

	/* find spike start */
	int start_fi;
	size_t start_pos;
	find_spike_start(frames, ch, peak_fi, peak_pos,
		&start_fi, &start_pos, peak_val);

	/* find spike end */
	int end_fi;
	size_t end_end;
	find_spike_end(frames, ch, peak_fi, peak_pos, fi2,
		&end_fi, &end_end, &peak_val);

	/* fix the spike */
	fix_spike1(inlink, frames, ch,
		start_fi, start_pos, end_fi, end_end, peak_val);

	if (end_fi == fi2 && end_end == f2_end)
	    break;
	fi1 = end_fi;
	f1_pos = end_end;
    }
}

/* find the end limit up to which the buffer can be processed;
 * that is, up to the last intersection with x-axis */
static size_t find_channel_end(AVFrame *frame, int ch)
{
    float *x = (float *) frame->extended_data[ch];
    ssize_t pos = frame->nb_samples - 1;
    if (x[pos] < 0) {
	if (x[0] < 0)
	    do
		pos--;
	    while (pos >= 0 && x[pos] < 0);
	else
	    do
		pos--;
	    while (x[pos] < 0);
    }
    else if (x[pos] > 0) {
	if (x[0] > 0)
	    do
		pos--;
	    while (pos >= 0 && x[pos] > 0);
	else
	    do
		pos--;
	    while (x[pos] > 0);
    }
    return pos + 1;
}

typedef struct QALimiterContext {
    AVFrame **frames;
    size_t nframes;
    int fi[8]; /* frame index, per channel */
    size_t fpos[8]; /* position in the frame up to which the input has been processed */
} QALimiterContext;

static int flush_frames(AVFilterContext *ctx, QALimiterContext *s, int nch)
{
    int fiend = s->fi[0];
    for (int ch = 1; ch < nch; ch++)
	if (s->fi[ch] < fiend)
	    fiend = s->fi[ch];

    if (fiend == 0)
	return 0;

    for (int fi = 0; fi < fiend; fi++)
	ff_filter_frame(ctx->outputs[0], s->frames[fi]);

    s->nframes -= fiend;
    memmove(s->frames, s->frames + fiend, s->nframes * sizeof(AVFrame *));

    for (int ch = 0; ch < nch; ch++)
	s->fi[ch] -= fiend;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    QALimiterContext *s = ctx->priv;

    s->frames = av_realloc_f(s->frames, s->nframes + 1, sizeof(AVFrame *));
    s->frames[s->nframes++] = frame;

    int nch = inlink->channels;
    for (int ch = 0; ch < nch; ch++) {
	size_t end = find_channel_end(frame, ch);
	if (end == 0)
	    // no intersection with x-axis
	    continue;
	int full = (end == frame->nb_samples);
	fix_spikes(inlink, s->frames, ch,
		s->fi[ch], s->fpos[ch], s->nframes - 1, end);
	if (full) {
	    s->fi[ch] = s->nframes;
	    s->fpos[ch] = 0;
	}
	else {
	    s->fi[ch] = s->nframes - 1;
	    s->fpos[ch] = end;
	}
    }

    return flush_frames(ctx, s, nch);
}

static int final_flush(AVFilterLink *inlink, AVFilterContext *ctx, QALimiterContext *s)
{
    int nch = inlink->channels;
    for (int ch = 0; ch < nch; ch++) {
	if (s->fi[ch] == s->nframes)
	    continue;
	fix_spikes(inlink, s->frames, ch,
		s->fi[ch], s->fpos[ch], s->nframes - 1,
		s->frames[s->nframes-1]->nb_samples - 1);
	s->fi[ch] = s->nframes;
	s->fpos[ch] = 0;
    }
    return flush_frames(ctx, s, nch);
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    QALimiterContext *s = ctx->priv;

    int ret = ff_request_frame(ctx->inputs[0]);
    if (ret == AVERROR_EOF && !ctx->is_disabled && s->nframes)
	ret = final_flush(outlink, ctx, s);

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    QALimiterContext *s = ctx->priv;
    for (int i = 0; i < s->nframes; i++)
	av_frame_free(&s->frames[i]);
    av_freep(&s->frames);
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts;

    ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    ff_set_common_formats(ctx, formats);

    layouts = ff_all_channel_layouts();
    ff_set_common_channel_layouts(ctx, layouts);

    formats = ff_all_samplerates();
    ff_set_common_samplerates(ctx, formats);

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .filter_frame  = filter_frame,
    },
    { NULL }
};

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .request_frame = request_frame,
    },
    { NULL }
};

AVFilter ff_af_qalimiter = {
    .name          = "qalimiter",
    .description   = NULL_IF_CONFIG_SMALL("qaac soft limiter"),
    .uninit        = uninit,
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
    .priv_size     = sizeof(QALimiterContext),
};
