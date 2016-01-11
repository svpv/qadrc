#include <stddef.h>
#include <math.h>
#include "libavutil/channel_layout.h"
#include "avfilter.h"
#include "audio.h"
#include "internal.h"

typedef struct QALimiterContext {
    float *buffer[8];
    size_t bufsiz[8];
    size_t processed[8];
} QALimiterContext;

#define m_thresh 0.9921875f

#define m_buffer s->buffer
#define m_bufsiz s->bufsiz
#define m_processed s->processed

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    QALimiterContext *s = ctx->priv;

    const int nin = frame->nb_samples;
    const int m_nchannels = inlink->channels;
    for (int n = 0; n < m_nchannels; ++n) {
	// fill the buffer
	float *x = m_buffer[n] = av_realloc_f(m_buffer[n], m_bufsiz[n] + nin, sizeof(float));
	memcpy(x + m_bufsiz[n], frame->extended_data[n], nin * sizeof(float));
	m_bufsiz[n] += nin;
	// find the end limit up to which the buffer can be processed;
	// that is, up to the last intersection with x-axis
	ssize_t limit = m_bufsiz[n];
	if (limit > 0 && nin > 0) {
	    float last = x[limit-1];
	    for (; limit > 0 && x[limit-1] * last > 0; --limit)
		;
	}
	// recall the last limit, the end at which we stopped last time
	ssize_t end = m_processed[n];
	// fix the spikes between the last end and the new end
	while (end < limit) {
	    ssize_t peak_pos = end;
	    for (; peak_pos < limit; ++peak_pos)
		if (x[peak_pos] > m_thresh || x[peak_pos] < -m_thresh)
		    break;
	    if (peak_pos == limit)
		break;
	    ssize_t start = peak_pos;
	    float peak = fabsf(x[peak_pos]);

	    while (start > end && x[peak_pos] * x[start] >= 0)
		--start;
	    ++start;
	    for (end = peak_pos + 1; end < limit; ++end) {
		if (x[peak_pos] * x[end] < 0)
		    break;
		float y = fabsf(x[end]);
		if (y > peak) {
		    peak = y;
		    peak_pos = end;
		}
	    }
	    if (peak < m_thresh * 2.0) {
		float a = (peak - m_thresh) / (peak * peak);
		if (x[peak_pos] > 0) a = - a;
		for (ssize_t i = start; i < end; ++i)
		    x[i] = x[i] + a * x[i] * x[i];
	    } else {
		float u = peak, v = m_thresh;
		float a = (u - 2 * v) / (u * u * u);
		float b = (3 * v - 2 * u) / (u * u);
		if (x[peak_pos] < 0)
		    b *= -1.0;
		for (ssize_t i = start; i < end; ++i)
		    x[i] = x[i] + b * x[i] * x[i] + a * x[i] * x[i] * x[i];
	    }
	}
	m_processed[n] = limit;
    }
    size_t prod = 2 * nin;
    for (int n = 0; n < m_nchannels; ++n)
	if (m_processed[n] < prod)
	    prod = m_processed[n];

    if (prod < 1) {
	av_frame_free(&frame);
	return 0;
    }

    AVFrame *out_frame = ff_get_audio_buffer(inlink, prod);
    av_frame_copy_props(out_frame, frame);
    av_frame_free(&frame);

    for (int n = 0; n < m_nchannels; ++n)
	memcpy(out_frame->extended_data[n], s->buffer[n], prod * sizeof(float));

    for (int n = 0; n < m_nchannels; ++n) {
	float *x = m_buffer[n];
	if (m_bufsiz[n] && prod) {
	    memmove(x, x + prod, (m_bufsiz[n] - prod) * sizeof(float));
	    m_bufsiz[n] -= prod;
	    m_processed[n] -= prod;
	}
    }

    return ff_filter_frame(ctx->outputs[0], out_frame);
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts;

    ff_add_format(&formats, AV_SAMPLE_FMT_FLT);
    ff_set_common_formats(ctx, formats);

    layouts = ff_all_channel_layouts();
    ff_set_common_channel_layouts(ctx, layouts);

    formats = ff_all_samplerates();
    ff_set_common_samplerates(ctx, formats);

    return 0;
}

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
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

AVFilter ff_af_qalimiter = {
    .name          = "qalimiter",
    .description   = NULL_IF_CONFIG_SMALL("qaac soft limiter"),
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
    .priv_size     = sizeof(QALimiterContext),
};
