/*
 * mono parts - apply mono effect to audio parts
 *
 * Written by Alexey Tourbin.
 * This file is distributed as Public Domain.
 */

#include <stdbool.h>
#include "libavutil/opt.h"
#include "libavutil/channel_layout.h"
#include "avfilter.h"
#include "audio.h"
#include "internal.h"

static void stereo2mono_fltp(AVFrame *frame)
{
    float **data = (float **) frame->extended_data;
    for (int i = 0; i < frame->nb_samples; i++) {
	double c0 = data[0][i];
	double c1 = data[1][i];
	double step = 0.5 / (frame->nb_samples + 1);
	double attack = (i + 1) * step; // from 0 to 0.5
	double release = 1 - attack; // from 1 down to 0.5
	data[0][i] = c0 * release + c1 * attack;
	data[1][i] = c1 * release + c0 * attack;
    }
}

static void mono2stereo_fltp(AVFrame *frame)
{
    float **data = (float **) frame->extended_data;
    for (int i = 0; i < frame->nb_samples; i++) {
	double c0 = data[0][i];
	double c1 = data[1][i];
	double step = 0.5 / (frame->nb_samples + 1);
	double attack = 0.5 + (i + 1) * step; // from 0.5 to 1
	double release = 1 - attack; // from 0.5 down to 0
	data[0][i] = c0 * attack + c1 * release;
	data[1][i] = c1 * attack + c0 * release;
    }
}

static void full_mono_fltp(AVFrame *frame)
{
    float **data = (float **) frame->extended_data;
    for (int i = 0; i < frame->nb_samples; i++) {
	double c0 = data[0][i];
	double c1 = data[1][i];
	double avg = (c0 + c1) / 2;
	data[0][i] = avg;
	data[1][i] = avg;
    }
}

typedef struct MonoPartsContext {
    const AVClass *class;
    const char *parts0;
    const char *parts;
    int part1, part2;
    int current_part;
    void (*stereo2mono)(AVFrame *frame);
    void (*mono2stereo)(AVFrame *frame);
    void (*full_mono)(AVFrame *frame);
} MonoPartsContext;

#define OFFSET(x) offsetof(MonoPartsContext, x)
#define FLAGS AV_OPT_FLAG_AUDIO_PARAM|AV_OPT_FLAG_FILTERING_PARAM
static const AVOption monoparts_options[] = {
    { "parts", "list of parts to be made mono", OFFSET(parts0), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(monoparts);

static bool scan_part(AVFilterContext *ctx)
{
    MonoPartsContext *s = ctx->priv;
    if (s->parts == NULL)
	s->parts = s->parts0;
    if (s->parts == NULL)
	goto error;
    int part1, part2, size;
    int ret = sscanf(s->parts, "%d-%d%n", &part1, &part2, &size);
    if (ret != 2)
	goto error;
    if (s->parts[size] == '|')
	s->parts += size + 1;
    else if (s->parts[size] == '\0')
	s->parts += size;
    else
	goto error;
    if (part1 < 0 || part2 < 0)
	goto error;
    if (part1 >= part2)
	    goto error;
    if (part1 <= s->part2 && s->part2)
	goto error;
    s->part1 = part1;
    s->part2 = part2;
    return true;
error:
    av_log(ctx, AV_LOG_ERROR, "cannot parse part spec: %s\n", s->parts);
    return false;
}

static int init(AVFilterContext *ctx)
{
    MonoPartsContext *s = ctx->priv;
    s->mono2stereo = mono2stereo_fltp;
    s->stereo2mono = stereo2mono_fltp;
    s->full_mono = full_mono_fltp;
    if (!scan_part(ctx))
        return AVERROR(EINVAL);
    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    MonoPartsContext *s = ctx->priv;

    if (inlink->sample_rate % 10) {
        av_log(ctx, AV_LOG_ERROR, "weird sample rate: %d\n", inlink->sample_rate);
        return AVERROR(EINVAL);
    }
    inlink->min_samples =
    inlink->max_samples =
    inlink->partial_buf_size = inlink->sample_rate / 10;

    return 0;
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts = NULL;
    int ret;

    ret = ff_add_channel_layout(&layouts, AV_CH_LAYOUT_STEREO);
    if (ret < 0) return ret;
    ret = ff_set_common_channel_layouts(ctx, layouts);
    if (ret < 0) return ret;

#if 0
    ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLT);
    if (ret < 0) return ret;
#endif
    ret = ff_add_format(&formats, AV_SAMPLE_FMT_FLTP);
    if (ret < 0) return ret;
    ret = ff_set_common_formats(ctx, formats);
    if (ret < 0) return ret;

    return ff_set_common_samplerates(ctx, ff_all_samplerates());
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    MonoPartsContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    int part = s->current_part++;
    if (part < s->part1) // stereo passthru
	return ff_filter_frame(outlink, frame);

    if (!av_frame_is_writable(frame)) {
	AVFrame *copy = ff_get_audio_buffer(inlink, frame->nb_samples);
	av_frame_copy_props(copy, frame);
	av_frame_copy(copy, frame);
	av_frame_free(&frame);
	frame = copy;
    }

    if (part == s->part1) {
	if (part == 0)
	    s->full_mono(frame);
	else
	    s->stereo2mono(frame);
    }
    else if (part < s->part2)
	s->full_mono(frame);
    else if (part == s->part2) {
	bool smallframe = frame->nb_samples < inlink->min_samples;
	bool lastpart = *s->parts == '\0';
	if (smallframe && lastpart)
	    s->full_mono(frame);
	else
	    s->mono2stereo(frame);
	if (lastpart)
	    s->part1 = s->part2 = INT_MAX;
	else if (!scan_part(ctx))
	    return AVERROR(EINVAL);
    }

    return ff_filter_frame(outlink, frame);
}

static const AVFilterPad inputs[] = {
    {
	.name		= "default",
	.type		= AVMEDIA_TYPE_AUDIO,
	.filter_frame	= filter_frame,
	.config_props	= config_input,
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

AVFilter ff_af_monoparts = {
    .name          = "monoparts",
    .description   = NULL_IF_CONFIG_SMALL("make audio parts mono"),
    .query_formats = query_formats,
    .priv_size     = sizeof(MonoPartsContext),
    .init          = init,
    .inputs        = inputs,
    .outputs       = outputs,
    .priv_class    = &monoparts_class,
};
