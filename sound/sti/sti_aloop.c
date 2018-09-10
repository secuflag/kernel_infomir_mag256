/*
 *   Copyright(c) 2014 STMicroelectronics
 *
 *   ST Loopback soundcard
 *
 *   It is fully inspired from aloop code:
 *   Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.
 *
 */

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <linux/of.h>
#if defined CONFIG_BPA2
#include <linux/io.h>
#include <linux/bpa2.h>
#endif

#define MAX_PCM_SUBSTREAMS	8

static int pcm_notify[SNDRV_CARDS];

module_param_array(pcm_notify, int, NULL, 0444);
MODULE_PARM_DESC(pcm_notify, "Break capture when PCM format/rate/channels changes.");

#define NO_PITCH 100000

#define STI_ALOOP_DEFAULT_NUM_DEVICES	1
#define STI_ALOOP_MAX_NUM_DEVICES	10

#define CABLE_VALID_PLAYBACK	BIT(SNDRV_PCM_STREAM_PLAYBACK)
#define CABLE_VALID_CAPTURE	BIT(SNDRV_PCM_STREAM_CAPTURE)
#define CABLE_VALID_BOTH	(CABLE_VALID_PLAYBACK|CABLE_VALID_CAPTURE)

#define RATE_LOCK_CTRL_MIN	8000
#define RATE_LOCK_CTRL_MAX	192000

enum {
	STI_ALOOP_MODE_RT,
	STI_ALOOP_MODE_NRT,
};

struct loopback_pcm;

struct loopback_cable {
	spinlock_t lock;
	struct loopback_pcm *streams[2];
	struct snd_pcm_hardware hw;
	/* flags */
	unsigned int valid;
	unsigned int running;
	unsigned int pause;
	struct timer_list timer_nrt;
	struct timer_list timer_capt_nrt;
	bool timer_nrt_running;
	bool timer_capt_nrt_running;
};

struct loopback_setup {
	unsigned int notify: 1;
	unsigned int rate_shift;
	unsigned int format;
	unsigned int rate;
	unsigned int channels;
	unsigned int rate_lock;
	struct snd_aes_iec958 iec958;
	struct snd_ctl_elem_id active_id;
	struct snd_ctl_elem_id format_id;
	struct snd_ctl_elem_id rate_id;
	struct snd_ctl_elem_id channels_id;
	struct snd_ctl_elem_id iec958_id;
	struct snd_ctl_elem_id rate_lock_id;
};

struct loopback {
	struct snd_card *card;
	struct mutex cable_lock;
	struct loopback_cable **cables[MAX_PCM_SUBSTREAMS];
	int num_devices;
	struct snd_pcm **pcm;
	struct loopback_setup *setup[MAX_PCM_SUBSTREAMS];
#if defined CONFIG_BPA2
	const char *mem_partition_name;
	struct bpa2_part *bpa2_part;
#endif
	unsigned int mode;
};

struct loopback_pcm {
	struct loopback *loopback;
	struct snd_pcm_substream *substream;
	struct loopback_cable *cable;
	unsigned int pcm_buffer_size;
	unsigned int buf_pos;	/* position in buffer */
	unsigned int silent_size;
	/* PCM parameters */
	unsigned int pcm_period_size;
	unsigned int pcm_bps;		/* bytes per second */
	unsigned int pcm_salign;	/* bytes per sample * channels */
	unsigned int pcm_rate_shift;	/* rate shift value */
	/* flags */
	unsigned int period_update_pending :1;
	/* timer stuff */
	unsigned int irq_pos;		/* fractional IRQ position */
	unsigned int period_size_frac;
	unsigned int last_drift;
	unsigned long last_jiffies;
	struct timer_list timer;
};

static inline int get_property_hdl(struct device *dev, struct device_node *np,
				   const char *prop, int idx)
{
	int sz = 0;
	const __be32 *phandle;

	phandle = of_get_property(np, prop, &sz);

	if (!phandle) {
		dev_err(dev, "%s: ERROR: DT-property '%s' missing or invalid!",
			__func__, prop);
		return -EINVAL;
	}

	if (idx >= sz) {
		dev_err(dev, "%s: ERROR: Array-index (%u) >= array-size (%u)!",
			__func__, idx, sz);
		return -EINVAL;
	}

	return be32_to_cpup(phandle + idx);
}

static inline unsigned int byte_pos(struct loopback_pcm *dpcm, unsigned int x)
{
	if (dpcm->pcm_rate_shift == NO_PITCH) {
		x /= HZ;
	} else {
		x = div_u64(NO_PITCH * (unsigned long long)x,
			    HZ * (unsigned long long)dpcm->pcm_rate_shift);
	}
	return x - (x % dpcm->pcm_salign);
}

static inline unsigned int frac_pos(struct loopback_pcm *dpcm, unsigned int x)
{
	if (dpcm->pcm_rate_shift == NO_PITCH) {	/* no pitch */
		return x * HZ;
	} else {
		x = div_u64(dpcm->pcm_rate_shift * (unsigned long long)x * HZ,
			    NO_PITCH);
	}
	return x;
}

static inline struct loopback_setup *get_setup(struct loopback_pcm *dpcm)
{
	int device = dpcm->substream->pstr->pcm->device;

	return &dpcm->loopback->setup[dpcm->substream->number][device];
}

static inline unsigned int get_notify(struct loopback_pcm *dpcm)
{
	return get_setup(dpcm)->notify;
}

static inline unsigned int get_rate_shift(struct loopback_pcm *dpcm)
{
	return get_setup(dpcm)->rate_shift;
}

static unsigned int get_cable_index(struct snd_pcm_substream *substream)
{
	return substream->pcm->device;
}

/* call in cable->lock */
static void loopback_timer_start(struct loopback_pcm *dpcm)
{
	unsigned long tick;
	unsigned int rate_shift = get_rate_shift(dpcm);

	if (rate_shift != dpcm->pcm_rate_shift) {
		dpcm->pcm_rate_shift = rate_shift;
		dpcm->period_size_frac = frac_pos(dpcm, dpcm->pcm_period_size);
	}
	if (dpcm->period_size_frac <= dpcm->irq_pos) {
		dpcm->irq_pos %= dpcm->period_size_frac;
		dpcm->period_update_pending = 1;
	}
	tick = dpcm->period_size_frac - dpcm->irq_pos;
	tick = (tick + dpcm->pcm_bps - 1) / dpcm->pcm_bps;
	dpcm->timer.expires = jiffies + tick;
	add_timer(&dpcm->timer);
}

/* call in cable->lock */
static void loopback_timer_nrt_start(struct loopback_cable *cable)
{
	/* stop capt timer */
	del_timer(&cable->timer_capt_nrt);
	cable->timer_capt_nrt.expires = 0;
	cable->timer_capt_nrt_running = false;

	/* expires as soon as possible (i.e. in next tick) */
	cable->timer_nrt.expires = jiffies - 1;
	dev_dbg(NULL, "%s: timer expires immediatly\n", __func__);
	add_timer(&cable->timer_nrt);
	cable->timer_nrt_running = true;
}

/* call in cable->lock */
static void loopback_timer_capt_nrt_start(struct loopback_cable *cable)
{
	struct loopback_pcm *dpcm = cable->streams[SNDRV_PCM_STREAM_CAPTURE];
	unsigned long tick, period_size_frac;

	/* expires in 1 playback period */
	period_size_frac = dpcm->pcm_period_size * HZ;
	tick = (period_size_frac + dpcm->pcm_bps - 1) / dpcm->pcm_bps;
	cable->timer_capt_nrt.expires = jiffies + tick;
	dev_dbg(NULL, "%s: timer expires in %lu jiffies\n", __func__, tick);
	add_timer(&cable->timer_capt_nrt);
	cable->timer_capt_nrt_running = true;
}

/* call in cable->lock */
static inline void loopback_timer_stop(struct loopback_pcm *dpcm)
{
	del_timer(&dpcm->timer);
	dpcm->timer.expires = 0;
}

/* call in cable->lock */
static inline void loopback_timer_nrt_stop(struct loopback_cable *cable)
{
	dev_dbg(NULL, "%s\n", __func__);

	del_timer(&cable->timer_nrt);
	cable->timer_nrt.expires = 0;
	cable->timer_nrt_running = false;
}

/* call in cable->lock */
static inline void loopback_timer_capt_nrt_stop(struct loopback_cable *cable)
{
	dev_dbg(NULL, "%s\n", __func__);

	del_timer(&cable->timer_capt_nrt);
	cable->timer_capt_nrt.expires = 0;
	cable->timer_capt_nrt_running = false;
}

static int loopback_check_format(struct loopback_cable *cable, int stream)
{
	struct snd_pcm_runtime *runtime, *cruntime;
	struct loopback_setup *setup;
	struct snd_card *card;
	int check;

	if (cable->valid != CABLE_VALID_BOTH) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			goto __notify;
		return 0;
	}
	runtime = cable->streams[SNDRV_PCM_STREAM_PLAYBACK]->
							substream->runtime;
	cruntime = cable->streams[SNDRV_PCM_STREAM_CAPTURE]->
							substream->runtime;
	check = runtime->format != cruntime->format ||
		runtime->rate != cruntime->rate ||
		runtime->channels != cruntime->channels;
	if (!check)
		return 0;
	if (stream == SNDRV_PCM_STREAM_CAPTURE) {
		return -EIO;
	} else {
		snd_pcm_stop(cable->streams[SNDRV_PCM_STREAM_CAPTURE]->
					substream, SNDRV_PCM_STATE_DRAINING);
__notify:
		runtime = cable->streams[SNDRV_PCM_STREAM_PLAYBACK]->
							substream->runtime;
		setup = get_setup(cable->streams[SNDRV_PCM_STREAM_PLAYBACK]);
		card = cable->streams[SNDRV_PCM_STREAM_PLAYBACK]->
			loopback->card;
		if (setup->format != runtime->format) {
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &setup->format_id);
			setup->format = runtime->format;
		}
		if (setup->rate != runtime->rate) {
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &setup->rate_id);
			setup->rate = runtime->rate;
		}
		if (setup->channels != runtime->channels) {
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &setup->channels_id);
			setup->channels = runtime->channels;
		}
	}
	return 0;
}

static void loopback_active_notify(struct loopback_pcm *dpcm)
{
	snd_ctl_notify(dpcm->loopback->card,
		       SNDRV_CTL_EVENT_MASK_VALUE,
		       &get_setup(dpcm)->active_id);
}

static int loopback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback *loopback = dpcm->loopback;
	struct loopback_cable *cable = dpcm->cable;
	int err, stream = 1 << substream->stream;
	int dev_num = substream->pcm->device;

	dev_dbg(substream->pcm->dev,
		"%s: dev_num = %d, stream =%d, cmd = %d\n",
		__func__, dev_num, substream->stream, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		err = loopback_check_format(cable, substream->stream);
		if (err < 0)
			return err;
		dpcm->last_jiffies = jiffies;
		dpcm->pcm_rate_shift = 0;
		dpcm->last_drift = 0;
		spin_lock(&cable->lock);
		cable->running |= stream;
		cable->pause &= ~stream;
		if (loopback->mode == STI_ALOOP_MODE_RT)
			loopback_timer_start(dpcm);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* start playback */
			if ((loopback->mode == STI_ALOOP_MODE_NRT) &&
			    !cable->timer_nrt_running)
				loopback_timer_nrt_start(cable);
			loopback_active_notify(dpcm);
		} else if (loopback->mode == STI_ALOOP_MODE_NRT) {
			/* start capture in NRT */
			int running = cable->running ^ cable->pause;

			if (!(running & CABLE_VALID_PLAYBACK) &&
			    !cable->timer_nrt_running &&
			    !cable->timer_capt_nrt_running)
				/* playback is not running */
				/* start timer that elapses in 1 period */
				/* to unblock the capt user */
				loopback_timer_capt_nrt_start(cable);
		}
		spin_unlock(&cable->lock);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock(&cable->lock);
		cable->running &= ~stream;
		cable->pause &= ~stream;
		if (loopback->mode == STI_ALOOP_MODE_RT) {
			loopback_timer_stop(dpcm);
		} else {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				loopback_timer_nrt_stop(cable);
			else
				loopback_timer_capt_nrt_stop(cable);
		}
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* stop playback */
			int running = cable->running ^ cable->pause;

			loopback_active_notify(dpcm);
			if ((loopback->mode == STI_ALOOP_MODE_NRT) &&
			    (running & CABLE_VALID_CAPTURE) &&
			    !cable->timer_nrt_running &&
			    !cable->timer_capt_nrt_running) {
				/* capt is still running */
				struct loopback_pcm *d;
				struct snd_pcm_runtime *r;

				d = cable->streams[SNDRV_PCM_STREAM_CAPTURE];
				r = d->substream->runtime;
				/* clear capture buffer */
				d->silent_size = d->pcm_buffer_size;
				snd_pcm_format_set_silence(r->format,
							   r->dma_area,
							   r->buffer_size *
								r->channels);
				/* start timer that elapses in 1 period */
				/* to unblock the capt user */
				loopback_timer_capt_nrt_start(cable);
			}
		}
		spin_unlock(&cable->lock);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		spin_lock(&cable->lock);
		cable->pause |= stream;
		if (loopback->mode == STI_ALOOP_MODE_RT) {
			loopback_timer_stop(dpcm);
		} else {
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				loopback_timer_nrt_stop(cable);
			else
				loopback_timer_capt_nrt_stop(cable);
		}
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* suspend playback */
			int running = cable->running ^ cable->pause;

			if ((loopback->mode == STI_ALOOP_MODE_NRT) &&
			    (running & CABLE_VALID_CAPTURE) &&
			    !cable->timer_nrt_running &&
			    !cable->timer_capt_nrt_running) {
				/* capt is still running */
				struct loopback_pcm *d;
				struct snd_pcm_runtime *r;

				d = cable->streams[SNDRV_PCM_STREAM_CAPTURE];
				r = d->substream->runtime;
				/* clear capture buffer */
				d->silent_size = d->pcm_buffer_size;
				snd_pcm_format_set_silence(r->format,
							   r->dma_area,
							   r->buffer_size *
								r->channels);
				/* start timer that elapses in 1 period */
				/* to unblock the capt user */
				loopback_timer_capt_nrt_start(cable);
			}
		}
		spin_unlock(&cable->lock);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		spin_lock(&cable->lock);
		dpcm->last_jiffies = jiffies;
		cable->pause &= ~stream;
		if (loopback->mode == STI_ALOOP_MODE_RT) {
			loopback_timer_start(dpcm);
		} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			/* resume capture */
			int running = cable->running ^ cable->pause;

			if (!(running & CABLE_VALID_PLAYBACK) &&
			    !cable->timer_nrt_running &&
			    !cable->timer_capt_nrt_running)
				/* playback is not running */
				/* start timer that elapses in 1 period */
				/* to unblock the capt user */
				loopback_timer_capt_nrt_start(cable);
		}
		if ((loopback->mode == STI_ALOOP_MODE_NRT) &&
		    (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		    !cable->timer_nrt_running)
				/* resume playback */
				loopback_timer_nrt_start(cable);
		spin_unlock(&cable->lock);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void params_change_substream(struct loopback_pcm *dpcm,
				    struct snd_pcm_runtime *runtime)
{
	struct snd_pcm_runtime *dst_runtime;

	if (dpcm == NULL || dpcm->substream == NULL)
		return;
	dst_runtime = dpcm->substream->runtime;
	if (dst_runtime == NULL)
		return;
	dst_runtime->hw = dpcm->cable->hw;
}

static void params_change(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback_cable *cable = dpcm->cable;

	cable->hw.formats = pcm_format_to_bits(runtime->format);
	cable->hw.rate_min = runtime->rate;
	cable->hw.rate_max = runtime->rate;
	cable->hw.channels_min = runtime->channels;
	cable->hw.channels_max = runtime->channels;
	params_change_substream(cable->streams[SNDRV_PCM_STREAM_PLAYBACK],
				runtime);
	params_change_substream(cable->streams[SNDRV_PCM_STREAM_CAPTURE],
				runtime);
}

static int loopback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback_cable *cable = dpcm->cable;
	int bps, salign;

	salign = (snd_pcm_format_width(runtime->format) *
						runtime->channels) / 8;
	bps = salign * runtime->rate;
	if (bps <= 0 || salign <= 0)
		return -EINVAL;

	dpcm->buf_pos = 0;
	dpcm->pcm_buffer_size = frames_to_bytes(runtime, runtime->buffer_size);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* clear capture buffer */
		dpcm->silent_size = dpcm->pcm_buffer_size;
		snd_pcm_format_set_silence(runtime->format, runtime->dma_area,
					   runtime->buffer_size *
						runtime->channels);
	}

	dpcm->irq_pos = 0;
	dpcm->period_update_pending = 0;
	dpcm->pcm_bps = bps;
	dpcm->pcm_salign = salign;
	dpcm->pcm_period_size = frames_to_bytes(runtime, runtime->period_size);

	mutex_lock(&dpcm->loopback->cable_lock);
	if (!(cable->valid & ~(1 << substream->stream)) ||
	    (get_setup(dpcm)->notify &&
	     substream->stream == SNDRV_PCM_STREAM_PLAYBACK))
		params_change(substream);

	cable->valid |= 1 << substream->stream;
	mutex_unlock(&dpcm->loopback->cable_lock);

	return 0;
}

static void clear_capture_buf(struct loopback_pcm *dpcm, unsigned int bytes)
{
	struct snd_pcm_runtime *runtime = dpcm->substream->runtime;
	char *dst = runtime->dma_area;
	unsigned int dst_off = dpcm->buf_pos;

	if (dpcm->silent_size >= dpcm->pcm_buffer_size)
		return;
	if (dpcm->silent_size + bytes > dpcm->pcm_buffer_size)
		bytes = dpcm->pcm_buffer_size - dpcm->silent_size;

	for (;;) {
		unsigned int size = bytes;

		if (dst_off + size > dpcm->pcm_buffer_size)
			size = dpcm->pcm_buffer_size - dst_off;

		snd_pcm_format_set_silence(runtime->format, dst + dst_off,
					   bytes_to_frames(runtime, size) *
						runtime->channels);
		dpcm->silent_size += size;
		bytes -= size;
		if (!bytes)
			break;

		dst_off = 0;
	}
}

static void copy_play_buf(struct loopback_pcm *play,
			  struct loopback_pcm *capt,
			  unsigned int bytes)
{
	struct snd_pcm_runtime *runtime = play->substream->runtime;
	char *src = runtime->dma_area;
	char *dst = capt->substream->runtime->dma_area;
	unsigned int src_off = play->buf_pos;
	unsigned int dst_off = capt->buf_pos;
	unsigned int clear_bytes = 0;

	/* check if playback is draining, trim the capture copy size
	 * when our pointer is at the end of playback ring buffer */
	if (runtime->status->state == SNDRV_PCM_STATE_DRAINING &&
	    snd_pcm_playback_hw_avail(runtime) < runtime->buffer_size) {
		snd_pcm_uframes_t appl_ptr, appl_ptr1, diff;

		appl_ptr = runtime->control->appl_ptr;
		appl_ptr1 = runtime->control->appl_ptr;
		appl_ptr1 -= appl_ptr1 % runtime->buffer_size;
		appl_ptr1 += play->buf_pos / play->pcm_salign;

		if (appl_ptr < appl_ptr1)
			appl_ptr1 -= runtime->buffer_size;

		diff = (appl_ptr - appl_ptr1) * play->pcm_salign;

		if (diff < bytes) {
			clear_bytes = bytes - diff;
			bytes = diff;
		}
	}

	for (;;) {
		unsigned int size = bytes;

		if (src_off + size > play->pcm_buffer_size)
			size = play->pcm_buffer_size - src_off;

		if (dst_off + size > capt->pcm_buffer_size)
			size = capt->pcm_buffer_size - dst_off;

		memcpy(dst + dst_off, src + src_off, size);
		capt->silent_size = 0;
		bytes -= size;

		if (!bytes)
			break;

		src_off = (src_off + size) % play->pcm_buffer_size;
		dst_off = (dst_off + size) % capt->pcm_buffer_size;
	}

	if (clear_bytes > 0) {
		clear_capture_buf(capt, clear_bytes);
		capt->silent_size = 0;
	}
}

static inline unsigned int bytepos_delta(struct loopback_pcm *dpcm,
					 unsigned int jiffies_delta)
{
	unsigned long last_pos;
	unsigned int delta;

	last_pos = byte_pos(dpcm, dpcm->irq_pos);
	dpcm->irq_pos += jiffies_delta * dpcm->pcm_bps;
	delta = byte_pos(dpcm, dpcm->irq_pos) - last_pos;

	if (delta >= dpcm->last_drift)
		delta -= dpcm->last_drift;

	dpcm->last_drift = 0;

	if (dpcm->irq_pos >= dpcm->period_size_frac) {
		dpcm->irq_pos %= dpcm->period_size_frac;
		dpcm->period_update_pending = 1;
	}

	return delta;
}

static inline void bytepos_finish(struct loopback_pcm *dpcm,
				  unsigned int delta)
{
	dpcm->buf_pos += delta;
	dpcm->buf_pos %= dpcm->pcm_buffer_size;
}

/* call in cable->lock */
static unsigned int loopback_pos_update(struct loopback_cable *cable)
{
	struct loopback_pcm *dpcm_play =
			cable->streams[SNDRV_PCM_STREAM_PLAYBACK];
	struct loopback_pcm *dpcm_capt =
			cable->streams[SNDRV_PCM_STREAM_CAPTURE];
	unsigned long delta_play = 0, delta_capt = 0;
	unsigned int running, count1, count2;

	running = cable->running ^ cable->pause;
	if (running & CABLE_VALID_PLAYBACK) {
		delta_play = jiffies - dpcm_play->last_jiffies;
		dpcm_play->last_jiffies += delta_play;
	}

	if (running & CABLE_VALID_CAPTURE) {
		delta_capt = jiffies - dpcm_capt->last_jiffies;
		dpcm_capt->last_jiffies += delta_capt;
	}

	if (delta_play == 0 && delta_capt == 0)
		goto unlock;

	if (delta_play > delta_capt) {
		count1 = bytepos_delta(dpcm_play, delta_play - delta_capt);
		bytepos_finish(dpcm_play, count1);
		delta_play = delta_capt;
	} else if (delta_play < delta_capt) {
		count1 = bytepos_delta(dpcm_capt, delta_capt - delta_play);
		clear_capture_buf(dpcm_capt, count1);
		bytepos_finish(dpcm_capt, count1);
		delta_capt = delta_play;
	}

	if (delta_play == 0 && delta_capt == 0)
		goto unlock;

	/* note delta_capt == delta_play at this moment */
	count1 = bytepos_delta(dpcm_play, delta_play);
	count2 = bytepos_delta(dpcm_capt, delta_capt);
	if (count1 < count2) {
		dpcm_capt->last_drift = count2 - count1;
		count1 = count2;
	} else if (count1 > count2) {
		dpcm_play->last_drift = count1 - count2;
	}
	copy_play_buf(dpcm_play, dpcm_capt, count1);
	bytepos_finish(dpcm_play, count1);
	bytepos_finish(dpcm_capt, count1);

unlock:
	return running;
}

static void loopback_timer_function(unsigned long data)
{
	struct loopback_pcm *dpcm = (struct loopback_pcm *)data;
	unsigned long flags;

	spin_lock_irqsave(&dpcm->cable->lock, flags);
	if (loopback_pos_update(dpcm->cable) & (1 << dpcm->substream->stream)) {
		loopback_timer_start(dpcm);
		if (dpcm->period_update_pending) {
			dpcm->period_update_pending = 0;
			spin_unlock_irqrestore(&dpcm->cable->lock, flags);
			/* need to unlock before calling below */
			snd_pcm_period_elapsed(dpcm->substream);
			return;
		}
	}
	spin_unlock_irqrestore(&dpcm->cable->lock, flags);
}

static void loopback_timer_nrt_function(unsigned long data)
{
	unsigned long flags, count, c, limit;
	snd_pcm_uframes_t appl_ptr, hw_ptr;
	struct loopback_cable *cable = (struct loopback_cable *)data;
	int running = cable->running ^ cable->pause;
	struct loopback_pcm *dpcm_play =
		cable->streams[SNDRV_PCM_STREAM_PLAYBACK];
	struct loopback_pcm *dpcm_capt =
		cable->streams[SNDRV_PCM_STREAM_CAPTURE];
	struct device *play_dev = dpcm_play->substream->pcm->dev;
	struct snd_pcm_runtime *dpr = dpcm_play->substream->runtime;

	dev_dbg(play_dev, "%s\n", __func__);

	/* lock cable resource */
	spin_lock_irqsave(&cable->lock, flags);

	/* player should run */
	if (!(running & CABLE_VALID_PLAYBACK)) {
		dev_dbg(play_dev, "%s: player not running !!\n", __func__);
		cable->timer_nrt_running = false;
		/* unlock cable resource */
		spin_unlock_irqrestore(&cable->lock, flags);
		return;
	}

	/* lock substream resources */
	snd_pcm_stream_lock(dpcm_play->substream);

	/* get current ALSA buffer player position in bytes */
	appl_ptr = frames_to_bytes(dpr, dpr->control->appl_ptr);

	/* get ALSA buffer player base pointer (last period_elapsed ) */
	hw_ptr = frames_to_bytes(dpr, dpr->status->hw_ptr);

	/* unlock substream resources */
	snd_pcm_stream_unlock(dpcm_play->substream);

	/* calculate offset since last copy */
	if (appl_ptr >= hw_ptr)
		count = appl_ptr - hw_ptr;
	else
		count = appl_ptr +
			dpcm_play->pcm_buffer_size -
			hw_ptr;

	dev_dbg(play_dev, "%s: initial count = %lu\n", __func__, count);
	if (!count) {
		cable->timer_nrt_running = false;
		/* unlock cable resource */
		spin_unlock_irqrestore(&cable->lock, flags);
		return;
	}

	limit = count;
	for (c = 1; c <= limit; c++) {
		if (((dpcm_play->buf_pos + c) / dpcm_play->pcm_period_size) !=
		    (dpcm_play->buf_pos / dpcm_play->pcm_period_size)) {
			if (running & CABLE_VALID_CAPTURE) {
				struct snd_pcm_runtime *dcr =
					dpcm_capt->substream->runtime;
				snd_pcm_uframes_t capt_appl_ptr, capt_hw_ptr;
				/* get current ALSA buffer play pos in bytes */
				capt_appl_ptr =
					frames_to_bytes(dcr,
							dcr->control->appl_ptr);
				capt_hw_ptr =
					frames_to_bytes(dcr,
							dcr->status->hw_ptr);
				/* detect overrun early */
				if ((capt_hw_ptr + c - capt_appl_ptr) >=
				    dpcm_capt->pcm_buffer_size) {
					dev_dbg(play_dev,
						"%s: no place in capt buf\n",
						__func__);
					loopback_timer_nrt_start(cable);
					/* unlock cable resource */
					spin_unlock_irqrestore(&cable->lock,
							       flags);
					return;
				}
			}

			/* capture & player buffer period reached */
			dev_dbg(play_dev,
				"%s: capture & player period elapsed\n",
				__func__);
			dev_dbg(play_dev,
				"%s: play_byte_pos = %u\n",
				__func__, dpcm_play->buf_pos);

			if (running & CABLE_VALID_CAPTURE) {
				dev_dbg(play_dev,
					"%s: capt_byte_pos = %u\n",
					__func__, dpcm_capt->buf_pos);

				copy_play_buf(dpcm_play, dpcm_capt, c);
				bytepos_finish(dpcm_capt, c);
				spin_unlock_irqrestore(&cable->lock, flags);
				snd_pcm_period_elapsed(dpcm_capt->substream);
				spin_lock_irqsave(&cable->lock, flags);
			}

			bytepos_finish(dpcm_play, c);
			spin_unlock_irqrestore(&cable->lock, flags);
			snd_pcm_period_elapsed(dpcm_play->substream);
			spin_lock_irqsave(&cable->lock, flags);

			limit -= c;
			c = 0; /* will be set to 1 at the end of loop */
		}
	}

	cable->timer_nrt_running = false;

	/* unlock cable resource */
	spin_unlock_irqrestore(&cable->lock, flags);
}

static void loopback_timer_capt_nrt_function(unsigned long data)
{
	unsigned long flags;
	struct loopback_cable *cable = (struct loopback_cable *)data;
	int running = cable->running ^ cable->pause;
	struct loopback_pcm *dpcm_capt =
		cable->streams[SNDRV_PCM_STREAM_CAPTURE];

	dev_dbg(NULL, "%s\n", __func__);

	/* lock cable resource */
	spin_lock_irqsave(&cable->lock, flags);

	if (running & CABLE_VALID_CAPTURE) {
		bytepos_finish(dpcm_capt, dpcm_capt->pcm_period_size);
		spin_unlock_irqrestore(&cable->lock, flags);
		snd_pcm_period_elapsed(dpcm_capt->substream);
		spin_lock_irqsave(&cable->lock, flags);
	}

	cable->timer_capt_nrt_running = false;

	/* unlock cable resource */
	spin_unlock_irqrestore(&cable->lock, flags);
}


static snd_pcm_uframes_t loopback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct device *dev = substream->pcm->dev;
	int dev_num = get_cable_index(substream);
	snd_pcm_uframes_t pos;

	dev_dbg(dev, "%s: dev_num = %d, stream = %d\n", __func__,
		dev_num, substream->stream);

	spin_lock(&dpcm->cable->lock);

	if (dpcm->loopback->mode == STI_ALOOP_MODE_RT)
		loopback_pos_update(dpcm->cable);
	pos = dpcm->buf_pos;
	dev_dbg(dev, "%s: buffer pos = %lu\n", __func__, pos);

	spin_unlock(&dpcm->cable->lock);

	return bytes_to_frames(runtime, pos);
}

static struct snd_pcm_hardware loopback_pcm_hardware = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_PAUSE |
		 SNDRV_PCM_INFO_RESUME),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE |
		    SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE |
		    SNDRV_PCM_FMTBIT_FLOAT_LE | SNDRV_PCM_FMTBIT_FLOAT_BE),
	.rates = SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.channels_min =	1,
	.channels_max =	32,
	.buffer_bytes_max = 2 * 1024 * 1024,
	.period_bytes_min = 64,
	/* note check overflow in frac_pos() using pcm_rate_shift before
	   changing period_bytes_max value */
	.period_bytes_max = 1024 * 1024,
	.periods_min =	 1,
	.periods_max = 1024,
	.fifo_size = 0,
};

static void loopback_runtime_free(struct snd_pcm_runtime *runtime)
{
	struct loopback_pcm *dpcm = runtime->private_data;

	kfree(dpcm);
}

static int loopback_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct device *dev = substream->pcm->dev;
	int dev_num = get_cable_index(substream);
	int stream = substream->stream;
#ifdef CONFIG_BPA2
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback *loopback = dpcm->loopback;
	int pages;
	int size = params_buffer_bytes(params);
#endif

	dev_dbg(dev, "%s: dev num = %d, stream = %d\n",
		__func__, dev_num, stream);

#ifdef CONFIG_BPA2
	/* get BPA2 partition */
	loopback->bpa2_part = bpa2_find_part(loopback->mem_partition_name);
	if (loopback->bpa2_part) {
		dev_info(dev, "%s: Using BPA2 '%s'\n", __func__,
			 loopback->mem_partition_name);
	} else {
		loopback->bpa2_part = bpa2_find_part("bigphysarea");
		if (loopback->bpa2_part)
			dev_info(dev, "%s: Using 'bigphysarea'\n", __func__);
		else
			dev_info(dev, "BPA2 partition %s not found\n", "audio");
	}

	if (loopback->bpa2_part) {
		/* use BPA2 */
		pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

		/* free existing buffer before allocating new one */
		if (runtime->dma_area) {
			iounmap(runtime->dma_area);
			bpa2_free_pages(loopback->bpa2_part,
					runtime->dma_addr);
			runtime->dma_area = NULL;
			runtime->dma_addr = 0;
			runtime->dma_bytes = 0;
			dev_dbg(dev,
				"%s: existing buf freed before alloc new one\n",
				__func__);
		}

		runtime->dma_addr =
			bpa2_alloc_pages(loopback->bpa2_part, pages, 0,
					 GFP_KERNEL);

		if (!runtime->dma_addr) {
			dev_err(dev, "Can't get %d pages from BPA2!\n", pages);
			return -ENOMEM;
		}

			if (pfn_valid(__phys_to_pfn(runtime->dma_addr))) {
				dev_err(dev,
					"page %#x inside the kernel space\n",
				substream->runtime->dma_addr);
				bpa2_free_pages(loopback->bpa2_part,
						runtime->dma_addr);
				runtime->dma_addr = 0;
				return -ENOMEM;
		}

		runtime->dma_bytes = size;
		runtime->dma_area = ioremap_nocache(runtime->dma_addr, size);

		return 0;
	} else
#endif /* CONFIG_BPA2 */
		/* use pcm lib */
		return snd_pcm_lib_alloc_vmalloc_buffer(substream,
						params_buffer_bytes(params));
}

static int loopback_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback_pcm *dpcm = runtime->private_data;
	struct loopback *loopback = dpcm->loopback;
	struct loopback_cable *cable = dpcm->cable;
	struct device *dev = substream->pcm->dev;
	int dev_num = get_cable_index(substream);
	int stream = substream->stream;

	dev_dbg(dev, "%s: dev num = %d, stream = %d\n",
		__func__, dev_num, stream);

	mutex_lock(&dpcm->loopback->cable_lock);
	cable->valid &= ~(1 << substream->stream);
	mutex_unlock(&dpcm->loopback->cable_lock);
#ifdef CONFIG_BPA2
	if (loopback->bpa2_part) {
		if (runtime->dma_area) {
			iounmap(runtime->dma_area);
		bpa2_free_pages(loopback->bpa2_part, runtime->dma_addr);
			runtime->dma_area = NULL;
			runtime->dma_addr = 0;
			runtime->dma_bytes = 0;
			dev_dbg(dev, "%s: BPA2 pages freed\n", __func__);
		}
		return 0;
	} else
#endif /* CONFIG_BPA2 */
		return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int rule_format(struct snd_pcm_hw_params *params,
		       struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_hardware *hw = rule->private;
	struct snd_mask *maskp = hw_param_mask(params, rule->var);

	maskp->bits[0] &= (u_int32_t)hw->formats;
	maskp->bits[1] &= (u_int32_t)(hw->formats >> 32);
	memset(maskp->bits + 2, 0, (SNDRV_MASK_MAX-64) / 8); /* clear rest */
	if (!maskp->bits[0] && !maskp->bits[1])
		return -EINVAL;

	return 0;
}

static int rule_rate(struct snd_pcm_hw_params *params,
		     struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_hardware *hw = rule->private;
	struct snd_interval t;

	t.min = hw->rate_min;
	t.max = hw->rate_max;
	t.openmin = 0;
	t.openmax = 0;
	t.integer = 0;

	return snd_interval_refine(hw_param_interval(params, rule->var), &t);
}

static int rule_channels(struct snd_pcm_hw_params *params,
			 struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_hardware *hw = rule->private;
	struct snd_interval t;

	t.min = hw->channels_min;
	t.max = hw->channels_max;
	t.openmin = 0;
	t.openmax = 0;
	t.integer = 0;

	return snd_interval_refine(hw_param_interval(params, rule->var), &t);
}

static int loopback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct loopback *loopback = substream->private_data;
	int stream = substream->stream;
	struct loopback_pcm *dpcm;
	struct loopback_cable *cable;
	int err = 0;
	int dev_num = get_cable_index(substream);
	struct loopback_setup *setup =
		&(loopback->setup[substream->number][dev_num]);

	dev_dbg(substream->pcm->dev, "%s: dev_num = %d, stream = %d\n",
		__func__, dev_num, stream);

	mutex_lock(&loopback->cable_lock);
	dpcm = kzalloc(sizeof(*dpcm), GFP_KERNEL);
	if (!dpcm) {
		err = -ENOMEM;
		goto unlock;
	}
	dpcm->loopback = loopback;
	dpcm->substream = substream;
	setup_timer(&dpcm->timer, loopback_timer_function,
		    (unsigned long)dpcm);

	cable = loopback->cables[substream->number][dev_num];
	if (!cable) {
		cable = kzalloc(sizeof(*cable), GFP_KERNEL);
		if (!cable) {
			kfree(dpcm);
			err = -ENOMEM;
			goto unlock;
		}
		spin_lock_init(&cable->lock);
		cable->hw = loopback_pcm_hardware;
		loopback->cables[substream->number][dev_num] = cable;
		cable->timer_nrt_running = false;
		cable->timer_capt_nrt_running = false;
		setup_timer(&cable->timer_nrt,
			    loopback_timer_nrt_function,
			    (unsigned long)cable);
		setup_timer(&cable->timer_capt_nrt,
			    loopback_timer_capt_nrt_function,
			    (unsigned long)cable);
		*(loopback->cables[substream->number] + dev_num) = cable;
	}
	dpcm->cable = cable;
	cable->streams[substream->stream] = dpcm;

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	/* use dynamic rules based on actual runtime->hw values */
	/* note that the default rules created in the PCM midlevel code */
	/* are cached -> they do not reflect the actual state */
	err = snd_pcm_hw_rule_add(runtime, 0,
				  SNDRV_PCM_HW_PARAM_FORMAT,
				  rule_format, &runtime->hw,
				  SNDRV_PCM_HW_PARAM_FORMAT, -1);
	if (err < 0)
		goto unlock;
	err = snd_pcm_hw_rule_add(runtime, 0,
				  SNDRV_PCM_HW_PARAM_RATE,
				  rule_rate, &runtime->hw,
				  SNDRV_PCM_HW_PARAM_RATE, -1);
	if (err < 0)
		goto unlock;
	err = snd_pcm_hw_rule_add(runtime, 0,
				  SNDRV_PCM_HW_PARAM_CHANNELS,
				  rule_channels, &runtime->hw,
				  SNDRV_PCM_HW_PARAM_CHANNELS, -1);
	if (err < 0)
		goto unlock;

	runtime->private_data = dpcm;
	runtime->private_free = loopback_runtime_free;
	if (get_notify(dpcm))
		runtime->hw = loopback_pcm_hardware;
	else
		runtime->hw = cable->hw;
	/* apply rate_lock */
	if (setup->rate_lock) {
		runtime->hw.rate_min = setup->rate_lock;
		runtime->hw.rate_max = setup->rate_lock;
	}

 unlock:
	mutex_unlock(&loopback->cable_lock);

	return err;
}

static int loopback_close(struct snd_pcm_substream *substream)
{
	int stream = substream->stream;
	struct loopback *loopback = substream->private_data;
	struct loopback_pcm *dpcm = substream->runtime->private_data;
	struct loopback_cable *cable;
	int dev_num = get_cable_index(substream);

	dev_dbg(substream->pcm->dev, "%s: dev_num = %d, stream = %d\n",
		__func__, dev_num, stream);

	mutex_lock(&loopback->cable_lock);
	cable = loopback->cables[substream->number][dev_num];

	if (loopback->mode == STI_ALOOP_MODE_RT)
		loopback_timer_stop(dpcm);

	if (cable->streams[!substream->stream]) {
		/* other stream is still alive */
		cable->streams[substream->stream] = NULL;
	} else {
		/* free the cable */
		loopback->cables[substream->number][dev_num] = NULL;
		kfree(cable);
	}
	mutex_unlock(&loopback->cable_lock);

	return 0;
}

static int loopback_write_ack(struct snd_pcm_substream *substream)
{
	int dev_num = get_cable_index(substream);
	struct loopback *loopback = substream->private_data;
	struct loopback_cable *cable =
		*(loopback->cables[substream->number] + dev_num);
	struct device *dev = substream->pcm->dev;

	dev_dbg(dev, "%s: dev_num = %d\n", __func__, dev_num);

	spin_lock(&cable->lock);
	if ((loopback->mode == STI_ALOOP_MODE_NRT) && !cable->timer_nrt_running)
		loopback_timer_nrt_start(cable);
	spin_unlock(&cable->lock);

	return 0;
}

static int loopback_read_ack(struct snd_pcm_substream *substream)
{
	int dev_num = get_cable_index(substream);
	struct loopback *loopback = substream->private_data;
	struct loopback_cable *cable =
		*(loopback->cables[substream->number] + dev_num);
	struct device *dev = substream->pcm->dev;
	int running = cable->running ^ cable->pause;

	dev_dbg(dev, "%s: dev_num = %d\n", __func__, dev_num);

	spin_lock(&cable->lock);
	if ((loopback->mode == STI_ALOOP_MODE_NRT) &&
	    !(running & CABLE_VALID_PLAYBACK) &&
	    !cable->timer_nrt_running &&
	    !cable->timer_capt_nrt_running)
			/* playback is not running */
			/* start timer that elapses in 1 period */
			/* to unblock the capt user */
			loopback_timer_capt_nrt_start(cable);
	spin_unlock(&cable->lock);

	return 0;
}

static struct snd_pcm_ops loopback_playback_ops = {
	.open =	loopback_open,
	.close = loopback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = loopback_hw_params,
	.hw_free = loopback_hw_free,
	.prepare = loopback_prepare,
	.trigger = loopback_trigger,
	.pointer = loopback_pointer,
	.page =	snd_pcm_lib_get_vmalloc_page,
	.mmap =	snd_pcm_lib_mmap_vmalloc,
	.ack =		loopback_write_ack,
};

static struct snd_pcm_ops loopback_capture_ops = {
	.open =	loopback_open,
	.close = loopback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = loopback_hw_params,
	.hw_free = loopback_hw_free,
	.prepare = loopback_prepare,
	.trigger = loopback_trigger,
	.pointer = loopback_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
	.mmap = snd_pcm_lib_mmap_vmalloc,
	.ack =		loopback_read_ack,
};

static int loopback_pcm_new(struct loopback *loopback,
			    int device, int substreams)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(loopback->card, "Loopback PCM", device,
			  substreams, substreams, &pcm);
	if (err < 0)
		return err;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &loopback_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &loopback_capture_ops);

	pcm->private_data = loopback;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Loopback PCM");

	loopback->pcm[device] = pcm;

	return 0;
}

static int loopback_rate_shift_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 80000;
	uinfo->value.integer.max = 120000;
	uinfo->value.integer.step = 1;

	return 0;
}

static int loopback_rate_shift_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).rate_shift;

	return 0;
}

static int loopback_rate_shift_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	unsigned int val;
	int change = 0;

	val = ucontrol->value.integer.value[0];
	if (val < 80000)
		val = 80000;
	if (val > 120000)
		val = 120000;
	mutex_lock(&loopback->cable_lock);
	if (val != (loopback->setup[kcontrol->id.subdevice]
				   [kcontrol->id.device]).rate_shift) {
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).rate_shift = val;
		change = 1;
	}
	mutex_unlock(&loopback->cable_lock);

	return change;
}

static int loopback_notify_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).notify;

	return 0;
}

static int loopback_notify_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	unsigned int val;
	int change = 0;

	val = ucontrol->value.integer.value[0] ? 1 : 0;
	if (val != (loopback->setup[kcontrol->id.subdevice]
				   [kcontrol->id.device]).notify) {
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).notify = val;
		change = 1;
	}

	return change;
}

static int loopback_active_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	struct loopback_cable *cable =
		loopback->cables[kcontrol->id.subdevice]
				[kcontrol->id.device ^ 1];
	unsigned int val = 0;

	if (cable != NULL)
		val = (cable->running & CABLE_VALID_PLAYBACK) ? 1 : 0;
	ucontrol->value.integer.value[0] = val;

	return 0;
}

static int loopback_format_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SNDRV_PCM_FORMAT_LAST;
	uinfo->value.integer.step = 1;

	return 0;
}

static int loopback_format_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).format;

	return 0;
}

static int loopback_rate_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 192000;
	uinfo->value.integer.step = 1;

	return 0;
}

static int loopback_rate_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).rate;

	return 0;
}

static int loopback_channels_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 1024;
	uinfo->value.integer.step = 1;

	return 0;
}

static int loopback_channels_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).channels;

	return 0;
}

static int loopback_ctl_iec958_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int loopback_ctl_iec958_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	struct snd_aes_iec958 *iec958 =
		&((loopback->setup[kcontrol->id.subdevice] +
			kcontrol->id.device)->iec958);

	if (!iec958)
		return -ENOMEM;

	mutex_lock(&loopback->cable_lock);
	ucontrol->value.iec958.status[0] = iec958->status[0];
	ucontrol->value.iec958.status[1] = iec958->status[1];
	ucontrol->value.iec958.status[2] = iec958->status[2];
	ucontrol->value.iec958.status[3] = iec958->status[3];
	mutex_unlock(&loopback->cable_lock);

	return 0;
}

static int loopback_ctl_iec958_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	struct snd_aes_iec958 *iec958 =
		&((loopback->setup[kcontrol->id.subdevice] +
			kcontrol->id.device)->iec958);

	if (!iec958)
		return -ENOMEM;

	mutex_lock(&loopback->cable_lock);
	iec958->status[0] = ucontrol->value.iec958.status[0];
	iec958->status[1] = ucontrol->value.iec958.status[1];
	iec958->status[2] = ucontrol->value.iec958.status[2];
	iec958->status[3] = ucontrol->value.iec958.status[3];
	mutex_unlock(&loopback->cable_lock);

	return 0;
}

static int loopback_rate_lock_info(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = RATE_LOCK_CTRL_MIN;
	uinfo->value.integer.max = RATE_LOCK_CTRL_MAX;
	uinfo->value.integer.step = 1;
	return 0;
}

static int loopback_rate_lock_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		(loopback->setup[kcontrol->id.subdevice]
				[kcontrol->id.device]).rate_lock;
	return 0;
}

static int loopback_rate_lock_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	unsigned int val;

	val = ucontrol->value.integer.value[0];
	if (val && (val < RATE_LOCK_CTRL_MIN))
		val = RATE_LOCK_CTRL_MIN;
	if (val && (val > RATE_LOCK_CTRL_MAX))
		val = RATE_LOCK_CTRL_MAX;

	mutex_lock(&loopback->cable_lock);

	(loopback->setup[kcontrol->id.subdevice]
			[kcontrol->id.device]).rate_lock = val;

	mutex_unlock(&loopback->cable_lock);

	return 0;
}

static struct snd_kcontrol_new loopback_controls[]  = {
{
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Rate Shift 100000",
	.info =         loopback_rate_shift_info,
	.get =          loopback_rate_shift_get,
	.put =          loopback_rate_shift_put,
},
{
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Notify",
	.info =         snd_ctl_boolean_mono_info,
	.get =          loopback_notify_get,
	.put =          loopback_notify_put,
},
#define ACTIVE_IDX 2
{
	.access =	SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Slave Active",
	.info =         snd_ctl_boolean_mono_info,
	.get =          loopback_active_get,
},
#define FORMAT_IDX 3
{
	.access =	SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Slave Format",
	.info =         loopback_format_info,
	.get =          loopback_format_get
},
#define RATE_IDX 4
{
	.access =	SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Slave Rate",
	.info =         loopback_rate_info,
	.get =          loopback_rate_get
},
#define CHANNELS_IDX 5
{
	.access =	SNDRV_CTL_ELEM_ACCESS_READ,
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Slave Channels",
	.info =         loopback_channels_info,
	.get =          loopback_channels_get
},
#define IEC958_IDX 6
{
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "IEC958",
	.info =         loopback_ctl_iec958_info,
	.get =          loopback_ctl_iec958_get,
	.put =		loopback_ctl_iec958_put,
},
#define RATE_LOCK_IDX 7
{
	.iface =        SNDRV_CTL_ELEM_IFACE_PCM,
	.name =         "PCM Rate Lock",
	.info =         loopback_rate_lock_info,
	.get =          loopback_rate_lock_get,
	.put =          loopback_rate_lock_put,
}
};

static int loopback_ctl_nrt_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = STI_ALOOP_MODE_RT;
	uinfo->value.integer.max = STI_ALOOP_MODE_NRT;

	return 0;
}

static int loopback_ctl_nrt_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	struct device *dev = loopback->pcm[0]->dev;

	dev_dbg(dev, "%s (kcontrol=%p, ucontrol=%p)", __func__,
		kcontrol, ucontrol);

	mutex_lock(&loopback->cable_lock);
	ucontrol->value.integer.value[0] = loopback->mode;
	mutex_unlock(&loopback->cable_lock);

	return 0;
}

static int loopback_ctl_nrt_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct loopback *loopback = snd_kcontrol_chip(kcontrol);
	struct device *dev = loopback->pcm[0]->dev;
	int ret = 0;

	dev_dbg(dev, "%s (kcontrol=%p, ucontrol=%p)", __func__,
		kcontrol, ucontrol);

	if ((ucontrol->value.integer.value[0] < STI_ALOOP_MODE_RT) ||
	    (ucontrol->value.integer.value[0] > STI_ALOOP_MODE_NRT))
		return -EINVAL;

	mutex_lock(&loopback->cable_lock);
	loopback->mode = ucontrol->value.integer.value[0];
	mutex_unlock(&loopback->cable_lock);

	return ret;
}

static struct snd_kcontrol_new loopback_nrt_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_CARD,
		.name = "Non Real Time mode",
		.info = loopback_ctl_nrt_info,
		.get = loopback_ctl_nrt_get,
		.put = loopback_ctl_nrt_put,
		.index = 0,
		.device = 0,
		.subdevice = 0,
	},
};

static int loopback_mixer_new(struct loopback *loopback, int notify)
{
	struct snd_card *card = loopback->card;
	struct snd_pcm *pcm;
	struct snd_kcontrol *kctl;
	struct loopback_setup *setup;
	int err, dev, substr, substr_count, idx;

	strcpy(card->mixername, "Loopback Mixer");

	err = snd_ctl_add(card, snd_ctl_new1(loopback_nrt_controls, loopback));
	if (err < 0) {
		dev_err(NULL, "%s: Failed to add %s (%d)\n",
			__func__, loopback_nrt_controls->name, err);
		return err;
	}

	for (dev = 0; dev < loopback->num_devices; dev++) {
		pcm = loopback->pcm[dev];
		substr_count =
		    pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream_count;
		for (substr = 0; substr < substr_count; substr++) {
			setup = loopback->setup[substr] + dev;
			setup->notify = notify;
			setup->rate_shift = NO_PITCH;
			setup->format = SNDRV_PCM_FORMAT_S16_LE;
			setup->rate = 48000;
			setup->channels = 2;
			memset(&(setup->iec958), 0,
			       sizeof(struct snd_aes_iec958));
			setup->rate_lock = 0;
			for (idx = 0; idx < ARRAY_SIZE(loopback_controls);
			     idx++) {
				kctl = snd_ctl_new1(&loopback_controls[idx],
						    loopback);
				if (!kctl)
					return -ENOMEM;

				kctl->id.device = dev;
				kctl->id.index = dev;
				kctl->id.subdevice = substr;
				switch (idx) {
				case ACTIVE_IDX:
					setup->active_id = kctl->id;
					break;
				case FORMAT_IDX:
					setup->format_id = kctl->id;
					break;
				case RATE_IDX:
					setup->rate_id = kctl->id;
					break;
				case CHANNELS_IDX:
					setup->channels_id = kctl->id;
					break;
				case IEC958_IDX:
					setup->iec958_id = kctl->id;
					break;
				case RATE_LOCK_IDX:
					setup->rate_lock_id = kctl->id;
					break;
				default:
					break;
				}
				err = snd_ctl_add(card, kctl);
				if (err < 0) {
					dev_err(pcm->dev,
						"%s: Failed to add %s (%d)\n",
						__func__,
						loopback_controls[idx].name,
						err);
					return err;
				}
			}
		}
	}
	return 0;
}

#ifdef CONFIG_PROC_FS

static void print_dpcm_info(struct snd_info_buffer *buffer,
			    struct loopback_pcm *dpcm,
			    const char *id)
{
	snd_iprintf(buffer, "  %s\n", id);
	if (dpcm == NULL) {
		snd_iprintf(buffer, "    inactive\n");
		return;
	}

	snd_iprintf(buffer, "    buffer_size:\t%u\n", dpcm->pcm_buffer_size);
	snd_iprintf(buffer, "    buffer_pos:\t\t%u\n", dpcm->buf_pos);
	snd_iprintf(buffer, "    silent_size:\t%u\n", dpcm->silent_size);
	snd_iprintf(buffer, "    period_size:\t%u\n", dpcm->pcm_period_size);
	snd_iprintf(buffer, "    bytes_per_sec:\t%u\n", dpcm->pcm_bps);
	snd_iprintf(buffer, "    sample_align:\t%u\n", dpcm->pcm_salign);
	snd_iprintf(buffer, "    rate_shift:\t\t%u\n", dpcm->pcm_rate_shift);
	snd_iprintf(buffer, "    update_pending:\t%u\n",
		    dpcm->period_update_pending);
	snd_iprintf(buffer, "    irq_pos:\t\t%u\n", dpcm->irq_pos);
	snd_iprintf(buffer, "    period_frac:\t%u\n", dpcm->period_size_frac);
	snd_iprintf(buffer, "    last_jiffies:\t%lu (%lu)\n",
		    dpcm->last_jiffies, jiffies);
	snd_iprintf(buffer, "    timer_expires:\t%lu\n", dpcm->timer.expires);
	snd_iprintf(buffer, "    timer_nrt_expires:\t%lu\n",
		    dpcm->cable->timer_nrt.expires);
	snd_iprintf(buffer, "    timer_capt_nrt_expires:\t%lu\n",
		    dpcm->cable->timer_capt_nrt.expires);
}

static void print_substream_info(struct snd_info_buffer *buffer,
				 struct loopback *loopback,
				 int sub,
				 int num)
{
	struct loopback_cable *cable = *(loopback->cables[sub] + num);

	snd_iprintf(buffer, "Cable %i substream %i:\n", num, sub);
	if (cable == NULL) {
		snd_iprintf(buffer, "  inactive\n");
		return;
	}
	snd_iprintf(buffer, "  valid: %u\n", cable->valid);
	snd_iprintf(buffer, "  running: %u\n", cable->running);
	snd_iprintf(buffer, "  pause: %u\n", cable->pause);
	print_dpcm_info(buffer, cable->streams[0], "Playback");
	print_dpcm_info(buffer, cable->streams[1], "Capture");
}

static void print_cable_info(struct snd_info_entry *entry,
			     struct snd_info_buffer *buffer)
{
	struct loopback *loopback = entry->private_data;
	int sub, num;

	mutex_lock(&loopback->cable_lock);
	num = entry->name[strlen(entry->name)-1];
	num = num == '0' ? 0 : 1;
	for (sub = 0; sub < MAX_PCM_SUBSTREAMS; sub++)
		print_substream_info(buffer, loopback, sub, num);
	mutex_unlock(&loopback->cable_lock);
}

static int loopback_proc_new(struct loopback *loopback, int cidx)
{
	char name[32];
	struct snd_info_entry *entry;
	int err;

	snprintf(name, sizeof(name), "cable#%d", cidx);
	err = snd_card_proc_new(loopback->card, name, &entry);
	if (err < 0)
		return err;

	snd_info_set_text_ops(entry, loopback, print_cable_info);

	return 0;
}

#else /* !CONFIG_PROC_FS */

#define loopback_proc_new(loopback, cidx) do { } while (0)

#endif

#define SND_LOOPBACK_DRIVER	"snd_sti_aloop"

static int loopback_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct loopback *loopback;
	int err, i;
	int dev = devptr->id;
	struct device_node *pnode = NULL;
	struct device *device = &devptr->dev;

	pnode = device->of_node;

	if (!pnode) {
		dev_err(device, "%s: invalid pnode", __func__);
		return -EINVAL;
	}

	/* create card */
	err = snd_card_create(-1, NULL, THIS_MODULE,
			      sizeof(struct loopback), &card);
	if (err < 0)
		return err;

	/* get private data */
	loopback = card->private_data;

	/* get number of devices from DT */
	of_property_read_u32(pnode, "num_devices", &loopback->num_devices);
	if (!loopback->num_devices) {
		dev_warn(device, "no num devices in DT: set to default");
		loopback->num_devices = STI_ALOOP_DEFAULT_NUM_DEVICES;
	}

	/* allocate memory for pcm */
	loopback->pcm =
		devm_kzalloc(device,
			     loopback->num_devices*sizeof(struct snd_pcm *),
			     GFP_KERNEL);
	if (!loopback->pcm) {
		dev_err(device, "cannot allocate memory for loopback->pcm");
		snd_card_free(card);
		return -ENOMEM;
	}

	for (i = 0; i < MAX_PCM_SUBSTREAMS; i++) {
		loopback->cables[i] =
			devm_kzalloc(device,
				     loopback->num_devices*
					sizeof(struct loopback_cable *),
				     GFP_KERNEL);
		if (!loopback->cables[i]) {
			dev_err(device,
				"can't alloc mem for loopback->cables[%d]", i);
			snd_card_free(card);
			return -ENOMEM;
		}

		loopback->setup[i] =
			devm_kzalloc(device,
				     loopback->num_devices*
					sizeof(struct loopback_setup),
				     GFP_KERNEL);
		if (!loopback->setup[i]) {
			dev_err(device,
				"can't alloc mem for loopback->setup[%d]", i);
			snd_card_free(card);
			return -ENOMEM;
		}
	}

	/* get BPA2 partition name from DT */
	of_property_read_string(pnode, "st,mem-partition-name",
				&loopback->mem_partition_name);

	/* create pcm devices, procfs, ctrl */
	loopback->card = card;
	mutex_init(&loopback->cable_lock);

	for (i = 0; i < loopback->num_devices; i++) {
		err = loopback_pcm_new(loopback, i, 1);
		if (err < 0)
			goto __nodev;

		loopback_proc_new(loopback, i);
	}

	err = loopback_mixer_new(loopback, pcm_notify[dev] ? 1 : 0);
	if (err < 0)
		goto __nodev;

	/* register card */
	strcpy(card->driver, "sti_aloop");
	strcpy(card->shortname, "aloop");
	sprintf(card->longname, "aloop %i", dev + 1);
	err = snd_card_register(card);

	/* set device private data */
	if (!err) {
		platform_set_drvdata(devptr, card);
		return 0;
	}

__nodev:
	dev_err(device, "%s: return error %d\n", __func__, err);
	snd_card_free(card);

	return err;
}

static int loopback_remove(struct platform_device *devptr)
{
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int loopback_suspend(struct device *pdev)
{
	struct snd_card *card = dev_get_drvdata(pdev);
	struct loopback *loopback = card->private_data;
	int i;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);

	for (i = 0; i < loopback->num_devices; i++)
		snd_pcm_suspend_all(loopback->pcm[i]);

	return 0;
}

static int loopback_resume(struct device *pdev)
{
	struct snd_card *card = dev_get_drvdata(pdev);

	snd_power_change_state(card, SNDRV_CTL_POWER_D0);

	return 0;
}

static SIMPLE_DEV_PM_OPS(loopback_pm, loopback_suspend, loopback_resume);
#define LOOPBACK_PM_OPS	(&loopback_pm)
#else
#define LOOPBACK_PM_OPS	NULL
#endif

static const struct of_device_id snd_sti_aloop_match[] = {
	{
		.compatible = "st,snd-sti-aloop",
	},
	{},
};

static struct platform_driver loopback_driver = {
	.probe = loopback_probe,
	.remove	= loopback_remove,
	.driver	= {
		.name = SND_LOOPBACK_DRIVER,
		.owner = THIS_MODULE,
		.of_match_table = snd_sti_aloop_match,
		.pm = LOOPBACK_PM_OPS,
	},
};

module_platform_driver(loopback_driver);

MODULE_AUTHOR("Moise Gergaud <moise.gergaud@st.com>");
MODULE_DESCRIPTION("A ST loopback soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,ST Loopback soundcard}}");
