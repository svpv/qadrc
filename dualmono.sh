#!/bin/bash
#
# dualmono.sh - adjust filter volumes for mono input
#
# When ffmpeg converts mono to stereo (such as when feeding mono input
# to replaygain, or using aresample=ocl=stereo explicitly), it tries
# to preserve rms volume while redistributing the signal among channels;
# thus absolute dbfs levels drop.  This file implements alternative
# "dualmono" logic: mono input should have been duplicated as is.
#
# Written by Alexey Tourbin.
# This file is distributed as Public Domain.

DualFix1()
{
	local old new
	eval "old=\$$1$2"
	new=$(Calc2f "$old $3 3.01")
	eval "$1$2=$new"
}

DualMono()
{
	local v
	for v; do
		case $v in
			# replaygain
			*rg[1-9])
				# rms is divided by twice as many samples;
				# rg suggests we increase the volume, but we don't
				DualFix1 $v gain -
				# absolute peak
				DualFix1 $v peak +
				continue ;;
			# ebur128
			*eb[1-9])
				# reverse-engeneered with ./dualtest
				DualFix1 $v gain -
				DualFix1 $v rlow +
				continue ;;
			# volumedetect
			*vd[1-9])
				# absoulte peak
				DualFix1 $v peak +
				# histogram quantile
				DualFix1 $v q999 +
				continue ;;
		esac
		echo "DualMono: unknown variables: $v" >&2
		return 1
	done
}
