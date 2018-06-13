#!/bin/bash
#
# ismono - mono autodetection routines
#
# Written by Alexey Tourbin.
# This file is distributed as Public Domain.

{
	local ff_decode_pre=
	if [[ $1 = *.[Mm][Pp]3 ]]; then
		ff_decode_pre='-acodec mp3float'
	fi

	local g1db downmix=
	local g1peak g1range g1rlow
	local q999s q995s q99s	# short-term, 3s
	local q999m q995m q99m	# momentary, 400ms

	ffmpeg $ff_decode_pre ${ff_ss:+-ss $ff_ss} -i "$1" ${ff_t:+-t $ff_t} -vn \
		-af "replaygain,ebur128=framelog=verbose,pan=1c|c0=0.5*c0+-0.5*c1,ebur128" \
		-f null - >$tmpdir/gain$$.log 2>&1 || { grep -i error $tmpdir/gain$$.log; false; }

	local vars
	vars=$($av0dir/aacgain15pp <$tmpdir/gain$$.log)
	local $vars

	local codec=${g0codec:?}
	if [ $codec = mp2 ]; then
		ff_decode_pre='-acodec mp2float'
	fi
	if [ $codec = mp3 ]; then
		ff_decode_pre='-acodec mp3float'
	fi

	if [ ${g0ch:?} = 1 ]; then
		DualMono rg1
		DualMono eb1
	elif [ $g0ch -gt 2 ]; then
		downmix='aresample=ocl=stereo'
	fi

	g1db=$(Calc2f "(${rg1gain:?} + ${eb1gain:?}) / 2")
	g1peak=$rg1peak g1range=$eb1range g1rlow=$eb1rlow

	if [ $g0ch -gt 1 ]; then
		q999s=$(Calc2f "${gain:-${g1db:?}} + ${eb2S999:?}")
		q995s=$(Calc2f "${gain:-${g1db:?}} + ${eb2S995:?}")
		 q99s=$(Calc2f "${gain:-${g1db:?}} + ${eb2S99:?} ")
		q999m=$(Calc2f "${gain:-${g1db:?}} + ${eb2M999:?}")
		q995m=$(Calc2f "${gain:-${g1db:?}} + ${eb2M995:?}")
		 q99m=$(Calc2f "${gain:-${g1db:?}} + ${eb2M99:?} ")
	fi

	local ismono=0
	if [ $g0ch = 1 ]; then
		ismono=3
		q999s=-91 q99s=-61
		q999m=-91 q99m=-61
	elif Cond "${q999m:?} <= -40"; then
		ismono=2
	elif Cond "${q99s:?} <= -20"; then
		ismono=1
	fi
}
