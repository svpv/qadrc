#!/bin/bash
#
# ismono - mono autodetection routines
#
# Written by Alexey Tourbin.
# This file is distributed as Public Domain.

MonoTry2()
{
	ffmpeg $ff_decode_pre ${ff_ss:+-ss $ff_ss} -i "$1" ${ff_t:+-t $ff_t} -vn \
		-af "${downmix:+$downmix,}$c0afs,replaygain,ebur128=framelog=verbose,
		pan=1c|c0=0.5*c0+-0.5*c1,ebur128" \
		-f null - >$tmpdir/gain$$.log 2>&1 || { grep -i error $tmpdir/gain$$.log; false; }
	local vars
	vars=$($av0dir/aacgain15pp <$tmpdir/gain$$.log)
	local $vars
	g1db=$(Calc2f "(${rg1gain:?} + ${eb1gain:?}) / 2")
	g1peak=$rg1peak g1range=$eb1range g1rlow=$eb1rlow
	q999=$(Calc2f "${gain:-${g1db:?}} + ${eb2S999:?}")
	q99=$( Calc2f "${gain:-${g1db:?}} + ${eb2S99:?} ")
}

{
	local ff_decode_pre=
	if [[ $1 = *.[Mm][Pp]3 ]]; then
		ff_decode_pre='-acodec mp3float'
	fi

	local g1db downmix=
	local g1peak g1range g1rlow
	local q999 q99 ismono=0

	# left channel needs correction?
	local c0db=0 c0afs= c0afm=

	ffmpeg $ff_decode_pre ${ff_ss:+-ss $ff_ss} -i "$1" ${ff_t:+-t $ff_t} -vn \
		-af "replaygain,asplit=3[i1][i2][i3];
		[i1]ebur128=framelog=verbose,
		    pan=1c|c0=c0,volumedetect,ebur128=framelog=verbose[o1];
		[i2]pan=1c|c0=c1,volumedetect,ebur128=framelog=verbose[o2];
		[i3]pan=1c|c0=0.5*c0+-0.5*c1,volumedetect,ebur128[o3];
		[o1][o2][o3]amix=3" \
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
		q999=$(Calc2f "${gain:-${g1db:?}} + ${eb4S999:?}")
		q99=$( Calc2f "${gain:-${g1db:?}} + ${eb4S99:?} ")

		local save_vars='g1db rg1peak eb1range eb1rlow q999 q99'
		c0db=$(Calc3f "($eb2gain - $eb3gain + $vd2mean - $vd1mean) / 2")
		if [ $c0db != 0 ]; then
			local c0w1 c0w2
			c0w1=$(Calc6f "1.0*10**(0.05*$c0db)")
			c0w2=$(Calc6f "0.5*10**(0.05*$c0db)")
			c0afs="pan=2c|c0=$c0w1*c0|c1=c1"
			c0afm="pan=1c|c0=$c0w2*c0+0.5*c1"
			for var in $save_vars; do
				local save_$var=${!var}
			done

			MonoTry2 "$1"

			# did left channel correction actually reduce the difference?
			if Cond "$q99 >= ${save_q99:?}"; then
				c0db=0 c0afs= c0afm=
				for var in $save_vars; do
					eval "$var=\$save_$var"
				done
			fi
		fi
	fi

	if [ $g0ch = 1 ]; then
		ismono=3 q999=-91 q99=-61
	elif Cond "${q999:?} <= -40"; then
		ismono=2
	elif Cond "${q99:?} <= -20.56"; then
		ismono=1
	fi
}
