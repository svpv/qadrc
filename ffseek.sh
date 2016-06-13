#!/bin/bash

# seek and duration support
ff_ss= ff_to= ff_t=

tc2ms()
{
	perl -le '
		@s = reverse split /:/, shift;
		$s = $s[0] + 60 * $s[1] + 3600 * $s[2];
		print int 0.5 + $s * 1000;
		' -- "$@"
}

ms2tc()
{
	perl -le "print $1 / 1000"
}

ffseek()
{
	local ms1 ms2 ms3
	# treat -ss 0 as no -ss
	if [ -n "$ff_ss" ]; then
		ms1=$(tc2ms "$ff_ss")
		[ $ms1 -gt 0 ] || ff_ss=
	fi
	# -t overrides -to
	if [ -n "$ff_t" ]; then
		if [ -z "$ff_ss" ]; then
			ff_to=$ff_t
		else
			ms1=$(tc2ms "$ff_ss")
			ms2=$(tc2ms "$ff_t")
			ms3=$(($ms1+$ms2))
			ff_to=$(ms2tc $ms3)
		fi
	elif [ -n "$ff_to" ]; then
		if [ -z "$ff_ss" ]; then
			ff_t=$ff_to
		else
			ms1=$(tc2ms "$ff_ss")
			ms2=$(tc2ms "$ff_to")
			ms3=$(($ms2-$ms1))
			ff_t=$(ms2tc $ms3)
		fi
	fi
}
