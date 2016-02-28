qadrc - dynamic range compression
=================================
Some program material, such as found on BBC Radio 3, uses wide dynamic
range (about 20 dB, as measured by EBU R-128 meter).  To make this material
more suitable for portable listening, the dynamic range should be reduced
towards 10 dB.  This project implements a few ffmpeg filters and further
builds on them to provide a general-purpose automatic procedure for reducing
dynamic range and transcoding for portable listening.

qalimiter
---------
This is a port of [qaac smart limiter](https://github.com/nu774/qaac/wiki/Sample-format-or-bit-depth-conversion,-and-limiter#about-limiter).
It should be applied to an audio signal which is already normalized and
generally well-limited, except possibly for a few stray spikes.  It then
fixes those spikes in the most seamless manner.

qadrc
-----
This is a port of [qaac compressor](https://github.com/nu774/qaac/wiki/Dynamic-range-compression).
It's a classic compressor (= with attack and release times, rooted in
analog designs).  The implementation is specifically based on
[Digital Dynamic Range Compressor Design - A Tutorial and Analysis](https://www.eecs.qmul.ac.uk/~josh/documents/GiannoulisMassbergReiss-dynamicrangecompression-JAES2012.pdf),
JAES2012.  In addition, this implementation provides the delay (aka lookahead)
control to cope better with sharp attacks.

mydrc
-----
Unlike classic compressors, `mydrc` doesn't do close envelope following
and rapid level changes.  Instead, it uses a huge lookahead buffer to adjust
the volume gradually, over the period of a few seconds.  The implementation
is based on LoRd\_MuldeR's [Dynamic Audio Normalizer](https://github.com/lordmulder/DynamicAudioNormalizer)
(ported to ffmpeg as `dynaudnorm`).  It uses Gaussian smoothing filter.
Unlike `dynaudnorm`, however, it doesn't do upward compression and actually
doesn't try to normalize the volume; that is, it does not "even out" the volume
of quiet and loud sections completely.  Instead, it uses downward compression
curve similar to that of `qadrc`, so that loud parts stay relatively loud,
and soft parts remain relatively soft.

transcode
---------
This is the script which puts it all together.  It checks to see
if the dynamic range of the input signal is wider than 10 dB, and sets up
[parallel compression](https://en.wikipedia.org/wiki/Parallel_compression),
mixing the outputs of `qadrc` and `mydrc`.  It also does automatic mono
detection and volume normalization.

aacgain15
---------
This is the volume normalization part which can be used separately.
It normalizes to the average of ReplayGain 1.0 and 2.0 specifications.
ReplayGain 2.0 uses EBU R-128 algorithm with -18 LUFS reference level,
and is generally considered to supersede ReplayGain 1.0.  However,
ReplayGain 1.0 is more sensitive to peak levels, which is important
for portable use.  The difference between ReplayGain 1.0 and 2.0,
after applying compression, is often around 2 dB (that is, ReplayGain 2.0
suggests volume go 1 dB up, while ReplayGain 1.0 suggests volume
go 1 dB down).

Other pieces
------------
You can use [apicker](https://github.com/svpv/apicker)
to pick the right part of the program (= its start and end times)
before passing it down to `transcode`.
