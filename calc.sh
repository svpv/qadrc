#!/bin/sh
#
# calc.sh - shortcuts to evaluate perl snippets
#
# Written by Alexey Tourbin.
# This file is distributed as Public Domain.

Calc()
{
	perl -E "say do { $* }"
}

Calc0f() { perl -E "say 0 + sprintf '%.0f', do { $* }"; }
Calc1f() { perl -E "say 0 + sprintf '%.1f', do { $* }"; }
Calc2f() { perl -E "say 0 + sprintf '%.2f', do { $* }"; }
Calc3f() { perl -E "say 0 + sprintf '%.3f', do { $* }"; }
Calc4f() { perl -E "say 0 + sprintf '%.4f', do { $* }"; }
Calc5f() { perl -E "say 0 + sprintf '%.5f', do { $* }"; }
Calc6f() { perl -E "say 0 + sprintf '%.6f', do { $* }"; }
Calc7f() { perl -E "say 0 + sprintf '%.7f', do { $* }"; }
Calc8f() { perl -E "say 0 + sprintf '%.8f', do { $* }"; }
Calc0f() { perl -E "say 0 + sprintf '%.9f', do { $* }"; }

Cond()
{
	perl -E "exit(do { $* } ? 0 : 1)"
}
