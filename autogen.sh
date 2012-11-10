#! /bin/sh

set -x

aclocal
autoconf
autoheader
libtoolize --copy --force
automake --foreign --add-missing --copy


