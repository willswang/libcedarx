#! /bin/sh

set -x

aclocal
autoconf
autoheader
libtoolize --automake --copy --force
automake --foreign --add-missing --copy


