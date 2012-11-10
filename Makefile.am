LIBCEDARX_LIBRARY_VERSION=1:0:0
AUTOMAKE_OPTIONS = foreign
lib_LTLIBRARIES = libcedarx.la

libcedarx_la_SOURCES = libcedarx.c avheap.c
libcedarx_la_LIBADD = libvecore.so
libcedarx_la_LDFLAGS = -version-info $(LIBCEDARX_LIBRARY_VERSION) 
libcedarx_la_includedir = $(includedir)/libcedarx
libcedarx_la_include_HEADERS = libcedarx.h

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libcedarx.pc
AM_CFLAGS = -Wall

install-data-hook:
	$(INSTALL_DATA) libvecore.so $(libdir)/libvecore.so