MAKEFLAGS+=-j 2
CC=mipsel-openwrt-linux-gcc
DEB=-Wall -g
REL=-Wall -O3 -s

deb: 00-deb
rel: 00-rel

00-deb:
	$(CC) $(DEB) -o 00-sens 00-sens.c
00-rel:
	$(CC) $(REL) -o 00-sens 00-sens.c

tags:
	ctags -R . $(STAGING_DIR)/toolchain-mipsel_24kc_gcc-7.3.0_musl/include

clean:
	rm -f 00-sens
	rm -f tags

