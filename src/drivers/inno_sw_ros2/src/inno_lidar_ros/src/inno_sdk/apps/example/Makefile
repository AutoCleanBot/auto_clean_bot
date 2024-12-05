###
#  Copyright (C) 2023 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##
.PHONY: all default pcap clean

all: default

default:
	@$(eval JOBS := $(shell echo $(MAKEFLAGS) | grep -o '\-j[[:digit:]]\+' | sed 's/-j//'))
	@$(if $(JOBS), , $(eval JOBS := 4))
	@$(eval MAKEFLAGS = "")
	mkdir -p build
	cd build && cmake -G "Unix Makefiles" .. && make -j$(JOBS) && make install
	rm -rf build

# Compile the example program supports pcap file input
pcap:
	@$(eval JOBS := $(shell echo $(MAKEFLAGS) | grep -o '\-j[[:digit:]]\+' | sed 's/-j//'))
	@$(if $(JOBS), , $(eval JOBS := 4))
	@$(eval MAKEFLAGS = "")
	mkdir -p build
	cd build && cmake -G "Unix Makefiles" -DPCAP_INPUT=1 .. && make -j$(JOBS) && make install
	rm -rf build

clean:
	rm -rf build
	rm -f demo
