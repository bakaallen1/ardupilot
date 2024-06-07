.PHONY: all configure copter clean

all: configure copter

configure:
	./waf configure --board CUAV-X7

copter:
	./waf copter

