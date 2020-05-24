all: clean
	afu_synth_setup --source hw/rtl/sources.txt build_synth
	cd build_synth; $(OPAE_PLATFORM_ROOT)/bin/run.sh
ifeq ($(SOFF_OPAE_CREATE_UNSIGNED_BITSTREAM), 1)
	cp config.gbs config_signed.gbs
	PACSign PR -t UPDATE -H openssl_manager -i config_signed.gbs -o config.gbs
endif

clean:
	rm -rf build build_synth design
