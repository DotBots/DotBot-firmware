.PHONY: all

SEGGER_DIR ?= /opt/segger
BUILD_CONFIG ?= Debug

all:
	"$(SEGGER_DIR)/bin/emBuild" projects/dotbot-firmware.emProject -config $(BUILD_CONFIG) -rebuild -verbose
