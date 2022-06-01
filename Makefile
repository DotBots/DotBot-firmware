.PHONY: all list-projects clean

SEGGER_DIR ?= /opt/segger
BUILD_CONFIG ?= Debug
PROJECTS ?= $(subst /,,$(subst ./projects/,,$(sort $(dir $(wildcard ./projects/*/)))))
.PHONY: $(PROJECTS)

all: $(PROJECTS)

$(PROJECTS):
	@echo "\e[1mBuilding project $@\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" projects/dotbot-firmware.emProject -project $@ -config $(BUILD_CONFIG) -rebuild -verbose
	@echo "\e[1mDone\e[0m\n"

list-projects:
	@echo "\e[1mAvailable projects:\e[0m"
	@echo $(PROJECTS) | tr ' ' '\n'

clean:
	"$(SEGGER_DIR)/bin/emBuild" projects/dotbot-firmware.emProject -config $(BUILD_CONFIG) -clean

