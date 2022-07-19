.PHONY: all list-projects clean

SEGGER_DIR ?= /opt/segger
BUILD_CONFIG ?= Debug

PROJECTS ?= $(shell git grep -h "project Name" projects/dotbot-firmware.emProject | sed -e s/"<project Name=\"/"/ | sed -e s/\"\>// | sed -e s/^[[:space:]]*// | sort)
SRCS ?= $(shell find bsp/ -name "*.[c|h]") $(shell find drv/ -name "*.[c|h]") $(shell find projects/ -name "*.[c|h]")
CLANG_FORMAT ?= clang-format
CLANG_FORMAT_TYPE ?= file

.PHONY: $(PROJECTS) format check-format

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

format:
	@$(CLANG_FORMAT) -i --style=$(CLANG_FORMAT_TYPE) $(SRCS)

check-format:
	@$(CLANG_FORMAT) --dry-run --Werror --style=$(CLANG_FORMAT_TYPE) $(SRCS)
