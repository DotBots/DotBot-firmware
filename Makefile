.PHONY: all list-projects clean

DOCKER_IMAGE ?= aabadie/dotbot:latest
DOCKER_TARGETS ?= all
PACKAGES_DIR_OPT ?=
SEGGER_DIR ?= /opt/segger
BUILD_CONFIG ?= Debug
BUILD_TARGET ?= dotbot-v1
PROJECT_FILE ?= $(BUILD_TARGET).emProject
BOOTLOADER ?= bootloader

ifeq (nrf5340dk-app,$(BUILD_TARGET))
  PROJECTS ?= \
    01bsp_gpio \
    01bsp_i2c \
    01bsp_lighthouse \
    01bsp_motors \
    01bsp_nvmc \
    01bsp_qdec \
    01bsp_radio_txrx \
    01bsp_radio_txrx_lr \
    01bsp_rgbled \
    01bsp_rng \
    01bsp_rpm \
    01bsp_timer \
    01bsp_timer_hf \
    01bsp_uart \
    01drv_lis2mdl \
    01drv_lis3mdl \
    01drv_move \
    01drv_pid \
    03app_dotbot \
    03app_dotbot_gateway \
    03app_dotbot_gateway_lr \
    03app_log_dump \
    03app_nrf5340_app \
    03app_sailbot \
    #
else ifeq (nrf5340dk-net,$(BUILD_TARGET))
  PROJECTS ?= \
    01bsp_gpio \
    01bsp_i2c \
    01bsp_motors \
    01bsp_nvmc \
    01bsp_radio_txrx \
    01bsp_radio_txrx_lr \
    01bsp_rgbled \
    01bsp_rng \
    01bsp_rpm \
    01bsp_timer \
    01bsp_timer_hf \
    01bsp_uart \
    01drv_lis2mdl \
    01drv_pid \
    03app_dotbot_gateway \
    03app_dotbot_gateway_lr \
    03app_log_dump \
    03app_nrf5340_net \
    #
  # Bootloader not supported on nrf5340 network core
  BOOTLOADER :=
else
  PROJECTS ?= $(shell find projects/ -maxdepth 1 -mindepth 1 -type d | tr -d "/" | sed -e s/projects// | sort)
endif

OTAP_APPS ?= $(shell find otap/ -maxdepth 1 -mindepth 1 -type d | tr -d "/" | sed -e s/otap// | sort)
OTAP_APPS := $(filter-out bootloader,$(OTAP_APPS))

# remove incompatible apps (nrf5340, sailbot gateway) for dotbot (v1, v2) builds
ifneq (,$(filter dotbot-v1,$(BUILD_TARGET)))
  PROJECTS := $(filter-out 01bsp_qdec 01drv_lis3mdl 01drv_move 03app_dotbot_gateway 03app_dotbot_gateway_lr 03app_sailbot 03app_nrf5340_%,$(PROJECTS))
  ARTIFACT_PROJECTS := 03app_dotbot
endif

ifneq (,$(filter dotbot-v2,$(BUILD_TARGET)))
  PROJECTS := $(filter-out 03app_dotbot_gateway 03app_dotbot_gateway_lr 03app_sailbot 03app_nrf5340_net,$(PROJECTS))
  ARTIFACT_PROJECTS := 03app_dotbot
endif

# remove incompatible apps (nrf5340, dotbot, gateway) for sailbot-v1 build
ifeq (sailbot-v1,$(BUILD_TARGET))
  PROJECTS := $(filter-out 01bsp_qdec 01drv_lis3mdl 01drv_move 03app_dotbot_gateway 03app_dotbot_gateway_lr 03app_dotbot 03app_nrf5340_%,$(PROJECTS))
  ARTIFACT_PROJECTS := 03app_sailbot
endif

# remove incompatible apps (nrf5340) for nrf52833dk/nrf52840dk build
ifneq (,$(filter nrf52833dk nrf52840dk,$(BUILD_TARGET)))
  PROJECTS := $(filter-out 01bsp_qdec 01drv_move 03app_nrf5340_%,$(PROJECTS))
  ARTIFACT_PROJECTS := 03app_dotbot_gateway 03app_dotbot_gateway_lr
endif

ifneq (,$(filter nrf5340dk-app,$(BUILD_TARGET)))
  ARTIFACT_PROJECTS := 03app_dotbot_gateway 03app_dotbot_gateway_lr
endif

ifneq (,$(filter nrf5340dk-net,$(BUILD_TARGET)))
  ARTIFACT_PROJECTS := 03app_nrf5340_net
endif

SRCS ?= $(shell find bsp/ -name "*.[c|h]") $(shell find crypto/ -name "*.[c|h]") $(shell find drv/ -name "*.[c|h]") $(shell find projects/ -name "*.[c|h]") $(shell find otap/ -name "*.[c|h]")
CLANG_FORMAT ?= clang-format
CLANG_FORMAT_TYPE ?= file

ARTIFACT_ELF = $(foreach app,$(ARTIFACT_PROJECTS),projects/$(app)/Output/$(BUILD_TARGET)/$(BUILD_CONFIG)/Exe/$(app)-$(BUILD_TARGET).elf)
ARTIFACT_HEX = $(ARTIFACT_ELF:.elf=.hex)
ARTIFACTS = $(ARTIFACT_ELF) $(ARTIFACT_HEX)


.PHONY: $(PROJECTS) $(ARTIFACT_PROJECTS) artifacts docker docker-release format check-format

all: $(PROJECTS) $(OTAP_APPS) $(BOOTLOADER)

$(PROJECTS):
	@echo "\e[1mBuilding project $@\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" $(PROJECT_FILE) -project $@ -config $(BUILD_CONFIG) $(PACKAGES_DIR_OPT) -rebuild -verbose
	@echo "\e[1mDone\e[0m\n"

$(OTAP_APPS):
	@echo "\e[1mBuilding otap application $@\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" $(PROJECT_FILE) -project $@ -config $(BUILD_CONFIG) $(PACKAGES_DIR_OPT) -rebuild -verbose
	@echo "\e[1mDone\e[0m\n"

$(BOOTLOADER):
	@echo "\e[1mBuilding bootloader application $@\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" otap/$(BUILD_TARGET)-bootloader.emProject -project $@ -config Release $(PACKAGES_DIR_OPT) -rebuild -verbose
	@echo "\e[1mDone\e[0m\n"

list-projects:
	@echo "\e[1mAvailable projects:\e[0m"
	@echo $(PROJECTS) | tr ' ' '\n'

clean:
	"$(SEGGER_DIR)/bin/emBuild" $(PROJECT_FILE) -config $(BUILD_CONFIG) -clean

distclean: clean
	rm -rf artifacts/

format:
	@$(CLANG_FORMAT) -i --style=$(CLANG_FORMAT_TYPE) $(SRCS)

check-format:
	@$(CLANG_FORMAT) --dry-run --Werror --style=$(CLANG_FORMAT_TYPE) $(SRCS)

artifacts: $(ARTIFACT_PROJECTS)
	@mkdir -p artifacts
	@for artifact in "$(ARTIFACTS)"; do \
		cp $${artifact} artifacts/.; \
		done
	@ls -l artifacts/

docker:
	docker run --rm -i \
		-e BUILD_TARGET="$(BUILD_TARGET)" \
		-e BUILD_CONFIG="$(BUILD_CONFIG)" \
		-e PACKAGES_DIR_OPT="-packagesdir $(SEGGER_DIR)/packages" \
		-e PROJECTS="$(PROJECTS)" \
		-e SEGGER_DIR="$(SEGGER_DIR)" \
		-v $(PWD):/dotbot $(DOCKER_IMAGE) \
		make $(DOCKER_TARGETS)

.PHONY: doc docclean
docclean:
	make -C doc/doxygen clean --no-print-directory
	make -C doc/sphinx clean --no-print-directory

doc:
	make -C doc/sphinx linkcheck html --no-print-directory SPHINXOPTS="-W --keep-going -n"
