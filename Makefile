.PHONY: all list-projects clean

DOCKER_IMAGE ?= aabadie/dotbot:latest
DOCKER_TARGETS ?= all
PACKAGES_DIR_OPT ?=
SEGGER_DIR ?= /opt/segger
BUILD_CONFIG ?= Debug
BUILD_TARGET ?= dotbot-v1
PROJECT_FILE ?= $(BUILD_TARGET).emProject
QUIET ?= 0
VERBOSE_OPTS ?= -verbose -echo
ifeq ($(QUIET),1)
  VERBOSE_OPTS =
endif

ifeq (nrf5340dk-app,$(BUILD_TARGET))
  PROJECTS ?= \
    dotbot \
    dotbot_gateway \
    dotbot_gateway_lr \
    log_dump \
    sailbot \
    lh2_mini_mote_app \
    lh2_mini_mote_test \
    #
else ifeq (nrf5340dk-net,$(BUILD_TARGET))
  PROJECTS ?= \
    dotbot_gateway \
    dotbot_gateway_lr \
    log_dump \
    nrf5340_net \
    #
else ifneq (,$(filter freebot-v%,$(BUILD_TARGET)))
  PROJECTS ?= \
    freebot \
    #
  ARTIFACT_PROJECTS := freebot
  # Bootloader not supported on freebot
  BOOTLOADER :=
else ifneq (,$(filter xgo%,$(BUILD_TARGET)))
  PROJECTS ?= \
    xgo \
    #
  ARTIFACT_PROJECTS := xgo
  # Bootloader not supported on xgo
else
  PROJECTS ?= $(shell find projects/ -maxdepth 1 -mindepth 1 -type d | tr -d "/" | sed -e s/projects// | sort)
endif

# remove incompatible apps (nrf5340, sailbot gateway) for dotbot (v1, v2) builds
ifneq (,$(filter dotbot-v1,$(BUILD_TARGET)))
  PROJECTS := $(filter-out dotbot_gateway dotbot_gateway_lr sailbot nrf5340_% freebot lh2_mini_mote% xgo,$(PROJECTS))
  ARTIFACT_PROJECTS := dotbot
endif

ifneq (,$(filter dotbot-v2 dotbot-v3,$(BUILD_TARGET)))
  PROJECTS := $(filter-out dotbot_gateway dotbot_gateway_lr sailbot xgo nrf5340_net freebot lh2_mini_mote%,$(PROJECTS))
  ARTIFACT_PROJECTS := dotbot
endif

# remove incompatible apps (nrf5340, sailbot, gateway, dotbot) for lh2-mini-mote builds
ifneq (,$(filter lh2-mini-mote,$(BUILD_TARGET)))
  PROJECTS := $(filter-out dotbot_gateway dotbot_gateway_lr dotbot sailbot nrf5340_% freebot xgo,$(PROJECTS))
  ARTIFACT_PROJECTS := lh2_mini_mote_app
endif

# remove incompatible apps (nrf5340, dotbot, gateway) for sailbot-v1 build
ifeq (sailbot-v1,$(BUILD_TARGET))
  PROJECTS := $(filter-out dotbot_gateway dotbot_gateway_lr dotbot nrf5340_% freebot lh2_mini_mote% xgo,$(PROJECTS))
  ARTIFACT_PROJECTS := sailbot
endif

ifneq (,$(filter nrf52833dk,$(BUILD_TARGET)))
  PROJECTS := $(filter-out 01crypto_% 01bsp_qspi,$(PROJECTS))
endif

# remove incompatible apps (nrf5340) for nrf52833dk/nrf52840dk build
ifneq (,$(filter nrf52833dk nrf52840dk,$(BUILD_TARGET)))
  PROJECTS := $(filter-out nrf5340_% freebot xgo,$(PROJECTS))
  ARTIFACT_PROJECTS := dotbot_gateway dotbot_gateway_lr
endif

ifneq (,$(filter nrf5340dk-app,$(BUILD_TARGET)))
  ARTIFACT_PROJECTS := dotbot_gateway dotbot_gateway_lr
endif

ifneq (,$(filter nrf5340dk-net,$(BUILD_TARGET)))
  ARTIFACT_PROJECTS := nrf5340_net
endif

DIRS ?= projects
SRCS ?= $(foreach dir,$(DIRS),$(shell find $(dir) -name "*.[c|h]"))
CLANG_FORMAT ?= clang-format
CLANG_FORMAT_TYPE ?= file

ARTIFACT_ELF = $(foreach app,$(ARTIFACT_PROJECTS),projects/$(app)/Output/$(BUILD_TARGET)/$(BUILD_CONFIG)/Exe/$(app)-$(BUILD_TARGET).elf)
ARTIFACT_HEX = $(ARTIFACT_ELF:.elf=.hex)
ARTIFACTS = $(ARTIFACT_ELF) $(ARTIFACT_HEX)


.PHONY: $(PROJECTS) $(ARTIFACT_PROJECTS) artifacts docker docker-release format check-format

all: $(PROJECTS)

$(PROJECTS):
	@echo "\e[1mBuilding project $@\e[0m"
	"$(SEGGER_DIR)/bin/emBuild" $(PROJECT_FILE) -project $@ -config $(BUILD_CONFIG) $(PACKAGES_DIR_OPT) -rebuild $(VERBOSE_OPTS)
	@echo "\e[1mDone\e[0m\n"

list-projects:
	@echo "\e[1mAvailable projects:\e[0m"
	@echo $(PROJECTS) | tr ' ' '\n'

clean:
	"$(SEGGER_DIR)/bin/emBuild" $(PROJECT_FILE) -config $(BUILD_CONFIG) -clean $(VERBOSE_OPTS)

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

UID := $(shell id -u)
GID := $(shell id -g)

docker:
	docker run --rm -i \
		-u $(UID):$(GID) \
		-e BUILD_TARGET="$(BUILD_TARGET)" \
		-e BUILD_CONFIG="$(BUILD_CONFIG)" \
		-e PACKAGES_DIR_OPT="-packagesdir $(SEGGER_DIR)/packages" \
		-e PROJECTS="$(PROJECTS)" \
		-e SEGGER_DIR="$(SEGGER_DIR)" \
		-v $(PWD):/dotbot $(DOCKER_IMAGE) \
		make $(DOCKER_TARGETS)

.PHONY: doc docclean
docclean:
	make -C doc/sphinx clean --no-print-directory
	rm -rf $(addprefix doc/sphinx/_,api examples projects)

# doc:
# 	make -C doc/sphinx linkcheck html --no-print-directory SPHINXOPTS="-W --keep-going -n"
doc:
	make -C doc/sphinx html --no-print-directory SPHINXOPTS="-W --keep-going -n"
