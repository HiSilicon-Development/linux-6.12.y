# SPDX-License-Identifier: GPL-2.0-only

PYTHON ?= python3
SRC_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BSP_CORE := $(SRC_DIR)bsp/Hi3798CV200/LNX_CFG/32bit/core
OUT ?= $(SRC_DIR)

TARGETS := $(OUT)/histb-avsp.bin \
	$(OUT)/histb-h264-cabac.bin \
	$(OUT)/histb-hevc-cabac.bin

.PHONY: all
all: $(TARGETS)

$(OUT)/histb-avsp.bin: $(BSP_CORE)/driver/HiSCDV200/scd_drv.S \
		$(SRC_DIR)generate-avsp.py
	@mkdir -p $(@D)
	$(PYTHON) $(SRC_DIR)generate-avsp.py $< $@

$(OUT)/histb-h264-cabac.bin: $(BSP_CORE)/syntax/h264.S \
		$(SRC_DIR)generate-h264-cabac.py
	@mkdir -p $(@D)
	$(PYTHON) $(SRC_DIR)generate-h264-cabac.py $< $@

$(OUT)/histb-hevc-cabac.bin: $(BSP_CORE)/driver/HiVDHV4R3C1/vdm_hal_hevc.S \
		$(SRC_DIR)generate-hevc-cabac.py
	@mkdir -p $(@D)
	$(PYTHON) $(SRC_DIR)generate-hevc-cabac.py $< $@
