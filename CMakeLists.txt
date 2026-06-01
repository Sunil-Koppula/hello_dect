#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_dect)

target_sources(app PRIVATE
  src/main.c
  src/main_sub.c
  src/radio.c
  src/queue.c
  src/product_info.c
  src/mesh.c
  src/mesh_layers/mesh_pairing.c
  src/mesh_layers/mesh_routing.c
  src/mesh_layers/mesh_session.c
  src/timeout.c
  src/tracker.c
  src/psram.c
  src/data.c
  src/config.c
  src/large_data.c
  src/spi_bus.c
  src/storage.c
  src/slm_at_uart.c
  src/slm_at_sub.c
  src/slm_at_main.c
)

# Testing utilities
target_sources(app PRIVATE src/testing/buttons.c)