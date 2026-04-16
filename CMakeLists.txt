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
  src/radio.c
  src/queue.c
  src/product_info.c
  src/mesh.c
  src/storage.c
  src/sensor.c
  src/gateway.c
  src/anchor.c
  src/timeout.c
  src/tracker.c
  src/fragment.c
)
