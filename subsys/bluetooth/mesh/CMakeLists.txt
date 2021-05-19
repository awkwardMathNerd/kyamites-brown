#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(model_utils.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR_SRV sensor_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR_CLI sensor_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR sensor_types.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR sensor.c)

zephyr_linker_sources(SECTIONS sensor_types.ld)
