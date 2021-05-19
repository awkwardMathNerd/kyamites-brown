
zephyr_library()

add_subdirectory(boards)

zephyr_library_sources(model_utils.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR_SRV sensor_srv.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR_CLI sensor_cli.c)

zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR sensor_types.c)
zephyr_library_sources_ifdef(CONFIG_BT_MESH_SENSOR sensor.c)

zephyr_linker_sources(SECTIONS sensor_types.ld)
