
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BT_MESH_BOARD_ARGON_PROV argon_prov.c)