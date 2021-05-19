
add_subdirectory_ifdef(CONFIG_BT            bluetooth)

# add_subdirectory_ifdef(CONFIG_SUBSYS        some/subsys/location)  # adds when configured (needs be in zephyr, module or app layers to work)
# add_subdirectory(some/subsys/location)  # always added