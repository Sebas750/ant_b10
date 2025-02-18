zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ANT_B10 ant_b10.c)

# Add dependencies if nedeed
# zephyr_library_add_dependencies_ifdef(CONFIG_ANT_B10 <dependency>)
