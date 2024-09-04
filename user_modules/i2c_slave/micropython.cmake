# Create interface library
add_library(i2c_slave INTERFACE)

# add sourfes
target_sources(i2c_slave INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/i2c_slave.c
)

# add includes
target_include_directories(i2c_slave INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/include
)

# Link our interface library to the usermod
target_link_libraries(usermod INTERFACE i2c_slave)
