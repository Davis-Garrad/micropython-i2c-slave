I2C_SLAVE_MOD_DIR := $(USERMOD_DIR)

# add all C files
SRC_USERMOD += $(I2C_SLAVE_MOD_DIR)/i2c_slave.c

# add module folder to include paths
CFLAGS_USERMOD += -I$(I2C_SLAVE_MOD_DIR)/include
