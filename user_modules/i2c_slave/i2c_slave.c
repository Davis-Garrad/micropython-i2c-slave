/*
 * Copyright (c) 2025 Davis V. Garrad  <dvlast@uwaterloo.ca>
 *
 * SPDX-License-Identifier: MIT
 */

#include "py/runtime.h"

#include <i2c_slave.h>
#include <i2c_fifo.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>

static const int I2C_FREQ = 400000;

typedef struct i2c_slave_t
{
    i2c_inst_t *i2c;
    i2c_slave_handler_t handler;
    bool transfer_in_progress;
} i2c_slave_t;

static i2c_slave_t i2c_slaves[2];

static inline void finish_transfer(i2c_slave_t *slave) {
    if (slave->transfer_in_progress) {
        slave->handler(slave->i2c, I2C_SLAVE_FINISH);
        slave->transfer_in_progress = false;
    }
}

static void __not_in_flash_func(i2c_slave_irq_handler)(i2c_slave_t *slave) {
    i2c_inst_t *i2c = slave->i2c;
    i2c_hw_t *hw = i2c_get_hw(i2c);

    uint32_t intr_stat = hw->intr_stat;
    if (intr_stat == 0) {
        return;
    }
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        hw->clr_tx_abrt;
        finish_transfer(slave);
    }
    if (intr_stat & I2C_IC_INTR_STAT_R_START_DET_BITS) {
        hw->clr_start_det;
        finish_transfer(slave);
    }
    if (intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        hw->clr_stop_det;
        finish_transfer(slave);
    }
    if (intr_stat & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        slave->transfer_in_progress = true;
        slave->handler(i2c, I2C_SLAVE_RECEIVE);
    }
    if (intr_stat & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        hw->clr_rd_req;
        slave->transfer_in_progress = true;
        slave->handler(i2c, I2C_SLAVE_REQUEST);
    }
}

static void __not_in_flash_func(i2c0_slave_irq_handler)() {
    i2c_slave_irq_handler(&i2c_slaves[0]);
}

static void __not_in_flash_func(i2c1_slave_irq_handler)() {
    i2c_slave_irq_handler(&i2c_slaves[1]);
}


static uint8_t selected_bundle = 0;
static uint8_t selected_sensor = 0;
static uint16_t temperature_ticks = 0;
static bool read_required = false;

static mp_obj_t is_read_required() {
	return mp_obj_new_int(read_required);
}

static mp_obj_t set_read_required(mp_obj_t read) {
	read_required = mp_obj_get_int(read);
	return mp_obj_new_int(read_required);
}

static MP_DEFINE_CONST_FUN_OBJ_0(is_read_required_obj, is_read_required);
static MP_DEFINE_CONST_FUN_OBJ_1(set_read_required_obj, set_read_required);

static mp_obj_t set_temperature_ticks(mp_obj_t ticks_) {
	temperature_ticks = mp_obj_get_int(ticks_);
	return mp_obj_new_int(temperature_ticks);
}

static MP_DEFINE_CONST_FUN_OBJ_1(set_temperature_ticks_obj, set_temperature_ticks);

static mp_obj_t get_bundle() {
	return mp_obj_new_int(selected_bundle);
}

static mp_obj_t get_sensor() {
	return mp_obj_new_int(selected_sensor);
}

static MP_DEFINE_CONST_FUN_OBJ_0(get_bundle_obj, get_bundle);
static MP_DEFINE_CONST_FUN_OBJ_0(get_sensor_obj, get_sensor);

static void handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch(event) {
	case I2C_SLAVE_RECEIVE: {
		// Read the address of the sensor/bundle we should read (4 LSBs determine sensor, 4 MSBs determine bundle)
		uint8_t byte = i2c_read_byte(i2c);

		uint8_t word_sensor = byte & 0xf;
		uint8_t word_bundle = (byte >> 4) & 0xf; // 0xf=15=0b00001111
		
		// pass info along to the pico
		selected_sensor  = word_sensor;
		selected_bundle = word_bundle;
		temperature_ticks = 0; // reset temperature.

		read_required = true; // signal to the pico that we need to read from the given address.
		break;
				}
	case I2C_SLAVE_REQUEST: {
		// Read the stored temperature ticks
		i2c_write_byte(i2c, (uint8_t)(temperature_ticks >> 4)); // MSD-first
		i2c_write_byte(i2c, (uint8_t)(temperature_ticks & 0xf));
		break;
	}
	case I2C_SLAVE_FINISH: {
		break;
	}
    }
}

void i2c_slave_init(i2c_inst_t *i2c, uint8_t address, i2c_slave_handler_t handler) {
    mp_printf(MP_PYTHON_PRINTER, "Starting I2C slave setup\n");

    i2c = i2c0;

    gpio_init(4); // 4
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_pull_up(4);

    gpio_init(5); // 5
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(5); // PICO_DEFAULT_I2C_SCL_PIN

    i2c_init(i2c, I2C_FREQ);

    mp_printf(MP_PYTHON_PRINTER, "Set up pins\n");

    assert(i2c == i2c0 || i2c == i2c1);
    assert(handler != NULL);

    uint i2c_index = i2c_hw_index(i2c);
    i2c_slave_t *slave = &i2c_slaves[i2c_index];
    slave->i2c = i2c;
    slave->handler = handler;

    mp_printf(MP_PYTHON_PRINTER, "Created slave instance\n");

    // Note: The I2C slave does clock stretching implicitly after a RD_REQ, while the Tx FIFO is empty.
    // There is also an option to enable clock stretching while the Rx FIFO is full, but we leave it
    // disabled since the Rx FIFO should never fill up (unless slave->handler() is way too slow).
    i2c_set_slave_mode(i2c, true, address);

    i2c_hw_t *hw = i2c_get_hw(i2c);
    // unmask necessary interrupts
    hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS | I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS | I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_START_DET_BITS;

    // enable interrupt for current core
    uint num = I2C0_IRQ + i2c_index;
    irq_set_exclusive_handler(num, i2c_index == 0 ? i2c0_slave_irq_handler : i2c1_slave_irq_handler);
    irq_set_enabled(num, true);
}

// Instead of a proper handler, just takes a void (f*)(uctypes.PTR *i2c, event_type (defined in module))
mp_obj_t i2c_slave_init_(mp_obj_t i2c_, mp_obj_t address_) {
    i2c_inst_t* i2c = i2c_slaves[mp_obj_get_int(i2c_)].i2c;
    uint8_t address = mp_obj_get_int(address_);
    i2c_slave_init(i2c, address, &handler);

    return mp_obj_new_int(0);
}

static MP_DEFINE_CONST_FUN_OBJ_2(i2c_slave_init_obj, i2c_slave_init_);

void i2c_slave_deinit(i2c_inst_t *i2c) {
    i2c = i2c0;

    assert(i2c == i2c0 || i2c == i2c1);

    uint i2c_index = i2c_hw_index(i2c);
    i2c_slave_t *slave = &i2c_slaves[i2c_index];
    assert(slave->i2c == i2c); // should be called after i2c_slave_init()

    slave->i2c = NULL;
    slave->handler = NULL;
    slave->transfer_in_progress = false;

    uint num = I2C0_IRQ + i2c_index;
    irq_set_enabled(num, false);
    irq_remove_handler(num, i2c_index == 0 ? i2c0_slave_irq_handler : i2c1_slave_irq_handler);

    i2c_hw_t *hw = i2c_get_hw(i2c);
    hw->intr_mask = I2C_IC_INTR_MASK_RESET;

    i2c_set_slave_mode(i2c, false, 0);
}

mp_obj_t i2c_slave_deinit_(mp_obj_t i2c_) {
    i2c_inst_t* i2c = i2c_slaves[mp_obj_get_int(i2c_)].i2c;
    i2c_slave_deinit(i2c);

    return mp_obj_new_int(0);
}

static MP_DEFINE_CONST_FUN_OBJ_1(i2c_slave_deinit_obj, i2c_slave_deinit_);

static const mp_rom_map_elem_t slave_i2c_module_globals_table[] = {
	{ MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_i2c_slave) },
	{ MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&i2c_slave_init_obj) },
	{ MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&i2c_slave_deinit_obj) },
	{ MP_ROM_QSTR(MP_QSTR_is_read_required), MP_ROM_PTR(&is_read_required_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_read_required), MP_ROM_PTR(&set_read_required_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_temperature_ticks), MP_ROM_PTR(&set_temperature_ticks_obj) },
	{ MP_ROM_QSTR(MP_QSTR_get_bundle), MP_ROM_PTR(&get_bundle_obj) },
	{ MP_ROM_QSTR(MP_QSTR_get_sensor), MP_ROM_PTR(&get_sensor_obj) },
	{ MP_ROM_QSTR(MP_QSTR_I2C_FREQ), MP_ROM_INT(I2C_FREQ) },
};

static MP_DEFINE_CONST_DICT(slave_i2c_module_globals, slave_i2c_module_globals_table);

const mp_obj_module_t i2c_slave_module = {
	.base = { &mp_type_module },
	.globals = (mp_obj_dict_t *)&slave_i2c_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_i2c_slave, i2c_slave_module);
