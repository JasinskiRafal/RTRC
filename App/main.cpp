#include <array>
#include <vector>

#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <pico/i2c_slave.h>

#include "task.hpp"
#include "atomic.hpp"
#include "memory.hpp"

constexpr uint I2C_BAUDRATE = 100 * 1000;
constexpr uint I2C_ADDRESS = 33;

constexpr uint I2C_SDA_PIN = 18;
constexpr uint I2C_SCL_PIN = 19;

constexpr uint PIN_MASK = (1 << 16) - 1;

struct relay_metadata
{
    uint pin_number;
    std::uint16_t *delay_time;
};

memory::Layout rtrc_registers;

void i2c_setup(void);

void blinker_callback(void *);
void main_controller_callback(void *);

int main()
{
    bool start_scheduler = false;

    i2c_setup();
    rtos::task::Dynamic blinker_task("Blinker Task", blinker_callback);
    rtos::task::Dynamic relay_controller_task("Controller Task", main_controller_callback);

    if (relay_controller_task)
    {
        start_scheduler = true;
    }

    if (start_scheduler)
    {
        vTaskStartScheduler();
    }

    while (true)
    {
        // NOP
    };
}

void i2c_slave_irq_handler(i2c_inst_t *i2c, i2c_slave_event_t event);

void i2c_setup(void)
{
    uint32_t set_baud;
    set_baud = i2c_init(i2c0, I2C_BAUDRATE);

    i2c_slave_init(i2c0, I2C_ADDRESS, &i2c_slave_irq_handler);

    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);

    set_baud = i2c_init(i2c1, I2C_BAUDRATE);
    hard_assert(set_baud == I2C_BAUDRATE);
}

void i2c_slave_irq_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    static bool address_received = false;
    static uint8_t register_address = 0;

    switch (event)
    {
    case I2C_SLAVE_RECEIVE:
        if (!address_received)
        {
            register_address = i2c_read_byte_raw(i2c);
            address_received = true;
        }
        else
        {
            if (register_address < rtrc_registers.bytes.size())
            {
                rtrc_registers.bytes[register_address++] = i2c_read_byte_raw(i2c);
            }
        }
        break;
    case I2C_SLAVE_REQUEST:
        if (register_address < rtrc_registers.bytes.size())
        {
            i2c_write_byte_raw(i2c, rtrc_registers.bytes[register_address++]);
        }
        break;

    case I2C_SLAVE_FINISH:
        address_received = false;
        register_address = 0;
        break;
    default:
        break;
    }
}

void blinker_callback(void *)
{
    bool state = false;
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);
    gpio_pull_up(PICO_DEFAULT_LED_PIN);

    while (1)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, state);
        state = !state;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


relay_metadata metadata[16] =
    {
        {0, &rtrc_registers.registers.time_registers[0]},
        {1, &rtrc_registers.registers.time_registers[1]},
        {2, &rtrc_registers.registers.time_registers[2]},
        {3, &rtrc_registers.registers.time_registers[3]},
        {4, &rtrc_registers.registers.time_registers[4]},
        {5, &rtrc_registers.registers.time_registers[5]},
        {6, &rtrc_registers.registers.time_registers[6]},
        {7, &rtrc_registers.registers.time_registers[7]},
        {8, &rtrc_registers.registers.time_registers[8]},
        {9, &rtrc_registers.registers.time_registers[9]},
        {10, &rtrc_registers.registers.time_registers[10]},
        {11, &rtrc_registers.registers.time_registers[11]},
        {12, &rtrc_registers.registers.time_registers[12]},
        {13, &rtrc_registers.registers.time_registers[13]},
        {14, &rtrc_registers.registers.time_registers[14]},
        {15, &rtrc_registers.registers.time_registers[15]},
};

void relay_controller_callback(void *);

void main_controller_callback(void *)
{
    gpio_init_mask(PIN_MASK);
    gpio_set_dir_out_masked(PIN_MASK);

    std::array<rtos::task::Dynamic, 16> relay_tasks{
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[0]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[1]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[2]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[3]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[4]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[5]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[6]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[7]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[8]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[9]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[10]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[11]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[12]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[13]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[14]},
        rtos::task::Dynamic{"Relay", relay_controller_callback, &metadata[15]}};

    while (1)
    {
        if (rtrc_registers.registers.status > 0)
        {
            for (auto &task : relay_tasks)
            {
                vTaskResume(task.get_handle());
            }
        }
    }
}

void relay_controller_callback(void *argument)
{
    relay_metadata data = *(relay_metadata *)argument;

    while (1)
    {
        vTaskSuspend(NULL);
        if (data.delay_time > 0)
        {
            gpio_put(data.pin_number, true);
            vTaskDelay(pdMS_TO_TICKS(data.delay_time));
            gpio_put(data.pin_number, false);
        }
    }
}
