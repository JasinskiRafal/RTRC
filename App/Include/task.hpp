#pragma once

#include <FreeRTOS.h>
#include <task.h>

namespace rtos
{
    namespace task
    {
        class ITask
        {
        public:
            virtual bool is_created() = 0;
            virtual operator bool() = 0;
            virtual ~ITask() = default;
        };

        class Dynamic : public ITask
        {
            TaskHandle_t _handle;
            bool _is_created;

        public:
            Dynamic(const char *name,
                    TaskFunction_t callback,
                    void *parameter = NULL,
                    UBaseType_t priority = tskIDLE_PRIORITY,
                    uint16_t stack_size = configMINIMAL_STACK_SIZE)
            {
                _is_created = (xTaskCreate(callback, name, stack_size, parameter, priority, &_handle) == pdPASS);
            }

            ~Dynamic()
            {
                vTaskDelete(NULL);
            }
            Dynamic(const Dynamic &) = delete;
            Dynamic &operator=(const Dynamic &) = delete;
            Dynamic(Dynamic &&) = default;
            Dynamic &operator=(Dynamic &&) = default;

            TaskHandle_t get_handle() { return _handle; }

            bool is_created() override { return _is_created; }
            operator bool() override { return _is_created; }
        };

        template <size_t Size>
        class Static : public ITask
        {
            TaskHandle_t _handle;
            StaticTask_t _task;
            StackType_t _buffer[Size];
            bool _is_created = false;

        public:
            Static(const char *name,
                   TaskFunction_t callback,
                   void *parameter = NULL,
                   uint16_t priority = tskIDLE_PRIORITY,
                   uint32_t stack_size = configMINIMAL_STACK_SIZE)
            {
                if (stack_size > Size)
                {
                    return;
                }

                _handle = xTaskCreateStatic(callback, name, stack_size, parameter, priority, _buffer, &_task);

                _is_created = (_handle != NULL);
            }

            ~Static()
            {
                vTaskDelete(NULL);
            }
            Static(const Static &) = delete;
            Static &operator=(const Static &) = delete;
            Static(Static &&) = default;
            Static &operator=(Static &&) = default;

            bool is_created() override { return _is_created; }
            operator bool() override { return _is_created; }
        };
    } // namespace task
} // namespace rtos
