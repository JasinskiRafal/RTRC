#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

namespace rtos
{
    namespace atomic
    {
        template <typename Type>
        class Variable
        {
            StaticSemaphore_t _buffer;
            SemaphoreHandle_t _mutex = xSemaphoreCreateMutexStatic(&_buffer);
            Type _value;

        public:
            Type get()
            {
                Type ret;
                xSemaphoreTake(_mutex, pdMS_TO_TICKS(portMAX_DELAY));
                ret = _value;
                xSemaphoreGive(_mutex);
                return ret;
            }

            void set(Type value)
            {
                xSemaphoreTake(_mutex, pdMS_TO_TICKS(portMAX_DELAY));
                _value = value;
                xSemaphoreGive(_mutex);
            }

            void operator=(Type value)
            {
                set(value);
            }
            Type *operator->()
            {
                return &_value;
            }
        };
    } // namespace atomic
} // namespace rtos
