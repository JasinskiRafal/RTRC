#pragma once

#include <cstdint>
#include <array>

namespace memory
{
    union Layout
    {
        struct Registers
        {
            std::array<std::uint16_t, 16> time_registers;
            std::uint8_t status;
        } registers;
        std::array<std::uint8_t, sizeof(Registers)> bytes;
    };

} // namespace memory
