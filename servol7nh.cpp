#include <chrono>
#include <iostream>
#include <thread>

#include "servol7nh.h"

void ServoL7NH::processData()
{
}

void ServoL7NH::start()
{
    auto* controlWord = reinterpret_cast<uint16_t*>(ec_slave[m_slaveId].outputs);

    // Switch on Disabled -> Ready to Switch on
    *controlWord = 0x0006;
    if (!waitForState(0x0061, 0x0021)) {
        std::cout << "Failed to reach 'Ready to Switch On' state." << std::endl;
    }

    // Ready to Switch on -> Switched on
    *controlWord = 0x0007;
    if (!waitForState(0x0023, 0x0023)) {
        std::cout << "Failed to reach 'Switched On' state." << std::endl;
    }

    // Switched on -> Operation
    *controlWord = 0x000F;
    if (!waitForState(0x0027, 0x0027)) {
        std::cout << "Failed to reach 'Operation Enabled' state." << std::endl;
    }
}

void ServoL7NH::stop()
{
    auto* controlWord = reinterpret_cast<uint16_t*>(ec_slave[m_slaveId].outputs);

    *controlWord = 0x0006;
}

bool ServoL7NH::waitForState(uint16_t statusMask, uint16_t expectedStatus, int timeOutMs)
{
    constexpr int cycleTimeUs = 1000; // 1ms

    auto* statusWord
        = reinterpret_cast<uint16_t*>(ec_slave[m_slaveId].inputs);
    auto start = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeOutMs) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        if ((*statusWord & statusMask) == expectedStatus) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }

    std::cout << "[ServoL7NH::waitForState] Timeout waiting for state. Current status : 0x" << std::hex
              << static_cast<uint16_t>(*statusWord) << std::endl;
    return false;
}
