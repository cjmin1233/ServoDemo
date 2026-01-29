#include <chrono>
#include <iostream>
#include <thread>

#include "cia402.h"
#include "servol7nh.h"

bool ServoL7NH::checkL7NH(int slaveId)
{
    const auto& slave = ec_slave[slaveId];
    return slave.eep_man == 0x00007595
           && slave.eep_id == 0x00010001;
}

int ServoL7NH::setupL7NH(uint16 slaveId)
{
    std::cout << "[ServoL7NH::setupL7NH] Setup servo " << slaveId << " start" << std::endl;

    // 1. 이름 체크
    if (std::string(ec_slave[slaveId].name).find("L7NH") != 0) return 0;

    int success = 1; // SOEM 콜백은 보통 성공 시 1을 기대함

    // --- [STEP 1] RXPDO Mapping Content (0x1600) 설정 ---
    // 내용을 먼저 채워야 합니다!
    uint16_t rxpdoIndex = cia402::IDX_RXPDO_MAPPING_1;
    uint8_t  zero       = 0;

    // 매핑 개수 0으로 초기화 (수정 모드 진입)
    success &= (ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t pdoEntries[] = {
        cia402::ENTRY_RX_CONTROL_WORD,
        cia402::ENTRY_RX_MODES_OF_OP,
        cia402::ENTRY_RX_TARGET_POSITION,
        cia402::ENTRY_RX_TARGET_VELOCITY,
        cia402::ENTRY_RX_DIGITAL_OUTPUTS
    };
    uint8_t entryCount = sizeof(pdoEntries) / sizeof(pdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        success &= (ec_SDOwrite(slaveId, rxpdoIndex, i + 1, FALSE, sizeof(pdoEntries[i]), &pdoEntries[i], EC_TIMEOUTRXM) > 0);
    }

    // 매핑 개수 확정
    success &= (ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 2] Sync Manager 2 Assignment (0x1C12) 설정 ---
    // 내용이 채워진 0x1600을 실제 채널에 연결합니다.
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 1, FALSE, sizeof(rxpdoIndex), &rxpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    uint32_t profileVel = 1'310'720;
    uint32_t profileAcc = 2'621'440;
    uint32_t profileDec = 2'621'440;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_VELOCITY, 0, FALSE, sizeof(profileVel), &profileVel, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_ACCEL, 0, FALSE, sizeof(profileAcc), &profileAcc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_DECEL, 0, FALSE, sizeof(profileDec), &profileDec, EC_TIMEOUTRXM) > 0);

    std::cout << "[ServoL7NH::setupL7NH] Result: " << (success ? "Success" : "Failed") << std::endl;
    return success;
}
void ServoL7NH::processData()
{
    stateCheck();

    const uint16_t& statusWord = reinterpret_cast<TxPDO*>(ec_slave[m_slaveId].inputs)->status_word;
    // std::cout << "[ServoL7NH::processData] servo status: " << *statusWord << std::endl;

    // bool isFault = *statusWord & (1 << 3);
    // if (isFault) {
    //     std::cout << "[ServoL7NH::processData] servo fault detected" << std::endl;
    // }

    bool isTargetReached = statusWord & (cia402::SW_BIT_TARGET_REACHED);

    if (isTargetReached) {
        auto* rxpdo = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);

        // setRelativeMove(false);

        // rxpdo->control_word |= (1 << 5);
        rxpdo->control_word &= ~(cia402::CW_BIT_NEW_SETPOINT);

        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);

        rxpdo->control_word |= (cia402::CW_BIT_NEW_SETPOINT);
    }
}

void ServoL7NH::start()
{
    RxPDO* rxpdo         = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);
    rxpdo->modes_of_op   = static_cast<int8_t>(cia402::Mode::PP);
    rxpdo->control_word &= ~(cia402::CW_BIT_ABS_REL);
}

void ServoL7NH::stop()
{
    RxPDO* rxpdo        = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);
    rxpdo->control_word = cia402::CW_SHUTDOWN;
}

void ServoL7NH::setTargetPosition(int32_t pos)
{
    RxPDO* rxpdo = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);

    rxpdo->target_position = pos;
}

// void ServoL7NH::setRelativeMove(bool relative)
// {
//     auto* pdo_outputs = reinterpret_cast<Outputs_PP*>(ec_slave[m_slaveId].outputs);

//     if (relative) {
//         pdo_outputs->controlWord |= 1 << 6;
//     } else {
//         pdo_outputs->controlWord &= ~(1 << 6);
//     }
// }

void ServoL7NH::stateCheck()
{
    if (ec_slave[m_slaveId].state != EC_STATE_OPERATIONAL) {
        return;
    }

    if (--m_stateCheckCounter >= 0) return;

    uint16_t&       controlWord = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs)->control_word;
    const uint16_t& statusWord  = reinterpret_cast<TxPDO*>(ec_slave[m_slaveId].inputs)->status_word;

    if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_OP_ENABLED) {
        // servo op enabled, early return
        return;
    }

    if ((statusWord & cia402::SW_STATE_MASK1) == cia402::SW_STATE_SWITCH_ON_DISABLED) {
        controlWord = controlWord & 0xf0 | cia402::CW_SHUTDOWN;

        m_stateCheckCounter = 20;
    } else if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_READY_SWITCH_ON) {
        controlWord = controlWord & 0xf0 | cia402::CW_SWITCH_ON;

        m_stateCheckCounter = 20;
    } else if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_SWITCHED_ON) {
        if ((controlWord & 0x0f) == cia402::CW_ENABLE_OP) {
            // Already tried to enable op. Drop to shutdown
            controlWord = controlWord & 0xf0 | cia402::CW_SHUTDOWN;

            std::cout << "[ServoL7NH::stateCheck] Already tried to enable op. Drop to shutdown" << std::endl;
        } else {
            controlWord = controlWord & 0xf0 | cia402::CW_ENABLE_OP;
        }
        m_stateCheckCounter = 50;
    }
}
