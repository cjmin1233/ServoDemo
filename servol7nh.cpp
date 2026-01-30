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
    auto*     rxpdo       = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);
    uint16_t& controlWord = rxpdo->control_word;

    const TxPDO*    txpdo       = reinterpret_cast<TxPDO*>(ec_slave[m_slaveId].inputs);
    const uint16_t& statusWord  = txpdo->status_word;
    const auto&     currentMode = static_cast<cia402::Mode>(txpdo->modes_of_op_disp);

    if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_OP_ENABLED) {
        // main operation
        switch (currentMode) {
        case cia402::Mode::PP: {
            // Profile position
            const bool isNewSetpointRequested = controlWord & cia402::CW_BIT_NEW_SETPOINT;
            const bool isSetpointAck          = statusWord & cia402::SW_BIT_SET_POINT_ACK;

            if (isNewSetpointRequested) {
                if (isSetpointAck) {
                    // new setpoint requested and acknowledged
                    controlWord &= ~(cia402::CW_BIT_NEW_SETPOINT);
                }
            } else if (m_flagNewSetpoint) {
                // request new setpoint
                controlWord |= cia402::CW_BIT_NEW_SETPOINT;
                // flag off
                m_flagNewSetpoint = false;
            }

            break;
        }
        case cia402::Mode::HM: {
            // Homing
            break;
        }
        default:
            break;
        }

        if (statusWord & cia402::SW_BIT_TARGET_REACHED) {
            // target reached
        }
    }
    // check state machine if not operational yet
    else {
        stateCheck();
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

    rxpdo->modes_of_op      = static_cast<int8_t>(cia402::Mode::PP);
    rxpdo->target_position  = pos;
    rxpdo->control_word    |= ~(cia402::CW_BIT_ABS_REL); // absolute move

    m_flagNewSetpoint = true;
}

void ServoL7NH::stateCheck()
{
    static constexpr int      stateCheckCycleCounter = 20;
    static constexpr uint16_t bit4to7                = 0xF0;
    static constexpr uint16_t bit0to3                = 0x0F;

    if (ec_slave[m_slaveId].state != EC_STATE_OPERATIONAL) {
        return;
    }

    if (m_stateCheckCounter > 0) {
        --m_stateCheckCounter;
        return;
    }

    uint16_t&       controlWord = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs)->control_word;
    const uint16_t& statusWord  = reinterpret_cast<TxPDO*>(ec_slave[m_slaveId].inputs)->status_word;

    if ((statusWord & cia402::SW_STATE_MASK1) == cia402::SW_STATE_SWITCH_ON_DISABLED) {
        controlWord = controlWord & bit4to7 | cia402::CW_SHUTDOWN;

        m_stateCheckCounter = stateCheckCycleCounter;
    } else if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_READY_SWITCH_ON) {
        controlWord = controlWord & bit4to7 | cia402::CW_SWITCH_ON;

        m_stateCheckCounter = stateCheckCycleCounter;
    } else if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_SWITCHED_ON) {
        if ((controlWord & bit0to3) == cia402::CW_ENABLE_OP) {
            // Already tried to enable op. Drop to shutdown
            controlWord = controlWord & bit4to7 | cia402::CW_SHUTDOWN;

            std::cout << "[ServoL7NH::stateCheck] Already tried to enable op. Drop to shutdown" << std::endl;
        } else {
            controlWord = controlWord & bit4to7 | cia402::CW_ENABLE_OP;
        }

        m_stateCheckCounter = stateCheckCycleCounter * 2;
    }
}
