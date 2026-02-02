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

    // check name
    if (std::string(ec_slave[slaveId].name).find("L7NH") != 0) return 0;

    int success = 1; // SOEM 콜백은 보통 성공 시 1을 기대함

    // --- [STEP 1] RXPDO Mapping Content (0x1600) ---
    uint16_t rxpdoIndex = cia402::IDX_RXPDO_MAPPING_1;
    uint8_t  zero       = 0;

    // 매핑 개수 0으로 초기화 (수정 모드 진입)
    success &= (ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t rxpdoEntries[] = {
        cia402::ENTRY_RX_CONTROL_WORD,
        cia402::ENTRY_RX_MODES_OF_OP,
        cia402::ENTRY_RX_TARGET_POSITION,
        cia402::ENTRY_RX_TARGET_VELOCITY,
        cia402::ENTRY_RX_DIGITAL_OUTPUTS
    };
    uint8_t entryCount = sizeof(rxpdoEntries) / sizeof(rxpdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        success &= (ec_SDOwrite(slaveId, rxpdoIndex, i + 1, FALSE, sizeof(rxpdoEntries[i]), &rxpdoEntries[i], EC_TIMEOUTRXM) > 0);
    }

    // 매핑 개수 확정
    success &= (ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 2] TXPDO Mapping Content (0x1A00) 설정 ---
    uint16_t txpdoIndex  = cia402::IDX_TXPDO_MAPPING_1;
    success             &= (ec_SDOwrite(slaveId, txpdoIndex, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    uint32_t txpdoEntries[] = {
        cia402::ENTRY_TX_STATUS_WORD,
        cia402::ENTRY_TX_MODES_OF_OP_DISP,
        cia402::ENTRY_TX_ACTUAL_POSITION,
        cia402::ENTRY_TX_ACTUAL_VELOCITY,
        cia402::ENTRY_TX_ACTUAL_TORQUE,
        cia402::ENTRY_TX_DIGITAL_INPUTS
    };
    entryCount = sizeof(txpdoEntries) / sizeof(txpdoEntries[0]);

    for (uint8_t i = 0; i < entryCount; ++i) {
        success &= (ec_SDOwrite(slaveId, txpdoIndex, i + 1, FALSE, sizeof(txpdoEntries[i]), &txpdoEntries[i], EC_TIMEOUTRXM) > 0);
    }
    success &= (ec_SDOwrite(slaveId, txpdoIndex, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // --- [STEP 3] Sync Manager 2 (RxPDO) & 3 (TxPDO) Assignment 설정 ---
    // RxPDO
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 1, FALSE, sizeof(rxpdoIndex), &rxpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM2_RXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    // TxPDO
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 1, FALSE, sizeof(txpdoIndex), &txpdoIndex, EC_TIMEOUTRXM) > 0);
    entryCount  = 1;
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_SM3_TXPDO_ASSIGN, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    uint32_t profileVel = s_encoderResolution;
    uint32_t profileAcc = s_encoderResolution * 2;
    uint32_t profileDec = s_encoderResolution * 2;
    uint32_t stopDec    = s_encoderResolution * 10;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_VELOCITY, 0, FALSE, sizeof(profileVel), &profileVel, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_ACCEL, 0, FALSE, sizeof(profileAcc), &profileAcc, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_PROFILE_DECEL, 0, FALSE, sizeof(profileDec), &profileDec, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_STOP_DECEL, 0, FALSE, sizeof(stopDec), &stopDec, EC_TIMEOUTRXM) > 0);

    // Set homing objects
    int32_t  homeOffset   = 0;
    int8_t   homingMethod = cia402::HM_NEG_LIMIT_SWITCH_AND_INDEX;
    uint32_t spdSwitch    = s_encoderResolution;
    uint32_t spdZero      = s_encoderResolution / 10;
    uint32_t homingAcc    = s_encoderResolution * 2;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOME_OFFSET, 0, FALSE, sizeof(homeOffset), &homeOffset, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_METHOD, 0, FALSE, sizeof(homingMethod), &homingMethod, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_SPEED, 0, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_SPEED, 1, FALSE, sizeof(spdSwitch), &spdSwitch, EC_TIMEOUTRXM) > 0);
    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_SPEED, 2, FALSE, sizeof(spdZero), &spdZero, EC_TIMEOUTRXM) > 0);

    entryCount  = 2;
    success    &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_SPEED, 0, FALSE, sizeof(entryCount), &entryCount, EC_TIMEOUTRXM) > 0);

    success &= (ec_SDOwrite(slaveId, cia402::IDX_HOMING_ACCEL, 0, FALSE, sizeof(homingAcc), &homingAcc, EC_TIMEOUTRXM) > 0);

    // etc...
    uint32_t posWindow = 100;

    success &= (ec_SDOwrite(slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, sizeof(posWindow), &posWindow, EC_TIMEOUTRXM) > 0);

    std::cout << "[ServoL7NH::setupL7NH] Result: " << (success ? "Success" : "Failed") << std::endl;
    return success;
}

void ServoL7NH::processData()
{
    auto* rxpdo       = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);
    auto& controlWord = rxpdo->control_word;

    const auto* txpdo       = reinterpret_cast<TxPDO*>(ec_slave[m_slaveId].inputs);
    const auto& statusWord  = txpdo->status_word;
    const auto& currentMode = static_cast<cia402::Mode>(txpdo->mode_disp);

    if ((statusWord & cia402::SW_STATE_MASK2) == cia402::SW_STATE_OP_ENABLED) {
        const bool isTargetReached = statusWord & cia402::SW_BIT_TARGET_REACHED;

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
        case cia402::Mode::PV:
        case cia402::Mode::PT:
            break;
        case cia402::Mode::HM: {
            processHM(rxpdo, txpdo);

            break;
        }
        case cia402::Mode::CSP:
        case cia402::Mode::CST:
        case cia402::Mode::CSV:
            break;
        default:
            std::cout << "[ServoL7NH::processData] invalid mode : " << static_cast<int8_t>(currentMode) << std::endl;
            break;
        }

        if (isTargetReached) {
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
    rxpdo->mode          = static_cast<int8_t>(cia402::Mode::HM);
    rxpdo->control_word &= ~(cia402::CW_BIT_ABS_REL); // Absolute move

    int psize = sizeof(m_posWindow);
    ec_SDOread(m_slaveId, cia402::IDX_POSITION_WINDOW, 0, FALSE, &psize, &m_posWindow, EC_TIMEOUTRXM);

    m_flagHomingStart = true;
}

void ServoL7NH::stop()
{
    RxPDO* rxpdo        = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);
    rxpdo->control_word = cia402::CW_SHUTDOWN; // 0x0006
}

void ServoL7NH::setTargetPosition(float ratio)
{
    static constexpr int32_t maxPosition = s_encoderResolution * 4;

    int32_t pos = static_cast<int32_t>(ratio * (maxPosition));

    setTargetPosition(pos);
}

void ServoL7NH::setTargetPosition(int32_t pos)
{
    RxPDO* rxpdo = reinterpret_cast<RxPDO*>(ec_slave[m_slaveId].outputs);

    rxpdo->mode             = static_cast<int8_t>(cia402::Mode::PP);
    rxpdo->target_position  = pos;
    rxpdo->control_word    &= ~(cia402::CW_BIT_ABS_REL); // Absolute move

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

void ServoL7NH::processHM(RxPDO* rxpdo, const TxPDO* txpdo)
{
    //
    static constexpr int HOMING_SETTLING_TIMEOUT = 5000;
    static constexpr int HOMING_STABLE_COUNT     = 50;

    auto&       controlWord = rxpdo->control_word;
    const auto& statusWord  = txpdo->status_word;
    const auto& currentMode = static_cast<cia402::Mode>(txpdo->mode_disp);

    // Homing
    const bool isHomingStart    = controlWord & cia402::CW_BIT_HOMING_START;
    const bool isHomingAttained = statusWord & cia402::SW_BIT_HOMING_ATTAINED;
    const bool isHomingError    = statusWord & cia402::SW_BIT_HOMING_ERROR;

    if (isHomingError) {
        std::cout << "[ServoL7NH::processData] homing error occurred!" << std::endl;

        // error handling logic
        controlWord        &= ~(cia402::CW_BIT_HOMING_START);
        m_flagHomingStart   = false;
        m_isHomingSettling  = false;
        return;
    }

    // 1. Homing start request
    if (!isHomingStart && m_flagHomingStart) {
        controlWord |= cia402::CW_BIT_HOMING_START;

        m_flagHomingStart  = false;
        m_isHomingSettling = false; // Homing 새로 시작 시 안정화 상태 초기화
        return;
    }

    // 2. Homing processing
    if (isHomingStart) {
        // 2-1. Homing Attained, 안정화 단계 진입
        if (isHomingAttained && !m_isHomingSettling) {
            std::cout << "[ServoL7NH::processHM] Homing attained. Start settling check..." << std::endl;

            m_isHomingSettling      = true;
            m_homingSettlingCounter = HOMING_SETTLING_TIMEOUT;
            m_homingStableCounter   = 0;
        }

        // 2-2. 안정화 단계 로직
        if (m_isHomingSettling) {
            --m_homingSettlingCounter; // 타임아웃 카운터 감소

            const int32_t  pos    = txpdo->actual_position;
            const uint32_t absPos = pos < 0 ? -pos : pos;

            // Position Window 내에 있는지 확인
            if (absPos <= m_posWindow) {
                ++m_homingStableCounter; // 안정화 카운터 증가
            } else {
                m_homingStableCounter = 0; // 벗어나면 리셋
            }

            // 안정화 성공: 충분한 시간 동안 Window 내에 머무름
            if (m_homingStableCounter >= HOMING_STABLE_COUNT) {
                std::cout << "[ServoL7NH::processHM] Homing Succeeded (Stable in window)" << std::endl;

                controlWord        &= ~(cia402::CW_BIT_HOMING_START);
                m_isHomingSettling  = false;
            }
            // 타임아웃 실패: 시간 내에 안정화되지 못함
            else if (m_homingSettlingCounter <= 0) {
                std::cout << "[ServoL7NH::processHM] Homing Failed (Timeout, not stable)" << std::endl;

                controlWord        &= ~(cia402::CW_BIT_HOMING_START);
                m_isHomingSettling  = false;
            }
        }
    }
}
