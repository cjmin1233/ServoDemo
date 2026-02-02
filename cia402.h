#ifndef CIA402_H
#define CIA402_H

/**
 * CiA402 Standard Definitions for EtherCAT Servo Drives
 * Optimized for L7NH and similar Servo Drives
 * Organized in namespace and enum class for C++ safety.
 */
#include <cstdint>

namespace cia402 {

// --- [1] Control Word Commands (0x6040) ---
constexpr uint16_t CW_SHUTDOWN    = 0x0006; // Ready to Switch On
constexpr uint16_t CW_SWITCH_ON   = 0x0007; // Switched On
constexpr uint16_t CW_ENABLE_OP   = 0x000F; // Operation Enabled (Servo ON)
constexpr uint16_t CW_FAULT_RESET = 0x0080;

// --- [2] Status Word Masks & States (0x6041) ---
constexpr uint16_t SW_STATE_MASK1 = 0x004F;
constexpr uint16_t SW_STATE_MASK2 = 0x006F;

constexpr uint16_t SW_STATE_NOT_READY          = 0x0000;
constexpr uint16_t SW_STATE_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t SW_STATE_READY_SWITCH_ON    = 0x0021;
constexpr uint16_t SW_STATE_SWITCHED_ON        = 0x0023;
constexpr uint16_t SW_STATE_OP_ENABLED         = 0x0027;
constexpr uint16_t SW_STATE_QUICK_STOP_ACTIVE  = 0x0007;
constexpr uint16_t SW_STATE_FAULT_REACTION     = 0x000F;
constexpr uint16_t SW_STATE_FAULT              = 0x0008;

// --- Control Word (0x6040) 주요 개별 비트 마스크 ---
constexpr uint16_t CW_BIT_NEW_SETPOINT           = (1 << 4); // 4번 비트: 신규 목표값 적용 (PP 모드, Rising edge)
constexpr uint16_t CW_BIT_HOMING_START           = (1 << 4); // 4번 비트: 원점 복귀 시작 (HM 모드)
constexpr uint16_t CW_BIT_CHANGE_SET_IMMEDIATELY = (1 << 5); // 5번 비트: 즉시 변경 여부
constexpr uint16_t CW_BIT_ABS_REL                = (1 << 6); // 6번 비트: 절대(0) / 상대(1) 위치 제어
constexpr uint16_t CW_BIT_HALT                   = (1 << 8); // 8번 비트: 일시 정지

// Status Word Bit Flags
constexpr uint16_t SW_BIT_VOLTAGE_ENABLED = (1 << 4);
constexpr uint16_t SW_BIT_WARNING_OCCURED = (1 << 7);
constexpr uint16_t SW_BIT_TARGET_REACHED  = (1 << 10);
constexpr uint16_t SW_BIT_SET_POINT_ACK   = (1 << 12); // PP 모드에서 New Setpoint 수신 확인

// --- Homing Mode (HM) 관련 비트 ---
constexpr uint16_t SW_BIT_HOMING_ATTAINED = (1 << 12); // homing 완료
constexpr uint16_t SW_BIT_HOMING_ERROR    = (1 << 13); // homing 에러 발생

// --- [3] Modes of Operation (0x6060) ---
enum class Mode : int8_t {
    PP  = 1, // Profile Position Mode
    PV  = 3, // Profile Velocity Mode
    PT  = 4, // Profile Torque Mode
    HM  = 6, // Homing Mode
    CSP = 8, // Cyclic Synchronous Position Mode
    CSV = 9, // Cyclic Synchronous Velocity Mode
    CST = 10 // Cyclic Synchronous Torque Mode
};

// --- [4] Object Dictionary Index ---
constexpr uint16_t IDX_CONTROL_WORD     = 0x6040;
constexpr uint16_t IDX_STATUS_WORD      = 0x6041;
constexpr uint16_t IDX_OP_MODE          = 0x6060;
constexpr uint16_t IDX_OP_MODE_DISPLAY  = 0x6061;
constexpr uint16_t IDX_TARGET_POSITION  = 0x607A;
constexpr uint16_t IDX_ACTUAL_POSITION  = 0x6064;
constexpr uint16_t IDX_POSITION_WINDOW  = 0x6067;
constexpr uint16_t IDX_PROFILE_VELOCITY = 0x6081;
constexpr uint16_t IDX_PROFILE_ACCEL    = 0x6083;
constexpr uint16_t IDX_PROFILE_DECEL    = 0x6084;
constexpr uint16_t IDX_STOP_DECEL       = 0x6085;
constexpr uint16_t IDX_DIGITAL_INPUTS   = 0x60FD;
// constexpr uint16_t IDX_ENCODER_RESOLUTION = 0x2002;

constexpr uint16_t IDX_HOME_OFFSET   = 0x607C;
constexpr uint16_t IDX_HOMING_METHOD = 0x6098;
constexpr uint16_t IDX_HOMING_SPEED  = 0x6099;
constexpr uint16_t IDX_HOMING_ACCEL  = 0x609A;

// --- [5] RxPDO Mapping Objects (Master -> Slave) ---
// 실제 어떤 데이터를 받을지 리스트를 작성하는 공간입니다.
constexpr uint16_t IDX_RXPDO_MAPPING_1 = 0x1600;
constexpr uint16_t IDX_RXPDO_MAPPING_2 = 0x1601;
constexpr uint16_t IDX_RXPDO_MAPPING_3 = 0x1602;
constexpr uint16_t IDX_RXPDO_MAPPING_4 = 0x1603;

// --- [6] TxPDO Mapping Objects (Slave -> Master) ---
// 슬레이브가 어떤 데이터를 보낼지 리스트를 작성하는 공간입니다.
constexpr uint16_t IDX_TXPDO_MAPPING_1 = 0x1A00;
constexpr uint16_t IDX_TXPDO_MAPPING_2 = 0x1A01;
constexpr uint16_t IDX_TXPDO_MAPPING_3 = 0x1A02;
constexpr uint16_t IDX_TXPDO_MAPPING_4 = 0x1A03;

// --- [7] Sync Manager PDO Assignment Objects ---
// 위에서 만든 매핑 리스트(1600, 1A00 등)를 실제 통신 채널(SM)에 할당합니다.
constexpr uint16_t IDX_SM2_RXPDO_ASSIGN = 0x1C12; // SM2: 출력(Rx) 담당
constexpr uint16_t IDX_SM3_TXPDO_ASSIGN = 0x1C13; // SM3: 입력(Tx) 담당

// --- [참고] 매핑 시 사용되는 Sub-Index 규칙 ---
// Sub-Index 0: 해당 매핑 객체에 담긴 오브젝트의 개수 (uint8_t)
// Sub-Index 1~n: 실제 매핑될 오브젝트 정보 (uint32_t: Index/Subindex/BitLen 조합)

// --- [8] RxPDO Mapping Entries (Master -> Slave) ---
// 형식: Index(16bit) + SubIndex(8bit) + BitLen(8bit)

// Control Word (0x6040:00, 16bit) -> 0x60400010
constexpr uint32_t ENTRY_RX_CONTROL_WORD = 0x60400010;

// Modes of Operation (0x6060:00, 8bit) -> 0x60600008
constexpr uint32_t ENTRY_RX_MODES_OF_OP = 0x60600008;

// Target Position (0x607A:00, 32bit) -> 0x607A0020
constexpr uint32_t ENTRY_RX_TARGET_POSITION = 0x607A0020;

// Target Velocity (0x60FF:00, 32bit) -> 0x60FF0020
constexpr uint32_t ENTRY_RX_TARGET_VELOCITY = 0x60FF0020;

// Digital Outputs (0x60FE:01, 32bit) -> 0x60FE0120
constexpr uint32_t ENTRY_RX_DIGITAL_OUTPUTS = 0x60FE0120;

// --- [9] TxPDO Mapping Entries (Slave -> Master) ---

// Status Word (0x6041:00, 16bit) -> 0x60410010
constexpr uint32_t ENTRY_TX_STATUS_WORD = 0x60410010;

// Modes of Op Display (0x6061:00, 8bit) -> 0x60610008
constexpr uint32_t ENTRY_TX_MODES_OF_OP_DISP = 0x60610008;

// Actual Position (0x6064:00, 32bit) -> 0x60640020
constexpr uint32_t ENTRY_TX_ACTUAL_POSITION = 0x60640020;

// Actual Velocity (0x606C:00, 32bit) -> 0x606C0020
constexpr uint32_t ENTRY_TX_ACTUAL_VELOCITY = 0x606C0020;

// Actual Torque (0x6077:00, 16bit) -> 0x60770010
constexpr uint32_t ENTRY_TX_ACTUAL_TORQUE = 0x60770010;

// Digital Inputs (0x60FD:00, 32bit) -> 0x60FD0020
constexpr uint32_t ENTRY_TX_DIGITAL_INPUTS = 0x60FD0020;

// --- [10] Homing Methods (0x6098) ---
constexpr int8_t HM_NEG_LIMIT_SWITCH_AND_INDEX = 1;  // (-)방향 리미트 스위치 접촉 후 Index Pulse 찾기
constexpr int8_t HM_POS_LIMIT_SWITCH_AND_INDEX = 2;  // (+)방향 리미트 스위치 접촉 후 Index Pulse 찾기
constexpr int8_t HM_HOME_SWITCH_NEG_AND_INDEX  = 11; // (-)방향으로 Home 스위치 접촉 후 Index Pulse 찾기
constexpr int8_t HM_HOME_SWITCH_POS_AND_INDEX  = 7;  // (+)방향으로 Home 스위치 접촉 후 Index Pulse 찾기
constexpr int8_t HM_CURRENT_POS_AS_HOME        = 35; // 현재 위치를 원점으로 설정 (이동 없음)

} // namespace cia402

#endif // CIA402_H
