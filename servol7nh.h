#ifndef SERVOL7NH_H
#define SERVOL7NH_H

#include "slave.h"

// EtherCAT CiA402 Modes of Operation (0x6060)
enum class ModeServoL7NH : int8_t {
    ProfilePosition    = 1, // PP : Profile Position Mode
    Velocity           = 3, // PV : Profile Velocity Mode
    Torque             = 4, // PT : Profile Torque Mode
    Homing             = 6, // HM : Homing Mode
    CyclicSyncPosition = 8, // CSP : Cyclic Synchronous Position
    CyclicSyncVelocity = 9, // CSV : Cyclic Synchronous Velocity
    CyclicSyncTorque   = 10 // CST : Cyclic Synchronous Torque
};

class EcatMaster;

class ServoL7NH : public Slave {
public:
    ServoL7NH(uint16_t slaveId)
        : Slave(slaveId)
    {
    }

    virtual void processData() override;

    virtual void start() override;
    virtual void stop() override;

    void setTargetPosition(int32_t pos);

private:
#pragma pack(push, 1)
    struct Outputs_PP {
        uint16_t controlWord;
        int32_t  targetPos;
        uint32_t profileVel;
        uint32_t profileAcc;
        uint32_t profileDec;
    };
#pragma pack(pop)

private:
    bool waitForState(uint16_t statusMask, uint16_t expectedStatus, int timeOutMs = 10'000);
};

#endif // SERVOL7NH_H
