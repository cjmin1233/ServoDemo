#ifndef ECATMASTER_H
#define ECATMASTER_H

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "slave.h"

extern "C" {
#include "ethercat.h"
}

class EcatMaster {
public:
    EcatMaster() = default;
    ~EcatMaster()
    {
        if (m_Running) {
            stop();
        }
    }

    bool init(const std::string& ifname);
    bool start();
    void stop();

    void launchServoMove(float ratio);

private:
    void processLoop();
    bool reqOpState();
    void ecatCheck();
    void slavesCheck();

    bool checkL7NH(const ec_slavet& slave)
    {
        return slave.eep_man == 0x00007595
               && slave.eep_id == 0x00010001;
    }

private:
    std::atomic<bool> m_Running { false };

    std::thread m_Worker;
    std::thread m_ErrorHandler;

    char m_IOmap[4096] = {};

    int              m_ExpectedWKC = 0;
    std::atomic<int> m_CurrentWKC { 0 };
    int              m_CurrentGroup = 0;

    std::vector<std::unique_ptr<Slave>> m_Slaves = {};
};

#endif // ECATMASTER_H
