#include "ecatmaster.h"
#include "servol7nh.h"

#include <iostream>

/** timeout value in us for return "Operational" state */
#define EC_TIMEOUTOP 50000

#define EC_TIMEOUTCONFIG (EC_TIMEOUTSTATE * 4)

int setupL7NH(uint16 slaveId)
{
    std::cout << "[setupL7NH] setup servo" << std::endl;

    std::string slave_name(ec_slave[slaveId].name);
    std::string prefix("L7NH");

    if (slave_name.find(prefix, 0) != 0) {
        return 0;
    }

    int wkc = 0;

    int8_t mode  = static_cast<int8_t>(ModeServoL7NH::Velocity);
    wkc         += ec_SDOwrite(slaveId, 0x6060, 0, FALSE,
                               sizeof(mode), &mode, EC_TIMEOUTRXM);

    uint8_t  nEntries    = 0;
    uint16_t rxpdoIndex  = 0x1601;
    wkc                 += ec_SDOwrite(slaveId, 0x1c12, 0, FALSE, sizeof(nEntries), &nEntries, EC_TIMEOUTRXM);
    wkc                 += ec_SDOwrite(slaveId, 0x1c12, 1, FALSE, sizeof(rxpdoIndex), &rxpdoIndex, EC_TIMEOUTRXM);
    nEntries             = 1;
    wkc                 += ec_SDOwrite(slaveId, 0x1c12, 0, FALSE, sizeof(nEntries), &nEntries, EC_TIMEOUTRXM);

    // pdo mapping
    uint32_t pdoEntries[] = {
        0x60400010, // Controlword (0x6040, sub 0, 16 bits)
        0x607A0020, // Target Position (0x607A, sub 0, 32 bits)
        0x60810020, // Profile Velocity (0x6081, sub 0, 32 bits)
        0x60830020, // Profile Acceleration (0x6083, sub 0, 32 bits)
        0x60840020  // Profile Deceleration (0x6084, sub 0, 32 bits)
    };
    nEntries = 0;

    wkc      += ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(nEntries), &nEntries, EC_TIMEOUTRXM);
    nEntries  = sizeof(pdoEntries) / sizeof(pdoEntries[0]);
    for (int i = 0; i < nEntries; ++i) {
        wkc += ec_SDOwrite(slaveId, rxpdoIndex, i + 1, FALSE, sizeof(pdoEntries[i]), &pdoEntries[i], EC_TIMEOUTRXM);
    }
    wkc += ec_SDOwrite(slaveId, rxpdoIndex, 0, FALSE, sizeof(nEntries), &nEntries, EC_TIMEOUTRXM);

    return wkc;
}

bool EcatMaster::init(const std::string& ifname)
{
    if (!ec_init(ifname.c_str())) {
        std::cout << "[EcatMaster::init] ec_init failed on " << ifname << std::endl;
        return false;
    }

    std::cout << "[EcatMaster::init] ec_init on " << ifname << " succeeded" << std::endl;

    if (ec_config_init(FALSE) <= 0) {
        std::cout << "[EcatMaster::init] No Slaves found" << std::endl;
        ec_close();
        return false;
    }

    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    std::cout << "[EcatMaster::init] " << ec_slavecount << " slaves found" << std::endl;

    m_Slaves.clear();
    m_Slaves.resize(ec_slavecount + 1);
    for (int i = 1; i <= ec_slavecount; ++i) {
        auto& slave = ec_slave[i];

        // servo L7NH
        if (slave.eep_man == 0x00007595
            && slave.eep_id == 0x00010001) {
            // PO2SOconfig
            slave.PO2SOconfig = setupL7NH;

            // make_unique
            m_Slaves[i] = std::make_unique<ServoL7NH>(i);
        } else {
            m_Slaves[i] = nullptr;
        }
    }

    ec_config_map(&m_IOmap);
    ec_configdc();

    std::cout << "[EcatMaster::init] Slaves mapped, state to SAFE_OP" << std::endl;

    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTCONFIG);

    m_ExpectedWKC = (ec_group[m_CurrentGroup].outputsWKC * 2) + ec_group[m_CurrentGroup].inputsWKC;
    std::cout << "[EcatMaster::init] Expected WKC : " << m_ExpectedWKC << std::endl;

    return reqOpState();
}

bool EcatMaster::start()
{
    if (m_Running) {
        return false;
    }

    m_Running = true;

    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] != nullptr) {
            m_Slaves[i]->start();
        }
    }

    m_Worker       = std::thread(&EcatMaster::processLoop, this);
    m_ErrorHandler = std::thread(&EcatMaster::ecatCheck, this);

    return true;
}

void EcatMaster::stop()
{
    if (!m_Running) {
        return;
    }

    m_Running = false;

    if (m_Worker.joinable()) m_Worker.join();
    if (m_ErrorHandler.joinable()) m_ErrorHandler.join();

    for (int i = 1; i <= ec_slavecount; ++i) {
        if (m_Slaves[i] != nullptr) {
            m_Slaves[i]->stop();
        }
    }

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();
}

void EcatMaster::processLoop()
{
    constexpr int cycleTimeUs = 1000; // 1ms

    while (m_Running) {
        ec_send_processdata();
        m_CurrentWKC.store(ec_receive_processdata(EC_TIMEOUTRET));

        for (int i = 1; i <= ec_slavecount; ++i) {
            if (m_Slaves[i] != nullptr) {
                m_Slaves[i]->processData();
            }
        }

        // sleep
        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
}

bool EcatMaster::reqOpState()
{
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    // send one valid process data to make outputs in slaves happy
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // request OP state for all slaves
    ec_writestate(0);

    // wait for all slaves to reach OP state
    int chk = 200;
    do {
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTOP);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    // check if all slaves are in OP state
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        std::cout << "[EcatMaster::reqOpState] Failed to reach OP state" << std::endl;
        return false;
    }

    std::cout << "[EcatMaster::reqOpState] All slaves in OP state" << std::endl;

    return true;
}

void EcatMaster::ecatCheck()
{
    constexpr int cycleTimeUs = 10000; // 10ms;

    while (m_Running) {
        if (m_CurrentWKC.load() < m_ExpectedWKC
            || ec_group[m_CurrentGroup].docheckstate) {
            // clear check state flag
            ec_group[m_CurrentGroup].docheckstate = FALSE;
            // read state of all slaves
            ec_readstate();

            slavesCheck();

            // all slaves resumed OP state
            if (!ec_group[m_CurrentGroup].docheckstate) {
                std::cout << "[EcatMaster::ecatCheck] OK : all slaves resumed OPERATIONAL" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(cycleTimeUs));
    }
}

void EcatMaster::slavesCheck()
{
    for (int i = 1; i <= ec_slavecount; ++i) {
        auto& slave = ec_slave[i];

        if (slave.group == m_CurrentGroup
            && slave.state != EC_STATE_OPERATIONAL) {
            std::cout << "[EcatMaster::slavesCheck] Slave " << i
                      << " state = " << slave.state
                      << " ALStatusCode = " << slave.ALstatuscode << std::endl;

            ec_group[m_CurrentGroup].docheckstate = TRUE;

            // one of the slaves is not in OP state
            if (slave.state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                std::cout << "[EcatMaster::slavesCheck] ERROR : slave " << i
                          << " SAFE_OP + ERROR, request ACK" << std::endl;
                // ACK the error
                slave.state = EC_STATE_PRE_OP + EC_STATE_ACK;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_INIT) {
                std::cout << "[EcatMaster::slavesCheck] INFO : slave " << i
                          << " INIT -> PRE_OP" << std::endl;
                slave.state = EC_STATE_PRE_OP;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_PRE_OP) {
                std::cout << "[EcatMaster::slavesCheck] INFO : slave " << i
                          << " PRE_OP -> SAFE_OP" << std::endl;
                slave.state = EC_STATE_SAFE_OP;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_BOOT) {
                std::cout << "[EcatMaster::slavesCheck] INFO : slave " << i
                          << " BOOT -> INIT" << std::endl;
                slave.state = EC_STATE_INIT;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_INIT, EC_TIMEOUTSTATE);
            } else if (slave.state == EC_STATE_SAFE_OP) {
                std::cout << "[EcatMaster::slavesCheck] WARNING : slave " << i
                          << " SAFE_OP -> OPERATIONAL" << std::endl;

                // ec_send_processdata();
                // ec_receive_processdata(EC_TIMEOUTRET);

                slave.state = EC_STATE_OPERATIONAL;
                ec_writestate(i);

                ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            }
            // try to reconfigure the slave
            else if (slave.state > EC_STATE_NONE) {
                if (ec_reconfig_slave(i, EC_TIMEOUTSAFE)) {
                    slave.islost = FALSE;
                    std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                              << " reconfigured" << std::endl;
                }
            }
            // check for lost slave
            else if (!slave.islost) {
                ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                if (slave.state == EC_STATE_NONE) {
                    slave.islost = TRUE;
                    std::cout << "[EcatMaster::slavesCheck] ERROR : slave " << i
                              << " lost" << std::endl;
                }
            }
        }

        // slave is lost then try to recover
        if (slave.islost) {
            // check state of lost slave
            if (slave.state != EC_STATE_NONE) {
                slave.islost = FALSE;
                std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                          << " found" << std::endl;
                continue;
            }

            // try to recover the slave
            if (ec_recover_slave(i, EC_TIMEOUTSAFE)) {
                slave.islost = FALSE;
                std::cout << "[EcatMaster::slavesCheck] MESSAGE : slave " << i
                          << " recovered" << std::endl;
            }
        }
    }
}
