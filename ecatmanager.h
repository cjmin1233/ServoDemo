#ifndef ECATMANAGER_H
#define ECATMANAGER_H

#include <QObject>

#include "ecatmaster.h"

class EcatManager : public QObject {
    Q_OBJECT
public:
    explicit EcatManager(QObject* parent = nullptr);
    ~EcatManager();

    const int getSlaveCount() const { return ec_slavecount; }

    bool connectMaster(const QString& ifname);
    void disconnectMaster();

    void launchServoMove(float ratio);

signals:

private:
    EcatMaster m_Master;
};

#endif // ECATMANAGER_H
