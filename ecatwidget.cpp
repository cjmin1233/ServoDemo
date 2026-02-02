#include "ecatwidget.h"
#include "ui_ecatwidget.h"

#include "ecatmanager.h"

EcatWidget::EcatWidget(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::EcatWidget)
{
    ui->setupUi(this);

    // const QString ifname = "\\Device\\NPF_{133B147A-976D-4DD4-A9FC-3360A1F84C90}";
    const QString ifname = "\\Device\\NPF_{F80FCB79-A945-4A5A-BD77-B5076391E949}";

    m_Manager = new EcatManager(this);

    if (m_Manager == nullptr) {
        return;
    }

    if (!m_Manager->connectMaster(ifname)) {
        // connect failed
        return;
    }

    connect(ui->pushButton, &QPushButton::clicked,
            this, [this]() {
                int sliderValue = ui->verticalSlider->value();
                int diff        = ui->verticalSlider->maximum() - ui->verticalSlider->minimum();

                float ratio = (float)sliderValue / diff;

                m_Manager->launchServoMove(ratio);
            });

    connect(ui->btnHome, &QPushButton::clicked,
            this, [this]() {
                m_Manager->setHome();
            });
}

EcatWidget::~EcatWidget()
{
    delete ui;
}
