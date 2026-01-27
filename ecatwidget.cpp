#include "ecatwidget.h"
#include "ui_ecatwidget.h"

#include "ecatmanager.h"

EcatWidget::EcatWidget(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::EcatWidget)
{
    ui->setupUi(this);

    const QString ifname = "\\Device\\NPF_{1ADF9951-9FC1-4966-866A-BD3F84ECB3D6}";

    m_Manager = new EcatManager(this);

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
}

EcatWidget::~EcatWidget()
{
    delete ui;
}
