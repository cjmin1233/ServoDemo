#ifndef ECATWIDGET_H
#define ECATWIDGET_H

#include <QWidget>

namespace Ui {
class EcatWidget;
}

class EcatManager;

class EcatWidget : public QWidget {
    Q_OBJECT

public:
    explicit EcatWidget(QWidget* parent = nullptr);
    ~EcatWidget();

private:
    Ui::EcatWidget* ui;

    EcatManager* m_Manager = nullptr;
};

#endif // ECATWIDGET_H
