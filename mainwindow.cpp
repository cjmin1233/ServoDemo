#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include "ecatwidget.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Create ecat widget
    EcatWidget* ecatWidget = new EcatWidget(this);
    setCentralWidget(ecatWidget);
}

MainWindow::~MainWindow()
{
    delete ui;
}
