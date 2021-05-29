#include "main_window.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
}

// void MainWindow::on_inputSpinBox1_valueChanged(int value)
// {
//     ui.outputWidget->setText(QString::number(value + ui.inputSpinBox2->value()));
// }

// void MainWindow::on_inputSpinBox2_valueChanged(int value)
// {
//     ui.outputWidget->setText(QString::number(value + ui.inputSpinBox1->value()));
// }
