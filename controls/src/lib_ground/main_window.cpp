#include "main_window.hpp"

//! [0]
CalculatorForm::CalculatorForm(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
}
//! [0]

//! [1]
void CalculatorForm::on_inputSpinBox1_valueChanged(int value)
{
    ui.outputWidget->setText(QString::number(value + ui.inputSpinBox2->value()));
}
//! [1]

//! [2]
void CalculatorForm::on_inputSpinBox2_valueChanged(int value)
{
    ui.outputWidget->setText(QString::number(value + ui.inputSpinBox1->value()));
}
//! [2]
