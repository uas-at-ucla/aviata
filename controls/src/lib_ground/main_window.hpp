#ifndef CALCULATORFORM_H
#define CALCULATORFORM_H

//! [0]
#include "ui_main_window.h"
//! [0]

//! [1]
class CalculatorForm : public QWidget
{
    Q_OBJECT

public:
    explicit CalculatorForm(QWidget *parent = nullptr);

private slots:
    void on_inputSpinBox1_valueChanged(int value);
    void on_inputSpinBox2_valueChanged(int value);

private:
    Ui::CalculatorForm ui;
};
//! [1]

#endif
