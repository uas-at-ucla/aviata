#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include "ui_main_window.h"

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    // void on_inputSpinBox1_valueChanged(int value);
    // void on_inputSpinBox2_valueChanged(int value);

private:
    Ui::MainWindow ui;
};

#endif
