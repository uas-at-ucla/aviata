#ifndef DIALOG_H
#define DIALOG_H
#include "ui_dialog.h"
#include <QDialog>

const int MAX_LOGS = 10;

QT_BEGIN_NAMESPACE
namespace Ui { class Dialog; }
QT_END_NAMESPACE

class Dialog : public QDialog
{
    Q_OBJECT

public:
    Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void on_tableWidget_cellClicked(int row, int column);
    void on_updateButtonPressed();

private:
    Ui::Dialog *ui;

    void initialize_update_fields();
    void log(std::string text);

    std::vector<std::string> logs;
    QTextEdit* id_fields[8];
    QComboBox* status_fields[8];
    QSpinBox* docking_fields[8];
};
#endif // DIALOG_H
