#include "dialog.h"
#include "ui_dialog.h"
#include <iostream>
using namespace std;

Dialog::Dialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog)
{
    ui->setupUi(this);
    cout << "Started ground control station" << endl;

    //Populate table
    //set left column (drone number)
    int numDrones = 8;      //READ THIS IN?
    ui->tableWidget->setRowCount(numDrones);

    //PUT THESE INTO THE UPDATE LOOP?
    //set drone hostnames
    int col = 0;
    QString drones[] = {"192.168.7.74", "192.241.4.32", "14.198.7.74", "0", "0", "0", "0", "0"};
    for(int i=0; i<numDrones; i++){
        QTableWidgetItem *temp = ui->tableWidget->item(i, col);
        if(!temp){
            temp = new QTableWidgetItem;
            ui->tableWidget->setItem(i, col, temp);
        }
        //set hostname
        //ui->tableWidget->item(i, col)->setText("Drone" + QString::number(i+1) + "_HOSTNAME");
        ui->tableWidget->item(i, col)->setText(drones[i]);
    }
    //set drone statuses
    col = 1;
    for(int i=0; i<ui->tableWidget->rowCount(); i++){
        QTableWidgetItem *temp = ui->tableWidget->item(i, col);
        if(!temp){
            temp = new QTableWidgetItem;
            ui->tableWidget->setItem(i, col, temp);
        }
        //set status
        //ui->tableWidget->item(i, col)->setText("example");
    }

    //CONSOLE
    QPalette p = ui->textBrowser_2->palette();
    p.setColor(QPalette::Active, QPalette::Base, Qt::black);
    ui->textBrowser_2->setPalette(p);
    QPalette q = ui->textBrowser_3->palette();
    q.setColor(QPalette::Active, QPalette::Base, Qt::black);
    ui->textBrowser_3->setPalette(q);
    ui->textBrowser_3->setTextColor(Qt::white);
    //ui->textBrowser_3->setText?;     //CHECK IF THIS EXISTS, might have to change from a text browser to something else?
    //round console edges
    //ui->textBrowser_2->setStyleSheet("border: 1px solid; border-radius:1px; background-color: palette(black)");
    //ui->textBrowser_3->setStyleSheet("border: 1px solid; border-radius:10px; background-color: palette(black)");
    connect(ui->updateButton, &QPushButton::pressed, this, &Dialog::on_updateButtonPressed);
    initialize_update_fields();
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::initialize_update_fields(){
    id_fields[0] = ui->drone_id_0;
    id_fields[1] = ui->drone_id_1;
    id_fields[2] = ui->drone_id_2;
    id_fields[3] = ui->drone_id_3;
    id_fields[4] = ui->drone_id_4;
    id_fields[5] = ui->drone_id_5;
    id_fields[6] = ui->drone_id_6;
    id_fields[7] = ui->drone_id_7;

    status_fields[0] = ui->drone_state_0;
    status_fields[1] = ui->drone_state_1;
    status_fields[2] = ui->drone_state_2;
    status_fields[3] = ui->drone_state_3;
    status_fields[4] = ui->drone_state_4;
    status_fields[5] = ui->drone_state_5;
    status_fields[6] = ui->drone_state_6;
    status_fields[7] = ui->drone_state_7;

    docking_fields[0] = ui->drone_slot_0;
    docking_fields[1] = ui->drone_slot_1;
    docking_fields[2] = ui->drone_slot_2;
    docking_fields[3] = ui->drone_slot_3;
    docking_fields[4] = ui->drone_slot_4;
    docking_fields[5] = ui->drone_slot_5;
    docking_fields[6] = ui->drone_slot_6;
    docking_fields[7] = ui->drone_slot_7;
}

void Dialog::on_updateButtonPressed(){
    int num_drones = ui->num_drones_spinner->value();
    cout << "Updating drone software for " << num_drones << " drones" << endl;
    for(int i = 0; i < num_drones; i++){
        cout << id_fields[i]->toPlainText().toStdString() << ": " << status_fields[i]->currentText().toStdString() << ", " << docking_fields[i]->value() << std::endl;
    }
}

void Dialog::on_tableWidget_cellClicked(int row, int column)
{
    if(row == 0 && column == 2){
        //send command to update all raspberry pis
        cout << "updating all raspberry pis" << endl;
    }
    else if (column == 2){
        //update the ROWth + 1 drone's raspberry pi

    }
}

