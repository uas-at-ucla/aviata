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
    ui->tableWidget->setRowCount(numDrones+1);
    for(int i=0; i<=numDrones; i++){
        QString num = QString::number(i);
        if(i==0){
            num = "";
        }
        QTableWidgetItem *temp = new QTableWidgetItem;
        temp->setText(num);
        ui->tableWidget->setVerticalHeaderItem(i, temp);
    }

    //PUT THESE INTO THE UPDATE LOOP?
    //set drone hostnames
    int col = 0;
    QString drones[] = {"192.168.7.74", "192.241.4.32", "14.198.7.74", "0", "0", "0", "0", "0"};
    for(int i=1; i<=numDrones; i++){
        QTableWidgetItem *temp = ui->tableWidget->item(i, col);
        if(!temp){
            temp = new QTableWidgetItem;
            ui->tableWidget->setItem(i, col, temp);
        }
        //set hostname
        //ui->tableWidget->item(i, col)->setText("Drone" + QString::number(i+1) + "_HOSTNAME");
        ui->tableWidget->item(i, col)->setText(drones[i-1]);
    }
    //set drone statuses
    col = 1;
    for(int i=1; i<ui->tableWidget->rowCount(); i++){
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
    //round console edges
    ui->textBrowser_2->setStyleSheet("border: 1px solid; border-radius:1px; background-color: palette(black)");
    ui->textBrowser_3->setStyleSheet("border: 1px solid; border-radius:10px; background-color: palette(black)");
}

Dialog::~Dialog()
{
    delete ui;
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

