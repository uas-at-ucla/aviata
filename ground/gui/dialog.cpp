#include "dialog.h"
#include "ui_dialog.h"
#include "util.h"

#include <ctime>
#include <iostream>
#include <string>
#include <cstring>
#include <filesystem>
#include <libssh/libssh.h>
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
    QPalette p = ui->log_text->palette();
    p.setColor(QPalette::Active, QPalette::Base, Qt::black);
    ui->log_text->setPalette(p);
    QPalette q = ui->textBrowser_3->palette();
    q.setColor(QPalette::Active, QPalette::Base, Qt::black);
    ui->textBrowser_3->setPalette(q);
    ui->textBrowser_3->setTextColor(Qt::white);
    ui->log_text->setTextColor(Qt::white);
    //ui->textBrowser_3->setText?;     //CHECK IF THIS EXISTS, might have to change from a text browser to something else?
    //round console edges
    //ui->log_text->setStyleSheet("border: 1px solid; border-radius:1px; background-color: palette(black)");
    //ui->textBrowser_3->setStyleSheet("border: 1px solid; border-radius:10px; background-color: palette(black)");
    connect(ui->updateButton, &QPushButton::pressed, this, &Dialog::on_updateButtonPressed);
    initialize_update_fields();
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::log(std::string text){
    std::string str;
    if(logs.size() > MAX_LOGS){
        logs.pop_back();
    }
    logs.insert(logs.begin(), text);
    for(int i = 0; i < logs.size(); i++){
        str+=logs.at(i);
        str+="\n";
    }

    QString t = QString::fromStdString(str);
    ui->log_text->setText(t);
}

void Dialog::log(char* text){
    std::string st(text);
    log(st);
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
    //Print drone updates
    int num_nodes = ui->num_drones_spinner->value();
    log("Updating drone software for " + std::to_string(num_nodes)+" drones");
    for(int i = 0; i < num_nodes; i++){
        log(id_fields[i]->toPlainText().toStdString() + ": " + status_fields[i]->currentText().toStdString() + ", " + std::to_string(docking_fields[i]->value()));
    }

    //Initialize standard values;
    ssh_session my_ssh_session;
    int rc;
    const char password[] = "raspberry";
    std::string* hosts = new std::string[num_nodes];
    for(int i = 0; i < num_nodes; i++){
        hosts[i] = id_fields[i]->toPlainText().toStdString();
    }
    std::string p_str(password);
    std::string h_str = "pi@rpi-" + id_fields[0]->toPlainText().toStdString();
    char* connection_string = strcpy(new char[h_str.length()+1], h_str.c_str());
    std::string path = std::filesystem::current_path().string() + "/central_node";

    //Copy central node files to central node
    log("Copying central node files");
    const std::string file_commands[] = {"chmod +x file_copy.sh && expect file_copy.sh ",
        path+" ",
        h_str+" ",
        p_str
    };
    char* file_trans = str_to_chararr(file_commands, 4);
    log(file_trans);
    int file_copy = system(file_trans);
    if(file_copy != 0){
        log("File copying failed, aborting");
        return;
    }
    delete file_trans;
    log("Files copied successfully");

    //Create new SSH session to central node
    my_ssh_session = ssh_new();
    if (my_ssh_session == NULL)
        return;

    ssh_options_set(my_ssh_session, SSH_OPTIONS_HOST, connection_string);

    log("SSH connecting to: " + h_str);
    rc = ssh_connect(my_ssh_session);
    if (rc != SSH_OK)
    {
        fprintf(stderr, "Error connecting to localhost: %s\n",
                ssh_get_error(my_ssh_session));
        ssh_free(my_ssh_session);
        return;
    }
    log("SSH connection succes, authenticating");

    rc = ssh_userauth_password(my_ssh_session, NULL, password);
    if (rc != SSH_AUTH_SUCCESS)
    {
        fprintf(stderr, "Error authenticating with password: %s\n",
                ssh_get_error(my_ssh_session));
        ssh_disconnect(my_ssh_session);
        ssh_free(my_ssh_session);
        return;
    }
    log("Authentication success");

    //Execute commands
    rc = execute_commands(my_ssh_session, connection_string, hosts, password, num_nodes);

    //Cleanup and close
    ssh_disconnect(my_ssh_session);
    ssh_free(my_ssh_session);
    delete[] hosts;
    delete[] connection_string;
    log("SSH session closed, updates complete");

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

