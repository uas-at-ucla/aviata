#include <iostream>
#include <string>
#include <cstring>
#include <filesystem>
#include <libssh/libssh.h>

using std::cout;
using std::endl;

char *get_command_string(const std::string *commands, int num_commands)
{
    std::string ans = "";
    std::string prefix = "";
    for (int i = 0; i < num_commands; i++)
    {
        ans += prefix;
        ans += "echo '" + commands[i] + "' && ";
        ans += commands[i];
        prefix = " && ";
    }
    char *cstr = new char[ans.length() + 1];
    std::strcpy(cstr, ans.c_str());
    return cstr;
}

std::string arr_to_str(const char** arr, int num_nodes){
    std::string out;
    out += "[";
    std::string prefix;

    for(int i = 0; i < num_nodes; i++){
        out+=prefix;
        out+=arr[i];
        prefix = ",";
    }

    out += "]";
    return out;
}

int execute_commands(ssh_session session, const char* central_host, const char **hosts, const char ** passwords, const int num_nodes, std::string release)
{
    ssh_channel channel;
    int rc;

    channel = ssh_channel_new(session);
    if (channel == NULL)
        return SSH_ERROR;

    rc = ssh_channel_open_session(channel);
    if (rc != SSH_OK)
    {
        ssh_channel_free(channel);
        return rc;
    }
    /*
    const std::string commands[] = {"rm -rf central_node",
                                    "scp -r"  
                                   };
                                   */

    const std::string commands[] = {"cd ~/central_node",
                              //"rm -rf " + release + ".zip",
                              //"rm -rf aviata-rpi-node-" + release,
                              //"wget https://github.com/chirag-singh1/aviata-rpi-node/archive/refs/tags/" + release + ".zip",
                              //"unzip " + release + ".zip",
                              //"cd aviata-rpi-node-" + release,
                              "chmod +x run.sh",
                              "./run.sh " + std::to_string(num_nodes)+" "+arr_to_str(hosts, num_nodes) + " "+ arr_to_str(passwords, num_nodes),
                              "rm -rf " + release + ".zip",
                              "rm -rf aviata-rpi-node-" + release};
    char* comm_str = get_command_string(commands, sizeof(commands)/sizeof(commands[0]));
                              
    rc = ssh_channel_request_exec(channel, comm_str);
    if (rc != SSH_OK)
    {
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        return rc;
    }

    char buffer[256];
    int nbytes;

    nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
    while (nbytes > 0)
    {
        if (fwrite(buffer, 1, nbytes, stdout) != nbytes)
        {
            ssh_channel_close(channel);
            ssh_channel_free(channel);
            return SSH_ERROR;
        }
        nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
    }

    if (nbytes < 0)
    {
        ssh_channel_close(channel);
        ssh_channel_free(channel);
        return SSH_ERROR;
    }

    ssh_channel_send_eof(channel);
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    delete comm_str;

    return SSH_OK;
}

char *str_to_chararr(const std::string *commands, int num_commands)
{
    std::string ans = "";
    for (int i = 0; i < num_commands; i++)
    {
        ans += commands[i];
    }
    char *cstr = new char[ans.length() + 1];
    std::strcpy(cstr, ans.c_str());
    return cstr;
}

int main(int argc, char* argv[])
{
    std::string release = "0.0.3";
    if(argc > 1){
        std::string release(argv[1]);
    }

    ssh_session my_ssh_session;
    int rc;
    const char password[] = "password";
    const char connection_string[] = "chirag-singh@localhost";
    const char *hosts[] = {"localhost", "localhost", "localhost", "localhost"};
    const char *passwords[] = {"password", "password", "password", "password"};
    const int num_nodes = 4;

    std::string p_str(password);
    std::string h_str(connection_string);
    std::string path = std::filesystem::current_path().string() + "/central_node";

    const std::string file_commands[] = {"chmod +x file_copy.sh && expect file_copy.sh ",
        path+" ",
        h_str+" ",
        p_str
    };
    char* file_trans = str_to_chararr(file_commands, 4);
    int file_copy = system(file_trans);
    delete file_trans;

    my_ssh_session = ssh_new();
    if (my_ssh_session == NULL)
        return 1;

    ssh_options_set(my_ssh_session, SSH_OPTIONS_HOST, connection_string);

    cout << "SSH connecting to: " << connection_string << endl;
    rc = ssh_connect(my_ssh_session);
    if (rc != SSH_OK)
    {
        fprintf(stderr, "Error connecting to localhost: %s\n",
                ssh_get_error(my_ssh_session));
        ssh_free(my_ssh_session);
        return 1;
    }

    cout << "SSH connection succes, authenticating" << endl;

    rc = ssh_userauth_password(my_ssh_session, NULL, password);
    if (rc != SSH_AUTH_SUCCESS)
    {
        fprintf(stderr, "Error authenticating with password: %s\n",
                ssh_get_error(my_ssh_session));
        ssh_disconnect(my_ssh_session);
        ssh_free(my_ssh_session);
        return 1;
    }

    cout << "Authentication success" << endl;

    rc = execute_commands(my_ssh_session, connection_string, hosts, passwords, num_nodes, release);

    ssh_disconnect(my_ssh_session);
    ssh_free(my_ssh_session);

    cout << "SSH session closed" << endl;
    return rc;
}