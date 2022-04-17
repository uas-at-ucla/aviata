#ifndef UTIL_H
#define UTIL_H

#include <cstring>
#include <string>
#include <libssh/libssh.h>

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

std::string arr_to_str(const std::string* hosts, int n){
    std::string out;
    out+="[";
    std::string prefix = "";

    for(int i = 0; i < n; i++){
        out+=prefix;
        out+=hosts[i];
        prefix=",";
    }

    out+="]";
    return out;
}

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

int execute_commands(ssh_session session, const char* central_host, const std::string* hosts, const char* password, const int num_nodes)
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

    std::string p(password);
    const std::string commands[] = {"cd ~/central_node",
                              "chmod +x run.sh",
                              "./run.sh " + std::to_string(num_nodes)+" "+arr_to_str(hosts, num_nodes) + " "+ p,
                              "cd ..",
                              "rm -rf central_node"};
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

#endif // UTIL_H
