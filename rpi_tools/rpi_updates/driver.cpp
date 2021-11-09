#include <iostream>
#include <string>
#include <cstring>
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

int execute_commands(ssh_session session, const char **hosts, const int num_nodes, const std::string release)
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

    const std::string commands[] = {"cd ~",
                              "rm -rf " + release + ".zip",
                              "rm -rf release-test-" + release,
                              "wget https://github.com/chirag-singh1/release-test/archive/refs/tags/" + release + ".zip",
                              "unzip " + release + ".zip",
                              "cd release-test-" + release,
                              "chmod +x test.sh",
                              "./test.sh ",
                              "rm -rf " + release + ".zip",
                              "rm -rf release-test-" + release};
    rc = ssh_channel_request_exec(channel, get_command_string(commands, sizeof(commands)/sizeof(commands[0])));
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

    return SSH_OK;
}

int main()
{
    ssh_session my_ssh_session;
    int rc;
    const char password[] = "password";
    const char connection_string[] = "localhost";
    const char *hosts[] = {"localhost", "localhost", "localhost", "localhost"};
    const std::string release = "0.0.2";
    const int num_nodes = 4;

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

    rc = execute_commands(my_ssh_session, hosts, num_nodes, release);

    ssh_disconnect(my_ssh_session);
    ssh_free(my_ssh_session);

    cout << "SSH session closed" << endl;
    return rc;
}