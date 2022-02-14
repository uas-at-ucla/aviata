#!/usr/bin/expect
        set pass [lindex $argv 2];
        set filepath [lindex $argv 0];
        set username [lindex $argv 1];
        spawn scp  -r $filepath $username:~/central_node

        expect {
        password: {send "$pass\r"; exp_continue}
                  }