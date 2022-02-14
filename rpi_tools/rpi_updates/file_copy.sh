#!/usr/bin/expect
        set pass $1 $2 $3
        spawn scp  -r $1 $2:~/central_node

        expect {
        password: {send "$3\r"; exp_continue}
                  }