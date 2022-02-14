echo 'Executing central node script'
echo 'Installing libssh dependency'
echo "password" | sudo -S apt-get install -y libssh-dev
echo 'Building central node'
make && \
chmod +x execute_ground && \
echo 'Executing central node program' && \
./execute_ground $1 $2 $3
echo 'Central node program complete, cleaning up'
make clean
echo 'Done!'