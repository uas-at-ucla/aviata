#!/bin/bash

while ! ifconfig | grep -F "10.10.0." > /dev/null; do
    sleep 1
done
