#!/bin/bash

\cp -n 99-radferret.rules /etc/udev/rules.d
/etc/init.d/udev restart
