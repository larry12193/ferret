#!/bin/bash

\cp -n 99-ferret.rules /etc/udev/rules.d
/etc/init.d/udev restart
