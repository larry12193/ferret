#!/bin/bash

RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0;0m'

echo -e "${RED}Installing lib files${NC}"
\cp -n flycapture.2.11.3.121_armhf/lib/libflycapture* /usr/lib

echo -e "${RED}Creating /usr/include/flycapture directory${NC}"
mkdir /usr/include/flycapture
echo -e "${RED}Installing include files ${NC}"
\cp -n flycapture.2.11.3.121_armhf/include/* /usr/include/flycapture 

echo -e "${RED}Installing binary files${NC}"
\cp -n flycapture.2.11.3.121_armhf/bin/* /usr/bin

echo -e "${BLUE}Calling configuration script${NC}"
sh flycapture.2.11.3.121_armhf/flycap2-conf
