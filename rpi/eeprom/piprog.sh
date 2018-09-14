#!/bin/bash

FILE="$1"
SIZE=$(stat -c'%s' "$1")

CHIP=${CHIP:-24c64}

I2C_BUS=${I2C_BUS:-0}
I2C_ADDR=${I2C_ADDR:-50}

SYS=/sys/class/i2c-adapter/i2c-${I2C_BUS}
DEV=${SYS}/${I2C_BUS}-00${I2C_ADDR}

modprobe i2c_dev  || exit $?
modprobe at24 write_timeout=250  || exit $?

test -f ${FILE}   || exit 1
test -d ${SYS}    || exit 1

if [ ! -d ${DEV} ]
then
    echo "${CHIP} 0x${I2C_ADDR}" > ${SYS}/new_device
fi

dd if="${FILE}" of=${DEV}/eeprom oflag=sync status=none

cmp -n ${SIZE} -s ${FILE} ${DEV}/eeprom ; RET=$?

echo "0x${I2C_ADDR}" > ${SYS}/delete_device

rmmod at24
rmmod i2c_dev

exit ${RET}
