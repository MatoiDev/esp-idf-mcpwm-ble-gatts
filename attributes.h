//
// Created by Matoi on 10.01.2024.
//

#include "../../../.espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/xtensa-esp-elf/sys-include/stdio.h"
#include "../../../.espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/xtensa-esp-elf/sys-include/stdlib.h"
#include "../../../.espressif/tools/xtensa-esp-elf/esp-13.2.0_20230928/xtensa-esp-elf/xtensa-esp-elf/sys-include/string.h"


/* Attributes State Machine */
enum
{
    KEIRA_IDX_SVC,

    KEIRA_IDX_MSG_CHARACTERISTIC,
    KEIRA_IDX_MSG_CHARACTERISTIC_VAL,

    KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC,
    KEIRA_IDX_LEFT_JOYSTICK_CHARACTERISTIC_VAL,

    KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC,
    KEIRA_IDX_RIGHT_JOYSTICK_CHARACTERISTIC_VAL,

    KEIRA_IDX_NB,
};
