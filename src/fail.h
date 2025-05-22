#ifndef FAIL_H
#define FAIL_H

enum FailStates
{
    FAIL_FLASH_INIT=1,
    FAIL_FLASH_CHIP_ERASE=2,
    FAIL_FINISHED_DATA_DUMP=3,
};

// this is defined in main so that it has access to the neopixel context
// this header file is just so that the enum constants and the function
// definition can be accessed from other files
void fail(int n);

#endif