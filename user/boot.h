//
//  boot.h
//
//  Created by Delmore Lee on 5/8/16.
//

#ifndef BOOT_H
#define BOOT_H

#include "py32f0xx_hal.h"

void vCheckBootArg(void);
void vRunEnterBootloader(void);
void vReboot(void);

#endif

