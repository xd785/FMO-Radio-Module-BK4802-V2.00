//
//  boot.c
//
//  Created by Delmore Lee on 5/8/16.


#include "boot.h"
//#include "types.h#ifndef _SIZE_T
//#define _SIZE_T
//typedef unsigned int size_t;
//#endif

/**
 * The 8-bit signed data type.
 */
typedef char int8;
/**
 * The volatile 8-bit signed data type.
 */
typedef volatile char vint8;
/**
 * The 8-bit unsigned data type.
 */
typedef unsigned char uint8;
/**
 * The volatile 8-bit unsigned data type.
 */
typedef volatile unsigned char vuint8;

/**
 * The 16-bit signed data type.
 */
typedef int int16;
/**
 * The volatile 16-bit signed data type.
 */
//typedef volatile int vint16;
/**
 * The 16-bit unsigned data type.
 */
typedef unsigned short uint16;
/**
 * The volatile 16-bit unsigned data type.
 */
//typedef volatile unsigned int vuint16;
/**
 * The 32-bit signed data type.
 */
typedef long int32;
/**
 * The volatile 32-bit signed data type.
 */
//typedef volatile long vint32;
/**
 * The 32-bit unsigned data type.
 */
typedef unsigned long uint32;
/**
 * The volatile 32-bit unsigned data type.
 */
//typedef volatile unsigned long vuint32;

/* bsd */
typedef uint8			u_char;		/**< 8-bit value */
typedef uint8 			SOCKET;
typedef uint16			u_short;	/**< 16-bit value */
typedef uint16			u_int;		/**< 16-bit value */
typedef uint32			u_long;		/**< 32-bit value */

#define SYSMEM_RESET_VECTOR            0x1fff0004    
#define BOOTLOADER_STACK_POINTER       (*(volatile unsigned int *)0x1fff0000)
// #define SYSMEM_RESET_VECTOR            0x2ffe0004    
// #define BOOTLOADER_STACK_POINTER       (*(volatile unsigned int *)0x2ffe0000)
#define BOOT_ARG_ADDRESS               0x20001FFC


#define RESET_TO_BOOTLOADER_MAGIC_CODE 0xDEADBEEF
#define FINISH_BOOTLOADER_MAGIC_CODE   0xCAFEBEEF

//uint32 uBootArg;
extern void systemClockInit(void);
	
//void vCheckBootArg(void)
//{
//    uint32 *pArg = (uint32 *)BOOT_ARG_ADDRESS;

//    if ((*pArg) == RESET_TO_BOOTLOADER_MAGIC_CODE) 
//    {
//        *pArg = 0;
//        void (*bootloader)(void) = (void (*)(void)) (*((uint32 *) SYSMEM_RESET_VECTOR));
//        
//        HAL_RCC_DeInit();
//        SysTick->CTRL = 0;
//        SysTick->LOAD = 0;
//        SysTick->VAL = 0;
//        systemClockInit();
//        __set_PRIMASK(1);
//        __set_MSP(BOOTLOADER_STACK_POINTER);
//        bootloader();
//        while (1);
//    }
//    else if((*pArg) == 0)
//    {
//        *pArg = FINISH_BOOTLOADER_MAGIC_CODE;
//        vReboot();
//    }
//}

void vCheckBootArg(void)
{
    uint32 *pArg = (uint32 *)BOOT_ARG_ADDRESS;

    if ((*pArg) == RESET_TO_BOOTLOADER_MAGIC_CODE) 
    {
         vReboot();

    }
    else if((*pArg) == 0)
    {
        *pArg = RESET_TO_BOOTLOADER_MAGIC_CODE;
        vReboot();
    }
}

void (*SysMemBootJump)(void);

void vRunEnterBootloader(void)
{
    uint32 *pArg = (uint32 *)BOOT_ARG_ADDRESS;
    *pArg = RESET_TO_BOOTLOADER_MAGIC_CODE;
    NVIC_SystemReset();
}

void vReboot(void)
{
    NVIC_SystemReset();
}
