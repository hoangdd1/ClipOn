/*
 * version.h
 *
 *  Created on: May 31, 2024
 *      Author: ephra
 */

#ifndef VERSION_H_
#define VERSION_H_


// Define firmware version macros
#define VERSION_MAJOR 2
#define VERSION_MINOR 0
#define VERSION_PATCH 0


#define ENTRY_POINT_ADDRESS 0x791C  // Define the entry point address based on the FR6047_USSSWLib_AQSMRT_V2.0.0.map file

// Define the password of the BSL found in the reset vector 0xFFE0 - 0XFFFF
#define BSL_PW {0xA6, 0x83, 0xF4, 0x83, 0x56, 0x84, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x10, 0x7F, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x60, 0x84, 0x36, 0x84}

#define START_CODE_ADDRESS 0x6000

#define MAX_CODE_BLOCK_SIZE 256

#define TOTAL_CODE_SIZE 128000

#endif /* VERSION_H_ */
