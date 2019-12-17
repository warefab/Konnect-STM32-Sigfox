/*
 * W25Q64JV.h
 *
 * w25q64jv 64mb flash library
 * for Warefab Konnect STM32L0-SIGFOX Development Board
 *
 * Created on: Nov 14, 2019
 * Author: Muchiri John
 * (c) wwww.warefab.com
 *
 * This software component is licensed by warefab under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 * opensource.org/licenses/BSD-3-Clause
 */

#ifndef W25Q64JV_H_
#define W25Q64JV_H_

#include "spi.h"

#define MANF_ID 0x90

#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04

#define READ_DATA 0x03
#define FAST_READ 0x0B

#define PAGE_PROG 0x02

#define SECTOR_ERASE_4KB 0x20
#define BLOCK_ERASE_32KB 0x52
#define BLOCK_ERASE_64KB 0xD8
#define CHIP_ERASE 0xC7 //0x60

#endif /* W25Q64JV_H_ */
