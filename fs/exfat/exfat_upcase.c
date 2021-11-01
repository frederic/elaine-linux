/*
 *  Copyright (C) 2012-2013 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/************************************************************************/
/*                                                                      */
/*  PROJECT : exFAT & FAT12/16/32 File System                           */
/*  FILE    : exfat_upcase.c                                            */
/*  PURPOSE : exFAT Up-case Table                                       */
/*                                                                      */
/*----------------------------------------------------------------------*/
/*  NOTES                                                               */
/*                                                                      */
/*----------------------------------------------------------------------*/
/*  REVISION HISTORY (Ver 0.9)                                          */
/*                                                                      */
/*  - 2010.11.15 [Joosun Hahn] : first writing                          */
/*                                                                      */
/************************************************************************/

#include "exfat_config.h"

#include "exfat_nls.h"

const u8 uni_upcase[NUM_UPCASE << 1] = {
	0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00,
	0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x07, 0x00,
	0x08, 0x00, 0x09, 0x00, 0x0A, 0x00, 0x0B, 0x00,
	0x0C, 0x00, 0x0D, 0x00, 0x0E, 0x00, 0x0F, 0x00,
	0x10, 0x00, 0x11, 0x00, 0x12, 0x00, 0x13, 0x00,
	0x14, 0x00, 0x15, 0x00, 0x16, 0x00, 0x17, 0x00,
	0x18, 0x00, 0x19, 0x00, 0x1A, 0x00, 0x1B, 0x00,
	0x1C, 0x00, 0x1D, 0x00, 0x1E, 0x00, 0x1F, 0x00,
	0x20, 0x00, 0x21, 0x00, 0x22, 0x00, 0x23, 0x00,
	0x24, 0x00, 0x25, 0x00, 0x26, 0x00, 0x27, 0x00,
	0x28, 0x00, 0x29, 0x00, 0x2A, 0x00, 0x2B, 0x00,
	0x2C, 0x00, 0x2D, 0x00, 0x2E, 0x00, 0x2F, 0x00,
	0x30, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00,
	0x34, 0x00, 0x35, 0x00, 0x36, 0x00, 0x37, 0x00,
	0x38, 0x00, 0x39, 0x00, 0x3A, 0x00, 0x3B, 0x00,
	0x3C, 0x00, 0x3D, 0x00, 0x3E, 0x00, 0x3F, 0x00,
	0x40, 0x00, 0x41, 0x00, 0x42, 0x00, 0x43, 0x00,
	0x44, 0x00, 0x45, 0x00, 0x46, 0x00, 0x47, 0x00,
	0x48, 0x00, 0x49, 0x00, 0x4A, 0x00, 0x4B, 0x00,
	0x4C, 0x00, 0x4D, 0x00, 0x4E, 0x00, 0x4F, 0x00,
	0x50, 0x00, 0x51, 0x00, 0x52, 0x00, 0x53, 0x00,
	0x54, 0x00, 0x55, 0x00, 0x56, 0x00, 0x57, 0x00,
	0x58, 0x00, 0x59, 0x00, 0x5A, 0x00, 0x5B, 0x00,
	0x5C, 0x00, 0x5D, 0x00, 0x5E, 0x00, 0x5F, 0x00,
	0x60, 0x00, 0x41, 0x00, 0x42, 0x00, 0x43, 0x00,
	0x44, 0x00, 0x45, 0x00, 0x46, 0x00, 0x47, 0x00,
	0x48, 0x00, 0x49, 0x00, 0x4A, 0x00, 0x4B, 0x00,
	0x4C, 0x00, 0x4D, 0x00, 0x4E, 0x00, 0x4F, 0x00,
	0x50, 0x00, 0x51, 0x00, 0x52, 0x00, 0x53, 0x00,
	0x54, 0x00, 0x55, 0x00, 0x56, 0x00, 0x57, 0x00,
	0x58, 0x00, 0x59, 0x00, 0x5A, 0x00, 0x7B, 0x00,
	0x7C, 0x00, 0x7D, 0x00, 0x7E, 0x00, 0x7F, 0x00,
	0x80, 0x00, 0x81, 0x00, 0x82, 0x00, 0x83, 0x00,
	0x84, 0x00, 0x85, 0x00, 0x86, 0x00, 0x87, 0x00,
	0x88, 0x00, 0x89, 0x00, 0x8A, 0x00, 0x8B, 0x00,
	0x8C, 0x00, 0x8D, 0x00, 0x8E, 0x00, 0x8F, 0x00,
	0x90, 0x00, 0x91, 0x00, 0x92, 0x00, 0x93, 0x00,
	0x94, 0x00, 0x95, 0x00, 0x96, 0x00, 0x97, 0x00,
	0x98, 0x00, 0x99, 0x00, 0x9A, 0x00, 0x9B, 0x00,
	0x9C, 0x00, 0x9D, 0x00, 0x9E, 0x00, 0x9F, 0x00,
	0xA0, 0x00, 0xA1, 0x00, 0xA2, 0x00, 0xA3, 0x00,
	0xA4, 0x00, 0xA5, 0x00, 0xA6, 0x00, 0xA7, 0x00,
	0xA8, 0x00, 0xA9, 0x00, 0xAA, 0x00, 0xAB, 0x00,
	0xAC, 0x00, 0xAD, 0x00, 0xAE, 0x00, 0xAF, 0x00,
	0xB0, 0x00, 0xB1, 0x00, 0xB2, 0x00, 0xB3, 0x00,
	0xB4, 0x00, 0xB5, 0x00, 0xB6, 0x00, 0xB7, 0x00,
	0xB8, 0x00, 0xB9, 0x00, 0xBA, 0x00, 0xBB, 0x00,
	0xBC, 0x00, 0xBD, 0x00, 0xBE, 0x00, 0xBF, 0x00,
	0xC0, 0x00, 0xC1, 0x00, 0xC2, 0x00, 0xC3, 0x00,
	0xC4, 0x00, 0xC5, 0x00, 0xC6, 0x00, 0xC7, 0x00,
	0xC8, 0x00, 0xC9, 0x00, 0xCA, 0x00, 0xCB, 0x00,
	0xCC, 0x00, 0xCD, 0x00, 0xCE, 0x00, 0xCF, 0x00,
	0xD0, 0x00, 0xD1, 0x00, 0xD2, 0x00, 0xD3, 0x00,
	0xD4, 0x00, 0xD5, 0x00, 0xD6, 0x00, 0xD7, 0x00,
	0xD8, 0x00, 0xD9, 0x00, 0xDA, 0x00, 0xDB, 0x00,
	0xDC, 0x00, 0xDD, 0x00, 0xDE, 0x00, 0xDF, 0x00,
	0xC0, 0x00, 0xC1, 0x00, 0xC2, 0x00, 0xC3, 0x00,
	0xC4, 0x00, 0xC5, 0x00, 0xC6, 0x00, 0xC7, 0x00,
	0xC8, 0x00, 0xC9, 0x00, 0xCA, 0x00, 0xCB, 0x00,
	0xCC, 0x00, 0xCD, 0x00, 0xCE, 0x00, 0xCF, 0x00,
	0xD0, 0x00, 0xD1, 0x00, 0xD2, 0x00, 0xD3, 0x00,
	0xD4, 0x00, 0xD5, 0x00, 0xD6, 0x00, 0xF7, 0x00,
	0xD8, 0x00, 0xD9, 0x00, 0xDA, 0x00, 0xDB, 0x00,
	0xDC, 0x00, 0xDD, 0x00, 0xDE, 0x00, 0x78, 0x01,
	0x00, 0x01, 0x00, 0x01, 0x02, 0x01, 0x02, 0x01,
	0x04, 0x01, 0x04, 0x01, 0x06, 0x01, 0x06, 0x01,
	0x08, 0x01, 0x08, 0x01, 0x0A, 0x01, 0x0A, 0x01,
	0x0C, 0x01, 0x0C, 0x01, 0x0E, 0x01, 0x0E, 0x01,
	0x10, 0x01, 0x10, 0x01, 0x12, 0x01, 0x12, 0x01,
	0x14, 0x01, 0x14, 0x01, 0x16, 0x01, 0x16, 0x01,
	0x18, 0x01, 0x18, 0x01, 0x1A, 0x01, 0x1A, 0x01,
	0x1C, 0x01, 0x1C, 0x01, 0x1E, 0x01, 0x1E, 0x01,
	0x20, 0x01, 0x20, 0x01, 0x22, 0x01, 0x22, 0x01,
	0x24, 0x01, 0x24, 0x01, 0x26, 0x01, 0x26, 0x01,
	0x28, 0x01, 0x28, 0x01, 0x2A, 0x01, 0x2A, 0x01,
	0x2C, 0x01, 0x2C, 0x01, 0x2E, 0x01, 0x2E, 0x01,
	0x30, 0x01, 0x31, 0x01, 0x32, 0x01, 0x32, 0x01,
	0x34, 0x01, 0x34, 0x01, 0x36, 0x01, 0x36, 0x01,
	0x38, 0x01, 0x39, 0x01, 0x39, 0x01, 0x3B, 0x01,
	0x3B, 0x01, 0x3D, 0x01, 0x3D, 0x01, 0x3F, 0x01,
	0x3F, 0x01, 0x41, 0x01, 0x41, 0x01, 0x43, 0x01,
	0x43, 0x01, 0x45, 0x01, 0x45, 0x01, 0x47, 0x01,
	0x47, 0x01, 0x49, 0x01, 0x4A, 0x01, 0x4A, 0x01,
	0x4C, 0x01, 0x4C, 0x01, 0x4E, 0x01, 0x4E, 0x01,
	0x50, 0x01, 0x50, 0x01, 0x52, 0x01, 0x52, 0x01,
	0x54, 0x01, 0x54, 0x01, 0x56, 0x01, 0x56, 0x01,
	0x58, 0x01, 0x58, 0x01, 0x5A, 0x01, 0x5A, 0x01,
	0x5C, 0x01, 0x5C, 0x01, 0x5E, 0x01, 0x5E, 0x01,
	0x60, 0x01, 0x60, 0x01, 0x62, 0x01, 0x62, 0x01,
	0x64, 0x01, 0x64, 0x01, 0x66, 0x01, 0x66, 0x01,
	0x68, 0x01, 0x68, 0x01, 0x6A, 0x01, 0x6A, 0x01,
	0x6C, 0x01, 0x6C, 0x01, 0x6E, 0x01, 0x6E, 0x01,
	0x70, 0x01, 0x70, 0x01, 0x72, 0x01, 0x72, 0x01,
	0x74, 0x01, 0x74, 0x01, 0x76, 0x01, 0x76, 0x01,
	0x78, 0x01, 0x79, 0x01, 0x79, 0x01, 0x7B, 0x01,
	0x7B, 0x01, 0x7D, 0x01, 0x7D, 0x01, 0x7F, 0x01,
	0x43, 0x02, 0x81, 0x01, 0x82, 0x01, 0x82, 0x01,
	0x84, 0x01, 0x84, 0x01, 0x86, 0x01, 0x87, 0x01,
	0x87, 0x01, 0x89, 0x01, 0x8A, 0x01, 0x8B, 0x01,
	0x8B, 0x01, 0x8D, 0x01, 0x8E, 0x01, 0x8F, 0x01,
	0x90, 0x01, 0x91, 0x01, 0x91, 0x01, 0x93, 0x01,
	0x94, 0x01, 0xF6, 0x01, 0x96, 0x01, 0x97, 0x01,
	0x98, 0x01, 0x98, 0x01, 0x3D, 0x02, 0x9B, 0x01,
	0x9C, 0x01, 0x9D, 0x01, 0x20, 0x02, 0x9F, 0x01,
	0xA0, 0x01, 0xA0, 0x01, 0xA2, 0x01, 0xA2, 0x01,
	0xA4, 0x01, 0xA4, 0x01, 0xA6, 0x01, 0xA7, 0x01,
	0xA7, 0x01, 0xA9, 0x01, 0xAA, 0x01, 0xAB, 0x01,
	0xAC, 0x01, 0xAC, 0x01, 0xAE, 0x01, 0xAF, 0x01,
	0xAF, 0x01, 0xB1, 0x01, 0xB2, 0x01, 0xB3, 0x01,
	0xB3, 0x01, 0xB5, 0x01, 0xB5, 0x01, 0xB7, 0x01,
	0xB8, 0x01, 0xB8, 0x01, 0xBA, 0x01, 0xBB, 0x01,
	0xBC, 0x01, 0xBC, 0x01, 0xBE, 0x01, 0xF7, 0x01,
	0xC0, 0x01, 0xC1, 0x01, 0xC2, 0x01, 0xC3, 0x01,
	0xC4, 0x01, 0xC5, 0x01, 0xC4, 0x01, 0xC7, 0x01,
	0xC8, 0x01, 0xC7, 0x01, 0xCA, 0x01, 0xCB, 0x01,
	0xCA, 0x01, 0xCD, 0x01, 0xCD, 0x01, 0xCF, 0x01,
	0xCF, 0x01, 0xD1, 0x01, 0xD1, 0x01, 0xD3, 0x01,
	0xD3, 0x01, 0xD5, 0x01, 0xD5, 0x01, 0xD7, 0x01,
	0xD7, 0x01, 0xD9, 0x01, 0xD9, 0x01, 0xDB, 0x01,
	0xDB, 0x01, 0x8E, 0x01, 0xDE, 0x01, 0xDE, 0x01,
	0xE0, 0x01, 0xE0, 0x01, 0xE2, 0x01, 0xE2, 0x01,
	0xE4, 0x01, 0xE4, 0x01, 0xE6, 0x01, 0xE6, 0x01,
	0xE8, 0x01, 0xE8, 0x01, 0xEA, 0x01, 0xEA, 0x01,
	0xEC, 0x01, 0xEC, 0x01, 0xEE, 0x01, 0xEE, 0x01,
	0xF0, 0x01, 0xF1, 0x01, 0xF2, 0x01, 0xF1, 0x01,
	0xF4, 0x01, 0xF4, 0x01, 0xF6, 0x01, 0xF7, 0x01,
	0xF8, 0x01, 0xF8, 0x01, 0xFA, 0x01, 0xFA, 0x01,
	0xFC, 0x01, 0xFC, 0x01, 0xFE, 0x01, 0xFE, 0x01,
	0x00, 0x02, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02,
	0x04, 0x02, 0x04, 0x02, 0x06, 0x02, 0x06, 0x02,
	0x08, 0x02, 0x08, 0x02, 0x0A, 0x02, 0x0A, 0x02,
	0x0C, 0x02, 0x0C, 0x02, 0x0E, 0x02, 0x0E, 0x02,
	0x10, 0x02, 0x10, 0x02, 0x12, 0x02, 0x12, 0x02,
	0x14, 0x02, 0x14, 0x02, 0x16, 0x02, 0x16, 0x02,
	0x18, 0x02, 0x18, 0x02, 0x1A, 0x02, 0x1A, 0x02,
	0x1C, 0x02, 0x1C, 0x02, 0x1E, 0x02, 0x1E, 0x02,
	0x20, 0x02, 0x21, 0x02, 0x22, 0x02, 0x22, 0x02,
	0x24, 0x02, 0x24, 0x02, 0x26, 0x02, 0x26, 0x02,
	0x28, 0x02, 0x28, 0x02, 0x2A, 0x02, 0x2A, 0x02,
	0x2C, 0x02, 0x2C, 0x02, 0x2E, 0x02, 0x2E, 0x02,
	0x30, 0x02, 0x30, 0x02, 0x32, 0x02, 0x32, 0x02,
	0x34, 0x02, 0x35, 0x02, 0x36, 0x02, 0x37, 0x02,
	0x38, 0x02, 0x39, 0x02, 0x65, 0x2C, 0x3B, 0x02,
	0x3B, 0x02, 0x3D, 0x02, 0x66, 0x2C, 0x3F, 0x02,
	0x40, 0x02, 0x41, 0x02, 0x41, 0x02, 0x43, 0x02,
	0x44, 0x02, 0x45, 0x02, 0x46, 0x02, 0x46, 0x02,
	0x48, 0x02, 0x48, 0x02, 0x4A, 0x02, 0x4A, 0x02,
	0x4C, 0x02, 0x4C, 0x02, 0x4E, 0x02, 0x4E, 0x02,
	0x50, 0x02, 0x51, 0x02, 0x52, 0x02, 0x81, 0x01,
	0x86, 0x01, 0x55, 0x02, 0x89, 0x01, 0x8A, 0x01,
	0x58, 0x02, 0x8F, 0x01, 0x5A, 0x02, 0x90, 0x01,
	0x5C, 0x02, 0x5D, 0x02, 0x5E, 0x02, 0x5F, 0x02,
	0x93, 0x01, 0x61, 0x02, 0x62, 0x02, 0x94, 0x01,
	0x64, 0x02, 0x65, 0x02, 0x66, 0x02, 0x67, 0x02,
	0x97, 0x01, 0x96, 0x01, 0x6A, 0x02, 0x62, 0x2C,
	0x6C, 0x02, 0x6D, 0x02, 0x6E, 0x02, 0x9C, 0x01,
	0x70, 0x02, 0x71, 0x02, 0x9D, 0x01, 0x73, 0x02,
	0x74, 0x02, 0x9F, 0x01, 0x76, 0x02, 0x77, 0x02,
	0x78, 0x02, 0x79, 0x02, 0x7A, 0x02, 0x7B, 0x02,
	0x7C, 0x02, 0x64, 0x2C, 0x7E, 0x02, 0x7F, 0x02,
	0xA6, 0x01, 0x81, 0x02, 0x82, 0x02, 0xA9, 0x01,
	0x84, 0x02, 0x85, 0x02, 0x86, 0x02, 0x87, 0x02,
	0xAE, 0x01, 0x44, 0x02, 0xB1, 0x01, 0xB2, 0x01,
	0x45, 0x02, 0x8D, 0x02, 0x8E, 0x02, 0x8F, 0x02,
	0x90, 0x02, 0x91, 0x02, 0xB7, 0x01, 0x93, 0x02,
	0x94, 0x02, 0x95, 0x02, 0x96, 0x02, 0x97, 0x02,
	0x98, 0x02, 0x99, 0x02, 0x9A, 0x02, 0x9B, 0x02,
	0x9C, 0x02, 0x9D, 0x02, 0x9E, 0x02, 0x9F, 0x02,
	0xA0, 0x02, 0xA1, 0x02, 0xA2, 0x02, 0xA3, 0x02,
	0xA4, 0x02, 0xA5, 0x02, 0xA6, 0x02, 0xA7, 0x02,
	0xA8, 0x02, 0xA9, 0x02, 0xAA, 0x02, 0xAB, 0x02,
	0xAC, 0x02, 0xAD, 0x02, 0xAE, 0x02, 0xAF, 0x02,
	0xB0, 0x02, 0xB1, 0x02, 0xB2, 0x02, 0xB3, 0x02,
	0xB4, 0x02, 0xB5, 0x02, 0xB6, 0x02, 0xB7, 0x02,
	0xB8, 0x02, 0xB9, 0x02, 0xBA, 0x02, 0xBB, 0x02,
	0xBC, 0x02, 0xBD, 0x02, 0xBE, 0x02, 0xBF, 0x02,
	0xC0, 0x02, 0xC1, 0x02, 0xC2, 0x02, 0xC3, 0x02,
	0xC4, 0x02, 0xC5, 0x02, 0xC6, 0x02, 0xC7, 0x02,
	0xC8, 0x02, 0xC9, 0x02, 0xCA, 0x02, 0xCB, 0x02,
	0xCC, 0x02, 0xCD, 0x02, 0xCE, 0x02, 0xCF, 0x02,
	0xD0, 0x02, 0xD1, 0x02, 0xD2, 0x02, 0xD3, 0x02,
	0xD4, 0x02, 0xD5, 0x02, 0xD6, 0x02, 0xD7, 0x02,
	0xD8, 0x02, 0xD9, 0x02, 0xDA, 0x02, 0xDB, 0x02,
	0xDC, 0x02, 0xDD, 0x02, 0xDE, 0x02, 0xDF, 0x02,
	0xE0, 0x02, 0xE1, 0x02, 0xE2, 0x02, 0xE3, 0x02,
	0xE4, 0x02, 0xE5, 0x02, 0xE6, 0x02, 0xE7, 0x02,
	0xE8, 0x02, 0xE9, 0x02, 0xEA, 0x02, 0xEB, 0x02,
	0xEC, 0x02, 0xED, 0x02, 0xEE, 0x02, 0xEF, 0x02,
	0xF0, 0x02, 0xF1, 0x02, 0xF2, 0x02, 0xF3, 0x02,
	0xF4, 0x02, 0xF5, 0x02, 0xF6, 0x02, 0xF7, 0x02,
	0xF8, 0x02, 0xF9, 0x02, 0xFA, 0x02, 0xFB, 0x02,
	0xFC, 0x02, 0xFD, 0x02, 0xFE, 0x02, 0xFF, 0x02,
	0x00, 0x03, 0x01, 0x03, 0x02, 0x03, 0x03, 0x03,
	0x04, 0x03, 0x05, 0x03, 0x06, 0x03, 0x07, 0x03,
	0x08, 0x03, 0x09, 0x03, 0x0A, 0x03, 0x0B, 0x03,
	0x0C, 0x03, 0x0D, 0x03, 0x0E, 0x03, 0x0F, 0x03,
	0x10, 0x03, 0x11, 0x03, 0x12, 0x03, 0x13, 0x03,
	0x14, 0x03, 0x15, 0x03, 0x16, 0x03, 0x17, 0x03,
	0x18, 0x03, 0x19, 0x03, 0x1A, 0x03, 0x1B, 0x03,
	0x1C, 0x03, 0x1D, 0x03, 0x1E, 0x03, 0x1F, 0x03,
	0x20, 0x03, 0x21, 0x03, 0x22, 0x03, 0x23, 0x03,
	0x24, 0x03, 0x25, 0x03, 0x26, 0x03, 0x27, 0x03,
	0x28, 0x03, 0x29, 0x03, 0x2A, 0x03, 0x2B, 0x03,
	0x2C, 0x03, 0x2D, 0x03, 0x2E, 0x03, 0x2F, 0x03,
	0x30, 0x03, 0x31, 0x03, 0x32, 0x03, 0x33, 0x03,
	0x34, 0x03, 0x35, 0x03, 0x36, 0x03, 0x37, 0x03,
	0x38, 0x03, 0x39, 0x03, 0x3A, 0x03, 0x3B, 0x03,
	0x3C, 0x03, 0x3D, 0x03, 0x3E, 0x03, 0x3F, 0x03,
	0x40, 0x03, 0x41, 0x03, 0x42, 0x03, 0x43, 0x03,
	0x44, 0x03, 0x45, 0x03, 0x46, 0x03, 0x47, 0x03,
	0x48, 0x03, 0x49, 0x03, 0x4A, 0x03, 0x4B, 0x03,
	0x4C, 0x03, 0x4D, 0x03, 0x4E, 0x03, 0x4F, 0x03,
	0x50, 0x03, 0x51, 0x03, 0x52, 0x03, 0x53, 0x03,
	0x54, 0x03, 0x55, 0x03, 0x56, 0x03, 0x57, 0x03,
	0x58, 0x03, 0x59, 0x03, 0x5A, 0x03, 0x5B, 0x03,
	0x5C, 0x03, 0x5D, 0x03, 0x5E, 0x03, 0x5F, 0x03,
	0x60, 0x03, 0x61, 0x03, 0x62, 0x03, 0x63, 0x03,
	0x64, 0x03, 0x65, 0x03, 0x66, 0x03, 0x67, 0x03,
	0x68, 0x03, 0x69, 0x03, 0x6A, 0x03, 0x6B, 0x03,
	0x6C, 0x03, 0x6D, 0x03, 0x6E, 0x03, 0x6F, 0x03,
	0x70, 0x03, 0x71, 0x03, 0x72, 0x03, 0x73, 0x03,
	0x74, 0x03, 0x75, 0x03, 0x76, 0x03, 0x77, 0x03,
	0x78, 0x03, 0x79, 0x03, 0x7A, 0x03, 0xFD, 0x03,
	0xFE, 0x03, 0xFF, 0x03, 0x7E, 0x03, 0x7F, 0x03,
	0x80, 0x03, 0x81, 0x03, 0x82, 0x03, 0x83, 0x03,
	0x84, 0x03, 0x85, 0x03, 0x86, 0x03, 0x87, 0x03,
	0x88, 0x03, 0x89, 0x03, 0x8A, 0x03, 0x8B, 0x03,
	0x8C, 0x03, 0x8D, 0x03, 0x8E, 0x03, 0x8F, 0x03,
	0x90, 0x03, 0x91, 0x03, 0x92, 0x03, 0x93, 0x03,
	0x94, 0x03, 0x95, 0x03, 0x96, 0x03, 0x97, 0x03,
	0x98, 0x03, 0x99, 0x03, 0x9A, 0x03, 0x9B, 0x03,
	0x9C, 0x03, 0x9D, 0x03, 0x9E, 0x03, 0x9F, 0x03,
	0xA0, 0x03, 0xA1, 0x03, 0xA2, 0x03, 0xA3, 0x03,
	0xA4, 0x03, 0xA5, 0x03, 0xA6, 0x03, 0xA7, 0x03,
	0xA8, 0x03, 0xA9, 0x03, 0xAA, 0x03, 0xAB, 0x03,
	0x86, 0x03, 0x88, 0x03, 0x89, 0x03, 0x8A, 0x03,
	0xB0, 0x03, 0x91, 0x03, 0x92, 0x03, 0x93, 0x03,
	0x94, 0x03, 0x95, 0x03, 0x96, 0x03, 0x97, 0x03,
	0x98, 0x03, 0x99, 0x03, 0x9A, 0x03, 0x9B, 0x03,
	0x9C, 0x03, 0x9D, 0x03, 0x9E, 0x03, 0x9F, 0x03,
	0xA0, 0x03, 0xA1, 0x03, 0xA3, 0x03, 0xA3, 0x03,
	0xA4, 0x03, 0xA5, 0x03, 0xA6, 0x03, 0xA7, 0x03,
	0xA8, 0x03, 0xA9, 0x03, 0xAA, 0x03, 0xAB, 0x03,
	0x8C, 0x03, 0x8E, 0x03, 0x8F, 0x03, 0xCF, 0x03,
	0xD0, 0x03, 0xD1, 0x03, 0xD2, 0x03, 0xD3, 0x03,
	0xD4, 0x03, 0xD5, 0x03, 0xD6, 0x03, 0xD7, 0x03,
	0xD8, 0x03, 0xD8, 0x03, 0xDA, 0x03, 0xDA, 0x03,
	0xDC, 0x03, 0xDC, 0x03, 0xDE, 0x03, 0xDE, 0x03,
	0xE0, 0x03, 0xE0, 0x03, 0xE2, 0x03, 0xE2, 0x03,
	0xE4, 0x03, 0xE4, 0x03, 0xE6, 0x03, 0xE6, 0x03,
	0xE8, 0x03, 0xE8, 0x03, 0xEA, 0x03, 0xEA, 0x03,
	0xEC, 0x03, 0xEC, 0x03, 0xEE, 0x03, 0xEE, 0x03,
	0xF0, 0x03, 0xF1, 0x03, 0xF9, 0x03, 0xF3, 0x03,
	0xF4, 0x03, 0xF5, 0x03, 0xF6, 0x03, 0xF7, 0x03,
	0xF7, 0x03, 0xF9, 0x03, 0xFA, 0x03, 0xFA, 0x03,
	0xFC, 0x03, 0xFD, 0x03, 0xFE, 0x03, 0xFF, 0x03,
	0x00, 0x04, 0x01, 0x04, 0x02, 0x04, 0x03, 0x04,
	0x04, 0x04, 0x05, 0x04, 0x06, 0x04, 0x07, 0x04,
	0x08, 0x04, 0x09, 0x04, 0x0A, 0x04, 0x0B, 0x04,
	0x0C, 0x04, 0x0D, 0x04, 0x0E, 0x04, 0x0F, 0x04,
	0x10, 0x04, 0x11, 0x04, 0x12, 0x04, 0x13, 0x04,
	0x14, 0x04, 0x15, 0x04, 0x16, 0x04, 0x17, 0x04,
	0x18, 0x04, 0x19, 0x04, 0x1A, 0x04, 0x1B, 0x04,
	0x1C, 0x04, 0x1D, 0x04, 0x1E, 0x04, 0x1F, 0x04,
	0x20, 0x04, 0x21, 0x04, 0x22, 0x04, 0x23, 0x04,
	0x24, 0x04, 0x25, 0x04, 0x26, 0x04, 0x27, 0x04,
	0x28, 0x04, 0x29, 0x04, 0x2A, 0x04, 0x2B, 0x04,
	0x2C, 0x04, 0x2D, 0x04, 0x2E, 0x04, 0x2F, 0x04,
	0x10, 0x04, 0x11, 0x04, 0x12, 0x04, 0x13, 0x04,
	0x14, 0x04, 0x15, 0x04, 0x16, 0x04, 0x17, 0x04,
	0x18, 0x04, 0x19, 0x04, 0x1A, 0x04, 0x1B, 0x04,
	0x1C, 0x04, 0x1D, 0x04, 0x1E, 0x04, 0x1F, 0x04,
	0x20, 0x04, 0x21, 0x04, 0x22, 0x04, 0x23, 0x04,
	0x24, 0x04, 0x25, 0x04, 0x26, 0x04, 0x27, 0x04,
	0x28, 0x04, 0x29, 0x04, 0x2A, 0x04, 0x2B, 0x04,
	0x2C, 0x04, 0x2D, 0x04, 0x2E, 0x04, 0x2F, 0x04,
	0x00, 0x04, 0x01, 0x04, 0x02, 0x04, 0x03, 0x04,
	0x04, 0x04, 0x05, 0x04, 0x06, 0x04, 0x07, 0x04,
	0x08, 0x04, 0x09, 0x04, 0x0A, 0x04, 0x0B, 0x04,
	0x0C, 0x04, 0x0D, 0x04, 0x0E, 0x04, 0x0F, 0x04,
	0x60, 0x04, 0x60, 0x04, 0x62, 0x04, 0x62, 0x04,
	0x64, 0x04, 0x64, 0x04, 0x66, 0x04, 0x66, 0x04,
	0x68, 0x04, 0x68, 0x04, 0x6A, 0x04, 0x6A, 0x04,
	0x6C, 0x04, 0x6C, 0x04, 0x6E, 0x04, 0x6E, 0x04,
	0x70, 0x04, 0x70, 0x04, 0x72, 0x04, 0x72, 0x04,
	0x74, 0x04, 0x74, 0x04, 0x76, 0x04, 0x76, 0x04,
	0x78, 0x04, 0x78, 0x04, 0x7A, 0x04, 0x7A, 0x04,
	0x7C, 0x04, 0x7C, 0x04, 0x7E, 0x04, 0x7E, 0x04,
	0x80, 0x04, 0x80, 0x04, 0x82, 0x04, 0x83, 0x04,
	0x84, 0x04, 0x85, 0x04, 0x86, 0x04, 0x87, 0x04,
	0x88, 0x04, 0x89, 0x04, 0x8A, 0x04, 0x8A, 0x04,
	0x8C, 0x04, 0x8C, 0x04, 0x8E, 0x04, 0x8E, 0x04,
	0x90, 0x04, 0x90, 0x04, 0x92, 0x04, 0x92, 0x04,
	0x94, 0x04, 0x94, 0x04, 0x96, 0x04, 0x96, 0x04,
	0x98, 0x04, 0x98, 0x04, 0x9A, 0x04, 0x9A, 0x04,
	0x9C, 0x04, 0x9C, 0x04, 0x9E, 0x04, 0x9E, 0x04,
	0xA0, 0x04, 0xA0, 0x04, 0xA2, 0x04, 0xA2, 0x04,
	0xA4, 0x04, 0xA4, 0x04, 0xA6, 0x04, 0xA6, 0x04,
	0xA8, 0x04, 0xA8, 0x04, 0xAA, 0x04, 0xAA, 0x04,
	0xAC, 0x04, 0xAC, 0x04, 0xAE, 0x04, 0xAE, 0x04,
	0xB0, 0x04, 0xB0, 0x04, 0xB2, 0x04, 0xB2, 0x04,
	0xB4, 0x04, 0xB4, 0x04, 0xB6, 0x04, 0xB6, 0x04,
	0xB8, 0x04, 0xB8, 0x04, 0xBA, 0x04, 0xBA, 0x04,
	0xBC, 0x04, 0xBC, 0x04, 0xBE, 0x04, 0xBE, 0x04,
	0xC0, 0x04, 0xC1, 0x04, 0xC1, 0x04, 0xC3, 0x04,
	0xC3, 0x04, 0xC5, 0x04, 0xC5, 0x04, 0xC7, 0x04,
	0xC7, 0x04, 0xC9, 0x04, 0xC9, 0x04, 0xCB, 0x04,
	0xCB, 0x04, 0xCD, 0x04, 0xCD, 0x04, 0xC0, 0x04,
	0xD0, 0x04, 0xD0, 0x04, 0xD2, 0x04, 0xD2, 0x04,
	0xD4, 0x04, 0xD4, 0x04, 0xD6, 0x04, 0xD6, 0x04,
	0xD8, 0x04, 0xD8, 0x04, 0xDA, 0x04, 0xDA, 0x04,
	0xDC, 0x04, 0xDC, 0x04, 0xDE, 0x04, 0xDE, 0x04,
	0xE0, 0x04, 0xE0, 0x04, 0xE2, 0x04, 0xE2, 0x04,
	0xE4, 0x04, 0xE4, 0x04, 0xE6, 0x04, 0xE6, 0x04,
	0xE8, 0x04, 0xE8, 0x04, 0xEA, 0x04, 0xEA, 0x04,
	0xEC, 0x04, 0xEC, 0x04, 0xEE, 0x04, 0xEE, 0x04,
	0xF0, 0x04, 0xF0, 0x04, 0xF2, 0x04, 0xF2, 0x04,
	0xF4, 0x04, 0xF4, 0x04, 0xF6, 0x04, 0xF6, 0x04,
	0xF8, 0x04, 0xF8, 0x04, 0xFA, 0x04, 0xFA, 0x04,
	0xFC, 0x04, 0xFC, 0x04, 0xFE, 0x04, 0xFE, 0x04,
	0x00, 0x05, 0x00, 0x05, 0x02, 0x05, 0x02, 0x05,
	0x04, 0x05, 0x04, 0x05, 0x06, 0x05, 0x06, 0x05,
	0x08, 0x05, 0x08, 0x05, 0x0A, 0x05, 0x0A, 0x05,
	0x0C, 0x05, 0x0C, 0x05, 0x0E, 0x05, 0x0E, 0x05,
	0x10, 0x05, 0x10, 0x05, 0x12, 0x05, 0x12, 0x05,
	0x14, 0x05, 0x15, 0x05, 0x16, 0x05, 0x17, 0x05,
	0x18, 0x05, 0x19, 0x05, 0x1A, 0x05, 0x1B, 0x05,
	0x1C, 0x05, 0x1D, 0x05, 0x1E, 0x05, 0x1F, 0x05,
	0x20, 0x05, 0x21, 0x05, 0x22, 0x05, 0x23, 0x05,
	0x24, 0x05, 0x25, 0x05, 0x26, 0x05, 0x27, 0x05,
	0x28, 0x05, 0x29, 0x05, 0x2A, 0x05, 0x2B, 0x05,
	0x2C, 0x05, 0x2D, 0x05, 0x2E, 0x05, 0x2F, 0x05,
	0x30, 0x05, 0x31, 0x05, 0x32, 0x05, 0x33, 0x05,
	0x34, 0x05, 0x35, 0x05, 0x36, 0x05, 0x37, 0x05,
	0x38, 0x05, 0x39, 0x05, 0x3A, 0x05, 0x3B, 0x05,
	0x3C, 0x05, 0x3D, 0x05, 0x3E, 0x05, 0x3F, 0x05,
	0x40, 0x05, 0x41, 0x05, 0x42, 0x05, 0x43, 0x05,
	0x44, 0x05, 0x45, 0x05, 0x46, 0x05, 0x47, 0x05,
	0x48, 0x05, 0x49, 0x05, 0x4A, 0x05, 0x4B, 0x05,
	0x4C, 0x05, 0x4D, 0x05, 0x4E, 0x05, 0x4F, 0x05,
	0x50, 0x05, 0x51, 0x05, 0x52, 0x05, 0x53, 0x05,
	0x54, 0x05, 0x55, 0x05, 0x56, 0x05, 0x57, 0x05,
	0x58, 0x05, 0x59, 0x05, 0x5A, 0x05, 0x5B, 0x05,
	0x5C, 0x05, 0x5D, 0x05, 0x5E, 0x05, 0x5F, 0x05,
	0x60, 0x05, 0x31, 0x05, 0x32, 0x05, 0x33, 0x05,
	0x34, 0x05, 0x35, 0x05, 0x36, 0x05, 0x37, 0x05,
	0x38, 0x05, 0x39, 0x05, 0x3A, 0x05, 0x3B, 0x05,
	0x3C, 0x05, 0x3D, 0x05, 0x3E, 0x05, 0x3F, 0x05,
	0x40, 0x05, 0x41, 0x05, 0x42, 0x05, 0x43, 0x05,
	0x44, 0x05, 0x45, 0x05, 0x46, 0x05, 0x47, 0x05,
	0x48, 0x05, 0x49, 0x05, 0x4A, 0x05, 0x4B, 0x05,
	0x4C, 0x05, 0x4D, 0x05, 0x4E, 0x05, 0x4F, 0x05,
	0x50, 0x05, 0x51, 0x05, 0x52, 0x05, 0x53, 0x05,
	0x54, 0x05, 0x55, 0x05, 0x56, 0x05, 0xFF, 0xFF,
	0xF6, 0x17, 0x63, 0x2C, 0x7E, 0x1D, 0x7F, 0x1D,
	0x80, 0x1D, 0x81, 0x1D, 0x82, 0x1D, 0x83, 0x1D,
	0x84, 0x1D, 0x85, 0x1D, 0x86, 0x1D, 0x87, 0x1D,
	0x88, 0x1D, 0x89, 0x1D, 0x8A, 0x1D, 0x8B, 0x1D,
	0x8C, 0x1D, 0x8D, 0x1D, 0x8E, 0x1D, 0x8F, 0x1D,
	0x90, 0x1D, 0x91, 0x1D, 0x92, 0x1D, 0x93, 0x1D,
	0x94, 0x1D, 0x95, 0x1D, 0x96, 0x1D, 0x97, 0x1D,
	0x98, 0x1D, 0x99, 0x1D, 0x9A, 0x1D, 0x9B, 0x1D,
	0x9C, 0x1D, 0x9D, 0x1D, 0x9E, 0x1D, 0x9F, 0x1D,
	0xA0, 0x1D, 0xA1, 0x1D, 0xA2, 0x1D, 0xA3, 0x1D,
	0xA4, 0x1D, 0xA5, 0x1D, 0xA6, 0x1D, 0xA7, 0x1D,
	0xA8, 0x1D, 0xA9, 0x1D, 0xAA, 0x1D, 0xAB, 0x1D,
	0xAC, 0x1D, 0xAD, 0x1D, 0xAE, 0x1D, 0xAF, 0x1D,
	0xB0, 0x1D, 0xB1, 0x1D, 0xB2, 0x1D, 0xB3, 0x1D,
	0xB4, 0x1D, 0xB5, 0x1D, 0xB6, 0x1D, 0xB7, 0x1D,
	0xB8, 0x1D, 0xB9, 0x1D, 0xBA, 0x1D, 0xBB, 0x1D,
	0xBC, 0x1D, 0xBD, 0x1D, 0xBE, 0x1D, 0xBF, 0x1D,
	0xC0, 0x1D, 0xC1, 0x1D, 0xC2, 0x1D, 0xC3, 0x1D,
	0xC4, 0x1D, 0xC5, 0x1D, 0xC6, 0x1D, 0xC7, 0x1D,
	0xC8, 0x1D, 0xC9, 0x1D, 0xCA, 0x1D, 0xCB, 0x1D,
	0xCC, 0x1D, 0xCD, 0x1D, 0xCE, 0x1D, 0xCF, 0x1D,
	0xD0, 0x1D, 0xD1, 0x1D, 0xD2, 0x1D, 0xD3, 0x1D,
	0xD4, 0x1D, 0xD5, 0x1D, 0xD6, 0x1D, 0xD7, 0x1D,
	0xD8, 0x1D, 0xD9, 0x1D, 0xDA, 0x1D, 0xDB, 0x1D,
	0xDC, 0x1D, 0xDD, 0x1D, 0xDE, 0x1D, 0xDF, 0x1D,
	0xE0, 0x1D, 0xE1, 0x1D, 0xE2, 0x1D, 0xE3, 0x1D,
	0xE4, 0x1D, 0xE5, 0x1D, 0xE6, 0x1D, 0xE7, 0x1D,
	0xE8, 0x1D, 0xE9, 0x1D, 0xEA, 0x1D, 0xEB, 0x1D,
	0xEC, 0x1D, 0xED, 0x1D, 0xEE, 0x1D, 0xEF, 0x1D,
	0xF0, 0x1D, 0xF1, 0x1D, 0xF2, 0x1D, 0xF3, 0x1D,
	0xF4, 0x1D, 0xF5, 0x1D, 0xF6, 0x1D, 0xF7, 0x1D,
	0xF8, 0x1D, 0xF9, 0x1D, 0xFA, 0x1D, 0xFB, 0x1D,
	0xFC, 0x1D, 0xFD, 0x1D, 0xFE, 0x1D, 0xFF, 0x1D,
	0x00, 0x1E, 0x00, 0x1E, 0x02, 0x1E, 0x02, 0x1E,
	0x04, 0x1E, 0x04, 0x1E, 0x06, 0x1E, 0x06, 0x1E,
	0x08, 0x1E, 0x08, 0x1E, 0x0A, 0x1E, 0x0A, 0x1E,
	0x0C, 0x1E, 0x0C, 0x1E, 0x0E, 0x1E, 0x0E, 0x1E,
	0x10, 0x1E, 0x10, 0x1E, 0x12, 0x1E, 0x12, 0x1E,
	0x14, 0x1E, 0x14, 0x1E, 0x16, 0x1E, 0x16, 0x1E,
	0x18, 0x1E, 0x18, 0x1E, 0x1A, 0x1E, 0x1A, 0x1E,
	0x1C, 0x1E, 0x1C, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
	0x20, 0x1E, 0x20, 0x1E, 0x22, 0x1E, 0x22, 0x1E,
	0x24, 0x1E, 0x24, 0x1E, 0x26, 0x1E, 0x26, 0x1E,
	0x28, 0x1E, 0x28, 0x1E, 0x2A, 0x1E, 0x2A, 0x1E,
	0x2C, 0x1E, 0x2C, 0x1E, 0x2E, 0x1E, 0x2E, 0x1E,
	0x30, 0x1E, 0x30, 0x1E, 0x32, 0x1E, 0x32, 0x1E,
	0x34, 0x1E, 0x34, 0x1E, 0x36, 0x1E, 0x36, 0x1E,
	0x38, 0x1E, 0x38, 0x1E, 0x3A, 0x1E, 0x3A, 0x1E,
	0x3C, 0x1E, 0x3C, 0x1E, 0x3E, 0x1E, 0x3E, 0x1E,
	0x40, 0x1E, 0x40, 0x1E, 0x42, 0x1E, 0x42, 0x1E,
	0x44, 0x1E, 0x44, 0x1E, 0x46, 0x1E, 0x46, 0x1E,
	0x48, 0x1E, 0x48, 0x1E, 0x4A, 0x1E, 0x4A, 0x1E,
	0x4C, 0x1E, 0x4C, 0x1E, 0x4E, 0x1E, 0x4E, 0x1E,
	0x50, 0x1E, 0x50, 0x1E, 0x52, 0x1E, 0x52, 0x1E,
	0x54, 0x1E, 0x54, 0x1E, 0x56, 0x1E, 0x56, 0x1E,
	0x58, 0x1E, 0x58, 0x1E, 0x5A, 0x1E, 0x5A, 0x1E,
	0x5C, 0x1E, 0x5C, 0x1E, 0x5E, 0x1E, 0x5E, 0x1E,
	0x60, 0x1E, 0x60, 0x1E, 0x62, 0x1E, 0x62, 0x1E,
	0x64, 0x1E, 0x64, 0x1E, 0x66, 0x1E, 0x66, 0x1E,
	0x68, 0x1E, 0x68, 0x1E, 0x6A, 0x1E, 0x6A, 0x1E,
	0x6C, 0x1E, 0x6C, 0x1E, 0x6E, 0x1E, 0x6E, 0x1E,
	0x70, 0x1E, 0x70, 0x1E, 0x72, 0x1E, 0x72, 0x1E,
	0x74, 0x1E, 0x74, 0x1E, 0x76, 0x1E, 0x76, 0x1E,
	0x78, 0x1E, 0x78, 0x1E, 0x7A, 0x1E, 0x7A, 0x1E,
	0x7C, 0x1E, 0x7C, 0x1E, 0x7E, 0x1E, 0x7E, 0x1E,
	0x80, 0x1E, 0x80, 0x1E, 0x82, 0x1E, 0x82, 0x1E,
	0x84, 0x1E, 0x84, 0x1E, 0x86, 0x1E, 0x86, 0x1E,
	0x88, 0x1E, 0x88, 0x1E, 0x8A, 0x1E, 0x8A, 0x1E,
	0x8C, 0x1E, 0x8C, 0x1E, 0x8E, 0x1E, 0x8E, 0x1E,
	0x90, 0x1E, 0x90, 0x1E, 0x92, 0x1E, 0x92, 0x1E,
	0x94, 0x1E, 0x94, 0x1E, 0x96, 0x1E, 0x97, 0x1E,
	0x98, 0x1E, 0x99, 0x1E, 0x9A, 0x1E, 0x9B, 0x1E,
	0x9C, 0x1E, 0x9D, 0x1E, 0x9E, 0x1E, 0x9F, 0x1E,
	0xA0, 0x1E, 0xA0, 0x1E, 0xA2, 0x1E, 0xA2, 0x1E,
	0xA4, 0x1E, 0xA4, 0x1E, 0xA6, 0x1E, 0xA6, 0x1E,
	0xA8, 0x1E, 0xA8, 0x1E, 0xAA, 0x1E, 0xAA, 0x1E,
	0xAC, 0x1E, 0xAC, 0x1E, 0xAE, 0x1E, 0xAE, 0x1E,
	0xB0, 0x1E, 0xB0, 0x1E, 0xB2, 0x1E, 0xB2, 0x1E,
	0xB4, 0x1E, 0xB4, 0x1E, 0xB6, 0x1E, 0xB6, 0x1E,
	0xB8, 0x1E, 0xB8, 0x1E, 0xBA, 0x1E, 0xBA, 0x1E,
	0xBC, 0x1E, 0xBC, 0x1E, 0xBE, 0x1E, 0xBE, 0x1E,
	0xC0, 0x1E, 0xC0, 0x1E, 0xC2, 0x1E, 0xC2, 0x1E,
	0xC4, 0x1E, 0xC4, 0x1E, 0xC6, 0x1E, 0xC6, 0x1E,
	0xC8, 0x1E, 0xC8, 0x1E, 0xCA, 0x1E, 0xCA, 0x1E,
	0xCC, 0x1E, 0xCC, 0x1E, 0xCE, 0x1E, 0xCE, 0x1E,
	0xD0, 0x1E, 0xD0, 0x1E, 0xD2, 0x1E, 0xD2, 0x1E,
	0xD4, 0x1E, 0xD4, 0x1E, 0xD6, 0x1E, 0xD6, 0x1E,
	0xD8, 0x1E, 0xD8, 0x1E, 0xDA, 0x1E, 0xDA, 0x1E,
	0xDC, 0x1E, 0xDC, 0x1E, 0xDE, 0x1E, 0xDE, 0x1E,
	0xE0, 0x1E, 0xE0, 0x1E, 0xE2, 0x1E, 0xE2, 0x1E,
	0xE4, 0x1E, 0xE4, 0x1E, 0xE6, 0x1E, 0xE6, 0x1E,
	0xE8, 0x1E, 0xE8, 0x1E, 0xEA, 0x1E, 0xEA, 0x1E,
	0xEC, 0x1E, 0xEC, 0x1E, 0xEE, 0x1E, 0xEE, 0x1E,
	0xF0, 0x1E, 0xF0, 0x1E, 0xF2, 0x1E, 0xF2, 0x1E,
	0xF4, 0x1E, 0xF4, 0x1E, 0xF6, 0x1E, 0xF6, 0x1E,
	0xF8, 0x1E, 0xF8, 0x1E, 0xFA, 0x1E, 0xFB, 0x1E,
	0xFC, 0x1E, 0xFD, 0x1E, 0xFE, 0x1E, 0xFF, 0x1E,
	0x08, 0x1F, 0x09, 0x1F, 0x0A, 0x1F, 0x0B, 0x1F,
	0x0C, 0x1F, 0x0D, 0x1F, 0x0E, 0x1F, 0x0F, 0x1F,
	0x08, 0x1F, 0x09, 0x1F, 0x0A, 0x1F, 0x0B, 0x1F,
	0x0C, 0x1F, 0x0D, 0x1F, 0x0E, 0x1F, 0x0F, 0x1F,
	0x18, 0x1F, 0x19, 0x1F, 0x1A, 0x1F, 0x1B, 0x1F,
	0x1C, 0x1F, 0x1D, 0x1F, 0x16, 0x1F, 0x17, 0x1F,
	0x18, 0x1F, 0x19, 0x1F, 0x1A, 0x1F, 0x1B, 0x1F,
	0x1C, 0x1F, 0x1D, 0x1F, 0x1E, 0x1F, 0x1F, 0x1F,
	0x28, 0x1F, 0x29, 0x1F, 0x2A, 0x1F, 0x2B, 0x1F,
	0x2C, 0x1F, 0x2D, 0x1F, 0x2E, 0x1F, 0x2F, 0x1F,
	0x28, 0x1F, 0x29, 0x1F, 0x2A, 0x1F, 0x2B, 0x1F,
	0x2C, 0x1F, 0x2D, 0x1F, 0x2E, 0x1F, 0x2F, 0x1F,
	0x38, 0x1F, 0x39, 0x1F, 0x3A, 0x1F, 0x3B, 0x1F,
	0x3C, 0x1F, 0x3D, 0x1F, 0x3E, 0x1F, 0x3F, 0x1F,
	0x38, 0x1F, 0x39, 0x1F, 0x3A, 0x1F, 0x3B, 0x1F,
	0x3C, 0x1F, 0x3D, 0x1F, 0x3E, 0x1F, 0x3F, 0x1F,
	0x48, 0x1F, 0x49, 0x1F, 0x4A, 0x1F, 0x4B, 0x1F,
	0x4C, 0x1F, 0x4D, 0x1F, 0x46, 0x1F, 0x47, 0x1F,
	0x48, 0x1F, 0x49, 0x1F, 0x4A, 0x1F, 0x4B, 0x1F,
	0x4C, 0x1F, 0x4D, 0x1F, 0x4E, 0x1F, 0x4F, 0x1F,
	0x50, 0x1F, 0x59, 0x1F, 0x52, 0x1F, 0x5B, 0x1F,
	0x54, 0x1F, 0x5D, 0x1F, 0x56, 0x1F, 0x5F, 0x1F,
	0x58, 0x1F, 0x59, 0x1F, 0x5A, 0x1F, 0x5B, 0x1F,
	0x5C, 0x1F, 0x5D, 0x1F, 0x5E, 0x1F, 0x5F, 0x1F,
	0x68, 0x1F, 0x69, 0x1F, 0x6A, 0x1F, 0x6B, 0x1F,
	0x6C, 0x1F, 0x6D, 0x1F, 0x6E, 0x1F, 0x6F, 0x1F,
	0x68, 0x1F, 0x69, 0x1F, 0x6A, 0x1F, 0x6B, 0x1F,
	0x6C, 0x1F, 0x6D, 0x1F, 0x6E, 0x1F, 0x6F, 0x1F,
	0xBA, 0x1F, 0xBB, 0x1F, 0xC8, 0x1F, 0xC9, 0x1F,
	0xCA, 0x1F, 0xCB, 0x1F, 0xDA, 0x1F, 0xDB, 0x1F,
	0xF8, 0x1F, 0xF9, 0x1F, 0xEA, 0x1F, 0xEB, 0x1F,
	0xFA, 0x1F, 0xFB, 0x1F, 0x7E, 0x1F, 0x7F, 0x1F,
	0x88, 0x1F, 0x89, 0x1F, 0x8A, 0x1F, 0x8B, 0x1F,
	0x8C, 0x1F, 0x8D, 0x1F, 0x8E, 0x1F, 0x8F, 0x1F,
	0x88, 0x1F, 0x89, 0x1F, 0x8A, 0x1F, 0x8B, 0x1F,
	0x8C, 0x1F, 0x8D, 0x1F, 0x8E, 0x1F, 0x8F, 0x1F,
	0x98, 0x1F, 0x99, 0x1F, 0x9A, 0x1F, 0x9B, 0x1F,
	0x9C, 0x1F, 0x9D, 0x1F, 0x9E, 0x1F, 0x9F, 0x1F,
	0x98, 0x1F, 0x99, 0x1F, 0x9A, 0x1F, 0x9B, 0x1F,
	0x9C, 0x1F, 0x9D, 0x1F, 0x9E, 0x1F, 0x9F, 0x1F,
	0xA8, 0x1F, 0xA9, 0x1F, 0xAA, 0x1F, 0xAB, 0x1F,
	0xAC, 0x1F, 0xAD, 0x1F, 0xAE, 0x1F, 0xAF, 0x1F,
	0xA8, 0x1F, 0xA9, 0x1F, 0xAA, 0x1F, 0xAB, 0x1F,
	0xAC, 0x1F, 0xAD, 0x1F, 0xAE, 0x1F, 0xAF, 0x1F,
	0xB8, 0x1F, 0xB9, 0x1F, 0xB2, 0x1F, 0xBC, 0x1F,
	0xB4, 0x1F, 0xB5, 0x1F, 0xB6, 0x1F, 0xB7, 0x1F,
	0xB8, 0x1F, 0xB9, 0x1F, 0xBA, 0x1F, 0xBB, 0x1F,
	0xBC, 0x1F, 0xBD, 0x1F, 0xBE, 0x1F, 0xBF, 0x1F,
	0xC0, 0x1F, 0xC1, 0x1F, 0xC2, 0x1F, 0xC3, 0x1F,
	0xC4, 0x1F, 0xC5, 0x1F, 0xC6, 0x1F, 0xC7, 0x1F,
	0xC8, 0x1F, 0xC9, 0x1F, 0xCA, 0x1F, 0xCB, 0x1F,
	0xC3, 0x1F, 0xCD, 0x1F, 0xCE, 0x1F, 0xCF, 0x1F,
	0xD8, 0x1F, 0xD9, 0x1F, 0xD2, 0x1F, 0xD3, 0x1F,
	0xD4, 0x1F, 0xD5, 0x1F, 0xD6, 0x1F, 0xD7, 0x1F,
	0xD8, 0x1F, 0xD9, 0x1F, 0xDA, 0x1F, 0xDB, 0x1F,
	0xDC, 0x1F, 0xDD, 0x1F, 0xDE, 0x1F, 0xDF, 0x1F,
	0xE8, 0x1F, 0xE9, 0x1F, 0xE2, 0x1F, 0xE3, 0x1F,
	0xE4, 0x1F, 0xEC, 0x1F, 0xE6, 0x1F, 0xE7, 0x1F,
	0xE8, 0x1F, 0xE9, 0x1F, 0xEA, 0x1F, 0xEB, 0x1F,
	0xEC, 0x1F, 0xED, 0x1F, 0xEE, 0x1F, 0xEF, 0x1F,
	0xF0, 0x1F, 0xF1, 0x1F, 0xF2, 0x1F, 0xF3, 0x1F,
	0xF4, 0x1F, 0xF5, 0x1F, 0xF6, 0x1F, 0xF7, 0x1F,
	0xF8, 0x1F, 0xF9, 0x1F, 0xFA, 0x1F, 0xFB, 0x1F,
	0xF3, 0x1F, 0xFD, 0x1F, 0xFE, 0x1F, 0xFF, 0x1F,
	0x00, 0x20, 0x01, 0x20, 0x02, 0x20, 0x03, 0x20,
	0x04, 0x20, 0x05, 0x20, 0x06, 0x20, 0x07, 0x20,
	0x08, 0x20, 0x09, 0x20, 0x0A, 0x20, 0x0B, 0x20,
	0x0C, 0x20, 0x0D, 0x20, 0x0E, 0x20, 0x0F, 0x20,
	0x10, 0x20, 0x11, 0x20, 0x12, 0x20, 0x13, 0x20,
	0x14, 0x20, 0x15, 0x20, 0x16, 0x20, 0x17, 0x20,
	0x18, 0x20, 0x19, 0x20, 0x1A, 0x20, 0x1B, 0x20,
	0x1C, 0x20, 0x1D, 0x20, 0x1E, 0x20, 0x1F, 0x20,
	0x20, 0x20, 0x21, 0x20, 0x22, 0x20, 0x23, 0x20,
	0x24, 0x20, 0x25, 0x20, 0x26, 0x20, 0x27, 0x20,
	0x28, 0x20, 0x29, 0x20, 0x2A, 0x20, 0x2B, 0x20,
	0x2C, 0x20, 0x2D, 0x20, 0x2E, 0x20, 0x2F, 0x20,
	0x30, 0x20, 0x31, 0x20, 0x32, 0x20, 0x33, 0x20,
	0x34, 0x20, 0x35, 0x20, 0x36, 0x20, 0x37, 0x20,
	0x38, 0x20, 0x39, 0x20, 0x3A, 0x20, 0x3B, 0x20,
	0x3C, 0x20, 0x3D, 0x20, 0x3E, 0x20, 0x3F, 0x20,
	0x40, 0x20, 0x41, 0x20, 0x42, 0x20, 0x43, 0x20,
	0x44, 0x20, 0x45, 0x20, 0x46, 0x20, 0x47, 0x20,
	0x48, 0x20, 0x49, 0x20, 0x4A, 0x20, 0x4B, 0x20,
	0x4C, 0x20, 0x4D, 0x20, 0x4E, 0x20, 0x4F, 0x20,
	0x50, 0x20, 0x51, 0x20, 0x52, 0x20, 0x53, 0x20,
	0x54, 0x20, 0x55, 0x20, 0x56, 0x20, 0x57, 0x20,
	0x58, 0x20, 0x59, 0x20, 0x5A, 0x20, 0x5B, 0x20,
	0x5C, 0x20, 0x5D, 0x20, 0x5E, 0x20, 0x5F, 0x20,
	0x60, 0x20, 0x61, 0x20, 0x62, 0x20, 0x63, 0x20,
	0x64, 0x20, 0x65, 0x20, 0x66, 0x20, 0x67, 0x20,
	0x68, 0x20, 0x69, 0x20, 0x6A, 0x20, 0x6B, 0x20,
	0x6C, 0x20, 0x6D, 0x20, 0x6E, 0x20, 0x6F, 0x20,
	0x70, 0x20, 0x71, 0x20, 0x72, 0x20, 0x73, 0x20,
	0x74, 0x20, 0x75, 0x20, 0x76, 0x20, 0x77, 0x20,
	0x78, 0x20, 0x79, 0x20, 0x7A, 0x20, 0x7B, 0x20,
	0x7C, 0x20, 0x7D, 0x20, 0x7E, 0x20, 0x7F, 0x20,
	0x80, 0x20, 0x81, 0x20, 0x82, 0x20, 0x83, 0x20,
	0x84, 0x20, 0x85, 0x20, 0x86, 0x20, 0x87, 0x20,
	0x88, 0x20, 0x89, 0x20, 0x8A, 0x20, 0x8B, 0x20,
	0x8C, 0x20, 0x8D, 0x20, 0x8E, 0x20, 0x8F, 0x20,
	0x90, 0x20, 0x91, 0x20, 0x92, 0x20, 0x93, 0x20,
	0x94, 0x20, 0x95, 0x20, 0x96, 0x20, 0x97, 0x20,
	0x98, 0x20, 0x99, 0x20, 0x9A, 0x20, 0x9B, 0x20,
	0x9C, 0x20, 0x9D, 0x20, 0x9E, 0x20, 0x9F, 0x20,
	0xA0, 0x20, 0xA1, 0x20, 0xA2, 0x20, 0xA3, 0x20,
	0xA4, 0x20, 0xA5, 0x20, 0xA6, 0x20, 0xA7, 0x20,
	0xA8, 0x20, 0xA9, 0x20, 0xAA, 0x20, 0xAB, 0x20,
	0xAC, 0x20, 0xAD, 0x20, 0xAE, 0x20, 0xAF, 0x20,
	0xB0, 0x20, 0xB1, 0x20, 0xB2, 0x20, 0xB3, 0x20,
	0xB4, 0x20, 0xB5, 0x20, 0xB6, 0x20, 0xB7, 0x20,
	0xB8, 0x20, 0xB9, 0x20, 0xBA, 0x20, 0xBB, 0x20,
	0xBC, 0x20, 0xBD, 0x20, 0xBE, 0x20, 0xBF, 0x20,
	0xC0, 0x20, 0xC1, 0x20, 0xC2, 0x20, 0xC3, 0x20,
	0xC4, 0x20, 0xC5, 0x20, 0xC6, 0x20, 0xC7, 0x20,
	0xC8, 0x20, 0xC9, 0x20, 0xCA, 0x20, 0xCB, 0x20,
	0xCC, 0x20, 0xCD, 0x20, 0xCE, 0x20, 0xCF, 0x20,
	0xD0, 0x20, 0xD1, 0x20, 0xD2, 0x20, 0xD3, 0x20,
	0xD4, 0x20, 0xD5, 0x20, 0xD6, 0x20, 0xD7, 0x20,
	0xD8, 0x20, 0xD9, 0x20, 0xDA, 0x20, 0xDB, 0x20,
	0xDC, 0x20, 0xDD, 0x20, 0xDE, 0x20, 0xDF, 0x20,
	0xE0, 0x20, 0xE1, 0x20, 0xE2, 0x20, 0xE3, 0x20,
	0xE4, 0x20, 0xE5, 0x20, 0xE6, 0x20, 0xE7, 0x20,
	0xE8, 0x20, 0xE9, 0x20, 0xEA, 0x20, 0xEB, 0x20,
	0xEC, 0x20, 0xED, 0x20, 0xEE, 0x20, 0xEF, 0x20,
	0xF0, 0x20, 0xF1, 0x20, 0xF2, 0x20, 0xF3, 0x20,
	0xF4, 0x20, 0xF5, 0x20, 0xF6, 0x20, 0xF7, 0x20,
	0xF8, 0x20, 0xF9, 0x20, 0xFA, 0x20, 0xFB, 0x20,
	0xFC, 0x20, 0xFD, 0x20, 0xFE, 0x20, 0xFF, 0x20,
	0x00, 0x21, 0x01, 0x21, 0x02, 0x21, 0x03, 0x21,
	0x04, 0x21, 0x05, 0x21, 0x06, 0x21, 0x07, 0x21,
	0x08, 0x21, 0x09, 0x21, 0x0A, 0x21, 0x0B, 0x21,
	0x0C, 0x21, 0x0D, 0x21, 0x0E, 0x21, 0x0F, 0x21,
	0x10, 0x21, 0x11, 0x21, 0x12, 0x21, 0x13, 0x21,
	0x14, 0x21, 0x15, 0x21, 0x16, 0x21, 0x17, 0x21,
	0x18, 0x21, 0x19, 0x21, 0x1A, 0x21, 0x1B, 0x21,
	0x1C, 0x21, 0x1D, 0x21, 0x1E, 0x21, 0x1F, 0x21,
	0x20, 0x21, 0x21, 0x21, 0x22, 0x21, 0x23, 0x21,
	0x24, 0x21, 0x25, 0x21, 0x26, 0x21, 0x27, 0x21,
	0x28, 0x21, 0x29, 0x21, 0x2A, 0x21, 0x2B, 0x21,
	0x2C, 0x21, 0x2D, 0x21, 0x2E, 0x21, 0x2F, 0x21,
	0x30, 0x21, 0x31, 0x21, 0x32, 0x21, 0x33, 0x21,
	0x34, 0x21, 0x35, 0x21, 0x36, 0x21, 0x37, 0x21,
	0x38, 0x21, 0x39, 0x21, 0x3A, 0x21, 0x3B, 0x21,
	0x3C, 0x21, 0x3D, 0x21, 0x3E, 0x21, 0x3F, 0x21,
	0x40, 0x21, 0x41, 0x21, 0x42, 0x21, 0x43, 0x21,
	0x44, 0x21, 0x45, 0x21, 0x46, 0x21, 0x47, 0x21,
	0x48, 0x21, 0x49, 0x21, 0x4A, 0x21, 0x4B, 0x21,
	0x4C, 0x21, 0x4D, 0x21, 0x32, 0x21, 0x4F, 0x21,
	0x50, 0x21, 0x51, 0x21, 0x52, 0x21, 0x53, 0x21,
	0x54, 0x21, 0x55, 0x21, 0x56, 0x21, 0x57, 0x21,
	0x58, 0x21, 0x59, 0x21, 0x5A, 0x21, 0x5B, 0x21,
	0x5C, 0x21, 0x5D, 0x21, 0x5E, 0x21, 0x5F, 0x21,
	0x60, 0x21, 0x61, 0x21, 0x62, 0x21, 0x63, 0x21,
	0x64, 0x21, 0x65, 0x21, 0x66, 0x21, 0x67, 0x21,
	0x68, 0x21, 0x69, 0x21, 0x6A, 0x21, 0x6B, 0x21,
	0x6C, 0x21, 0x6D, 0x21, 0x6E, 0x21, 0x6F, 0x21,
	0x60, 0x21, 0x61, 0x21, 0x62, 0x21, 0x63, 0x21,
	0x64, 0x21, 0x65, 0x21, 0x66, 0x21, 0x67, 0x21,
	0x68, 0x21, 0x69, 0x21, 0x6A, 0x21, 0x6B, 0x21,
	0x6C, 0x21, 0x6D, 0x21, 0x6E, 0x21, 0x6F, 0x21,
	0x80, 0x21, 0x81, 0x21, 0x82, 0x21, 0x83, 0x21,
	0x83, 0x21, 0xFF, 0xFF, 0x4B, 0x03, 0xB6, 0x24,
	0xB7, 0x24, 0xB8, 0x24, 0xB9, 0x24, 0xBA, 0x24,
	0xBB, 0x24, 0xBC, 0x24, 0xBD, 0x24, 0xBE, 0x24,
	0xBF, 0x24, 0xC0, 0x24, 0xC1, 0x24, 0xC2, 0x24,
	0xC3, 0x24, 0xC4, 0x24, 0xC5, 0x24, 0xC6, 0x24,
	0xC7, 0x24, 0xC8, 0x24, 0xC9, 0x24, 0xCA, 0x24,
	0xCB, 0x24, 0xCC, 0x24, 0xCD, 0x24, 0xCE, 0x24,
	0xCF, 0x24, 0xFF, 0xFF, 0x46, 0x07, 0x00, 0x2C,
	0x01, 0x2C, 0x02, 0x2C, 0x03, 0x2C, 0x04, 0x2C,
	0x05, 0x2C, 0x06, 0x2C, 0x07, 0x2C, 0x08, 0x2C,
	0x09, 0x2C, 0x0A, 0x2C, 0x0B, 0x2C, 0x0C, 0x2C,
	0x0D, 0x2C, 0x0E, 0x2C, 0x0F, 0x2C, 0x10, 0x2C,
	0x11, 0x2C, 0x12, 0x2C, 0x13, 0x2C, 0x14, 0x2C,
	0x15, 0x2C, 0x16, 0x2C, 0x17, 0x2C, 0x18, 0x2C,
	0x19, 0x2C, 0x1A, 0x2C, 0x1B, 0x2C, 0x1C, 0x2C,
	0x1D, 0x2C, 0x1E, 0x2C, 0x1F, 0x2C, 0x20, 0x2C,
	0x21, 0x2C, 0x22, 0x2C, 0x23, 0x2C, 0x24, 0x2C,
	0x25, 0x2C, 0x26, 0x2C, 0x27, 0x2C, 0x28, 0x2C,
	0x29, 0x2C, 0x2A, 0x2C, 0x2B, 0x2C, 0x2C, 0x2C,
	0x2D, 0x2C, 0x2E, 0x2C, 0x5F, 0x2C, 0x60, 0x2C,
	0x60, 0x2C, 0x62, 0x2C, 0x63, 0x2C, 0x64, 0x2C,
	0x65, 0x2C, 0x66, 0x2C, 0x67, 0x2C, 0x67, 0x2C,
	0x69, 0x2C, 0x69, 0x2C, 0x6B, 0x2C, 0x6B, 0x2C,
	0x6D, 0x2C, 0x6E, 0x2C, 0x6F, 0x2C, 0x70, 0x2C,
	0x71, 0x2C, 0x72, 0x2C, 0x73, 0x2C, 0x74, 0x2C,
	0x75, 0x2C, 0x75, 0x2C, 0x77, 0x2C, 0x78, 0x2C,
	0x79, 0x2C, 0x7A, 0x2C, 0x7B, 0x2C, 0x7C, 0x2C,
	0x7D, 0x2C, 0x7E, 0x2C, 0x7F, 0x2C, 0x80, 0x2C,
	0x80, 0x2C, 0x82, 0x2C, 0x82, 0x2C, 0x84, 0x2C,
	0x84, 0x2C, 0x86, 0x2C, 0x86, 0x2C, 0x88, 0x2C,
	0x88, 0x2C, 0x8A, 0x2C, 0x8A, 0x2C, 0x8C, 0x2C,
	0x8C, 0x2C, 0x8E, 0x2C, 0x8E, 0x2C, 0x90, 0x2C,
	0x90, 0x2C, 0x92, 0x2C, 0x92, 0x2C, 0x94, 0x2C,
	0x94, 0x2C, 0x96, 0x2C, 0x96, 0x2C, 0x98, 0x2C,
	0x98, 0x2C, 0x9A, 0x2C, 0x9A, 0x2C, 0x9C, 0x2C,
	0x9C, 0x2C, 0x9E, 0x2C, 0x9E, 0x2C, 0xA0, 0x2C,
	0xA0, 0x2C, 0xA2, 0x2C, 0xA2, 0x2C, 0xA4, 0x2C,
	0xA4, 0x2C, 0xA6, 0x2C, 0xA6, 0x2C, 0xA8, 0x2C,
	0xA8, 0x2C, 0xAA, 0x2C, 0xAA, 0x2C, 0xAC, 0x2C,
	0xAC, 0x2C, 0xAE, 0x2C, 0xAE, 0x2C, 0xB0, 0x2C,
	0xB0, 0x2C, 0xB2, 0x2C, 0xB2, 0x2C, 0xB4, 0x2C,
	0xB4, 0x2C, 0xB6, 0x2C, 0xB6, 0x2C, 0xB8, 0x2C,
	0xB8, 0x2C, 0xBA, 0x2C, 0xBA, 0x2C, 0xBC, 0x2C,
	0xBC, 0x2C, 0xBE, 0x2C, 0xBE, 0x2C, 0xC0, 0x2C,
	0xC0, 0x2C, 0xC2, 0x2C, 0xC2, 0x2C, 0xC4, 0x2C,
	0xC4, 0x2C, 0xC6, 0x2C, 0xC6, 0x2C, 0xC8, 0x2C,
	0xC8, 0x2C, 0xCA, 0x2C, 0xCA, 0x2C, 0xCC, 0x2C,
	0xCC, 0x2C, 0xCE, 0x2C, 0xCE, 0x2C, 0xD0, 0x2C,
	0xD0, 0x2C, 0xD2, 0x2C, 0xD2, 0x2C, 0xD4, 0x2C,
	0xD4, 0x2C, 0xD6, 0x2C, 0xD6, 0x2C, 0xD8, 0x2C,
	0xD8, 0x2C, 0xDA, 0x2C, 0xDA, 0x2C, 0xDC, 0x2C,
	0xDC, 0x2C, 0xDE, 0x2C, 0xDE, 0x2C, 0xE0, 0x2C,
	0xE0, 0x2C, 0xE2, 0x2C, 0xE2, 0x2C, 0xE4, 0x2C,
	0xE5, 0x2C, 0xE6, 0x2C, 0xE7, 0x2C, 0xE8, 0x2C,
	0xE9, 0x2C, 0xEA, 0x2C, 0xEB, 0x2C, 0xEC, 0x2C,
	0xED, 0x2C, 0xEE, 0x2C, 0xEF, 0x2C, 0xF0, 0x2C,
	0xF1, 0x2C, 0xF2, 0x2C, 0xF3, 0x2C, 0xF4, 0x2C,
	0xF5, 0x2C, 0xF6, 0x2C, 0xF7, 0x2C, 0xF8, 0x2C,
	0xF9, 0x2C, 0xFA, 0x2C, 0xFB, 0x2C, 0xFC, 0x2C,
	0xFD, 0x2C, 0xFE, 0x2C, 0xFF, 0x2C, 0xA0, 0x10,
	0xA1, 0x10, 0xA2, 0x10, 0xA3, 0x10, 0xA4, 0x10,
	0xA5, 0x10, 0xA6, 0x10, 0xA7, 0x10, 0xA8, 0x10,
	0xA9, 0x10, 0xAA, 0x10, 0xAB, 0x10, 0xAC, 0x10,
	0xAD, 0x10, 0xAE, 0x10, 0xAF, 0x10, 0xB0, 0x10,
	0xB1, 0x10, 0xB2, 0x10, 0xB3, 0x10, 0xB4, 0x10,
	0xB5, 0x10, 0xB6, 0x10, 0xB7, 0x10, 0xB8, 0x10,
	0xB9, 0x10, 0xBA, 0x10, 0xBB, 0x10, 0xBC, 0x10,
	0xBD, 0x10, 0xBE, 0x10, 0xBF, 0x10, 0xC0, 0x10,
	0xC1, 0x10, 0xC2, 0x10, 0xC3, 0x10, 0xC4, 0x10,
	0xC5, 0x10, 0xFF, 0xFF, 0x1B, 0xD2, 0x21, 0xFF,
	0x22, 0xFF, 0x23, 0xFF, 0x24, 0xFF, 0x25, 0xFF,
	0x26, 0xFF, 0x27, 0xFF, 0x28, 0xFF, 0x29, 0xFF,
	0x2A, 0xFF, 0x2B, 0xFF, 0x2C, 0xFF, 0x2D, 0xFF,
	0x2E, 0xFF, 0x2F, 0xFF, 0x30, 0xFF, 0x31, 0xFF,
	0x32, 0xFF, 0x33, 0xFF, 0x34, 0xFF, 0x35, 0xFF,
	0x36, 0xFF, 0x37, 0xFF, 0x38, 0xFF, 0x39, 0xFF,
	0x3A, 0xFF, 0x5B, 0xFF, 0x5C, 0xFF, 0x5D, 0xFF,
	0x5E, 0xFF, 0x5F, 0xFF, 0x60, 0xFF, 0x61, 0xFF,
	0x62, 0xFF, 0x63, 0xFF, 0x64, 0xFF, 0x65, 0xFF,
	0x66, 0xFF, 0x67, 0xFF, 0x68, 0xFF, 0x69, 0xFF,
	0x6A, 0xFF, 0x6B, 0xFF, 0x6C, 0xFF, 0x6D, 0xFF,
	0x6E, 0xFF, 0x6F, 0xFF, 0x70, 0xFF, 0x71, 0xFF,
	0x72, 0xFF, 0x73, 0xFF, 0x74, 0xFF, 0x75, 0xFF,
	0x76, 0xFF, 0x77, 0xFF, 0x78, 0xFF, 0x79, 0xFF,
	0x7A, 0xFF, 0x7B, 0xFF, 0x7C, 0xFF, 0x7D, 0xFF,
	0x7E, 0xFF, 0x7F, 0xFF, 0x80, 0xFF, 0x81, 0xFF,
	0x82, 0xFF, 0x83, 0xFF, 0x84, 0xFF, 0x85, 0xFF,
	0x86, 0xFF, 0x87, 0xFF, 0x88, 0xFF, 0x89, 0xFF,
	0x8A, 0xFF, 0x8B, 0xFF, 0x8C, 0xFF, 0x8D, 0xFF,
	0x8E, 0xFF, 0x8F, 0xFF, 0x90, 0xFF, 0x91, 0xFF,
	0x92, 0xFF, 0x93, 0xFF, 0x94, 0xFF, 0x95, 0xFF,
	0x96, 0xFF, 0x97, 0xFF, 0x98, 0xFF, 0x99, 0xFF,
	0x9A, 0xFF, 0x9B, 0xFF, 0x9C, 0xFF, 0x9D, 0xFF,
	0x9E, 0xFF, 0x9F, 0xFF, 0xA0, 0xFF, 0xA1, 0xFF,
	0xA2, 0xFF, 0xA3, 0xFF, 0xA4, 0xFF, 0xA5, 0xFF,
	0xA6, 0xFF, 0xA7, 0xFF, 0xA8, 0xFF, 0xA9, 0xFF,
	0xAA, 0xFF, 0xAB, 0xFF, 0xAC, 0xFF, 0xAD, 0xFF,
	0xAE, 0xFF, 0xAF, 0xFF, 0xB0, 0xFF, 0xB1, 0xFF,
	0xB2, 0xFF, 0xB3, 0xFF, 0xB4, 0xFF, 0xB5, 0xFF,
	0xB6, 0xFF, 0xB7, 0xFF, 0xB8, 0xFF, 0xB9, 0xFF,
	0xBA, 0xFF, 0xBB, 0xFF, 0xBC, 0xFF, 0xBD, 0xFF,
	0xBE, 0xFF, 0xBF, 0xFF, 0xC0, 0xFF, 0xC1, 0xFF,
	0xC2, 0xFF, 0xC3, 0xFF, 0xC4, 0xFF, 0xC5, 0xFF,
	0xC6, 0xFF, 0xC7, 0xFF, 0xC8, 0xFF, 0xC9, 0xFF,
	0xCA, 0xFF, 0xCB, 0xFF, 0xCC, 0xFF, 0xCD, 0xFF,
	0xCE, 0xFF, 0xCF, 0xFF, 0xD0, 0xFF, 0xD1, 0xFF,
	0xD2, 0xFF, 0xD3, 0xFF, 0xD4, 0xFF, 0xD5, 0xFF,
	0xD6, 0xFF, 0xD7, 0xFF, 0xD8, 0xFF, 0xD9, 0xFF,
	0xDA, 0xFF, 0xDB, 0xFF, 0xDC, 0xFF, 0xDD, 0xFF,
	0xDE, 0xFF, 0xDF, 0xFF, 0xE0, 0xFF, 0xE1, 0xFF,
	0xE2, 0xFF, 0xE3, 0xFF, 0xE4, 0xFF, 0xE5, 0xFF,
	0xE6, 0xFF, 0xE7, 0xFF, 0xE8, 0xFF, 0xE9, 0xFF,
	0xEA, 0xFF, 0xEB, 0xFF, 0xEC, 0xFF, 0xED, 0xFF,
	0xEE, 0xFF, 0xEF, 0xFF, 0xF0, 0xFF, 0xF1, 0xFF,
	0xF2, 0xFF, 0xF3, 0xFF, 0xF4, 0xFF, 0xF5, 0xFF,
	0xF6, 0xFF, 0xF7, 0xFF, 0xF8, 0xFF, 0xF9, 0xFF,
	0xFA, 0xFF, 0xFB, 0xFF, 0xFC, 0xFF, 0xFD, 0xFF,
	0xFE, 0xFF, 0xFF, 0xFF
};
