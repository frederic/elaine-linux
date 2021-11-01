/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** File:
**     tas2770-codec.h
**
** Description:
**     header file for tas2560-codec.c
**
** =============================================================================
*/

#ifndef _TAS2770_CODEC_H
#define _TAS2770_CODEC_H

#include "tas2770.h"

int tas2770_register_codec(struct tas2770_priv *pTAS2770);
int tas2770_deregister_codec(struct tas2770_priv *pTAS2770);
int tas2770_LoadConfig(struct tas2770_priv *pTAS2770, bool bPowerOn);

#endif /* _TAS2770_CODEC_H */
