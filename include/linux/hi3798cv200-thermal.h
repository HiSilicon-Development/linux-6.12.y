/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_HI3798CV200_THERMAL_H
#define _LINUX_HI3798CV200_THERMAL_H

/* Serialized against provider removal; returns millidegrees Celsius. */
int hi3798cv200_avs_get_temp(int *temperature);

#endif
