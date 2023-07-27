/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sbus2_telemetry_params.c
 *
 * Parameters defined for the sbus2 driver.
 *
 *
 *
 * @author Kevindsp  <kevindsp@qq.com>
 */


/**
 * BATTERY-SLOT-NUM
 *
 * sbus2 telemetry for the battery voltage and the current. It should be match to the "Curr.F1678"'s slot number in your RC controllerr,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 29
 * @group sbus2 driver
 */
PARAM_DEFINE_INT32(SBS2_BATT_SLOT, 0);

/**
 * AIRSPEED-SLOT-NUM
 *
 * sbus2 telemetry for airspeed. It should be match to the (air) "Speed sensor"'s slot number in your RC controllerr,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 32
 * @group sbus2 driver
 */
PARAM_DEFINE_INT32(SBS2_AS_SLOT, 0);

/**
 * RPM-SLOT-NUM
 *
 * sbus2 telemetry for the propeller's rpm. It should be match to the "rpm sensor"'s slot number in your RC controllerr,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 32
 * @group sbus2 driver
 */
PARAM_DEFINE_INT32(SBS2_RPM_SLOT, 0);

/**
 * TEMP-SLOT-NUM
 *
 * sbus2 telemetry for the temperature. It should be match to the "Temp-F1713"'s slot number in your RC controllerr,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 32
 *  @group sbus2 driver
 */
PARAM_DEFINE_INT32(SBS2_TEMP_SLOT, 0);

/**
 * GPS-SLOT-NUM
 *
 * sbus2 telemetry for GPS. It should be match to the "GPS-F1675"'s slot number in your RC controller. It can only be set to 8/16/24r,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 24
 * @group sbus2 driver
 */
PARAM_DEFINE_INT32(SBS2_GPS_SLOT, 0);

/**
 * YAW-SLOT-NUM
 *
 * sbus2 telemetry for the yaw. It is using the "Temp-F1713"'s slot number in your RC controllerr,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 32
 *  @group sbus2 driver
 */
PARAM_DEFINE_INT32(SBS2_YAW_SLOT, 0);

/**
 * JETCAT-SLOT-NUM
 *
 * sbus2 telemetry for jet engine's status as jetcat. It should be match to the "JetCat V10"'s slot number in your RC controller. It can only be set to 8/16r,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 16
 * @group sbus2 driver
 */

PARAM_DEFINE_INT32(SBS2_JC_SLOT, 0);

/**
 * TT
 *
 * sbus2 telemetry for GPS. It should be match to the "GPS-F1675"'s slot number in your RC controller. It can only be set to 8/16/24r,0 disable the slot
 *
 * @group sbus2_telemetery
 * @min 0
 * @max 8000
 * @reboot_required false
 * @group sbus2 driver
 *
PARAM_DEFINE_INT32(SBS2_TT, 4920);
*/
