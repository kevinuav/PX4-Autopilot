/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 *@file JetEngine_params.c
 *@author Kevin Chen <kevindsp@qq.com>
 *
 */

/**
 * Fuel consumption rate
 *
 * Fuel comsuption per second at max pump power (ml/s)
 *
 * @group Jet Engine
 * @min 2
 * @max 15.0
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(FUEL_CONSUM_RATE, 8.5f);

/**
 * Fuel tank capacity
 *
 * Fuel tank capacity (ml)
 *
 * @group Jet Engine
 * @min 500
 * @max 3000
 * @decimal 2
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(FUEL_TANK_CAP, 1500.0f);

/**
 * Fuel low
 *
 * Fuel low warning which defines the low fuel warning volume from the consumed fuel of the total fuel tank capacity (0-1000)
 *
 * @group Jet Engine
 * @min 0
 * @max 1000
 * @decimal 1
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FUEL_LOW, 300.0f);

/**
 * engine's min rpm
 *
 * The engine's idle rpm (the min rpm)
 *
 * @group Jet Engine
 * @min 30000
 * @max 100000
 * @decimal 0
 * @increment 100
 */
PARAM_DEFINE_FLOAT(RPM_MIN, 50000);
/**
 * engine's max rpm
 *
 * The engine's max rpm
 *
 * @group Jet Engine
 * @min 100000
 * @max 300000
 * @decimal 0
 * @increment 100
 */
PARAM_DEFINE_FLOAT(RPM_MAX, 160000);
/**
 * engine's type
 *
 * The engine's type including:
 * @value 4 gasengine
 * @value 2 jetcat
 * @value 1 linton jet engine
 * @value 0  not a fule powered engine
 *
 * @group Jet Engine
 * @min 0
 * @max 5
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(ENGINE_TYPE, 0);

/**
 * throttle's pwm while it is starting
 *
 *
 * @group Jet Engine
 * @min 900
 * @max 2000
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(PWM_START, 1200);

/**
 * starter's max pwm
 *
 *
 * @group Jet Engine
 * @min 900
 * @max 2000
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(MAX_STARTER, 2000);

/**
 * starter's min pwm
 *
 *
 * @group Jet Engine
 * @min 900
 * @max 2000
 * @decimal 0
 * @increment 1
 */
PARAM_DEFINE_INT32(MIN_STARTER, 900);

/**
 * revelution numbers of every round
 *
 * revelution (the ratio of n/round) every round of the propeller
 *
 * @group Jet Engine
 * @min 0
 * @max 50
 * @decimal 0
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(R_N, 1);

/**
 * engine's serial port baudrate
 *
 *
 * @group Jet Engine
 * @value 0      0
 * @value 9600   9600
 * @value 38400  38400
 * @value 57600  57600
 * @value 115200 115200
 * @value 230400 230400
 */
PARAM_DEFINE_INT32(JE_BR, 0);

/**
 * engine's control rate
 *
 *
 * @group Jet Engine
 * @value 20 20HZ
 * @value 10 10HZ
 * @value 5   5HZ
 */
PARAM_DEFINE_INT32(JE_R, 10);

/**
 * engine's control mode
 *
 * The engine's control mode, directly control by PWM or control by serial
 *
 * @group Jet Engine
 * @value 0 PWM
 * @value 1 serial
 */
PARAM_DEFINE_INT32(JE_M, 0);

/**
 * how many engine totally
 *
 *
 * @group Jet Engine
 * @min 1
 * @max 4
 */
PARAM_DEFINE_INT32(JE_N, 1);


