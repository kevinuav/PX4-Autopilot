/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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
 * @file gasengine.cpp
 * @author Kevin Chen <kevindsp@qq.com>
 */

#include "gasengine.hpp"
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>
#include <cstdint>


#include <uORB/topics/mavlink_log.h>
#include <systemlib/mavlink_log.h>

extern orb_advert_t mavlink_log_pub;



GasEngine::GasEngine(JECallbackPtr callback, void *callback_user,struct engine_status_s *engine_status)
:JEHelper(callback,callback_user),ModuleParams(nullptr)
{
	_engine_status=engine_status;
	_e_n=_E_N.get();

}

GasEngine::~GasEngine()
{

}



int
GasEngine::waiting()
{
	_engine_status->e_n=_e_n;
	return true;
}

int
GasEngine::start()
{
	static uint32_t starting_time=0;
	static uint8_t start_count=0;
	starting_time++;
	if(start_count<2) //we try to start the engine 2 times.
	{
		_engine_status->status[0]=engine_state::starting;
		if(starting_time<60)
		{
			if(starting_time<30) //starter runs 3s
			{
				PX4_INFO("gas engine start ");
//				_ge_ctl.throttle=_pwm_start.get();
//				_ge_ctl.starter=_max_start.get();
				setThr(_pwm_start.get(),_max_start.get());
			}
			else					//turn off the starter
			{
				PX4_INFO("gas engine starter stop ");
//				_ge_ctl.throttle=_pwm_start.get();
//				_ge_ctl.starter=_min_start.get();
				setThr(_pwm_start.get(),_min_start.get());
			}
			if(starting_time>50) //wait 2s to check if the engine can run without starter
			{
				PX4_INFO("check if the engine started!");
				_engine_starting=0;
			}
			else
			{
				_engine_starting=1; //set the starting flag between 0-4s,0-3s indicate the starter is working.4th second wait for the starter stop to check if the engine can run itself.
			}
		}
		if(starting_time>=60)
			{
				starting_time=0;
				start_count++;
			}
	}
	else if(!_engine_starting&&!_engine_started)
	{
		starting_time=0;
		start_count=0;
		PX4_INFO("engine start fail!");
		return -1;
	}
//	PX4_INFO("starting time %d starting count %d starting %d started %d",(int)starting_time,(int)start_count,(int)_engine_starting,(int)_engine_started);

/*	_ge_ctl.throttle=0;
	_ge_ctl.starter=0;

	_ge_ctl.throttle=1200;
	_ge_ctl.starter=2000;
	_ge_ctl_pub.publish(_ge_ctl);
*/

	if(_engine_starting&&!_engine_started)
	{
//		PX4_INFO("engine starting...");
		return 0; //in the starting procedure
	}

	if(!_engine_starting&&_engine_started)
	{
		starting_time=0;
		start_count=0;
		PX4_INFO("engine started!");
		return 1;
	}

	return 0;
}

int
GasEngine::runn()
{
	if(_rpm1>100){_engine_status->status[0]=engine_state::running;}else{_engine_status->status[0]=engine_state::stopping; _engine_status->status[0]=engine_fault::flameout;}
	if(_rpm2>100){_engine_status->status[1]=engine_state::running;}else{_engine_status->status[1]=engine_state::stopping; _engine_status->status[1]=engine_fault::flameout;}
	if(_rpm1>100&&_e_n==1)
	{
		if(!_engine_starting)_engine_started = true;
	}
	else if(_rpm1>100&&_rpm2>100&&_e_n==2)
	{
	 	if(!_engine_starting)_engine_started = true;
	}
	else _engine_started = false;

	if(_rpm1>3400){_engine_status->status[0]=engine_state::maximum_power;}
	if(_rpm2>3400){_engine_status->status[1]=engine_state::maximum_power;}
	return 0;
}

int
GasEngine::stop()
{
	static uint32_t stoping_time=0;
	stoping_time++;
	if(stoping_time<20)
	{
		_engine_status->status[0]=engine_state::stopping;
//		_ge_ctl.throttle=_para_set.pwm_stop;
//		_ge_ctl.starter=_para_set.min_start;
		setThr(_para_set.pwm_stop,_para_set.min_start);
		return 0;
	}
	return true;
}

int
GasEngine::test(uint8_t)
{
	return true;
}

int
GasEngine::collect()
{
	//static uint32_t		_pwm_unavailable_time = 0;
	//static uint16_t		pwm_counter=0;
	/*_pwm_in_sub.update(&_pwm_in);
		if(_pwm_in.pwm_counter==pwm_counter) //since the subscription side of uorb continuously returns the updated flag as true no matter if the publisher side has really published the new message. I have to make a flag to indicate if the message has actually been updated. :(
		{
			_pwm_unavailable_time++;
//			PX4_INFO("pwm unupdated");
		}
		else
		{
			_pwm_unavailable_time = 0;
			pwm_counter = _pwm_in.pwm_counter;
//			PX4_INFO("pwm msg updated^^^^^^");
		}
		if(_pwm_unavailable_time>3)
		{_pwm_in.period1 = 35000;
		_pwm_in.period2 = 35000;
		}
*/
	_rpm_sub.update(&_rpm);

	 double period1 = (double)_pwm_in.period;
	 double period2 = (double)_pwm_in.period;
	 _rpm1 = period2rpm(period1);
	 _rpm2 = period2rpm(period2);
	_engine_status->rpm[0] = _rpm.indicated_frequency_rpm;
	_engine_status->rpm[1] = _rpm2;

	//PX4_INFO("status:%d period1:%f period2:%f rpm1:%d rpm2:%d",(int)_pwm_in.status,(double)period1,(double)period2,(int)_engine_status->rpm[0],(int)_engine_status->rpm[1]);

	return true;
}

int
GasEngine::handlefault()
{
	return true;
}

int
GasEngine::getJEpara()
{
	return true;
}

int
GasEngine::setJEpara()
{
	return true;
}

double
GasEngine::period2rpm(double period)
{
	 if(period>30000) //这里需要改进，计时器只有16位，超过65535就溢出不准了，也就是低于50次/秒的计数就不行。但将计数周期定得太低又在很高速的时候就切为0.最后后期用时间间隔重新算
	 {
		 return 0.f;
	 }
	  else if(period>100) //there is interference which's period is smaller than 100
	 {
		double period_d = (double)period / 1000000;
	 	return 60/(period_d*(double)_r_n.get());
	}else return 0.f;
}



