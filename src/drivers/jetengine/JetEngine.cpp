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
 * @file JetEngine.cpp
 * @author Kevin Chen <kevindsp@qq.com>
 */

#include "JetEngine.hpp"
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>

#include "engine/linton.hpp"
#include "engine/Jetcat.hpp"
#include "engine/gasengine.hpp"
#include "engine/none.hpp"

#include <uORB/topics/mavlink_log.h>
#include <systemlib/mavlink_log.h>

  orb_advert_t mavlink_log_pub; /*除了V5飞控的编译中要加extern才能链接通过，
					其糨的飞控要去掉extern才能过，否则会被提示变量重定义*/




JetEngine::JetEngine(const char *device,unsigned baudrate) :
	ModuleParams(nullptr),
	Device(MODULE_NAME),
	_configured_baudrate(baudrate)


{
	// store port name
	strncpy(_port, device, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';



	set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

	char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2) 这里是取数组ttyS2的最后一个元素的内容2
	set_device_bus(c - 48); // sub 48 to convert char to integer

}

JetEngine::~JetEngine()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	delete _helper;
}


int
JetEngine::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point;

	entry_point = (px4_main_t)&run_trampoline;

	int task_id = px4_task_spawn_cmd("jetengine", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, TASK_STACK_SIZE,
				   entry_point, (char *const *)argv);
	if (task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	_task_id = task_id;


	return 0;

	//if(!_configured_baudrate)_configured_baudrate = 115200;

	//PX4_INFO("jetengine's port:%s baudrate:%d",device_name,_configured_baudrate);

	return PX4_ERROR;
}

JetEngine *JetEngine::instantiate(int argc, char *argv[])
{
	const char *device_name = nullptr;
	int baudrate=0;
	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;


	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}
			break;

		case 'd':
			device_name = myoptarg;
			break;
			}
	}

	JetEngine *jetengine = nullptr;

	if(device_name==nullptr)device_name= JETENGINE_DEFAULT_PORT;

	jetengine = new JetEngine(device_name,baudrate);

	if (error_flag) {
		return 0;
	}

	return jetengine;
}

int JetEngine::print_status()
{
	int n = _E_N.get();

	PX4_INFO("jet engine's port: %s, baudrate: %d",_port,_configured_baudrate);
	for(int i=0; i<n;i++)
	{
	PX4_INFO("ENG%d rpm:%d egt:%f enginestatus:%d engine fault:%d ",(int)i,(int)_engine_status.rpm[i],(double)_engine_status.egt[i],(int)_engine_status.status[i],(int)_engine_status.fault[i]);
	PX4_INFO("engine's load:%f pump's vol:%f pump's power:%f",(double)_engine_status.engine_load[i],(double)_engine_status.vol[i],(double)_engine_status.pump[i]);
	}
	return 0;
}



int
JetEngine::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
JetEngine::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
JetEngine driver module that handles the communication with the jet engines and publishes the engine's status via uORB.
It supports differnet type of engine. Such as Linton Jetcat and even the gas engine.

The module supports upto 4 engines at the running time.

### Examples

Starting jet engine driver (the serial port is on /dev/ttyS3):
$ jetengine start -d /dev/ttyS3

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("jetengine", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int jetengine_main(int argc, char *argv[])
{
	return JetEngine::main(argc, argv);
}

bool
JetEngine::init1()
{
	return true;
}

void
JetEngine::run()
{
	while(!should_exit()) {
		PX4_INFO("JE process begin");

	if (_helper != nullptr) {
			delete (_helper);
			_helper = nullptr;
		}

		if (! _uart.isOpen()) { //如果串口没开，就进行下面几个操作

			// Configure UART port
			if (!_uart.setPort(_port)) { //设置开哪个口，如果失败就报错
				PX4_ERR("Error configuring serial device on port %s", _port);
				px4_sleep(1);
				continue;
			}

			// Configure the desired baudrate if one was specified by the user.
			// Otherwise the default baudrate will be used.
			if (_configured_baudrate) {
				if (! _uart.setBaudrate(_configured_baudrate)) { //设置波特率，如果失败就报错
					PX4_ERR("Error setting baudrate to %u on %s", _configured_baudrate, _port);
					px4_sleep(1);
					continue;
				}
			}else
			{
				_baudrate = _je_br.get();
			}

			// Open the UART. If this is successful then the UART is ready to use.
			if (! _uart.open()) { //打开串口，如果失败报错
				PX4_ERR("Error opening serial device  %s", _port);
				px4_sleep(1);
				continue;
			}
		}


	if (_helper != nullptr) {
				delete (_helper);
				_helper = nullptr;
			}

	_e_type = _engine_type.get();

	_ec_rate = _ec_r.get();

	_engine_status.engine_type = _e_type;


	switch(_e_type)
	{
	case	je_mode_t::JE_LINTON:
		_helper= new Linton(&JetEngine::callback,this,&_engine_status,_serial_fd);
	break;

	case	je_mode_t::JE_JETCAT:
		_helper= new Jetcat(&JetEngine::callback,this,&_engine_status,_serial_fd);
	break;

	case	je_mode_t::GE:
		_helper = new GasEngine(&JetEngine::callback,this,&_engine_status);
	break;

	case	je_mode_t::JE_NONE:
		_helper = new None(&JetEngine::callback,this,&_engine_status);
	break;

		default:
		return;
	}

	if(_baudrate)_uart.setBaudrate(_baudrate);

	px4_sleep(1);

	static int    estatus=engine_status_s::UNINIT;
	static bool para_ini=0,para_got=0,para_setted=0;
	static int i=0, max_para=0;


//	set_opt(_serial_fd, _Baudrate, 8, 'N', 1);

//	PX4_INFO("para_got %d para_set%d estatus%d",(int)para_got,(int)para_ini,(int)estatus);

	if(estatus==engine_status_s::UNINIT){

		if(!para_got&&!para_ini)
		{
			if(_helper->getJEpara())
			{
			para_got=1; //get je para
			}
		}
		if(para_got &&!para_setted && !para_ini)
		{
			if(_helper->setJEpara())
			{
				para_got=0;
				para_setted=1;
			}
			else
			{
				i++;
				max_para=max_para>i ? max_para:i;
			}
		}
		if(!para_got&&para_setted&&!para_ini) //reset the flag, which makes the got and set procedure run again to check if the sent parameter correct.
		{
			if(i)
			{
				i=0;
				para_got=0;
				para_setted=0;
			}
			else
			{
				if(max_para)mavlink_log_critical(&mavlink_log_pub, "%d engine parameters have been setted consistent!",max_para);
				max_para=0;
				para_ini=1;
				estatus = engine_status_s::STANDBY;
			}
		}
		//once the parameter has been setted successfully, we should not query the ecu.

	}//engine uninitial

	if(para_ini)_helper->waiting(); //it asks the engine status every cycle. And it's command will be covered if the code below sends command via the serial port

	if(estatus==engine_status_s::STANDBY)
	{
		if(_et_command_sub.update(&_et)&& !_engine_started)
		{
		_helper->test(_et.cmd);
		}
	}

	if(estatus==engine_status_s::FAULT)
	{
	int h=0;
	h=_helper->handlefault();
	if(h)estatus=h;
	}

				if(estatus==engine_status_s::STANDBY)
				{
					estatus=engine_status_s::STARTING;
					warnx("engine start");
					mavlink_log_critical(&mavlink_log_pub, "engine start");
				}

				if(estatus==engine_status_s::STARTING)
				{
					int j=_helper->start();
					if(j==1)
					{
						estatus=engine_status_s::RUNNING;
						warnx("engine started");
						mavlink_log_critical(&mavlink_log_pub, "engine started");
					}
					else
					if(!j)
					{
						estatus=engine_status_s::STOPPING;
					}else
					if(j<0)
					{
						estatus=engine_status_s::FAULT;
					}
				}

				if(estatus==engine_status_s::RUNNING)
				{
					int j = _helper->runn();
					if(j==1)estatus=engine_status_s::STOPPING;
					else if(j<0)estatus=engine_status_s::FAULT;
				}

				if(estatus==engine_status_s::STOPPING)
				{
					_helper->stop();
					estatus=engine_status_s::STANDBY;
					warnx("engine stopped");
					mavlink_log_critical(&mavlink_log_pub, "engine stopped");

				}




/*	_engine_status.temp[0]=70;
	_engine_status.pump[0]=93;
	_engine_status.rpm[0]=94;
	_engine_status.status[0]=64;
	_engine_status.engine_type=3;
*/
	PX4_INFO("RPM:%d EGT:%f fuel remain:%f",(int)_engine_status.rpm[0],(double)_engine_status.egt[0],(double)_engine_status.fuel_remain);
	_engine_status_pub.publish(_engine_status);

	PX4_INFO("JE process");

	//px4_usleep( 1000000/_ec_rate);

	}
}




int JetEngine::callback(JECallbackType type, void *data1, int data2, void *user,int data3,float *data4)
{
	JetEngine *_je = (JetEngine *)user;


	switch (type) {
	case JECallbackType::readDeviceData: {
			int timeout;
			memcpy(&timeout, data1, sizeof(timeout));
			int num_read = _je->pollOrRead((uint8_t *)data1, data2, timeout);

			return num_read;
		}

	case JECallbackType::writeDeviceData:{

		int ret = 0;

				ret = _je->_uart.write((void *) data1, (size_t) data2);

		return ret;
	}

	case JECallbackType::setBaudrate:
		return _je->setBaudrate(data2);

	case JECallbackType::setPWM:
		return _je->setPWM((uint16_t)data2,(uint16_t)data3);

	case JECallbackType::publish:
		return _je->publish();

	case JECallbackType::getARM:
		return _je->getArm();

	case JECallbackType::getThr:
		return _je->getThr(data4);

	}

	return 0;
}

int JetEngine::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)//封装好后的pollOrRead
{
	int ret = 0;
	const size_t character_count = 32; // minimum bytes that we want to read
	const int max_timeout = 2000;
	int timeout_adjusted = math::min(max_timeout, timeout);

		ret = _uart.readAtLeast(buf, buf_length, math::min(character_count, buf_length), timeout_adjusted);

	return ret;
}

int JetEngine::setBaudrate(unsigned baud)
{
		if (_uart.setBaudrate(baud)) return 1;
	return -1;
}

int JetEngine::setPWM(uint16_t thr_pwm,uint16_t starter_pwm)
{
	_jec.throttle = thr_pwm;
	_jec.starter = starter_pwm;
	_je_ctl_pub.publish(_jec);
	return 1;
}

int
JetEngine::publish()
{
	_engine_status_pub.publish(_engine_status);
	return 1;
}

int
JetEngine::getArm()
{
	_armed_sub.update(&_armed);
	return _armed.armed;
//	if(_engine_status.rpm[0]== 12001) return 1;
//	else return 0;
}

int
JetEngine::getThr(float *thr)
{
	_armed_sub.update(&_armed);
	_motors_sub.update(&_motors);
	if(_armed.armed)*thr = _motors.control[0];
	else *thr = 0.0f;
	PX4_INFO("get------------THR:%f",(double)*thr);
	return 1;
}
