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
 * @file linton.cpp
 * @author Kevin Chen <kevindsp@qq.com>
 */


#include "linton.hpp"
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


Linton::Linton( JECallbackPtr  callback, void *callback_user,struct engine_status_s *engine_status,int uart4)
:JEHelper(callback,callback_user),ModuleParams(nullptr)
{
	_para_set.pwm_min=_pwm3_min.get();
	_para_set.pwm_max=_pwm3_max.get();
	_para_set.pwm_stop=_pwm3_stop.get();
	_para_set.rpm_min=(uint64_t)_rpm_min.get();
	_para_set.rpm_max=(uint64_t)_rpm_max.get();
	_para_set.consum_rate=_fuel_consum_rate.get();
	_para_set.tank_cap=_fuel_tank_cap.get();
	_para_set.fuel_low=_fuel_low.get();
	_engine_status=engine_status;
	_ec_m=_EC_M.get();
	_e_n=_E_N.get();
	setbaud();
}

Linton::~Linton()
{

}

int
Linton::waiting()
{

	_engine_status->e_n=_e_n;

	PX4_INFO("------------------waiting");

	while(1)
	{
	collect();

	publish();
	if(getarm())return 1;
	}
}

int
Linton::start()
{
	bool pwm_starting=0;

	char jet_start[7] = {"start"};

	uint16_t crc = crc16((uint8_t *)jet_start,sizeof(jet_start)-2);

	uint8_t crchi= (crc&0xff00)>>8;

	uint8_t crclo= crc&0x00ff;

	jet_start[5] = crchi;

	jet_start[6] = crclo;

	PX4_INFO("-----------------------------starting");

	while(1)
	{
		collect();

		publish();

		if(_fault>16&&_fault<30)return -1;

	       if(_status==18
		||_status==19
		||_status==20
		||_status==21
		||_status==22
	       )return true;
		else
		{
			if(!getarm()) return 0;
			if(_status==0
			  ||_status==1
			   )
			   {
				if(_ec_m) //start the engine with pwm or with serial command.Then we should not change the throttle's pwm value anymore
				{
				write(jet_start, sizeof(jet_start)); //if we finish the pwm starting proceduce,or we use serial controll mode, we should keep asking the starting status after it finishes the starting produce with pwm mode until the engine switches the status to running
				}
				else if(!pwm_starting)pwm_starting=pwm_start();
			   }
		}
	}
}

int
Linton::runn()
{
	PX4_INFO("----------------------------running");

	bool gotfault = 0;

	while(1)
	{
		collect();

		publish();

		gotfault = (_fault==30||_fault==31||_fault==32);

		if(gotfault)
	{
		mavlink_log_critical(&mavlink_log_pub, "engine flame out!");
		flame_out=1;
	}
	PX4_INFO("flame out%d _fault%d",(int)flame_out,_fault);

		if(!getarm())return 1;
		if(gotfault)return-1;
	}
}

int
Linton::stop()
{
	char jet_stop[6] = {"stop"};

	uint16_t crc = crc16((uint8_t *)jet_stop,sizeof(jet_stop)-2);

	uint8_t crchi= (crc&0xff00)>>8;

	uint8_t crclo= crc&0x00ff;

	jet_stop[4] = crchi;

	jet_stop[5] = crclo;

	PX4_INFO("-----------------------------stopping");

	while(1)
	{
		collect();

		publish();

		if(_status==18
		||_status==19
		||_status==20
		||_status==21
		||_status==22
	      	 )
		 {
			write(jet_stop, sizeof(jet_stop));
		 }
		 if(_status==0||_status==1)return 1;
	}
}

int
Linton::test(uint8_t cmd)
{
	const char mot_stop[]={"@C.HMI=34.0"};
	const char mot_start[]={"@C.HMI=34.1"};
	const char pump_stop[]={"@C.HMI=35.0"};
	const char pump_start[]={"@C.HMI=35.1"};
	const char glowor_stop[]={"@C.HMI=36.0"};
	const char glowor_start[]={"@C.HMI=36.1"};

	switch(cmd)
	{
	case 0: write(mot_start, sizeof(mot_start));break;
	case 1: write(pump_start, sizeof(pump_start));break;
	case 2: write(glowor_start, sizeof(glowor_start));break;
	case 3: write(mot_stop, sizeof(mot_stop));break;
	case 4: write(pump_stop, sizeof(pump_stop));break;
	case 5: write(glowor_stop, sizeof(glowor_stop));break;
	default : write(mot_stop, sizeof(mot_stop));
		  write(pump_stop, sizeof(pump_stop));
		  write(glowor_stop, sizeof(glowor_stop));
	}
	return true;
}

int
Linton::handlefault()
{
	static uint8_t last_status=0;
	bool run=(_status==10||_status==11||_status==12||_status==13||_status==14||_status==15||_status==16);
	PX4_INFO("status%d fault%d last status%d last fault",(int)_status,(int)_fault,(int)last_status);
	if(last_status==29)
	{
	if(run)
	{
		flame_out=0;
		last_status=_status;
		return engine_status_t::running;
	}
	else
	{
		last_status=_status;
		return engine_status_t::fault;
	}
	}

	if(_status==0||_status==24)
	{
		starting_fault=0;
		last_status=_status;
		return engine_status_t::standby;
	}
	else
	{
		last_status=_status;
		return engine_status_t::fault;
	}
}





int
Linton::collect()
{
			int j = 0;
			int bytes_count = 0;
			/* then poll or read for new data */
		//	warnx("before read");
			int ret=read(_buf, sizeof(_buf),1000);
			//warnx("%d",ret);
		//	warnx("after read");
			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */
				return 0;
			} else if (ret > 0) {
				/* if we have new data from ECU, go handle it */
				bytes_count = ret;
			}
		/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parseChar(_buf[j])) > 0) {
					/* return to configure during configuration or to the ECU driver during normal work
					 * if a packet has arrived */
					//warnx("parchar %d",l);
					handle(l);

				}

				j++;
			}
			/* everything is read */
			j = bytes_count = 0;


		return ret;


}

int Linton::parseChar(uint8_t b)
{
	int iRet = 0;
	static uint8_t a0,a1,a2;
	a2 = a1;
	a1 = a0;
	a0 = b;
	if(a0==0xff&&a1==0xff&&a2==0xff)
	{
		iRet = _rx_buffer_bytes;
		_rx_buffer_bytes = 0;

	}
	else
	{
		if(_rx_buffer_bytes>=JE_READ_BUFFER_SIZE-1)_rx_buffer_bytes = 0; //reset the buffer pointer if too long to get the needed '\r', or it will hard fault
		_rx_buffer[_rx_buffer_bytes++] = b;
		return 0;
	}

	return iRet;
}

int Linton::handle(int len)
{
	char *endp;

	char *bufptr = (char *)_rx_buffer+2;

	//PX4_INFO("after parse char:%.*s\n",len,_rx_buffer);

	if(memcmp(_rx_buffer,"Temp",4)==0)
	{
		uint16_t egt=0;
		egt = strtol(bufptr+4, &endp,10);
		_engine_status->egt[0]=(float)egt;
	} else
	if(memcmp(_rx_buffer,"Rpm",3)==0)
	{
		uint64_t rpm;
		rpm = strtol(bufptr+3, &endp,10);
		_engine_status->rpm[0]=rpm;
	} else
	if(memcmp(_rx_buffer,"State",5)==0)
	{
		_status = strtol(bufptr+5, &endp,10);

		_engine_status->status[0]=_status;
	} else
	if(memcmp(_rx_buffer,"Error",5)==0)
	{
		_fault = strtol(bufptr+5, &endp,10);
		_engine_status->fault[0]=_fault;
	} else
	if(memcmp(_rx_buffer,"Flow",4)==0)
	{
		double pump = strtod(bufptr+4, &endp);
		double fuel_comsuption=_fuelcomsued(pump);

		double fuel_remain = (double)_para_set.tank_cap - fuel_comsuption;

		_engine_status->fuel_remain = fuel_remain;
	}



	return 1;
}

int
Linton::setJEpara() // Although linton engine only has 2 groups of parameters. But we need to compare them one by one. So it needs to do 5 times
{
//	int je_br=_je_br.get();
//	setBaudrate(je_br);  //no need to set baudrate dynamically while the engine's running.
	char para[30]={};

	static int i=0;
	return 1;
/*	PX4_INFO("set");
	PX4_INFO("para got: pwm max%f min%f close%f rpm_max:%d rpm_min:%d",(double)_para_got.pwm_max,(double)_para_got.pwm_min,(double)_para_got.pwm_stop,(int)_para_got.rpm_max,(int)_para_got.rpm_min);
*/
	while(1)
	{
		if(i==0){ i++;		if(_para_set.rpm_min!=_para_got.rpm_min){sprintf(para,"@SetData=71,%d;\r\n",(int)_para_set.rpm_min);break;} }
		if(i==1){ i++;		if(_para_set.rpm_max!=_para_got.rpm_max){sprintf(para,"@SetData=70,%d;\r\n",(int)_para_set.rpm_max);break;} }
		if(i==2){ i++;		if(fabs(_para_set.pwm_stop-_para_got.pwm_stop)>.001){sprintf(para,"@SetData=12,%d;\r\n",(int)_para_set.pwm_stop);break;} }
		if(i==3){ i++;		if(fabs(_para_set.pwm_min-_para_got.pwm_min)>.001){sprintf(para,"@SetData=13,%d;\r\n",(int)_para_set.pwm_min);break;} }
		if(i==4){ i++;		if(fabs(_para_set.pwm_max-_para_got.pwm_max)>.001){sprintf(para,"@SetData=14,%d;\r\n",(int)_para_set.pwm_max);break;} }
		if(i>=5){ i=0;		return 1;}
	}
/*	PX4_INFO("i= %d",i);
	PX4_INFO("para set: pwm max%f min%f close%f rpm_max:%d rpm_min:%d ",(double)_para_set.pwm_max,(double)_para_set.pwm_min,(double)_para_set.pwm_stop,(int)_para_set.rpm_max,(int)_para_set.rpm_min);
	PX4_INFO("para got: pwm max%f min%f close%f rpm_max:%d rpm_min:%d",(double)_para_got.pwm_max,(double)_para_got.pwm_min,(double)_para_got.pwm_stop,(int)_para_got.rpm_max,(int)_para_got.rpm_min);

			warnx("got pwm max:%f,setted pwm max:%f",(double)_para_got.pwm_max,(double)_para_set.pwm_max);
			warnx("got pwm min:%f,setted pwm min:%f",(double)_para_got.pwm_min,(double)_para_set.pwm_min);
			warnx("got pwm stop:%f, setted pwm stop:%f",(double)_para_got.pwm_stop,(double)_para_set.pwm_stop);
			warnx("got rpm max:%f, setted rpm max:%f",(double)_para_got.rpm_max,(double)_para_set.rpm_max);
			warnx("got rpm min:%f, setted rpm min:%f",(double)_para_got.rpm_min,(double)_para_set.rpm_min);
		warnx("i=%d",i);
*/
		write(para,sizeof(para));
	return 0;
}

int
Linton::getJEpara() //linton engine only need to send 2 groups of parameters request
{
	return 1;
	static uint8_t para_num=0;
	//int para_got=0;
	const char get_pwm[]={"@HMI=13,0;\r\n"};
	const char get_rpm[]={"@HMI=10,0;\r\n"};
	if(para_num==0){write(get_pwm,sizeof(get_pwm));}
	if(para_num==1){write(get_rpm,sizeof(get_rpm));}
	if(para_num<2)para_num++;
	else
	{para_num = 0; //para_got = 1;}
	}
	return 1;
}

int
Linton::pwm_start() //the pwm starting mode is according to linton's SEQUENCE starting proceduce
{
//	px4_sleep(1); //1s
//		setThr(1,0);//set the pwm to the throttle's minimum value 这100是disarm值
//	px4_usleep(500000);//1.5s
		setThr(2,0);//set the throttle to minimum with the "trim" //其中100是disarm的值，200是微调值
	px4_sleep(1);//2.5s
		setThr(3,0);//"push" the throttle to maximum
	px4_sleep(1);//3.5s
		setThr(2,0); //finally "pull" it back to minimun with trim
	px4_sleep(1);//4.5s
		setThr(0,0);
		return 1;
}

int Linton::setbaud()
{
	return setBaudrate(LINTON_BAUDRATE);
}

uint16_t
Linton::crc16(uint8_t *updata, uint8_t len)
{
	uint8_t CRCHi = 0xff;
	uint8_t CRCLo = 0xff;
	uint8_t index;

	while(len--)
	{
		index = CRCHi ^ *updata++;
		CRCHi = CRCLo ^ auchCRCHi[index];
		CRCLo = auchCRCLo[index];
	}
	return (uint16_t)(CRCLo << 8 | CRCHi);
};
