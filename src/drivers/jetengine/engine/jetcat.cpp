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
 * @file jetcat.cpp
 * @author Kevin Chen <kevindsp@qq.com>
 */


#include "jetcat.hpp"
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

Jetcat::Jetcat( JECallbackPtr  callback, void *callback_user,struct engine_status_s *engine_status,int uart4)
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
	_ec_m=_EC_M.get();
	_e_n=_E_N.get();
	_engine_status=engine_status;
	uart_init();
}

Jetcat::~Jetcat()
{

}


int
Jetcat::uart_init()
{
	//setBaudrate(115200);
	//set_opt(_uart4, _Baudrate, 8, 'N', 1);
	return 1;
}



int
Jetcat::waiting()
{

	_engine_status->e_n=_e_n;

	uint8_t i = 1 ;//jetcat's engine number starts from 1

	while(1)
	{
	PX4_INFO("standby");
//	PX4_INFO("_status%d",(int)_status);
		cmdrac(i);
		cmdra1(i);
		cmdrfi(i);
//	cmdwpe(i,(double)getthr());
		_status[i-1] = 0;
		if(i<_e_n)i++;
		else i=1;

//	PX4_INFO("arm status:%d thr:%f",(int)getarm(),(double)getthr());

	publish();
	if(getarm())return 1;
	}
}

int
Jetcat::start()
{
	bool pwm_starting=0;

	bool all_finish=0;;

	uint8_t i=1;

	uint8_t count = 0;

	while(1)
	{
	PX4_INFO("starting");

/*	if(last_fault!=_fault&&start_fault)
	{
		PX4_INFO("starting fail");
		starting_fault=1;
		last_fault=_fault;
		return -1;
	}
*/

 //continuously do the starting proceduce before the engine is at the running status

		if(!_ec_m&&!pwm_starting) //start the engine with pwm or with serial command.Then we should not change the throttle's pwm value anymore
		{
		pwm_starting=pwm_start(); //if we finish the pwm starting proceduce,or we use serial controll mode, we should keep asking the starting status after it finishes the starting produce with pwm mode until the engine switches the status to running
		}else// the _send_token avoid the message sending frequency too high for the ECU
		{
			cmdtco(i,1);
			cmdrac(i);
			cmdra1(i);
			cmdrfi(i);
			bool start_finish = (_status[i-1]==JESTATUS::RUN)||(_status[i-1]==JESTATUS::ACCDELAY)||(_status[i-1]==JESTATUS::SPEEDREG)||(_status[i-1]==JESTATUS::TWOSHAFTREG);
			PX4_INFO("start_finish[%d]:%d all_finfish:%d",(int)i-1,(int)start_finish,(int)all_finish);
			count += start_finish;
			if(count>=_e_n)all_finish = 1;
			else all_finish = 0;
			if(i<_e_n)i++; else {i=1; count = 0;}
		}
		publish();
	if(getarm())
	{
	if(all_finish)return 1;
	}
	else return 0; //cancel by disarm
	}

}

int
Jetcat::runn()
{
	bool engine_fault[4];

	bool gotfault=0;

	uint8_t i=1;

	//char wpe_run[18]; //the serial port is not fast enough to send over 15 characters in such a short time. Or it will output the messy code. so we limit it to 13.

	while(1)
	{
	PX4_INFO("running");
		cmdrac(i);
		cmdra1(i);
		cmdrfi(i);
		if(_ec_m)cmdwpe(i,(double)getthr());
		PX4_INFO("%1c,wpe,%.1f\r",i2char(i),(double)getthr());
		engine_fault[i-1] = (_fault[i-1]!=FAULTSTATUS::PWMSHUTDOWN)&&(_fault[i-1]!=FAULTSTATUS::NODEFINE)&&(_fault[i-1]!=FAULTSTATUS::AUTOOFF)&&(_fault[i-1]!=FAULTSTATUS::MANUOFF);
		gotfault |= engine_fault[i-1];
		PX4_INFO("running:engine_fault:%d _fault:%d",(int)engine_fault[i-1],_fault[i-1]);
		if(i<_e_n)i++; else i=1;
		publish();
		if(!getarm())return 1;
		if(gotfault)return-1;
	}

}

int
Jetcat::stop()
{
	uint8_t i=1;

	uint8_t count = 0;

	bool all_stopped = 0;


	while(1)
	{
	PX4_INFO("stop");

	if(!_ec_m)
		{
			setThr(1,0);
		}
		PX4_INFO("sending tco");
		cmdtco(i,0);
		cmdrac(i);
		cmdra1(i);
		cmdrfi(i);
		bool stopped = (_status[i-1] == JESTATUS::OFF);
		count +=stopped;
		if(count>=_e_n)all_stopped = 1;
		else all_stopped = 0;
		if(i<_e_n)i++; else {i=1;count = 0;}
		publish();
		if(all_stopped)return 1;
	}
}

int
Jetcat::test(uint8_t cmd)  // unused
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
Jetcat::handlefault()
{
	uint8_t last_status=0;
	uint8_t i=1;
	bool run[4];
	while(1)
	{
		PX4_INFO("handlefault");
		cmdrac(i);
		cmdra1(i);
		cmdrfi(i);

	run[i]=(_status[i]==(uint8_t)JESTATUS::RUN||_status[i]==JESTATUS::ACCDELAY||_status[i]==JESTATUS::SPEEDREG||_status[i]==JESTATUS::TWOSHAFTREG||_status[i]==JESTATUS::PREHEAT1||_status[i]==JESTATUS::PREHEAT2);
	PX4_INFO("handle:status%d fault%d last status%d last fault",(int)_status[i],(int)_fault[i],(int)last_status);
	if(i<_e_n)i++; else i=1;
	publish();

	if(run[0])//the engine faults at the running stage
	{
		return engine_status_t::running;
	}

	if(_status[0]==JESTATUS::OFF)//the engine faults at the standby stage
	{
		return engine_status_t::standby;
	}
	}

}





int
Jetcat::collect()
{
			int j = 0;
			int bytes_count = 0;
			/* then poll or read for new data */
		//	warnx("before read");
			int ret=read(_buf, sizeof(_buf),50);
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
			PX4_INFO("before:%.*s\n received:%d",bytes_count,_buf,bytes_count);
		/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parseChar(_buf[j])) > 0) {
					/* return to configure during configuration or to the ecu driver during normal work
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

int Jetcat::parseChar(uint8_t b)
{
	int iRet = 0;
	static uint8_t a0,a1,a2,a3,a4;
	static bool getcmd=0;
	a4 = a3;
	a3 = a2;
	a2 = a1;
	a1 = a0;
	a0 = b;
	     if(a2=='R'&&a1=='A'&&a0=='C'){_sent_cmd=jetcat_cmd_t::RAC;getcmd=1;} //The reply command contains a "\r"
	else if(a2=='R'&&a1=='G'&&a0=='V'){_sent_cmd=jetcat_cmd_t::RGV;getcmd=1;}//,We should return 0 here ,or it will mess up with the next data
	else if(a2=='R'&&a1=='A'&&a0=='1'){_sent_cmd=jetcat_cmd_t::RA1;getcmd=1;}
	else if(a2=='R'&&a1=='F'&&a0=='I'){_sent_cmd=jetcat_cmd_t::RFI;getcmd=1;}
	else if(a2=='T'&&a1=='C'&&a0=='O'){_sent_cmd=jetcat_cmd_t::TCO;getcmd=1;}
	else if(a2=='t'&&a1=='c'&&a0=='o'){_sent_cmd=jetcat_cmd_t::tco;getcmd=1;}
	else if(a2=='W'&&a1=='R'&&a0=='P'){_sent_cmd=jetcat_cmd_t::WRP;getcmd=1;}
	else if(a2=='w'&&a1=='r'&&a0=='p'){_sent_cmd=jetcat_cmd_t::wrp;getcmd=1;}
	else if(a2=='W'&&a1=='P'&&a0=='E'){_sent_cmd=jetcat_cmd_t::WPE;getcmd=1;}
	else if(a2=='w'&&a1=='p'&&a0=='e'){_sent_cmd=jetcat_cmd_t::wpe;getcmd=1;}
	else if(a2=='X'&&a1=='L'&&a0=='O'){_sent_cmd=jetcat_cmd_t::XLO;getcmd=1;}
	else if(a2=='S'&&a1=='L'&&a0=='O'){_sent_cmd=jetcat_cmd_t::SLO;getcmd=1;}

	if(a2=='H'&&a1=='S'&&a0==',')
	{
		char e_id = (char)a4;
		_e_id = atoi(&e_id)-1; //jetcat's engine number starts from 1, we should sub 1 to fit the array
		_rx_buffer_bytes = 0;
		_engine_status->e_id = _e_id;
	}
	else if(a0=='\r')
	{
		if(!getcmd)
		{
		_rx_buffer[_rx_buffer_bytes++] = b;
		iRet = _rx_buffer_bytes;
		}
		else {getcmd=0; return 0;}
	}
	else
	{
		_rx_buffer[_rx_buffer_bytes++] = b;
		return 0;
	}

	return iRet;
}

int Jetcat::handle(int len)
{

	char *endp;

	if (len < 7) {
		return 0;
	}

	int uiCalcComma = 0;

	for (int i = 2 ; i < len; i++) { //the first [0]char is a comma ,so we begins from [1]
		if (_rx_buffer[i] == ',') { uiCalcComma++; } //the correct number of comma= number of parameters
	}

	char *bufptr = (char *)(_rx_buffer + 2);
	int ret = 0;

	PX4_INFO("received cmd %d ",(int)_sent_cmd);
	PX4_INFO("after:%.*s\n",len,_rx_buffer);

	static double pump_v=0.0;

	if ((_sent_cmd == jetcat_cmd_t::RAC)&&(memcmp(_rx_buffer , "OK,", 3) == 0) && (uiCalcComma == 6)) {
		/*
		  raeding the actually value from the engine
		  An example of the RAC message string is:

		  1,HS,OK,0,25,0,0,44.6,0.384

		  Field   Meaning
		  0
		  1   RPM
		  2   EGT C
		  3   pump voltage
		  4   engine status
		  5   PWM throttle %
		  6   engine current
		*/
		uint64_t rpm=0;

		double egt=0.0;

		if (bufptr && *(++bufptr) != ',') { rpm = strtol(bufptr, &endp,10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { egt = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { pump_v = strtod(bufptr, &endp); bufptr = endp; }

	//	if (bufptr && *(++bufptr) != ',') { status = strtol(bufptr, &endp,10); bufptr = endp; }


		_engine_status->rpm[_e_id]=rpm;

		_engine_status->egt[_e_id]=(float)egt;

		_engine_status->vol[_e_id]=(float)pump_v;

	//	_engine_status->status=(uint8_t)status;

		ret=1;

	}else if ((_sent_cmd == jetcat_cmd_t::RA1)&&(memcmp(_rx_buffer, "OK,", 3) == 0) && (uiCalcComma == 4)) {
		/*
		  raeding the fault message from the engine
		  An example of the RA1 message string is:

		  1,HS,OK,1,20,0.645,4.66
		  Field   Meaning
		  0
		  1   the reason why the engine shuts down
		  2   environment temperature C
		  3   pump voltage min
		  4   pump voltage max
		*/
		int efault=0;

		if (bufptr && *(++bufptr) != ',') { efault = strtol(bufptr, &endp,10); bufptr = endp; }

		_engine_status->fault[_e_id]=efault;

		_fault[_e_id]=efault;

		ret=1;

	}else if ((_sent_cmd == jetcat_cmd_t::RFI)&&(memcmp(_rx_buffer, "OK,", 3) == 0) && (uiCalcComma == 6)) {
		/*
		  raeding the fuel message
		  An example of the RFI message string is:

		  1,HS,OK,0,536,0,7.55,41,14
		  Field   Meaning
		  0
		  1   fuel comsume rate at the moment
		  2   the fuel volume left in tank
		  3   the setting RPM
		  4   the battery's voltage
		  5   the engine's running time
		  6   the fuel volume that the engine comsued
		*/
		static bool warned = 0;
		double fuel_comsuption= 0.0,fuel_remain = 0.0,bat_v = 0.0;
		static float fcomsu_single[4]={0.0};
		float fcomsu_all=0;

		if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { bat_v=strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { fuel_comsuption=strtod(bufptr, &endp); bufptr = endp; }

		fcomsu_single[_e_id]=(float)fuel_comsuption;

		uint i=0;
		for(i=0;i<4;i++)
		{
			fcomsu_all+=fcomsu_single[i];
		}

		fuel_remain = _para_set.tank_cap-fcomsu_all;

		_engine_status->fuel_remain = fuel_remain;

		_engine_status->fuel_consumed=fcomsu_all;

		_engine_status->pump[_e_id]=(float)(100.0*pump_v/(bat_v*0.618));//0.618 is the maximum voltage ratio of the jetcat's pump power-100% power



		if((double)_para_set.fuel_low>(fuel_remain) &&!warned)
			{
				mavlink_log_critical(&mavlink_log_pub, "remaining fuel has reached the low point!");
				warned = 1;
			}

		ret=1;

	}else if ((_sent_cmd == jetcat_cmd_t::tco)&&(memcmp(_rx_buffer, "OK,", 3) == 0) && (uiCalcComma == 3)) {
		/*
		  starting or stopping the engine with returning message
		  An example of the tco message string is:

		  1,HS,OK,1,3,4
		  Field   Meaning
		  0
		  1   if the engine is executing the command? 0 not doing 1 doing
		  2   the controll mode of the engine at the moment 2-PWM 3-EXT 4-COM 5-CAN 6-GSU
		  3   the engine status at the moment
		*/
		int estatus=0;

		if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { estatus=strtol(bufptr, &endp,10); bufptr = endp; }

		_engine_status->status[_e_id]=(uint8_t)estatus;

		_status[_e_id]=estatus;

		ret=1;

	}else if ((_sent_cmd == jetcat_cmd_t::wpe)&&(memcmp(_rx_buffer, "OK,", 3) == 0) && (uiCalcComma == 4)) {
		/*
		  controlling the engine thrust with returning message
		  An example of the wpe message string is:

		  1,HS,OK,1,3,0,2.01
		  Field   Meaning
		  0
		  1   if the engine is executing the command? 0 not doing 1 doing
		  2   the controll mode of the engine at the moment 2-PWM 3-EXT 4-COM 5-CAN 6-GSU
		  3   the valid setting thrust %
		  4   the actually thrust at the moment
		*/
		double eload=0.0;

		if (bufptr && *(++bufptr) != ',') { strtol(bufptr, &endp,10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { strtol(bufptr, &endp,10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { strtol(bufptr, &endp,10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { eload = strtod(bufptr, &endp); bufptr = endp; }

		_engine_status->engine_load[_e_id]=(float)eload;

		ret=1;

	}
	return ret;
}

int
Jetcat::setJEpara() // Although linton engine only has 2 groups of parameters. But we need to compare them one by one. So it needs to do 5 times
{
/*	int je_br=_je_br.get();
	setBaudrate(je_br);  //no need to set baudrate dynamically while the engine's running.
	char para[30]={};

	static int i=0;
	PX4_INFO("set");
	PX4_INFO("para got: pwm max%f min%f close%f rpm_max:%d rpm_min:%d",(double)_para_got.pwm_max,(double)_para_got.pwm_min,(double)_para_got.pwm_stop,(int)_para_got.rpm_max,(int)_para_got.rpm_min);

	while(1)
	{
		if(i==0){ i++;		if(_para_set.rpm_min!=_para_got.rpm_min){sprintf(para,"@SetData=71,%d;\r\n",(int)_para_set.rpm_min);break;} }
		if(i==1){ i++;		if(_para_set.rpm_max!=_para_got.rpm_max){sprintf(para,"@SetData=70,%d;\r\n",(int)_para_set.rpm_max);break;} }
		if(i==2){ i++;		if(fabs(_para_set.pwm_stop-_para_got.pwm_stop)>.001){sprintf(para,"@SetData=12,%d;\r\n",(int)_para_set.pwm_stop);break;} }
		if(i==3){ i++;		if(fabs(_para_set.pwm_min-_para_got.pwm_min)>.001){sprintf(para,"@SetData=13,%d;\r\n",(int)_para_set.pwm_min);break;} }
		if(i==4){ i++;		if(fabs(_para_set.pwm_max-_para_got.pwm_max)>.001){sprintf(para,"@SetData=14,%d;\r\n",(int)_para_set.pwm_max);break;} }
		if(i>=5){ i=0;		return 1;}
	}
	PX4_INFO("i= %d",i);
	PX4_INFO("para set: pwm max%f min%f close%f rpm_max:%d rpm_min:%d ",(double)_para_set.pwm_max,(double)_para_set.pwm_min,(double)_para_set.pwm_stop,(int)_para_set.rpm_max,(int)_para_set.rpm_min);
	PX4_INFO("para got: pwm max%f min%f close%f rpm_max:%d rpm_min:%d",(double)_para_got.pwm_max,(double)_para_got.pwm_min,(double)_para_got.pwm_stop,(int)_para_got.rpm_max,(int)_para_got.rpm_min);

			warnx("got pwm max:%f,setted pwm max:%f",(double)_para_got.pwm_max,(double)_para_set.pwm_max);
			warnx("got pwm min:%f,setted pwm min:%f",(double)_para_got.pwm_min,(double)_para_set.pwm_min);
			warnx("got pwm stop:%f, setted pwm stop:%f",(double)_para_got.pwm_stop,(double)_para_set.pwm_stop);
			warnx("got rpm max:%f, setted rpm max:%f",(double)_para_got.rpm_max,(double)_para_set.rpm_max);
			warnx("got rpm min:%f, setted rpm min:%f",(double)_para_got.rpm_min,(double)_para_set.rpm_min);
		warnx("i=%d",i);

		write(para,sizeof(para));
	return 0;
*/ return 1;
}

int
Jetcat::getJEpara() //linton engine only need to send 2 groups of parameters request
{
/*	static uint8_t para_num=0;
	int para_got=0;
	const char get_pwm[]={"@HMI=13,0;\r\n"};
	const char get_rpm[]={"@HMI=10,0;\r\n"};
	if(para_num==0){write(get_pwm,sizeof(get_pwm));}
	if(para_num==1){write(get_rpm,sizeof(get_rpm));}
	if(para_num<2)para_num++;
	else
	{para_num = 0; para_got = 1;}

	return para_got;
*/ return 1;
}
/*
int
Jetcat::send_cmd(uint8_t cmd)
{
	_sent_cmd = cmd;
	switch(cmd)
	{
		case jetcat_cmd_t::RAC	:
		case jetcat_cmd_t::RGV	:
		case jetcat_cmd_t::RA1	:
		case jetcat_cmd_t::RFI	:
		case jetcat_cmd_t::TCO	:
		case jetcat_cmd_t::tco	:
		case jetcat_cmd_t::WRP	:
		case jetcat_cmd_t::wrp	:
		case jetcat_cmd_t::WPE	:
		case jetcat_cmd_t::wpe	:
		case jetcat_cmd_t::XLO	:
		case jetcat_cmd_t::SLO	:

		default : _sent_cmd = cmd;

	}
	return 1;
}
*/

int
Jetcat::pwm_start() //the pwm starting mode is according to jetcat's SEQUENCE starting proceduce
{
//	px4_sleep(1); //1s
//		setThr(1,0);//set the pwm to the throttle's minimum value 这100是disarm值
//	px4_usleep(500000);//1.5s
		setThr(jet_engine_ctl_s::EC_MIN,0);//set the throttle to minimum with the "trim" //其中100是disarm的值，200是微调值
	px4_sleep(1);//2.5s
		setThr(jet_engine_ctl_s::EC_MAX,0);//"push" the throttle to maximum
	px4_sleep(1);//3.5s
		setThr(jet_engine_ctl_s::EC_MIN,0); //finally "pull" it back to minimun with trim
	px4_sleep(1);//4.5s
		setThr(jet_engine_ctl_s::EC_FREE,0);
		return 1;
}

char
Jetcat::i2char(int i)
{
	if(i==1)return'1';
	if(i==2)return'2';
	if(i==3)return'3';
	if(i==4)return'4';
	return false;
}

int
Jetcat::cmdrac(uint8_t en)
{
	char RAC_status[9]; //the engine normal running status
//	PX4_INFO("               cmdrac");
	sprintf(RAC_status,"%1c,RAC,1\r",i2char(en));
	write(RAC_status, sizeof(RAC_status)-1); //这里一定要注意，sprintf里用\r的话，最后一个元素会补0，那么ECU会不回复命令直接输出数据。所以要减1
//	PX4_INFO("%.*s\n",9,RAC_status);
//	_sent_cmd=jetcat_cmd_t::RAC;
	collect();
	return 1;
}

int
Jetcat::cmdra1(uint8_t en)
{
	char RA1_status[9]; //the engine's abnormal status
//	PX4_INFO("                cmdra1");
	sprintf(RA1_status,"%1c,RA1,1\r",i2char(en));
	write(RA1_status, sizeof(RA1_status)-1);
//	_sent_cmd=jetcat_cmd_t::RA1;
	collect();
	return 1;
}

int
Jetcat::cmdrfi(uint8_t en)
{
	char RFI_status[9]; //the engine's fuel status
//	PX4_INFO("                 cmdrfi");
	sprintf(RFI_status,"%1c,RFI,1\r",i2char(en));
	write(RFI_status, sizeof(RFI_status)-1);
//	_sent_cmd=jetcat_cmd_t::RFI;
	collect();
	return 1;
}

int
Jetcat::cmdtco(uint8_t en,bool on)
{
	char jet_stop[9]; //it can't be longer than the chariter you really need. or it will block the next command.
//	PX4_INFO("                  cmdtco");
	if(on)
	sprintf(jet_stop,"%1c,tco,1\r",i2char(en));
	else
	sprintf(jet_stop,"%1c,tco,0\r",i2char(en));
	write(jet_stop, sizeof(jet_stop)-1);
//	_sent_cmd=jetcat_cmd_t::tco;
	collect();
	return 1;
}

int
Jetcat::cmdwpe(uint8_t en,float thr)
{
	char wpe_run[13];
	sprintf(wpe_run,"%1c,wpe,%.2f\r",i2char(en),(double)thr*100);
	if(wpe_run[12]!='\r')wpe_run[12]='\r';
//	PX4_INFO(wpe_run,sizeof(wpe_run));
//	PX4_INFO("%1c,wpe,%.2f\r",i2char(en),(double)thr*100);
	write(wpe_run, sizeof(wpe_run));
//	_sent_cmd=jetcat_cmd_t::wpe;
	collect();
	return 1;
}
