//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       serial.cpp
 *
 *
 * Environment  :       g++
 * Latest Update:       2008/07/07
 *
 * Designer(s)  :       m.sanpei (AMSL)
 * Author(s)    :       m.sanpei (AMSL)
 *
 * CopyRight    :       2007, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2007/08/23
 *
 */
//-----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "serial.h"

#include <iostream>
using namespace std;

//#define DEBUG

Serial::Serial(int baudrate, const char* modemdevice, int vmin, int lflag)
{
#if 1 
		struct termios toptions;

		fd_ = open(modemdevice, O_RDWR | O_NOCTTY | O_NDELAY );
		if (fd_ == -1)  {     // Could not open the port.
			perror("roomba_init_serialport: Unable to open port ");
			exit(-1);
    }
    
		tcgetattr(fd_, &oldtio_);
		if (tcgetattr(fd_, &toptions) < 0) {
			perror("roomba_init_serialport: Couldn't get term attributes");
				exit(-1);
		}

		cfsetispeed(&toptions, baudrate);
		cfsetospeed(&toptions, baudrate);

		// 8N1
		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;
		// no flow control
		toptions.c_cflag &= ~CRTSCTS;

		toptions.c_cflag    |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
		toptions.c_iflag    &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

		toptions.c_lflag    &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
		toptions.c_oflag    &= ~OPOST; // make raw

		toptions.c_cc[VMIN]  = 26;
		toptions.c_cc[VTIME] = 2;           // FIXME: not sure about this

		if( tcsetattr(fd_, TCSANOW, &toptions) < 0) {
			perror("roomba_init_serialport: Couldn't set term attributes");
			exit(-1);
		}
#endif

}


Serial::~Serial()
{
	tcsetattr(fd_,TCSANOW,&oldtio_);
	close(fd_);
}


int Serial::read(unsigned char* p, int len)
{
	return ::read(fd_, p, len);
}

int Serial::write(const unsigned char* p, int len)
{
	return ::write(fd_, p, len);
}

void Serial::setVmin(int vmin) {
 
	newtio_.c_iflag = IGNPAR;

	newtio_.c_cc[VTIME]    = 1;   /* キャラクタ間タイマ未使用*/
	newtio_.c_cc[VMIN]     = vmin;   /* vmin文字受け取るまでブロックする*/

}

void Serial::setRts(int sw)
{
	int status;
	ioctl(fd_, TIOCMGET, &status); /* set the serial port status */

	if(sw)      /* set the RTS line */
		status &= ~TIOCM_RTS;
	else
		status |= TIOCM_RTS;

	ioctl(fd_, TIOCMSET, &status); /* set the serial port status */
}

#ifdef DEBUG
int main()
{
	char rdata[255];
	char sdata[255];


	Serial test(B19200, "/dev/ttyUSB0");

	sprintf(sdata, "V\r");
	
	test.write_serial(sdata, strlen(sdata));

	test.read_serial(rdata);

	printf("%s",rdata);

	return 0;
}


#endif
