/****************************************************************************
 * this sample is released as public domain.  It is distributed in the hope that 
 * it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * Author: Danyu Li
 * Date: 09/18/18 
*****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <limits.h>
#include <ctype.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <sys/stat.h>
#include <sys/fcntl.h>

/*********************************************************************************************************
**                               Global data & Function declaration
*********************************************************************************************************/
#define CLEAR(x) 							memset(&(x), 0, sizeof(x))
#define MAX_PAIR_FOR_SPI_FLASH				(64)

//define the Leopard Imaging USB3.0 Camera
// uvc extension id
#define LI_XU_SENSOR_REGISTER_CONFIGURATION (0x0c)
#define LI_XU_SENSOR_REG_RW    			    (0x0e)
#define LI_XU_GENERIC_I2C_RW 				(0x10)

// define the buffer for storage
unsigned char buf1[5] 		= {0}; 				//used for LI_XU_SENSOR_REG_RW 
unsigned char buf2[256] 	= {0}; 				//used for LI_XU_SENSOR_REGISTER_CONFIGURATION
unsigned char buf3[256+6] 	= {0}; 				//used for LI_XU_GENERIC_I2C_RW

//TODO: change according to your setup
const char dev_name[64] = "/dev/video0";

struct uvc_xu_control_query xu_query;

#if 0
//TODO: used in helper function
typedef struct reg_seq
{
	unsigned char reg_data_width;
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_seq;
//TODO: used in helper function, modify it for different camera
const reg_seq ChangConfig[]=
{
	{2,0x098e,0x7c00},
	{2,0xfc00,0x2800},
	{2,0x0040,0x8100}
};
const reg_seq TrigEnable[]=
{
	{2,0x098e,0xc890},
	{1,0xc890,0x03  },
	{1,0xc891,0x03  },
	{1,0xc892,0x00  }
};
const reg_seq TrigDisable[]=
{
	{2,0x098e,0xc890},
	{1,0xc890,0x00  }
};
#endif

//used for SPI flash
//notice it only support 16-bit data
typedef struct reg_pair
{
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_pair;

const reg_pair ChangConfigFromFlash[]=
{
	{0x098e,0x7c00},
	{0xfc00,0x2800},
	{0x0040,0x8100}
};



/*********************************************************************************************************
**                                  Function definition
*********************************************************************************************************/

/* 
 * open the /dev/video* uvc camera device
 * 
 * args: 
 * 		device_name 
 * returns: 
 * 		file descriptor v4l2_dev
 */
int open_v4l2_device(char *device_name)
{
	int v4l2_dev;

	if(device_name == NULL)
		return -5;
	v4l2_dev = open(device_name, 0);

	printf("%s opened, handle=%d\n", device_name, v4l2_dev);
	return v4l2_dev;
}

/*
 * handle the error for opening the device
 * args:
 * 		none
 * returns:
 * 		none
 */
void error_handle()
{
	int res = errno;

	const char *err;
	switch(res) 
	{
		case ENOENT:	err = "Extension unit or control not found"; break;
		case ENOBUFS:	err = "Buffer size does not match control size"; break;
		case EINVAL:	err = "Invalid request code"; break;
		case EBADRQC:	err = "Request not supported by control"; break;
		default:		err = strerror(res); break;
	}

	printf("failed %s. (System code: %d) \n", err, res);

	return ;
}

/*
 * helper function to commnunicate with FX3 UVC defined extension unit
 * args:
 * 		fd 			- file descriptor
 *  	property_id - defined Leopard Imaging USB3.0 Camera uvc extension id
 * 		length 		- size of defined extension unit
 * 		buffer		- pointer for buffer data
 *  
 */
static void writeToUVCExtension(int fd, int property_id, 
								int length, unsigned char* buffer) 
{
	
	CLEAR(xu_query);
	xu_query.unit 		= 3;				//has to be unit 3
	xu_query.query 		= UVC_SET_CUR;		//request code to send to the device
	xu_query.size 		= length;
	xu_query.selector 	= property_id; 		
	xu_query.data 		= buffer;			//control buffer

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) 
		error_handle();
}

/*
 * helper function to commnunicate with FX3 UVC defined extension unit
 * args:
 * 		fd 			- file descriptor
 *  	property_id - defined Leopard Imaging USB3.0 Camera uvc extension id
 * 		length 		- size of defined extension unit
 * 		buffer		- pointer for buffer data
 *  
 */
static void readFromUVCExtension(int fd, int property_id, 
								int length, unsigned char* buffer) 
{
	CLEAR(xu_query);
	xu_query.unit 		= 3;				//has to be unit 3
	xu_query.query 		= UVC_GET_CUR;		//request code to send to the device
	xu_query.size 		= length;
	xu_query.selector 	= property_id; 		
	xu_query.data 		= buffer;			//control buffer

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) 
		error_handle();
}

/*
 *  register read/write for different slaves on I2C line
 * 
 *	Byte0: bit7 0:read;1:write. bit[6:0] regAddr width, 1:8-bit register address;
 														2:16-bit register address
 *		   0x81: write, regAddr is 8-bit; 0x82: write, regAddr is 16-bit
 *		   0x01: read,  regAddr is 8-bit; 0x02: read,  regAddr is 16-bit
 *	Byte1: Length of register data,1~256
 *	Byte2: i2c salve address 8bit
 *	Byte3: register address
 *	Byte4: register address(16bit) or register data
 *
 *  Register data starts from Byte4(8bit address) or Byte5(16bit address)
 * args:	
 * 		fd 		 - file descriptor
 * 		rw_flag	 - please read the above
 * 		bufCnt	 - define register data length(8-bit/16-bit)
 * 		slaveAddr- I2C address for slave
 * 		regAddr  - register address want to access
 * 		i2c_data - pointer to register value   
 */
void generic_I2C_write(int fd, int rw_flag, int bufCnt, 
					   int slaveAddr, int regAddr, unsigned char *i2c_data) 
{
	int regVal = 0;

	CLEAR(buf3);

	buf3[0] = rw_flag;
	buf3[1] = bufCnt-1;
	buf3[2] = slaveAddr>>8;
	buf3[3] = slaveAddr&0xff;
	buf3[4] = regAddr>>8;
	buf3[5] = regAddr&0xff;

	if (bufCnt == 1 ) {
		buf3[6] = *i2c_data;
		regVal = buf3[6];
	}
	else
	{
		buf3[6] = *(i2c_data+1);
		buf3[7] = *i2c_data;
		regVal = (buf3[6]<<8) + buf3[7];
	}

	writeToUVCExtension(fd, LI_XU_GENERIC_I2C_RW, 256+6, buf3); 
	printf("I2C slave ADDR[0x%x], Write REG[0x%x]: 0x%x\r\n",slaveAddr, regAddr, regVal);

}

/*
 *  register read/write for different slaves on I2C line
 *
 *  Register data starts from Byte4(8bit address) or Byte5(16bit address)
 * args:	
 * 		fd 		 - file descriptor
 * 		rw_flag	 - please read the above for details
 * 		bufCnt	 - define register data length(8-bit/16-bit)
 * 		slaveAddr- I2C address for slave
 * 		regAddr  - register address want to access
 * 		i2c_data - pointer to register value   
 */
void generic_I2C_read(int fd, int rw_flag, int bufCnt, 
					  int slaveAddr, int regAddr, unsigned char *i2c_data) 

{
	int regVal = 0;

	CLEAR(buf3);
	buf3[0] = rw_flag;
	buf3[1] = bufCnt-1;
	buf3[2] = slaveAddr>>8;
	buf3[3] = slaveAddr&0xff;
	buf3[4] = regAddr>>8;
	buf3[5] = regAddr&0xff;
	writeToUVCExtension(fd, LI_XU_GENERIC_I2C_RW, 256+6, buf3);
	buf3[6] = 0;
	buf3[7] = 0;
	readFromUVCExtension(fd, LI_XU_GENERIC_I2C_RW, 256+6, buf3);
	if(bufCnt==1) {
		regVal = buf3[6];
	}
	else
	{
		regVal = (buf3[6] << 8) + buf3[7];
	}
	printf("I2C slave ADDR[0x%x], Read REG[0x%x]: 0x%x\r\n", slaveAddr, regAddr, regVal);
	
}

/* 
 * I2C register read/write for the sensor 
 * args: 
 * 		fd 		- file descriptor
 * 		regAddr - register address want to access
 *		regVal  - register value to write
 */
void sensor_reg_write(int fd, int regAddr, int regVal) {
	
	CLEAR(buf1);
	
	buf1[0] = 1; 						//1 indicates for write
	buf1[1] = (regAddr >> 8) & 0xff;
	buf1[2] = regAddr & 0xff;
	buf1[3] = (regVal >> 8) & 0xff;
	buf1[4] = regVal & 0xff;

	writeToUVCExtension(fd, LI_XU_SENSOR_REG_RW, 5, buf1);

	printf("Write Sensor REG[0x%x]: 0x%x\r\n",regAddr, regVal);
}

/* 
 * I2C register read/write for the sensor 
 * args: 
 * 		fd 		- file descriptor
 * 		regAddr - register address want to access
 */
void sensor_reg_read(int fd, int regAddr) {

	int regVal = 0;

	CLEAR(buf1); 

	buf1[0] = 0; 					 	//0 indicates for read
	buf1[1] = (regAddr >> 8) & 0xff;
	buf1[2] = regAddr & 0xff;
    
	writeToUVCExtension(fd, LI_XU_SENSOR_REG_RW, 5, buf1);
	buf1[0] = 0; 						//0 indicates for read
	buf1[3] = 0;
	buf1[4] = 0;
	readFromUVCExtension(fd, LI_XU_SENSOR_REG_RW, 5, buf1);

	regVal = (buf1[3] << 8) + buf1[4];
	printf("Read Sensor REG[0x%x] = 0x%x\r\n", regAddr, regVal);

}

/*
 *	save register to spi flash on FX3, load it automatically at boot time
 *  args:
 * 		fd 		 - file descriptor
 * 		regCount - pairs of regAddr and regVal (up to 62)
 * 		buffer   - sensor register configuration
 */
void update_register_setting_from_configuration(int fd, int regCount, const struct reg_pair *buffer) 
{	
	int i;
	CLEAR(buf2);

	//set flags to match definition in firmware
	buf2[0] = 0x11;
	buf2[1] = 0x22;
	buf2[2] = 0x33;
	buf2[3] = 0x44;

	//regCount can be less than 62, match with firmware
	buf2[4] = regCount & 0xff;
	buf2[5] = regCount >> 8;
	buf2[6] = regCount >> 16;
	buf3[7] = regCount >> 24;

	//max reg, addr pair #= (256 -8)/4 = 62
	for (i = 2; i < MAX_PAIR_FOR_SPI_FLASH; i++) {

		int addr 	= buffer[i-2].reg_addr;
		int val 	= buffer[i-2].reg_val;

		if ((i-2)  < regCount) {
			if (addr != 0xffff && val != 0xffff) {
				sensor_reg_write(fd, addr, val);
			}
			buf2[4*i] = addr & 0xff;
			buf2[4*i + 1] = addr >> 8;
			buf2[4*i + 2] = val & 0xff;
			buf2[4*i + 3] = val >> 8;
		}
		else {
			buf2[4*i] = 0xff;
			buf2[4*i + 1] = 0xff;
			buf2[4*i + 2] = 0xff;
			buf2[4*i + 3] = 0xff;
		}

		if  ((i+1)%64 == 0) {
			writeToUVCExtension(fd, LI_XU_SENSOR_REGISTER_CONFIGURATION, 256, buf2);
		}

		//readFromUVCExtension(fd, LI_XU_SENSOR_REGISTER_CONFIGURATION, 256, buf2);
	}
	
}

//for testing
void main(int argc, char *argv[])
{
	int v4l2_dev;
	int regval, opt;
	int size;
	int interval = 0;

	while ((opt = getopt(argc, argv, "hi:")) != -1) {
		switch (opt) {
			case 'i':
			interval = atoi(optarg);
			if (interval < 0 ) {
				printf("interval between reset must be >= 0\n");
				exit(-1);
			}
			break;
			case 'h':
			default:
			printf("Usage %s [-h] [-i <interval(us)>]\n", argv[0]);
			return;
		}
	}

	v4l2_dev = open_v4l2_device(dev_name);

	if(v4l2_dev < 0)
	{
		printf("open camera %s failed,err code:%d\n\r",dev_name, v4l2_dev);
		return ;
	}

	//test
	generic_I2C_read(v4l2_dev, 0x02, 2, 0xBA, 0x098e, 0x00);
	sensor_reg_read(v4l2_dev, 0xfc00);
	sensor_reg_read(v4l2_dev, 0x0040);

	size = sizeof(ChangConfigFromFlash)/sizeof(reg_pair);
	update_register_setting_from_configuration(v4l2_dev, size, ChangConfigFromFlash); 
	generic_I2C_read(v4l2_dev, 0x02, 2, 0xBA, 0x098e, 0x00);
	sensor_reg_read(v4l2_dev, 0xfc00);
	sensor_reg_read(v4l2_dev, 0x0040);

	close(v4l2_dev);	

	return;
}