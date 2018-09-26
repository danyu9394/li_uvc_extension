/****************************************************************************
  This sample is released as public domain.  It is distributed in the hope it
  will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

  This is the sample code for Leopard USB3.0 AR0231 AP0200 GMSL2 camera for 
  register control under Linux using V4L2. For supporting more UVC extension
  unit features, firmware will need to get updated.
  
  Author: Danyu Li
  Date: 09/18/18 
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

/****************************************************************************
**                      Global data & Function declaration
*****************************************************************************/
#define CLEAR(x) 							memset(&(x), 0, sizeof(x))
#define SIZE(a)								(sizeof(a)/sizeof(*a))

#define MAX_PAIR_FOR_SPI_FLASH				(64)
#define AP020X_I2C_ADDR						(0xBA)
#define MAX9295_SER_I2C_ADDR				(0x80)
#define MAX9296_DESER_I2C_ADDR				(0x90)


//define the Leopard Imaging USB3.0 Camera
// uvc extension id
#define LI_XU_SENSOR_REGISTER_CONFIGURATION (0x0c)
#define LI_XU_SENSOR_REG_RW    			    (0x0e)
#define LI_XU_GENERIC_I2C_RW 				(0x10)

// define the buffer for storage
unsigned char buf1[5] 		= {0};	//for LI_XU_SENSOR_REG_RW 
unsigned char buf2[256] 	= {0};	//for LI_XU_SENSOR_REGISTER_CONFIGURATION
unsigned char buf3[256+6] 	= {0};	//for LI_XU_GENERIC_I2C_RW

// TODO: change according to your setup
char dev_name[64] = "/dev/video0";

struct uvc_xu_control_query xu_query;

//#define AP0202_WRITE_REG_ON_THE_FLY
#define AP0202_WRITE_REG_IN_FLASH

//test registers on the fly
#ifdef AP0202_WRITE_REG_ON_THE_FLY
// TODO: used in helper function
typedef struct reg_seq
{
	unsigned char reg_data_width;
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_seq;
// TODO: used in helper function, modify it for different camera
const reg_seq ChangConfig[]=
{
	{2,0x098e,0x7c00},
	{2,0xfc00,0x2800},
	{2,0x0040,0x8100},
	{2,0x0058,0x4444}
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

#ifdef AP0202_WRITE_REG_IN_FLASH
// used for SPI flash 
// notice it only support 16-bit data with SPI flash
typedef struct reg_pair
{
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_pair;

// TODO: put the register you want to set in here
const reg_pair ChangConfigFromFlash[]=
{
	{0x0058,0x3443},//customer_rev		
};
#endif

int open_v4l2_device(char *device_name);
void error_handle();
static void write_to_UVC_extension(int fd, int property_id, 
								int length, unsigned char* buffer);
static void read_from_UVC_extension(int fd, int property_id, 
								int length, unsigned char* buffer);

void generic_I2C_write(int fd, int rw_flag, int bufCnt, 
					   int slaveAddr, int regAddr, unsigned char *i2c_data); 
void generic_I2C_read(int fd, int rw_flag, int bufCnt, 
					  int slaveAddr, int regAddr); 

void sensor_reg_write(int fd, int regAddr, int regVal);
void sensor_reg_read(int fd, int regAddr);

void load_register_setting_from_configuration(int fd, int regCount, 
										const struct reg_pair *buffer); 
void load_register_setting_from_flash_manually(int fd); 

/*****************************************************************************
**                           Function definition
*****************************************************************************/

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
static void write_to_UVC_extension(int fd, int property_id, 
								int length, unsigned char* buffer) 
{
	
	CLEAR(xu_query);
	xu_query.unit 		= 3;			//has to be unit 3
	xu_query.query 		= UVC_SET_CUR;	//request code to send to the device
	xu_query.size 		= length;
	xu_query.selector 	= property_id; 		
	xu_query.data 		= buffer;		//control buffer

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
static void read_from_UVC_extension(int fd, int property_id, 
								int length, unsigned char* buffer) 
{
	CLEAR(xu_query);
	xu_query.unit 		= 3;			//has to be unit 3
	xu_query.query 		= UVC_GET_CUR;	//request code to send to the device
	xu_query.size 		= length;
	xu_query.selector 	= property_id; 		
	xu_query.data 		= buffer;		//control buffer

	if(ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query) != 0) 
		error_handle();
}

/*
 *  register read/write for different slaves on I2C line
 * 
 *	Byte0: bit7 0:read;1:write. bit[6:0] regAddr width, 1:8-bit register addr;
 														2:16-bit register addr
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

	write_to_UVC_extension(fd, LI_XU_GENERIC_I2C_RW, 256+6, buf3); 
	printf("I2C slave ADDR[0x%x], Write REG[0x%x]: 0x%x\r\n",
			slaveAddr, regAddr, regVal);

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
					  int slaveAddr, int regAddr) 

{
	int regVal = 0;

	CLEAR(buf3);
	buf3[0] = rw_flag;
	buf3[1] = bufCnt-1;
	buf3[2] = slaveAddr>>8;
	buf3[3] = slaveAddr&0xff;
	buf3[4] = regAddr>>8;
	buf3[5] = regAddr&0xff;
	write_to_UVC_extension(fd, LI_XU_GENERIC_I2C_RW, 256+6, buf3);
	buf3[6] = 0;
	buf3[7] = 0;
	read_from_UVC_extension(fd, LI_XU_GENERIC_I2C_RW, 256+6, buf3);
	if(bufCnt==1) {
		regVal = buf3[6];
	}
	else
	{
		regVal = (buf3[6] << 8) + buf3[7];
	}
	printf("I2C slave ADDR[0x%x], Read REG[0x%x]: 0x%x\r\n", 
			slaveAddr, regAddr, regVal);
	
}

/* 
 * I2C register read/write for the sensor 
 * args: 
 * 		fd 		- file descriptor
 * 		regAddr - register address want to access
 *		regVal  - register value to write
 */
void sensor_reg_write(int fd, int regAddr, int regVal) 
{
	
	CLEAR(buf1);
	
	buf1[0] = 1; 						//1 indicates for write
	buf1[1] = (regAddr >> 8) & 0xff;
	buf1[2] = regAddr & 0xff;
	buf1[3] = (regVal >> 8) & 0xff;
	buf1[4] = regVal & 0xff;

	write_to_UVC_extension(fd, LI_XU_SENSOR_REG_RW, 5, buf1);

	printf("Write Sensor REG[0x%x]: 0x%x\r\n",regAddr, regVal);
}

/* 
 * I2C register read/write for the sensor 
 * args: 
 * 		fd 		- file descriptor
 * 		regAddr - register address want to access
 */
void sensor_reg_read(int fd, int regAddr) 
{

	int regVal = 0;

	CLEAR(buf1); 

	buf1[0] = 0; 					 	//0 indicates for read
	buf1[1] = (regAddr >> 8) & 0xff;
	buf1[2] = regAddr & 0xff;
    
	write_to_UVC_extension(fd, LI_XU_SENSOR_REG_RW, 5, buf1);
	buf1[0] = 0; 						//0 indicates for read
	buf1[3] = 0;
	buf1[4] = 0;
	read_from_UVC_extension(fd, LI_XU_SENSOR_REG_RW, 5, buf1);

	regVal = (buf1[3] << 8) + buf1[4];
	printf("Read Sensor REG[0x%x] = 0x%x\r\n", regAddr, regVal);

}

/*
 *	save register to spi flash on FX3, load it automatically when boot time
 *  flash for storage is set to be 256 bytes
 *  args:
 * 		fd 		 - file descriptor
 * 		regCount - pairs of regAddr and regVal (up to 62)
 * 		buffer   - sensor register configuration
 */
void load_register_setting_from_configuration(int fd, int regCount, 
										const struct reg_pair *buffer) 
{	
	int i;
	CLEAR(buf2);
	printf("save to flash\r\n");
	//set flags to match definition in firmware
	buf2[0] = 0x11;
	buf2[1] = 0x22;
	buf2[2] = 0x33;
	buf2[3] = 0x44;

	//regCount can be less than 62, match with firmware
	buf2[4] = regCount & 0xff;
	buf2[5] = regCount >> 8;
	buf2[6] = regCount >> 16;
	buf2[7] = regCount >> 24;

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

		if  ((i+1)%MAX_PAIR_FOR_SPI_FLASH == 0) {
			//store it to SPI flash
			write_to_UVC_extension(fd, LI_XU_SENSOR_REGISTER_CONFIGURATION,
				256, buf2);
			sleep(1);
		}		
	}
}

/*
 *	load register to spi flash on FX3 manually 
 *  #########FOR TEST ONLY###########
 *  flash for storage is set to be 256 bytes
 *  args:
 * 		fd 		 - file descriptor
 */
void load_register_setting_from_flash_manually(int fd)  
{
	int reg_flash_length, addr, val, i;
	printf("load from flash\r\n");
	CLEAR(buf2);
	read_from_UVC_extension(fd, LI_XU_SENSOR_REGISTER_CONFIGURATION, 
		256, buf2);
	if(buf2[0]!=0x11 && buf2[1]!=0x22 && buf2[2]!=0x33 && buf2[3]!=0x44)
	{
		return;
	}
	reg_flash_length = buf2[4] | buf2[5]<<8 | buf2[6]<<16 | buf2[7]<<24;
	
	if (reg_flash_length > 63) return;

	for(i=0; i< reg_flash_length; i++) {
		addr = buf2[i*4+8] | buf2[i*4+9]<<8;
		val = buf2[i*4+10] | buf2[i*4+11]<<8;
		if(addr != 0xffff && val!=0xffff) {
			sensor_reg_write(fd, addr, val);
		}
	}
	
}

// for testing
int main()
{
	int v4l2_dev;	

	v4l2_dev = open_v4l2_device(dev_name);

	if(v4l2_dev < 0)
	{
		printf("open camera %s failed,err code:%d\n\r",dev_name, v4l2_dev);
		return 0;
	}

	//test
	#ifdef AP0202_WRITE_REG_ON_THE_FLY
	unsigned int i;
	for(i=0 ; i< sizeof(ChangConfig)/sizeof(reg_seq); i++) 
		generic_I2C_write(v4l2_dev,0x82,ChangConfig[i].reg_data_width, 
			AP020X_I2C_ADDR,ChangConfig[i].reg_addr,					
			(unsigned char*)&(ChangConfig[i].reg_val));					
		
	for(i=0 ; i< sizeof(ChangConfig)/sizeof(reg_seq); i++) {
		generic_I2C_read(v4l2_dev,0x02,ChangConfig[i].reg_data_width,
		AP020X_I2C_ADDR,ChangConfig[i].reg_addr);
	}

	generic_I2C_read(v4l2_dev, 0x02, 1, MAX9295_SER_I2C_ADDR, 0x0000);
	generic_I2C_read(v4l2_dev, 0x02, 1, MAX9296_DESER_I2C_ADDR, 0x0000);
	#endif

	#ifdef AP0202_WRITE_REG_IN_FLASH
	
	//load_register_setting_from_configuration(v4l2_dev, 
	//	SIZE(ChangConfigFromFlash), ChangConfigFromFlash); 
	
	sleep(1);
	generic_I2C_read(v4l2_dev, 0x02, 2, AP020X_I2C_ADDR, 0x0058);
	sensor_reg_read(v4l2_dev, 0x0058);
	#endif

	close(v4l2_dev);	
	return 0;
}