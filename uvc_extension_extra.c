/****************************************************************************
  This sample is released as public domain.  It is distributed in the hope it
  will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

  This is the sample code for Leopard USB3.0 AR0231 AP0200 GMSL2 camera for 
  register control under Linux using V4L2. For supporting more UVC extension
  unit features, firmware will need to get updated.
  
  Author: Danyu Li
  Date: 10/10/18 
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
**                      	Global data 
*****************************************************************************/
#define CLEAR(x) 							memset(&(x), 0, sizeof(x))
#define SIZE(a)								(sizeof(a)/sizeof(*a))

//define the Leopard Imaging USB3.0 Camera
// uvc extension id
#define LI_XU_PTS_QUERY						(0x08)	// not suppoprted by all usb cameras
#define LI_XU_SENSOR_REGISTER_CONFIGURATION (0x0c)	// not supported by all usb cameras
#define LI_XU_SENSOR_REG_RW    			    (0x0e)
#define LI_XU_GENERIC_I2C_RW 				(0x10)

#define MAX_PAIR_FOR_SPI_FLASH				(64)	
// define the buffer for storage
unsigned char buf1[5] 		= {0};	//for LI_XU_SENSOR_REG_RW 
unsigned char buf2[256] 	= {0};	//for LI_XU_SENSOR_REGISTER_CONFIGURATION
unsigned char buf3[256+6] 	= {0};	//for LI_XU_GENERIC_I2C_RW
unsigned char buf4[4]		= {0};  //for LI_XU_PTS_QUERY

// TODO: change according to your setup
char dev_name[64] = "/dev/video0";

struct uvc_xu_control_query xu_query;

enum v4l2_buf_type type;

//#define AP0202_WRITE_REG_ON_THE_FLY
//#define AP0202_WRITE_REG_IN_FLASH

//test registers on the fly
#ifdef AP0202_WRITE_REG_ON_THE_FLY

#define AP020X_I2C_ADDR						(0xBA)
#define MAX9295_SER_I2C_ADDR				(0x80)
#define MAX9296_DESER_I2C_ADDR				(0x90)

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

//#define AR0231_MIPI_TESTING
#ifdef AR0231_MIPI_TESTING
#define AR0231_I2C_ADDR						(0x20)
typedef struct reg_seq
{
	unsigned char reg_data_width;
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_seq;

const reg_seq AR0231_MIPI_REG_TESTING[]=
{
	{2,0x3064,0x1802}, // SMIA_TEST
	{2,0x3056,0x0080}, // GREEN1_GAIN
	{2,0x3058,0x0080}, // BLUE_GAIN
	{2,0x305a,0x0080}, // RED_GAIN
	{2,0x305c,0x0080}, //GREEN2_GAIN
	{2,0x3138,0x000B}  //OTPM_TCFG_OPT
};
#endif

typedef struct reg_pair
{
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_pair;

#ifdef AP0202_WRITE_REG_IN_FLASH
// used for SPI flash 
// notice it only support 16-bit data with SPI flash
// TODO: put the register you want to set in here
const reg_pair ChangConfigFromFlash[]=
{
	{0x305e,0x0222},//customer_rev		
};
#endif

#define IMX334_MONO_MIPI_TESTING 
#ifdef IMX334_MONO_MIPI_TESTING
#define IMX334_I2C_ADDR						(0x34)
typedef struct reg_seq
{
	unsigned char reg_data_width;
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_seq;

const reg_seq IMX334_MIPI_REG_TESTING[]=
{
	{1,0x30E8,0x14}, // PROGRAMMABLE_GAIN_CONTROL
	{1,0x3302,0x32}, // BLKLEVEL[7:0]
	{1,0x3303,0x00}, // BLKLEVEL[1:0] range 0x0-0x3ff
};
#endif
/****************************************************************************
**							 Function declaration
*****************************************************************************/
static int open_v4l2_device(char *device_name);
static void error_handle();

static void write_to_UVC_extension(int fd, int property_id, 
								int length, unsigned char* buffer);
static void read_from_UVC_extension(int fd, int property_id, 
								int length, unsigned char* buffer);

static void uvc_get_control(int fd, unsigned int id);
static void uvc_set_control(int fd, unsigned int id, int value);


void generic_I2C_write(int fd, int rw_flag, int bufCnt, 
					   int slaveAddr, int regAddr, unsigned char *i2c_data); 
void generic_I2C_read(int fd, int rw_flag, int bufCnt, 
					  int slaveAddr, int regAddr); 

void sensor_reg_write(int fd, int regAddr, int regVal);
void sensor_reg_read(int fd, int regAddr);

void load_register_setting_from_configuration(int fd, int regCount, 
										const struct reg_pair *buffer); 
void load_register_setting_from_flash_manually(int fd); 

void get_pts(int fd);
void set_pts(int fd, unsigned long initVal);

void set_gain(int fd, int analog_gain);
void get_gain(int fd);

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
static int open_v4l2_device(char *device_name)
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
static void error_handle()
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

static void uvc_get_control(int fd, unsigned int id)
{
	struct v4l2_control ctrl;
	int ret;

	ctrl.id = id;

	ret = ioctl(fd, VIDIOC_G_CTRL, &ctrl);
	if (ret < 0) {
		printf("unable to get control: %s (%d).\n",
			strerror(errno), errno);
		return;
	}

	printf("Control 0x%08x value %u\n", id, ctrl.value);
}

static void uvc_set_control(int fd, unsigned int id, int value)
{
	struct v4l2_control ctrl;
	int ret;
	CLEAR(ctrl);
	ctrl.id = id;
	ctrl.value = value;

	ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);
	if (ret < 0) {
		printf("unable to set control: %s (%d).\n",
			strerror(errno), errno);
		return;
	}

	printf("Control 0x%08x set to %u, is %u\n", id, value,
		ctrl.value);
}

/*--------------------------------------------------------------------------- */

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

/*
 * currently PTS information are placed in 2 places
 * 1. UVC video data header 
 * 	- a 33-bit PTS timestamp as defined in ITU-REC-H.222.0/ISO/IEC 13818-1
 * 	- you can get the PTS info by looking at usb sniffing data
 * 	- uvc header byte 1(BFH) bit 2 for PTS indicator
 *  - PTS is stored in byte 2-5 of a 12-byte uvc header
 *  - PTS value stays the same in a frame
 * 2.  first 4 bytes in a given frame
 *  - you can get the PTS by grabbing the first 4 bytes of each frame
 *  - byte order: little endian
 *  
 * PTS info:
 * currently on FX3, PTS counter is sampling at frequency 403M/16 ~ 25MHz
 * for different camera boards, crystall will slightly drift over time
 * PTS increments over time can be calculated by 
 * 		(1/frame_rate)/(1/25MHz) = 25MHz/frame_rate
 * 
 * this method is an add-on extension unit for query PTS info
 * args:
 * 		fd 		 - file descriptor
 */
void get_pts(int fd){
	CLEAR(buf4);
	read_from_UVC_extension(fd, LI_XU_PTS_QUERY, 4, buf4);
	unsigned long pts = buf4[0] | buf4[1] << 8 | buf4[2] << 16| buf4[3] << 24;
	printf("get PTS = 0x%x\r\n", pts);
	
}

/* set PTS counter initial value
 * args:
 * 		fd 		- file descriptor
 * 		initVal	- initial counter start value for PTS(4-byte long)
 *
 */
void set_pts(int fd, unsigned long initVal){
	CLEAR(buf4);
	buf4[0] = (initVal>>24)&0xff;
	buf4[1] = (initVal>>16)&0xff;
	buf4[2] = (initVal>>8)&0xff;
	buf4[3] = initVal&0xff;
	write_to_UVC_extension(fd, LI_XU_PTS_QUERY, 4, buf4);
	printf("set init count value of PTS to 0x%x\r\n", initVal);
}


void set_gain(int fd, int analog_gain) {
	uvc_set_control(fd, V4L2_CID_GAIN, analog_gain);
}
void get_gain(int fd) {
	uvc_get_control(fd, V4L2_CID_GAIN);
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
	load_register_setting_from_configuration(v4l2_dev, 
		SIZE(ChangConfigFromFlash), ChangConfigFromFlash); 
	
	sleep(1);
	//generic_I2C_read(v4l2_dev, 0x02, 2, AP020X_I2C_ADDR, 0x0058);
	sensor_reg_write(v4l2_dev, 0x5080, 0x00);
	sensor_reg_read(v4l2_dev, 0x4308);
	sensor_reg_read(v4l2_dev, 0x5080);
	#endif

	#ifdef OS05A20_PTS_QUERY
	set_pts(v4l2_dev, 0);
	get_pts(v4l2_dev);
	#endif

	#ifdef AR0231_MIPI_TESTING
	unsigned int i;
	for(i=0 ; i< sizeof(AR0231_MIPI_REG_TESTING)/sizeof(reg_seq); i++) {
		//TODO: choose either one of the function below for register read
		generic_I2C_read(v4l2_dev,0x02,AR0231_MIPI_REG_TESTING[i].reg_data_width,
			AR0231_I2C_ADDR,AR0231_MIPI_REG_TESTING[i].reg_addr);

		sensor_reg_read(v4l2_dev, AR0231_MIPI_REG_TESTING[i].reg_addr);
	}
	#endif

	#ifdef IMX334_MONO_MIPI_TESTING
	unsigned int i;
	for(i=0 ; i< sizeof(IMX334_MIPI_REG_TESTING)/sizeof(reg_seq); i++) {
		//TODO: choose either one of the function below for register read
		generic_I2C_read(v4l2_dev,0x02,IMX334_MIPI_REG_TESTING[i].reg_data_width,
			IMX334_I2C_ADDR,IMX334_MIPI_REG_TESTING[i].reg_addr);

		sensor_reg_read(v4l2_dev, IMX334_MIPI_REG_TESTING[i].reg_addr);
	}
	set_gain(v4l2_dev,5);
	get_gain(v4l2_dev);
	#endif
	close(v4l2_dev);	
	return 0;
}