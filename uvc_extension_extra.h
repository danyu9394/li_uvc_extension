#pragma once
/****************************************************************************
**                      	Global data 
*****************************************************************************/
#define CLEAR(x) 							memset(&(x), 0, sizeof(x))
#define SIZE(a)								(sizeof(a)/sizeof(*a))

// define the Leopard Imaging USB3.0 Camera
// uvc extension id
#define LI_XU_SENSOR_MODES_SWITCH           (0x01)
#define LI_XU_SENSOR_WINDOW_REPOSITION      (0x02)
#define LI_XU_LED_MODES                     (0x03)
#define LI_XU_SENSOR_GAIN_CONTROL_RGB       (0x04)
#define LI_XU_SENSOR_GAIN_CONTROL_A         (0x05)
#define LI_XU_SENSOR_EXPOSURE_TIME          (0x06)
#define LI_XU_SENSOR_UUID_HWFW_REV          (0x07)
#define LI_XU_PTS_QUERY						(0x08)	// not suppoprted by all usb cameras
#define LI_XU_SOFT_TRIGGER                  (0x09)
#define LI_XU_TRIGGER_DELAY                 (0x0a)
#define LI_XU_EX_MAX_MIN_INFO               (0x0b)
#define LI_XU_SENSOR_REGISTER_CONFIGURATION (0x0c)	// not supported by all usb cameras
#define LI_XU_SENSOR_REG_RW    			    (0x0e)
#define LI_XU_ERASE_EEPROM                  (0x0f)
#define LI_XU_GENERIC_I2C_RW 				(0x10)
#define LI_XU_SENSOR_DEFECT_PIXEL_TABLE     (0x11)

// I2C slave address list
//  On-semi
#define AP020X_I2C_ADDR						(0xBA)
#define AR0231_I2C_ADDR						(0x20)
#define AR0144_I2C_ADDR                     (0x20)
// Sony
#define IMX334_I2C_ADDR						(0x34)
#define IMX390_I2C_ADDR                     (0x34)
#define IMX324_I2C_ADDR                     (0x34)
// Omnivision
#define OV2311_I2C_ADDR                     (0xC0)
#define OS05A20_I2C_ADDR                    (0x6C)
// Maxim
#define MAX9295_SER_I2C_ADDR				(0x80)
#define MAX9296_DESER_I2C_ADDR				(0x90)

struct uvc_xu_control_query xu_query;

typedef struct reg_pair
{
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_pair;

typedef struct reg_seq
{
	unsigned char reg_data_width;
	unsigned short reg_addr;
	unsigned short reg_val;
}reg_seq;

// define the buffer for storage
unsigned char buf1[5] 		= {0};	//for LI_XU_SENSOR_REG_RW 
unsigned char buf2[256] 	= {0};	//for LI_XU_SENSOR_REGISTER_CONFIGURATION
unsigned char buf3[256+6] 	= {0};	//for LI_XU_GENERIC_I2C_RW
unsigned char buf4[4]		= {0};  //for LI_XU_PTS_QUERY

#define MAX_PAIR_FOR_SPI_FLASH				(64)	

//test registers on the fly
#ifdef AP0202_WRITE_REG_ON_THE_FLY
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
// TODO: put the register you want to set in here
const reg_pair ChangConfigFromFlash[]=
{
	{0x305e,0x0222},//customer_rev		
};
#endif

//#define AR0231_MIPI_TESTING
#ifdef AR0231_MIPI_TESTING
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

#define IMX334_MONO_MIPI_TESTING 
#ifdef IMX334_MONO_MIPI_TESTING
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