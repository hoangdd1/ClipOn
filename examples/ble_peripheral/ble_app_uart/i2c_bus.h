#ifndef I2C_BUS_H
#define I2C_BUS_H



/* Common addresses definition for temperature sensor. */
#define EE_24F02_ADDR       (0xA0 >> 1) /* 1010000b */
#define SENSOR_ADDR         (0x48 /*>> 1*/) // (0x0AU /*>> 1*/)

typedef enum _EEPROM_VARS
{
  MANUF_DATE,
  DEV_STR_NAME,
  SERIAL_NUMBER,
  ACTIVATE_KEY_H,
  ACTIVATE_KEY_L,
  _4G_APN_H,
  _4G_APN_L,
  _IP_ADDRESS,
  _UDP_PORT,
  _4G_CID,  
  _4G_AUTH,
  _4G_PINMUN,
  _4G_USR,
  _4G_PASS,
  NUM_OF_EE_VARS
}EEPROM_VARS;
      
typedef struct
{
        EEPROM_VARS index;
        unsigned char ee_addr;
        char ee_len;
	char *ee_buff;
        char ee_resotre_buff[16];
}EEPROM_DB;

extern EEPROM_DB ee_db[NUM_OF_EE_VARS];

int AT24CXX_Write(unsigned char WriteAddr,unsigned char *pBuffer,int NumToWrite);
int AT24CXX_Read(unsigned char ReadAddr,unsigned char *pBuffer,int NumToRead);
unsigned char AT24CXX_ReadOneByte(unsigned char ReadAddr);
void AT24CXX_WriteOneByte(unsigned char WriteAddr,unsigned char DataToWrite);
int AT24CXX_Init(void);
void InitI2CBUS(void);
void DenitI2CBUS(void);
void AT24CXX_restore_default(void);
int AT24XX_modem_params_exist(void);
int AT24XX_standalone_name(void);
void Ti_calibrate(void);                 //connection with TI component
void TiReadData_byI2c_command(void);     //read data after connection is done
float TiReadData_byI2c(void);            //read data after connection is done
void StartTiRead(void);                  //start read status
void StopTiRead(void);                   //stop read status
void update_water_sense_ti(void);        //manager of updating water flow status
void Firmware_Version(void);             //to read from TI - Firmware Version && Software Entry Point 
void Bootloader_Mode(void);              //Sending Command to Put the TI into BSL (Bootloader) Mode
void Password_to_BSL_1(void);            //Sending Password to Open the BSL
int Password_to_BSL_2(void);             //get response from bsl of the password. 
void Password_to_BSL_1_original(void);
int Mass_Erase_2(void);                  //Sending Command to ask BSL (Bootloader) remove mass(flash).
void Mass_Erase_1(void);                 //Sending Command to ask BSL (Bootloader) remove mass(flash).  
void Read_BSL_Version(void);             //Reading the BSL Version
void enter_BSL_Mode_by_HW(void);
void Executing_Firmware(void);           //Executing New/Current Firmware - when TI in the BSL 
void write_DataStruct(void);
void Sensor_Init( void );
#endif
