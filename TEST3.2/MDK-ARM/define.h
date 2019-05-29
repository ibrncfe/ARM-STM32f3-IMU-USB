#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_spi.h"
#include "math.h"





#define ABS(x)         (x < 0) ? (-x) : x


/**
 * @brief LED Types Definition
 */
typedef enum
{
  LED3 = 0,
  LED4 = 1,
  LED5 = 2,
  LED6 = 3,
  LED7 = 4,
  LED8 = 5,
  LED9 = 6,
  LED10 = 7,
  
  LED_GREEN  = LED6, 
  LED_ORANGE = LED5, 
  LED_RED    = LED3, 
  LED_BLUE   = LED4,
  LED_GREEN_2  = LED7,
  LED_ORANGE_2 = LED8,
  LED_RED_2    = LED10,
  LED_BLUE_2   = LED9
}Led_TypeDef;

/**
 * @brief BUTTON Types Definition
 */
typedef enum 
{
  B1 = 0

}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1

}ButtonMode_TypeDef;

#define I2Cx_TIMEOUT_MAX                      0x10000     

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////Leds Definitions/////////////////////////////
///////////////////////////////////////////////////////////////
#define LEDn                             8

#define LED6_PIN                         GPIO_PIN_15
#define LED6_GPIO_PORT                   GPIOE
#define LED6_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE() 
#define LED6_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define LED8_PIN                         GPIO_PIN_14
#define LED8_GPIO_PORT                   GPIOE
#define LED8_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE() 
#define LED8_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()
  
#define LED10_PIN                        GPIO_PIN_13
#define LED10_GPIO_PORT                  GPIOE
#define LED10_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define LED10_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()
  
#define LED9_PIN                         GPIO_PIN_12
#define LED9_GPIO_PORT                   GPIOE
#define LED9_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define LED9_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()
  
#define LED7_PIN                         GPIO_PIN_11
#define LED7_GPIO_PORT                   GPIOE
#define LED7_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define LED7_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define LED5_PIN                         GPIO_PIN_10
#define LED5_GPIO_PORT                   GPIOE
#define LED5_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define LED5_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define LED3_PIN                         GPIO_PIN_9
#define LED3_GPIO_PORT                   GPIOE
#define LED3_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define LED4_PIN                         GPIO_PIN_8
#define LED4_GPIO_PORT                   GPIOE
#define LED4_GPIO_CLK_ENABLE()           __GPIOE_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __GPIOE_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__)   (((__LED__) == LED3) ? LED3_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED4) ? LED4_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED5) ? LED5_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED6) ? LED6_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED7) ? LED7_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED8) ? LED8_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED9) ? LED9_GPIO_CLK_ENABLE() :\
                                         ((__LED__) == LED10) ? LED10_GPIO_CLK_ENABLE() : 0 )

#define LEDx_GPIO_CLK_DISABLE(__LED__)  (((__LED__) == LED3) ? LED3_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED4) ? LED4_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED5) ? LED5_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED6) ? LED6_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED7) ? LED7_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED8) ? LED8_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED9) ? LED9_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED10) ? LED10_GPIO_CLK_DISABLE() : 0 )


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////Buttons Definitions//////////////////////////
///////////////////////////////////////////////////////////////
#define BUTTONn                          1  

#define USER_BUTTON_PIN                  GPIO_PIN_0
#define USER_BUTTON_GPIO_PORT            GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()   __GPIOA_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn            EXTI0_IRQn 

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__)  (((__BUTTON__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_ENABLE() : 0 )

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__) (((__BUTTON__) == BUTTON_USER) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0 )

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////I2C Definitions//////////////////////////////
///////////////////////////////////////////////////////////////
/*##################### I2C ###################################*/

/**
  * @brief  Definition for I2C Interface pins (I2C1 used)
  */
#define DISCOVERY_I2Cx                        I2C1
#define DISCOVERY_I2Cx_CLK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_I2Cx_CLK_DISABLE()          __HAL_RCC_I2C1_CLK_DISABLE()
#define DISCOVERY_I2Cx_FORCE_RESET()          __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()        __HAL_RCC_I2C1_RELEASE_RESET() 

#define DISCOVERY_I2Cx_SCL_PIN                GPIO_PIN_6                  /* PB.06 */
#define DISCOVERY_I2Cx_SDA_PIN                GPIO_PIN_7                  /* PB.07 */

#define DISCOVERY_I2Cx_GPIO_PORT              GPIOB                       /* GPIOB */
#define DISCOVERY_I2Cx_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 
#define DISCOVERY_I2Cx_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2Cx_AF                     GPIO_AF4_I2C1

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////SPI1 Definitions/////////////////////////////
///////////////////////////////////////////////////////////////
/*##################### SPIx ###################################*/

#define DISCOVERY_SPIx                        SPI1
#define DISCOVERY_SPIx_CLK_ENABLE()           __SPI1_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_PORT              GPIOA                      /* GPIOA */
#define DISCOVERY_SPIx_AF                     GPIO_AF5_SPI1
#define DISCOVERY_SPIx_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define DISCOVERY_SPIx_GPIO_CLK_DISABLE()     __GPIOA_CLK_DISABLE()
#define DISCOVERY_SPIx_SCK_PIN                GPIO_PIN_5                 /* PA.05 */
#define DISCOVERY_SPIx_MISO_PIN               GPIO_PIN_6                 /* PA.06 */
#define DISCOVERY_SPIx_MOSI_PIN               GPIO_PIN_7                 /* PA.07 */
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)
 

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////GYRO Definitions/////////////////////////////
///////////////////////////////////////////////////////////////
/*##################### GYRO ##########################*/
/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/* Chip Select macro definition */
#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  GYRO SPI Interface pins
  */
#define GYRO_CS_GPIO_PORT          		GPIOE                       /* GPIOE */
#define GYRO_CS_GPIO_CLK_ENABLE()		__GPIOE_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()		__GPIOE_CLK_DISABLE()
#define GYRO_CS_PIN                		GPIO_PIN_3                  /* PE.03 */

#define GYRO_INT_GPIO_PORT        		GPIOE                       /* GPIOE */
#define GYRO_INT_GPIO_CLK_ENABLE()		__GPIOE_CLK_ENABLE()
#define GYRO_INT_GPIO_CLK_DISABLE()		__GPIOE_CLK_DISABLE()
#define GYRO_INT1_PIN              		GPIO_PIN_1                  /* PE.00 */
#define GYRO_INT1_EXTI_IRQn        		EXTI1_IRQn 
#define GYRO_INT2_PIN              		GPIO_PIN_2                  /* PE.01 */
#define GYRO_INT2_EXTI_IRQn        		EXTI2_TS_IRQn 

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
///////////////////////////L3GD20 Definitions////////////////////
/////////////////////////////////////////////////////////////////
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */ 
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

/** @defgroup Power_Mode_selection 
  * @{
  */
#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)

/** @defgroup OutPut_DataRate_Selection 
  * @{
  */
#define L3GD20_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_4    ((uint8_t)0xC0)

/** @defgroup Axes_Selection 
  * @{
  */
#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)
#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        ((uint8_t)0x00)

/** @defgroup Bandwidth_Selection 
  * @{
  */
#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)

/** @defgroup Full_Scale_Selection 
  * @{
  */
#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20) 
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)

/** @defgroup Full_Scale_Sensitivity 
  * @{
  */
#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */
  
/** @defgroup Block_Data_Update 
  * @{
  */  
#define L3GD20_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GD20_BlockDataUpdate_Single      ((uint8_t)0x80)
  
/** @defgroup Endian_Data_selection
  * @{
  */  
#define L3GD20_BLE_LSB                     ((uint8_t)0x00)
#define L3GD20_BLE_MSB	                   ((uint8_t)0x40)

/** @defgroup High_Pass_Filter_status 
  * @{
  */   
#define L3GD20_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define L3GD20_HIGHPASSFILTER_ENABLE	   ((uint8_t)0x10)

/** @defgroup INT1_INT2_selection 
  * @{
  */   
#define L3GD20_INT1                        ((uint8_t)0x00)
#define L3GD20_INT2                        ((uint8_t)0x01)

/** @defgroup INT1_Interrupt_status 
  * @{
  */   
#define L3GD20_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)

/** @defgroup INT2_Interrupt_status 
  * @{
  */   
#define L3GD20_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)

/** @defgroup INT1_Interrupt_ActiveEdge 
  * @{
  */   
#define L3GD20_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GD20_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)

/** @defgroup Boot_Mode_selection 
  * @{
  */
#define L3GD20_BOOT_NORMALMODE             ((uint8_t)0x00)
#define L3GD20_BOOT_REBOOTMEMORY           ((uint8_t)0x80)
 
/** @defgroup High_Pass_Filter_Mode 
  * @{
  */   
#define L3GD20_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           ((uint8_t)0x30)

/** @defgroup High_Pass_CUT OFF_Frequency 
  * @{
  */   
#define L3GD20_HPFCF_0              0x00
#define L3GD20_HPFCF_1              0x01
#define L3GD20_HPFCF_2              0x02
#define L3GD20_HPFCF_3              0x03
#define L3GD20_HPFCF_4              0x04
#define L3GD20_HPFCF_5              0x05
#define L3GD20_HPFCF_6              0x06
#define L3GD20_HPFCF_7              0x07
#define L3GD20_HPFCF_8              0x08
#define L3GD20_HPFCF_9              0x09

typedef struct
{  
  void       (*Init)(uint16_t);
  uint8_t    (*ReadID)(void);
  void       (*Reset)(void);
  void       (*ConfigIT)(uint16_t); 
  void       (*EnableIT)(uint8_t);
  void       (*DisableIT)(uint8_t);  
  uint8_t    (*ITStatus)(uint16_t, uint16_t);   
  void       (*ClearIT)(uint16_t, uint16_t); 
  void       (*FilterConfig)(uint8_t);  
  void       (*FilterCmd)(uint8_t);  
  void       (*GetXYZ)(float *);
}GYRO_DrvTypeDef;

typedef struct
{
  uint8_t Power_Mode;                         /* Power-down/Sleep/Normal Mode */
  uint8_t Output_DataRate;                    /* OUT data rate */
  uint8_t Axes_Enable;                        /* Axes enable */
  uint8_t Band_Width;                         /* Bandwidth selection */
  uint8_t BlockData_Update;                   /* Block Data Update */
  uint8_t Endianness;                         /* Endian Data selection */
  uint8_t Full_Scale;                         /* Full Scale selection */
}GYRO_InitTypeDef;

/* GYRO High Pass Filter struct */
typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
}GYRO_FilterConfigTypeDef;

typedef enum 
{
  GYRO_OK = 0,
  GYRO_ERROR = 1,
  GYRO_TIMEOUT = 2
} 
GYRO_StatusTypeDef;

/*GYRO Interrupt struct */
typedef struct
{
  uint8_t Latch_Request;                      /* Latch interrupt request into CLICK_SRC register */
  uint8_t Interrupt_Axes;                     /* X, Y, Z Axes Interrupts */ 
  uint8_t Interrupt_ActiveEdge;               /* Interrupt Active edge */
}GYRO_InterruptConfigTypeDef;  


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
///////////////////////////ACCLEROMETER Definitions////////////////////
/////////////////////////////////////////////////////////////////
/**
  * @brief  ACCELEROMETER I2C1 Interface pins
  */
#define ACCELERO_I2C_ADDRESS             0x32

#define ACCELERO_DRDY_PIN                GPIO_PIN_2                  /* PE.02 */
#define ACCELERO_DRDY_GPIO_PORT          GPIOE                       /* GPIOE */
#define ACCELERO_DRDY_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE() 
#define ACCELERO_DRDY_GPIO_CLK_DISABLE() __HAL_RCC_GPIOE_CLK_DISABLE() 
#define ACCELERO_DRDY_EXTI_IRQn          EXTI2_TSC_IRQn              /*TAMP_STAMP_IRQn*/

#define ACCELERO_INT_GPIO_PORT           GPIOE                       /* GPIOE */
#define ACCELERO_INT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_INT_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_INT1_PIN                GPIO_PIN_4                  /* PE.04 */
#define ACCELERO_INT1_EXTI_IRQn          EXTI4_IRQn 
#define ACCELERO_INT2_PIN                GPIO_PIN_5                  /* PE.05 */
#define ACCELERO_INT2_EXTI_IRQn          EXTI9_5_IRQn 

typedef struct
{  
  void      (*Init)(uint16_t);
  uint8_t   (*ReadID)(void);
  void      (*Reset)(void);
  void      (*ConfigIT)(void);
  void      (*EnableIT)(uint8_t);
  void      (*DisableIT)(uint8_t);
  uint8_t   (*ITStatus)(uint16_t);
  void      (*ClearIT)(void);
  void      (*FilterConfig)(uint8_t);
  void      (*FilterCmd)(uint8_t);
  void      (*GetXYZ)(int16_t *);
}ACCELERO_DrvTypeDef;


/* ACCELERO struct */
typedef struct
{
  uint8_t Power_Mode;                         /* Power-down/Normal Mode */
  uint8_t AccOutput_DataRate;                 /* OUT data rate */
  uint8_t Axes_Enable;                        /* Axes enable */
  uint8_t High_Resolution;                    /* High Resolution enabling/disabling */
  uint8_t BlockData_Update;                   /* Block Data Update */
  uint8_t Endianness;                         /* Endian Data selection */
  uint8_t AccFull_Scale;                      /* Full Scale selection */
}ACCELERO_InitTypeDef;

/* ACCELERO High Pass Filter struct */
typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;      /* Internal filter mode */
  uint8_t HighPassFilter_CutOff_Frequency;    /* High pass filter cut-off frequency */
  uint8_t HighPassFilter_AOI1;                /* HPF_enabling/disabling for AOI function on interrupt 1 */
  uint8_t HighPassFilter_AOI2;                /* HPF_enabling/disabling for AOI function on interrupt 2 */
}ACCELERO_FilterConfigTypeDef;

/* ACCELERO Mag struct */
typedef struct
{
  uint8_t Temperature_Sensor;                /* Temperature sensor enable/disable */
  uint8_t MagOutput_DataRate;                /* OUT data rate */
  uint8_t Working_Mode;                      /* operating mode */
  uint8_t MagFull_Scale;                     /* Full Scale selection */
}LACCELERO_InitTypeDef;

typedef enum 
{
  ACCELERO_OK = 0,
  ACCELERO_ERROR = 1,
  ACCELERO_TIMEOUT = 2
} 
ACCELERO_StatusTypeDef;


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
///////////////////////////LSM303DLHC Definitions////////////////
/////////////////////////////////////////////////////////////////
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
/* Exported constant IO ------------------------------------------------------*/
#define ACC_I2C_ADDRESS                      0x32
#define MAG_I2C_ADDRESS                      0x3C

/* Acceleration Registers */
#define LSM303DLHC_WHO_AM_I_ADDR             0x0F  /* device identification register */
#define LSM303DLHC_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303DLHC_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303DLHC_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303DLHC_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303DLHC_CTRL_REG5_A               0x24  /* Control register 5 acceleration */
#define LSM303DLHC_CTRL_REG6_A               0x25  /* Control register 6 acceleration */
#define LSM303DLHC_REFERENCE_A               0x26  /* Reference register acceleration */
#define LSM303DLHC_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303DLHC_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303DLHC_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303DLHC_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303DLHC_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303DLHC_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */ 
#define LSM303DLHC_FIFO_CTRL_REG_A           0x2E  /* Fifo control Register acceleration */
#define LSM303DLHC_FIFO_SRC_REG_A            0x2F  /* Fifo src Register acceleration */

#define LSM303DLHC_INT1_CFG_A                0x30  /* Interrupt 1 configuration Register acceleration */
#define LSM303DLHC_INT1_SOURCE_A             0x31  /* Interrupt 1 source Register acceleration */
#define LSM303DLHC_INT1_THS_A                0x32  /* Interrupt 1 Threshold register acceleration */
#define LSM303DLHC_INT1_DURATION_A           0x33  /* Interrupt 1 DURATION register acceleration */

#define LSM303DLHC_INT2_CFG_A                0x34  /* Interrupt 2 configuration Register acceleration */
#define LSM303DLHC_INT2_SOURCE_A             0x35  /* Interrupt 2 source Register acceleration */
#define LSM303DLHC_INT2_THS_A                0x36  /* Interrupt 2 Threshold register acceleration */
#define LSM303DLHC_INT2_DURATION_A           0x37  /* Interrupt 2 DURATION register acceleration */

#define LSM303DLHC_CLICK_CFG_A               0x38  /* Click configuration Register acceleration */
#define LSM303DLHC_CLICK_SOURCE_A            0x39  /* Click 2 source Register acceleration */
#define LSM303DLHC_CLICK_THS_A               0x3A  /* Click 2 Threshold register acceleration */

#define LSM303DLHC_TIME_LIMIT_A              0x3B  /* Time Limit Register acceleration */
#define LSM303DLHC_TIME_LATENCY_A            0x3C  /* Time Latency Register acceleration */
#define LSM303DLHC_TIME_WINDOW_A             0x3D  /* Time window register acceleration */

/* Magnetic field Registers */
#define LSM303DLHC_CRA_REG_M                 0x00  /* Control register A magnetic field */
#define LSM303DLHC_CRB_REG_M                 0x01  /* Control register B magnetic field */
#define LSM303DLHC_MR_REG_M                  0x02  /* Control register MR magnetic field */
#define LSM303DLHC_OUT_X_H_M                 0x03  /* Output Register X magnetic field */
#define LSM303DLHC_OUT_X_L_M                 0x04  /* Output Register X magnetic field */
#define LSM303DLHC_OUT_Z_H_M                 0x05  /* Output Register Z magnetic field */
#define LSM303DLHC_OUT_Z_L_M                 0x06  /* Output Register Z magnetic field */ 
#define LSM303DLHC_OUT_Y_H_M                 0x07  /* Output Register Y magnetic field */
#define LSM303DLHC_OUT_Y_L_M                 0x08  /* Output Register Y magnetic field */

#define LSM303DLHC_SR_REG_M                  0x09  /* Status Register magnetic field */
#define LSM303DLHC_IRA_REG_M                 0x0A  /* IRA Register magnetic field */
#define LSM303DLHC_IRB_REG_M                 0x0B  /* IRB Register magnetic field */
#define LSM303DLHC_IRC_REG_M                 0x0C  /* IRC Register magnetic field */

#define LSM303DLHC_TEMP_OUT_H_M              0x31  /* Temperature Register magnetic field */
#define LSM303DLHC_TEMP_OUT_L_M              0x32  /* Temperature Register magnetic field */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_LMS303DLHC                   ((uint8_t)0x33)

/** @defgroup Acc_Power_Mode_selection
  * @{
  */
#define LSM303DLHC_NORMAL_MODE            ((uint8_t)0x00)
#define LSM303DLHC_LOWPOWER_MODE          ((uint8_t)0x08)
/**
  * @}
  */

/** @defgroup Acc_OutPut_DataRate_Selection
  * @{
  */
#define LSM303DLHC_ODR_1_HZ                ((uint8_t)0x10)  /*!< Output Data Rate = 1 Hz */
#define LSM303DLHC_ODR_10_HZ               ((uint8_t)0x20)  /*!< Output Data Rate = 10 Hz */
#define LSM303DLHC_ODR_25_HZ               ((uint8_t)0x30)  /*!< Output Data Rate = 25 Hz */
#define LSM303DLHC_ODR_50_HZ               ((uint8_t)0x40)  /*!< Output Data Rate = 50 Hz */
#define LSM303DLHC_ODR_100_HZ              ((uint8_t)0x50)  /*!< Output Data Rate = 100 Hz */
#define LSM303DLHC_ODR_200_HZ              ((uint8_t)0x60)  /*!< Output Data Rate = 200 Hz */
#define LSM303DLHC_ODR_400_HZ              ((uint8_t)0x70)  /*!< Output Data Rate = 400 Hz */
#define LSM303DLHC_ODR_1620_HZ_LP          ((uint8_t)0x80)  /*!< Output Data Rate = 1620 Hz only in Low Power Mode */
#define LSM303DLHC_ODR_1344_HZ             ((uint8_t)0x90)  /*!< Output Data Rate = 1344 Hz in Normal mode and 5376 Hz in Low Power Mode */
/**
  * @}
  */

/** @defgroup Acc_Axes_Selection
  * @{
  */
#define LSM303DLHC_X_ENABLE                ((uint8_t)0x01)
#define LSM303DLHC_Y_ENABLE                ((uint8_t)0x02)
#define LSM303DLHC_Z_ENABLE                ((uint8_t)0x04)
#define LSM303DLHC_AXES_ENABLE             ((uint8_t)0x07)
#define LSM303DLHC_AXES_DISABLE            ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Acc_High_Resolution
  * @{
  */
#define LSM303DLHC_HR_ENABLE               ((uint8_t)0x08)
#define LSM303DLHC_HR_DISABLE              ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Acc_Full_Scale_Selection
  * @{
  */
#define LSM303DLHC_FULLSCALE_2G            ((uint8_t)0x00)  /*!< ±2 g */
#define LSM303DLHC_FULLSCALE_4G            ((uint8_t)0x10)  /*!< ±4 g */
#define LSM303DLHC_FULLSCALE_8G            ((uint8_t)0x20)  /*!< ±8 g */
#define LSM303DLHC_FULLSCALE_16G           ((uint8_t)0x30)  /*!< ±16 g */
/**
  * @}
  */

/** @defgroup Acc_Full_Scale_Selection
  * @{
  */
#define LSM303DLHC_ACC_SENSITIVITY_2G     ((uint8_t)1)  /*!< accelerometer sensitivity with 2 g full scale [mg/LSB] */
#define LSM303DLHC_ACC_SENSITIVITY_4G     ((uint8_t)2)  /*!< accelerometer sensitivity with 4 g full scale [mg/LSB] */
#define LSM303DLHC_ACC_SENSITIVITY_8G     ((uint8_t)4)  /*!< accelerometer sensitivity with 8 g full scale [mg/LSB] */
#define LSM303DLHC_ACC_SENSITIVITY_16G    ((uint8_t)12) /*!< accelerometer sensitivity with 12 g full scale [mg/LSB] */
/**
  * @}
  */

/** @defgroup Acc_Block_Data_Update
  * @{
  */  
#define LSM303DLHC_BlockUpdate_Continous   ((uint8_t)0x00) /*!< Continuos Update */
#define LSM303DLHC_BlockUpdate_Single      ((uint8_t)0x80) /*!< Single Update: output registers not updated until MSB and LSB reading */
/**
  * @}
  */
  
/** @defgroup Acc_Endian_Data_selection
  * @{
  */  
#define LSM303DLHC_BLE_LSB                 ((uint8_t)0x00) /*!< Little Endian: data LSB @ lower address */
#define LSM303DLHC_BLE_MSB                 ((uint8_t)0x40) /*!< Big Endian: data MSB @ lower address */
/**
  * @}
  */
  
/** @defgroup Acc_Boot_Mode_selection
  * @{
  */
#define LSM303DLHC_BOOT_NORMALMODE         ((uint8_t)0x00)
#define LSM303DLHC_BOOT_REBOOTMEMORY       ((uint8_t)0x80)
/**
  * @}
  */  
 
/** @defgroup Acc_High_Pass_Filter_Mode
  * @{
  */   
#define LSM303DLHC_HPM_NORMAL_MODE_RES     ((uint8_t)0x00)
#define LSM303DLHC_HPM_REF_SIGNAL          ((uint8_t)0x40)
#define LSM303DLHC_HPM_NORMAL_MODE         ((uint8_t)0x80)
#define LSM303DLHC_HPM_AUTORESET_INT       ((uint8_t)0xC0)
/**
  * @}
  */

/** @defgroup Acc_High_Pass_CUT OFF_Frequency
  * @{
  */   
#define LSM303DLHC_HPFCF_8                 ((uint8_t)0x00)
#define LSM303DLHC_HPFCF_16                ((uint8_t)0x10)
#define LSM303DLHC_HPFCF_32                ((uint8_t)0x20)
#define LSM303DLHC_HPFCF_64                ((uint8_t)0x30)
/**
  * @}
  */
    
/** @defgroup Acc_High_Pass_Filter_status
  * @{
  */   
#define LSM303DLHC_HIGHPASSFILTER_DISABLE  ((uint8_t)0x00)
#define LSM303DLHC_HIGHPASSFILTER_ENABLE   ((uint8_t)0x08)
/**
  * @}
  */
  
/** @defgroup Acc_High_Pass_Filter_Click_status
  * @{
  */   
#define LSM303DLHC_HPF_CLICK_DISABLE       ((uint8_t)0x00)
#define LSM303DLHC_HPF_CLICK_ENABLE        ((uint8_t)0x04)
/**
  * @}
  */

/** @defgroup Acc_High_Pass_Filter_AOI1_status
  * @{
  */   
#define LSM303DLHC_HPF_AOI1_DISABLE        ((uint8_t)0x00)
#define LSM303DLHC_HPF_AOI1_ENABLE	       ((uint8_t)0x01)
/**
  * @}
  */
  
/** @defgroup Acc_High_Pass_Filter_AOI2_status
  * @{
  */   
#define LSM303DLHC_HPF_AOI2_DISABLE        ((uint8_t)0x00)
#define LSM303DLHC_HPF_AOI2_ENABLE         ((uint8_t)0x02)
/**
  * @}
  */ 

/** @defgroup Acc_Interrupt1_Configuration_definition
  * @{
  */
#define LSM303DLHC_IT1_CLICK               ((uint8_t)0x80)
#define LSM303DLHC_IT1_AOI1                ((uint8_t)0x40)
#define LSM303DLHC_IT1_AOI2                ((uint8_t)0x20)
#define LSM303DLHC_IT1_DRY1                ((uint8_t)0x10)
#define LSM303DLHC_IT1_DRY2                ((uint8_t)0x08)
#define LSM303DLHC_IT1_WTM                 ((uint8_t)0x04)
#define LSM303DLHC_IT1_OVERRUN             ((uint8_t)0x02)
/**
  * @}
  */  
 
/** @defgroup Acc_Interrupt2_Configuration_definition
  * @{
  */
#define LSM303DLHC_IT2_CLICK               ((uint8_t)0x80)
#define LSM303DLHC_IT2_INT1                ((uint8_t)0x40)
#define LSM303DLHC_IT2_INT2                ((uint8_t)0x20)
#define LSM303DLHC_IT2_BOOT                ((uint8_t)0x10)
#define LSM303DLHC_IT2_ACT                 ((uint8_t)0x08)
#define LSM303DLHC_IT2_HLACTIVE            ((uint8_t)0x02)
/**
  * @}
  */ 

/** @defgroup Acc_INT_Combination_Status
  * @{
  */   
#define LSM303DLHC_OR_COMBINATION          ((uint8_t)0x00)  /*!< OR combination of enabled IRQs */
#define LSM303DLHC_AND_COMBINATION         ((uint8_t)0x80)  /*!< AND combination of enabled IRQs */
#define LSM303DLHC_MOV_RECOGNITION         ((uint8_t)0x40)  /*!< 6D movement recognition */
#define LSM303DLHC_POS_RECOGNITION         ((uint8_t)0xC0)  /*!< 6D position recognition */
/**
  * @}
  */

/** @defgroup Acc_INT_Axes
  * @{
  */   
#define LSM303DLHC_Z_HIGH                  ((uint8_t)0x20)  /*!< Z High enabled IRQs */
#define LSM303DLHC_Z_LOW                   ((uint8_t)0x10)  /*!< Z low enabled IRQs */
#define LSM303DLHC_Y_HIGH                  ((uint8_t)0x08)  /*!< Y High enabled IRQs */
#define LSM303DLHC_Y_LOW                   ((uint8_t)0x04)  /*!< Y low enabled IRQs */
#define LSM303DLHC_X_HIGH                  ((uint8_t)0x02)  /*!< X High enabled IRQs */
#define LSM303DLHC_X_LOW                   ((uint8_t)0x01)  /*!< X low enabled IRQs */
/**
  * @}
  */
      
/** @defgroup Acc_INT_Click
* @{
*/   
#define LSM303DLHC_Z_DOUBLE_CLICK          ((uint8_t)0x20)  /*!< Z double click IRQs */
#define LSM303DLHC_Z_SINGLE_CLICK          ((uint8_t)0x10)  /*!< Z single click IRQs */
#define LSM303DLHC_Y_DOUBLE_CLICK          ((uint8_t)0x08)  /*!< Y double click IRQs */
#define LSM303DLHC_Y_SINGLE_CLICK          ((uint8_t)0x04)  /*!< Y single click IRQs */
#define LSM303DLHC_X_DOUBLE_CLICK          ((uint8_t)0x02)  /*!< X double click IRQs */
#define LSM303DLHC_X_SINGLE_CLICK          ((uint8_t)0x01)  /*!< X single click IRQs */
/**
* @}
*/
  
/** @defgroup Acc_INT1_Interrupt_status
  * @{
  */   
#define LSM303DLHC_INT1INTERRUPT_DISABLE   ((uint8_t)0x00)
#define LSM303DLHC_INT1INTERRUPT_ENABLE    ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup Acc_INT1_Interrupt_ActiveEdge
  * @{
  */   
#define LSM303DLHC_INT1INTERRUPT_LOW_EDGE  ((uint8_t)0x20)
#define LSM303DLHC_INT1INTERRUPT_HIGH_EDGE ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Mag_Data_Rate
  * @{
  */ 
#define LSM303DLHC_ODR_0_75_HZ              ((uint8_t) 0x00)  /*!< Output Data Rate = 0.75 Hz */
#define LSM303DLHC_ODR_1_5_HZ               ((uint8_t) 0x04)  /*!< Output Data Rate = 1.5 Hz */
#define LSM303DLHC_ODR_3_0_HZ               ((uint8_t) 0x08)  /*!< Output Data Rate = 3 Hz */
#define LSM303DLHC_ODR_7_5_HZ               ((uint8_t) 0x0C)  /*!< Output Data Rate = 7.5 Hz */
#define LSM303DLHC_ODR_15_HZ                ((uint8_t) 0x10)  /*!< Output Data Rate = 15 Hz */
#define LSM303DLHC_ODR_30_HZ                ((uint8_t) 0x14)  /*!< Output Data Rate = 30 Hz */
#define LSM303DLHC_ODR_75_HZ                ((uint8_t) 0x18)  /*!< Output Data Rate = 75 Hz */
#define LSM303DLHC_ODR_220_HZ               ((uint8_t) 0x1C)  /*!< Output Data Rate = 220 Hz */
/**
  * @}
  */
 
/** @defgroup Mag_Full_Scale
  * @{
  */ 
#define  LSM303DLHC_FS_1_3_GA               ((uint8_t) 0x20)  /*!< Full scale = ±1.3 Gauss */
#define  LSM303DLHC_FS_1_9_GA               ((uint8_t) 0x40)  /*!< Full scale = ±1.9 Gauss */
#define  LSM303DLHC_FS_2_5_GA               ((uint8_t) 0x60)  /*!< Full scale = ±2.5 Gauss */
#define  LSM303DLHC_FS_4_0_GA               ((uint8_t) 0x80)  /*!< Full scale = ±4.0 Gauss */
#define  LSM303DLHC_FS_4_7_GA               ((uint8_t) 0xA0)  /*!< Full scale = ±4.7 Gauss */
#define  LSM303DLHC_FS_5_6_GA               ((uint8_t) 0xC0)  /*!< Full scale = ±5.6 Gauss */
#define  LSM303DLHC_FS_8_1_GA               ((uint8_t) 0xE0)  /*!< Full scale = ±8.1 Gauss */
/**
  * @}
  */ 
 
/**
 * @defgroup Magnetometer_Sensitivity
 * @{
 */
#define LSM303DLHC_M_SENSITIVITY_XY_1_3Ga     1100  /*!< magnetometer X Y axes sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_1_9Ga     855   /*!< magnetometer X Y axes sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_2_5Ga     670   /*!< magnetometer X Y axes sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_4Ga       450   /*!< magnetometer X Y axes sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_4_7Ga     400   /*!< magnetometer X Y axes sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_5_6Ga     330   /*!< magnetometer X Y axes sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_XY_8_1Ga     230   /*!< magnetometer X Y axes sensitivity for 8.1 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_1_3Ga      980   /*!< magnetometer Z axis sensitivity for 1.3 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_1_9Ga      760   /*!< magnetometer Z axis sensitivity for 1.9 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_2_5Ga      600   /*!< magnetometer Z axis sensitivity for 2.5 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_4Ga        400   /*!< magnetometer Z axis sensitivity for 4 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_4_7Ga      355   /*!< magnetometer Z axis sensitivity for 4.7 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_5_6Ga      295   /*!< magnetometer Z axis sensitivity for 5.6 Ga full scale [LSB/Ga] */
#define LSM303DLHC_M_SENSITIVITY_Z_8_1Ga      205   /*!< magnetometer Z axis sensitivity for 8.1 Ga full scale [LSB/Ga] */

/** @defgroup Mag_Working_Mode
  * @{
  */ 
#define LSM303DLHC_CONTINUOS_CONVERSION      ((uint8_t) 0x00)   /*!< Continuous-Conversion Mode */
#define LSM303DLHC_SINGLE_CONVERSION         ((uint8_t) 0x01)   /*!< Single-Conversion Mode */
#define LSM303DLHC_SLEEP                     ((uint8_t) 0x02)   /*!< Sleep Mode */                       


/** @defgroup Mag_Temperature_Sensor
  * @{
  */ 
#define LSM303DLHC_TEMPSENSOR_ENABLE         ((uint8_t) 0x80)   /*!< Temp sensor Enable */
#define LSM303DLHC_TEMPSENSOR_DISABLE        ((uint8_t) 0x00)   /*!< Temp sensor Disable */
  
/** @defgroup LSM303DLHC_Exported_Functions
  * @{
  */
/* ACC functions */
void    LSM303DLHC_AccInit(uint16_t InitStruct);
uint8_t LSM303DLHC_AccReadID(void);
void    LSM303DLHC_AccRebootCmd(void);
void    LSM303DLHC_AccFilterConfig(uint8_t FilterStruct);
void    LSM303DLHC_AccFilterCmd(uint8_t HighPassFilterState);
void    LSM303DLHC_AccReadXYZ(int16_t* pData);
void    LSM303DLHC_AccFilterClickCmd(uint8_t HighPassFilterClickState);
void    LSM303DLHC_AccIT1Enable(uint8_t LSM303DLHC_IT);
void    LSM303DLHC_AccIT1Disable(uint8_t LSM303DLHC_IT);
void    LSM303DLHC_AccIT2Enable(uint8_t LSM303DLHC_IT);
void    LSM303DLHC_AccIT2Disable(uint8_t LSM303DLHC_IT);
void    LSM303DLHC_AccINT1InterruptEnable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303DLHC_AccINT1InterruptDisable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303DLHC_AccINT2InterruptEnable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303DLHC_AccINT2InterruptDisable(uint8_t ITCombination, uint8_t ITAxes);
void    LSM303DLHC_AccClickITEnable(uint8_t ITClick);
void    LSM303DLHC_AccClickITDisable(uint8_t ITClick);
void    LSM303DLHC_AccZClickITConfig(void);

#if 0
/* MAG functions */
void    LSM303DLHC_MagInit(LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct);
uint8_t LSM303DLHC_MagGetDataStatus(void);
void    LSM303DLHC_CompassReadAcc(int16_t* pData);
#endif

/* COMPASS / ACCELERO IO functions */
void    COMPASSACCELERO_IO_Init(void);
void    COMPASSACCELERO_IO_ITConfig(void);
void    COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value);
uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr);




///////////////////////////////////////////////////////
/*  L3GD20_Exported_Functions
// Sensor Configuration Functions */ 
void    L3GD20_Init(uint16_t InitStruct);
uint8_t L3GD20_ReadID(void);
void    L3GD20_RebootCmd(void);
uint8_t Temperature_ReadID(void);

/* Interrupt Configuration Functions */
void    L3GD20_INT1InterruptConfig(uint16_t Int1Config);
void    L3GD20_EnableIT(uint8_t IntSel);
void    L3GD20_DisableIT(uint8_t IntSel);

/* High Pass Filter Configuration Functions */
void    L3GD20_FilterConfig(uint8_t FilterStruct);
void    L3GD20_FilterCmd(uint8_t HighPassFilterState);
void    L3GD20_ReadXYZAngRate(float *pfData);
uint8_t L3GD20_GetDataStatus(void);

/* Gyroscope IO functions */
void    GYRO_IO_Init(void);
void    GYRO_IO_DeInit(void);
void    GYRO_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void    GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

/* Sensor Configuration Functions */ 
uint8_t BSP_GYRO_Init(void);
void BSP_GYRO_Reset(void);
uint8_t BSP_GYRO_ReadID(void);
void    BSP_GYRO_ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct);
void BSP_GYRO_EnableIT(uint8_t IntPin);
void BSP_GYRO_DisableIT(uint8_t IntPin);
void BSP_GYRO_GetXYZ(float* pfData);

/* Acc functions */
uint8_t   BSP_ACCELERO_Init(void);
void      BSP_ACCELERO_Reset(void);
void      BSP_ACCELERO_GetXYZ(int16_t *pDataXYZ);

/* SPIx bus function */
static void     SPIx_Init(void);
static uint8_t  SPIx_WriteRead(uint8_t byte);
static void     SPIx_Error (void);
static void     SPIx_MspInit(SPI_HandleTypeDef *hspi);

/* I2Cx bus function */
static void     I2Cx_Init(void);
static void     I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  I2Cx_ReadData(uint16_t Addr, uint8_t Reg);
static void     I2Cx_Error (void);
static void     I2Cx_MspInit(I2C_HandleTypeDef *hi2c);

/* Functions Test */
void ACCELERO_MEMS_Test(void);
void GYRO_MEMS_Test(void);


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////FUNCTIONS////////////////////////////////////
///////////////////////////////////////////////////////////////
void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void Toggle_Leds(void);
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
uint32_t BSP_PB_GetState(Button_TypeDef Button);

