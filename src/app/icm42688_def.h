#ifndef __ICM42688_REG_H__
#define __ICM42688_REG_H__

#include <stdint.h>

typedef enum ICM42688_REG
{
    // Accesible from all user banks
    REG_BANK_SEL                 = 0x76,
        
    // User Bank 0    
    UB0_REG_DEVICE_CONFIG         = 0x11,
    // break    
    UB0_REG_DRIVE_CONFIG         = 0x13,
    UB0_REG_INT_CONFIG           = 0x14,
    // break    
    UB0_REG_FIFO_CONFIG         = 0x16,
    // break
    UB0_REG_TEMP_DATA1            = 0x1D,
    UB0_REG_TEMP_DATA0            = 0x1E,
    UB0_REG_ACCEL_DATA_X1         = 0x1F,
    UB0_REG_ACCEL_DATA_X0         = 0x20,
    UB0_REG_ACCEL_DATA_Y1         = 0x21,
    UB0_REG_ACCEL_DATA_Y0         = 0x22,
    UB0_REG_ACCEL_DATA_Z1         = 0x23,
    UB0_REG_ACCEL_DATA_Z0         = 0x24,
    UB0_REG_GYRO_DATA_X1          = 0x25,
    UB0_REG_GYRO_DATA_X0          = 0x26,
    UB0_REG_GYRO_DATA_Y1          = 0x27,
    UB0_REG_GYRO_DATA_Y0          = 0x28,
    UB0_REG_GYRO_DATA_Z1          = 0x29,
    UB0_REG_GYRO_DATA_Z0          = 0x2A,
    UB0_REG_TMST_FSYNCH           = 0x2B,
    UB0_REG_TMST_FSYNCL           = 0x2C,
    UB0_REG_INT_STATUS            = 0x2D,
    UB0_REG_FIFO_COUNTH           = 0x2E,
    UB0_REG_FIFO_COUNTL           = 0x2F,
    UB0_REG_FIFO_DATA             = 0x30,
    UB0_REG_APEX_DATA0            = 0x31,
    UB0_REG_APEX_DATA1            = 0x32,
    UB0_REG_APEX_DATA2            = 0x33,
    UB0_REG_APEX_DATA3            = 0x34,
    UB0_REG_APEX_DATA4            = 0x35,
    UB0_REG_APEX_DATA5            = 0x36,
    UB0_REG_INT_STATUS2           = 0x37,
    UB0_REG_INT_STATUS3           = 0x38,
    // break
    UB0_REG_SIGNAL_PATH_RESET      = 0x4B,
    UB0_REG_INTF_CONFIG0           = 0x4C,
    UB0_REG_INTF_CONFIG1           = 0x4D,
    UB0_REG_PWR_MGMT0              = 0x4E,
    UB0_REG_GYRO_CONFIG0           = 0x4F,
    UB0_REG_ACCEL_CONFIG0          = 0x50,
    UB0_REG_GYRO_CONFIG1           = 0x51,
    UB0_REG_GYRO_ACCEL_CONFIG0     = 0x52,
    UB0_REG_ACCEFL_CONFIG1         = 0x53,
    UB0_REG_TMST_CONFIG            = 0x54,
    // break
    UB0_REG_APEX_CONFIG0         = 0x56,
    UB0_REG_SMD_CONFIG           = 0x57,
    // break        
    UB0_REG_FIFO_CONFIG1         = 0x5F,
    UB0_REG_FIFO_CONFIG2         = 0x60,
    UB0_REG_FIFO_CONFIG3         = 0x61,
    UB0_REG_FSYNC_CONFIG         = 0x62,
    UB0_REG_INT_CONFIG0          = 0x63,
    UB0_REG_INT_CONFIG1          = 0x64,
    UB0_REG_INT_SOURCE0          = 0x65,
    UB0_REG_INT_SOURCE1          = 0x66,
    // break
    UB0_REG_INT_SOURCE3         = 0x68,
    UB0_REG_INT_SOURCE4         = 0x69,
    // break
    UB0_REG_FIFO_LOST_PKT0         = 0x6C,
    UB0_REG_FIFO_LOST_PKT1         = 0x6D,
    // break
    UB0_REG_SELF_TEST_CONFIG     = 0x70,
    // break
    UB0_REG_WHO_AM_I             = 0x75,
    
    // User Bank 1
    UB1_REG_SENSOR_CONFIG0         = 0x03,
    // break
    UB1_REG_GYRO_CONFIG_STATIC2  = 0x0B,
    UB1_REG_GYRO_CONFIG_STATIC3  = 0x0C,
    UB1_REG_GYRO_CONFIG_STATIC4  = 0x0D,
    UB1_REG_GYRO_CONFIG_STATIC5  = 0x0E,
    UB1_REG_GYRO_CONFIG_STATIC6  = 0x0F,
    UB1_REG_GYRO_CONFIG_STATIC7  = 0x10,
    UB1_REG_GYRO_CONFIG_STATIC8  = 0x11,
    UB1_REG_GYRO_CONFIG_STATIC9  = 0x12,
    UB1_REG_GYRO_CONFIG_STATIC10 = 0x13,
    // break
    UB1_REG_XG_ST_DATA             = 0x5F,
    UB1_REG_YG_ST_DATA             = 0x60,
    UB1_REG_ZG_ST_DATA             = 0x61,
    UB1_REG_TMSTVAL0               = 0x62,
    UB1_REG_TMSTVAL1               = 0x63,
    UB1_REG_TMSTVAL2               = 0x64,
    // break
    UB1_REG_INTF_CONFIG4         = 0x7A,
    UB1_REG_INTF_CONFIG5         = 0x7B,
    UB1_REG_INTF_CONFIG6         = 0x7C,
    
    // User Bank 2
    UB2_REG_ACCEL_CONFIG_STATIC2 = 0x03,
    UB2_REG_ACCEL_CONFIG_STATIC3 = 0x04,
    UB2_REG_ACCEL_CONFIG_STATIC4 = 0x05,
    // break
    UB2_REG_XA_ST_DATA             = 0x3B,
    UB2_REG_YA_ST_DATA             = 0x3C,
    UB2_REG_ZA_ST_DATA             = 0x3D,
    
    // User Bank 4
    UB4_REG_APEX_CONFIG1         = 0x40,
    UB4_REG_APEX_CONFIG2         = 0x41,
    UB4_REG_APEX_CONFIG3         = 0x42,
    UB4_REG_APEX_CONFIG4         = 0x43,
    UB4_REG_APEX_CONFIG5         = 0x44,
    UB4_REG_APEX_CONFIG6         = 0x45,
    UB4_REG_APEX_CONFIG7         = 0x46,
    UB4_REG_APEX_CONFIG8         = 0x47,
    UB4_REG_APEX_CONFIG9         = 0x48,
    // break
    UB4_REG_ACCEL_WOM_X_THR     = 0x4A,
    UB4_REG_ACCEL_WOM_Y_THR     = 0x4B,
    UB4_REG_ACCEL_WOM_Z_THR     = 0x4C,
    UB4_REG_INT_SOURCE6         = 0x4D,
    UB4_REG_INT_SOURCE7         = 0x4E,
    UB4_REG_INT_SOURCE8         = 0x4F,
    UB4_REG_INT_SOURCE9         = 0x50,
    UB4_REG_INT_SOURCE10        = 0x51,
    // break
    UB4_REG_OFFSET_USER0         = 0x77,
    UB4_REG_OFFSET_USER1         = 0x78,
    UB4_REG_OFFSET_USER2         = 0x79,
    UB4_REG_OFFSET_USER3         = 0x7A,
    UB4_REG_OFFSET_USER4         = 0x7B,
    UB4_REG_OFFSET_USER5         = 0x7C,
    UB4_REG_OFFSET_USER6         = 0x7D,
    UB4_REG_OFFSET_USER7         = 0x7E,
    UB4_REG_OFFSET_USER8         = 0x7F,

} ICM42688_REG;

/* ACCEL_CONFIG0 Settings */
#define ACCEL_FS_SEL_16G    (0x00 << 5) 
#define ACCEL_FS_SEL_8G     (0x01 << 5) 
#define ACCEL_FS_SEL_4G     (0x02 << 5) 
#define ACCEL_FS_SEL_2G     (0x03 << 5)

#define ACCEL_ODR_SEL_32KHZ (0x01)
#define ACCEL_ODR_SEL_16KHZ (0x02)
#define ACCEL_ODR_SEL_8KHZ  (0x03)
#define ACCEL_ODR_SEL_4KHZ  (0x04)
#define ACCEL_ODR_SEL_2KHZ  (0x05)
#define ACCEL_ODR_SEL_1KHZ  (0x06)  // default
#define ACCEL_ODR_SEL_200HZ (0x07)  
#define ACCEL_ODR_SEL_100HZ (0x08)  
#define ACCEL_ODR_SEL_50HZ  (0x09)  
#define ACCEL_ODR_SEL_25HZ  (0x0A)  
#define ACCEL_ODR_SEL_12HZ  (0x0B)  
#define ACCEL_ODR_SEL_6HZ   (0x0C)  
#define ACCEL_ODR_SEL_3HZ   (0x0D)  
#define ACCEL_ODR_SEL_1HZ   (0x0E)  
#define ACCEL_ODR_SEL_500HZ (0x0F)  

/* default ID */
#define ICM42688_DEFAULT_ID 0x47

/* GYRO_CONFIG0 Settings */
#define GYRO_FS_SEL_2000DPS (0x00 << 5)    //default
#define GYRO_FS_SEL_1000DPS (0x01 << 5)    //
#define GYRO_FS_SEL_500DPS  (0x02 << 5)
#define GYRO_FS_SEL_250DPS  (0x03 << 5)
#define GYRO_FS_SEL_125DPS  (0x04 << 5)
#define GYRO_FS_SEL_62DPS   (0x05 << 5)
#define GYRO_FS_SEL_31DPS   (0x06 << 5)
#define GYRO_FS_SEL_15DPS   (0x07 << 5)

#define GYRO_ODR_SEL_32KHZ  0x01
#define GYRO_ODR_SEL_16KHZ  0x02
#define GYRO_ODR_SEL_8KHZ   0x03
#define GYRO_ODR_SEL_4KHZ   0x04
#define GYRO_ODR_SEL_2KHZ   0x05
#define GYRO_ODR_SEL_1KHZ   0x06
#define GYRO_ODR_SEL_200HZ  0x07
#define GYRO_ODR_SEL_100HZ  0x08
#define GYRO_ODR_SEL_50HZ   0x09
#define GYRO_ODR_SEL_25HZ   0x0A
#define GYRO_ODR_SEL_12HZ   0x0B
#define GYRO_ODR_SEL_500HZ  0x0F


/* PWR_MGMT0 Settings */

#define PWR_MGMT0_ACCEL_OFF 0x00
#define PWR_MGMT0_ACCEL_ON  0x01
#define PWR_MGMT0_ACCEL_LP  0x02
#define PWR_MGMT0_ACCEL_LN  0x03

#define PWR_MGMT0_GYRO_OFF  (0x00 << 2)
#define PWR_MGMT0_GYRO_ON   (0x01 << 2)
#define PWR_MGMT0_GYRO_LN   (0x03 << 2)

#define PWR_MGMT0_IDLE      (0x01 << 4)

#define PWR_MGMT0_TEMP_DIS  (0x01 << 5)


/* INT config */
#define INT1_POLARITY_HIGH  0x01
#define INT1_DRIVE_PP       0x02
#define INT1_DRIVE_OD       0x00
#define INT1_MODE_LATCHED   0x04


#define INT2_POLARITY_HIGH  (INT1_POLARITY_HIGH << 3)
#define INT2_DRIVE_PP       (INT1_DRIVE_PP << 3)
#define INT2_DRIVE_OD       (INT1_DRIVE_OD << 3)
#define INT2_MODE_LATCHED   (INT1_MODE_LATCHED << 3)

/*G = 9.80665 */
#define LSB_ACC_16G         (9.80665f / 2048)
#define LSB_ACC_8G          (LSB_ACC_16G / 2)
#define LSB_ACC_4G          (LSB_ACC_16G / 4)
#define LSB_ACC_2G          (LSB_ACC_16G / 8)

/*Turn Into Radian*/
#define LSB_GYRO_15DPS      ( 1.0f / 2097.2f)
#define LSB_GYRO_31DPS      (LSB_GYRO_15DPS * 2)
#define LSB_GYRO_62DPS      (LSB_GYRO_15DPS * 4)
#define LSB_GYRO_125DPS     (LSB_GYRO_15DPS * 8)
#define LSB_GYRO_250DPS     (LSB_GYRO_15DPS * 16)
#define LSB_GYRO_500DPS     (LSB_GYRO_15DPS * 32)
#define LSB_GYRO_1000DPS    (LSB_GYRO_15DPS * 64)
#define LSB_GYRO_2000DPS    (LSB_GYRO_15DPS * 128)

#endif  // __ICM42688_REGISTERS_H__
