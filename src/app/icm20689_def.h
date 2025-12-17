#ifndef __ICM42688_REG_H__
#define __ICM42688_REG_H__

#include <stdint.h>


typedef enum ICM20689_REG
{
    REG_SELF_TEST_X_GYRO     = 0x00, //  R/W N XG_ST_DATA[7:0] 
    REG_SELF_TEST_Y_GYRO     = 0x01, //  R/W N YG_ST_DATA[7:0] 
    REG_SELF_TEST_Z_GYRO     = 0x02, //  R/W N ZG_ST_DATA[7:0] 
    REG_SELF_TEST_X_ACCEL    = 0x0D, //  R/W N XA_ST_DATA[7:0] 
    REG_SELF_TEST_Y_ACCEL    = 0x0E, //  R/W N YA_ST_DATA[7:0] 
    REG_SELF_TEST_Z_ACCEL    = 0x0F, //  R/W N ZA_ST_DATA[7:0] 
    REG_XG_OFFS_USRH         = 0x13, //  R/W N X_OFFS_USR [15:8] 
    REG_XG_OFFS_USRL         = 0x14, //  R/W N X_OFFS_USR [7:0] 
    REG_YG_OFFS_USRH         = 0x15, //  R/W N Y_OFFS_USR [15:8] 
    REG_YG_OFFS_USRL         = 0x16, //  R/W N Y_OFFS_USR [7:0] 
    REG_ZG_OFFS_USRH         = 0x17, //  R/W N Z_OFFS_USR [15:8] 
    REG_ZG_OFFS_USRL         = 0x18, //  R/W N Z_OFFS_USR [7:0] 
    REG_SMPLRT_DIV           = 0x19, //  R/W N SMPLRT_DIV[7:0] 
    REG_CONFIG               = 0x1A, //  R/W N - FIFO_ MODE EXT_SYNC_SET[2:0] DLPF_CFG[2:0] 
    REG_GYRO_CONFIG          = 0x1B, //  R/W N XG_ST YG_ST ZG_ST FS_SEL [1:0] - FCHOICE_B[1:0] 
    REG_ACCEL_CONFIG         = 0x1C, //  R/W N XA_ST YA_ST ZA_ST ACCEL_FS_SEL[1:0] - 
    REG_ACCEL_CONFIG_2       = 0x1D, //  R/W N FIFO_SIZE DEC2_CFG ACCEL_FCHOI CE_B A_DLPF_CFG 
    REG_LP_MODE_CFG          = 0x1E, //  R/W N GYRO_CYCL E G_AVGCFG[2:0] - 
    REG_ACCEL_WOM_X_THR      = 0x20, //  R/W N WOM_X_TH[7:0] 
    REG_ACCEL_WOM_Y_THR      = 0x21, //  R/W N WOM_Y_TH[7:0] 
    REG_ACCEL_WOM_Z_THR      = 0x22, //  R/W N WOM_Z_TH[7:0] 
    REG_FIFO_EN              = 0x23, //  R/W N TEMP _FIFO_EN XG_FIFO_EN YG_FIFO_EN ZG_FIFO_EN ACCEL_FIFO_ EN - - - 
    REG_FSYNC_INT            = 0x36, //  R/C N FSYNC_INT - - - - - - - 
    REG_INT_PIN_CFG          = 0x37, //  R/W Y INT_LEVEL INT_OPEN LATCH _INT_EN INT_RD _CLEAR FSYNC_INT_L EVEL FSYNC _INT_MODE_EN - - 
    REG_INT_ENABLE           = 0x38, //  R/W Y WOM_INT_EN[7:5] FIFO _OFLOW _EN - GDRIVE_INT_ EN DMP_INT_EN DATA_RDY_I NT_EN 
    REG_DMP_INT_STATUS       = 0x39, //  R/C N - DMP_INT[5:0] 
    REG_INT_STATUS           = 0x3A, //  R/C N WOM_INT[7:5] FIFO _OFLOW _INT - GDRIVE_INT DMP_INT DATA _RDY_INT 
    REG_ACCEL_XOUT_H         = 0x3B, //  R N ACCEL_XOUT_H[15:8] 
    REG_ACCEL_XOUT_L         = 0x3C, //  R N ACCEL_XOUT_L[7:0] 
    REG_ACCEL_YOUT_H         = 0x3D, //  R N ACCEL_YOUT_H[15:8] 
    REG_ACCEL_YOUT_L         = 0x3E, //  R N ACCEL_YOUT_L[7:0] 
    REG_ACCEL_ZOUT_H         = 0x3F, //  R N ACCEL_ZOUT_H[15:8] 
    REG_ACCEL_ZOUT_L         = 0x40, //  R N ACCEL_ZOUT_L[7:0] 
    REG_TEMP_OUT_H           = 0x41, //  R N TEMP_OUT[15:8] 
    REG_TEMP_OUT_L           = 0x42, //  R N TEMP_OUT[7:0] 
    REG_GYRO_XOUT_H          = 0x43, //  R N GYRO_XOUT[15:8] 
    REG_GYRO_XOUT_L          = 0x44, //  R N GYRO_XOUT[7:0] 
    REG_GYRO_YOUT_H          = 0x45, //  R N GYRO_YOUT[15:8] 
    REG_GYRO_YOUT_L          = 0x46, //  R N GYRO_YOUT[7:0] 
    REG_GYRO_ZOUT_H          = 0x47, //  R N GYRO_ZOUT[15:8] 
    REG_GYRO_ZOUT_L          = 0x48, //  R N GYRO_ZOUT[7:0] 
    REG_SIGNAL_PATH_RESET    = 0x68, // R/W N - - - - - - ACCEL 
    REG_ACCEL_INTEL_CTRL     = 0x69, // R/W N ACCEL_INTE
    REG_USER_CTRL            = 0x6A, // R/W N DMP_EN FIFO_EN - I2C_IF 
    REG_PWR_MGMT_1           = 0x6B, // R/W Y DEVICE_RES
    REG_PWR_MGMT_2           = 0x6C, // R/W Y FIFO_LP_EN DMP_LP_DIS STBY_XA STBY_YA STBY_ZA STBY_XG STBY_YG STBY_ZG 
    REG_FIFO_COUNTH          = 0x72, // R N - FIFO_COUNT[12:8] 
    REG_FIFO_COUNTL          = 0x73, // R N FIFO_COUNT[7:0] 
    REG_FIFO_R_W             = 0x74, // R/W N FIFO_DATA[7:0] 
    REG_WHO_AM_I             = 0x75, // R N WHOAMI[7:0] 
    REG_XA_OFFSET_H          = 0x77, // R/W N XA_OFFS [14:7] 
    REG_XA_OFFSET_L          = 0x78, // R/W N XA_OFFS [6:0] - 
    REG_YA_OFFSET_H          = 0x7A, // R/W N YA_OFFS [14:7] 
    REG_YA_OFFSET_L          = 0x7B, // R/W N YA_OFFS [6:0] - 
    REG_ZA_OFFSET_H          = 0x7D, // R/W N ZA_OFFS [14:7] 
    REG_ZA_OFFSET_L          = 0x7E, // R/W N ZA_OFFS [6:0] - 

} ICM20689_REG;

#define PWR_MGMT_1_CLK_SEL_PLL  0x01

#define WHO_I_AM_DEFAULT        0x98

#define ACCEL_DLPF_218HZ        0x01
#define ACCEL_DLPF_99HZ         0x02
#define ACCEL_DLPF_45HZ         0x03
#define ACCEL_DLPF_21HZ         0x04
#define ACCEL_DLPF_10HZ         0x05
#define ACCEL_DLPF_5HZ          0x06
#define ACCEL_DLPF_420HZ        0x07
#define ACCEL_FCHOICE_B         0x08

#define GYRO_DLPF_250HZ         0x00
#define GYRO_DLPF_176HZ         0x01
#define GYRO_DLPF_92HZ          0x02
#define GYRO_DLPF_41HZ          0x03
#define GYRO_DLPF_20HZ          0x04
#define GYRO_DLPF_10HZ          0x05
#define GYRO_DLPF_5HZ           0x06


#define GYRO_RANGE_250DPS   (0x00 << 3)     // FS_SEL[1:0] 00
#define GYRO_RANGE_500DPS   (0x01 << 3)     // FS_SEL[1:0] 01
#define GYRO_RANGE_1000DPS  (0x02 << 3)     // FS_SEL[1:0] 10
#define GYRO_RANGE_2000DPS  (0x03 << 3)     // FS_SEL[1:0] 11

#define ACCEL_RANGE_2G      (0x00 << 3)     // ACCEL_FS_SEL[1:0] 00
#define ACCEL_RANGE_4G      (0x01 << 3)     // ACCEL_FS_SEL[1:0] 01
#define ACCEL_RANGE_8G      (0x02 << 3)     // ACCEL_FS_SEL[1:0] 10
#define ACCEL_RANGE_16G     (0x03 << 3)     // ACCEL_FS_SEL[1:0] 11

/*G = 9.80665 */
#define LSB_ACC_16G         (9.80665f / 2048)
#define LSB_ACC_8G          (LSB_ACC_16G / 2)
#define LSB_ACC_4G          (LSB_ACC_16G / 4)
#define LSB_ACC_2G          (LSB_ACC_16G / 8)

/*Turn Into Radian*/
#define LSB_GYRO_250DPS     (1.0f / 131.0f)
#define LSB_GYRO_500DPS     (LSB_GYRO_250DPS * 2)
#define LSB_GYRO_1000DPS    (LSB_GYRO_250DPS * 4)
#define LSB_GYRO_2000DPS    (LSB_GYRO_250DPS * 8)

#endif  // __ICM42688_REGISTERS_H__
