#ifndef _ESP_IDF_WSEN_PADS_REGISTERS_H_
#define _ESP_IDF_WSEN_PADS_REGISTERS_H_

/* Register address list. */
#define WSEN_PADS_INT_CFG           0x0B /**< (RW) Interrupt configuration register. */
#define WSEN_PADS_THR_P_L           0x0C /**< (RW) Threshold pressure register. (low byte) */
#define WSEN_PADS_THR_P_H           0x0D /**< (RW) Threshold pressure register. (high byte) */
#define WSEN_PADS_INTERFACE_CTRL    0x0E /**< (RW) Interface control register. */
#define WSEN_PADS_DEVICE_ID         0x0F /**< (RO) Device ID register. */
#define WSEN_PADS_CTRL_1            0x10 /**< (RW) Control register 1. */
#define WSEN_PADS_CTRL_2            0x11 /**< (RW) Control register 2. */
#define WSEN_PADS_CTRL_3            0x12 /**< (RW) Control register 3. */
#define WSEN_PADS_FIFO_CTRL         0x13 /**< (RW) FIFO control register. */
#define WSEN_PADS_FIFO_WTM          0x14 /**< (RW) FIFO threshold level. */
#define WSEN_PADS_REF_P_L           0x15 /**< (RO) Reference pressure register. (low byte) */
#define WSEN_PADS_REF_P_H           0x16 /**< (RO) Reference pressure register. (high byte) */
#define WSEN_PADS_OPC_P_L           0x18 /**< (RW) Pressure offset register. (low byte) */
#define WSEN_PADS_OPC_P_H           0x19 /**< (RW) Pressure offset register. (high byte) */
#define WSEN_PADS_INT_SRC           0x24 /**< (RO) Interrupt source register. */
#define WSEN_PADS_FIFO_STATUS_1     0x25 /**< (RO) FIFO fill level.  */
#define WSEN_PADS_FIFO_STATUS_2     0x26 /**< (RO) FIFO status register. */
#define WSEN_PADS_STATUS            0x27 /**< (RO) Status register. */
#define WSEN_PADS_DATA_P_XL         0x28 /**< (RO) Pressure data register. (lowest byte) */
#define WSEN_PADS_DATA_P_L          0x29 /**< (RO) Pressure data register. (low byte) */
#define WSEN_PADS_DATA_P_H          0x30 /**< (RO) Pressure data register. (high byte) */
#define WSEN_PADS_DATA_T_L          0x2B /**< (RO) Temperature data register. (low byte) */
#define WSEN_PADS_DATA_T_H          0x2C /**< (RO) Temperature data register. (high byte) */
#define WSEN_PADS_FIFO_DATA_P_XL    0x78 /**< (RO) Pressure FIFO register. (lowest byte) */
#define WSEN_PADS_FIFO_DATA_P_L     0x79 /**< (RO) Pressure FIFO register. (low byte) */
#define WSEN_PADS_FIFO_DATA_P_H     0x7A /**< (RO) Pressure FIFO register. (high byte) */
#define WSEN_PADS_FIFO_DATA_T_L     0x7B /**< (RO) Temperature FIFO register. (low byte) */
#define WSEN_PADS_FIFO_DATA_T_H     0x7C /**< (RO) Temperature FIFO register. (high byte) */

/* INT_CFG bits. */
#define WSEN_PADS_INT_CFG_AUTOREFP  (1 << 7) /**< Enable AUTOREFP function. */
#define WSEN_PADS_INT_CFG_RESET_ARP (1 << 6) /**< Reset AUTOREFP function. */
#define WSEN_PADS_INT_CFG_AUTOZERO  (1 << 5) /**< Enable AUTOZERO function. */
#define WSEN_PADS_INT_CFG_RESET_AZ  (1 << 4) /**< Reset AUTOZERO function. */
#define WSEN_PADS_INT_CFG_DIFF_EN   (1 << 3) /**< Enable differential pressure interrupt. */
#define WSEN_PADS_INT_CFG_LIR       (1 << 2) /**< Latch interrupt request signal. */
#define WSEN_PADS_INT_CFG_PLE       (1 << 1) /**< Enable pressure lower than threshold interrupt. */
#define WSEN_PADS_INT_CFG_PHE       (1 << 0) /**< Enable pressure higher than threshold interrupt. */

/* INTERFACE_CTRL bits. */
#define WSEN_PADS_INTERFACE_CTRL_SDA_PU_EN  (1 << 4) /**< Enable pull up on SDA pin. */
#define WSEN_PADS_INTERFACE_CTRL_SAO_PU_EN  (1 << 3) /**< Enable pull up on SAO pin. */
#define WSEN_PADS_INTERFACE_CTRL_PD_DIS_INT (1 << 2) /**< Disable pull down on INT pin. */
#define WSEN_PADS_INTERACE_CTRL_I2C_DISABLE (1 << 0) /**< Disable I2C interface. */

/* Device ID value for WSEN-PADS. */
#define WSEN_PADS_DEVICE_ID_VALUE 0xB3

/* CTRL_1 bits. */
#define WSEN_PADS_CTRL_1_ODR_POS    4        /**< Position of ODR bits. */
#define WSEN_PADS_CTRL_1_ODR_MASK   (7 << 4) /**< Mask for all ODR bits. */
#define WSEN_PADS_CTRL_1_ODR_OFF    (0 << 4) /**< Power down / single conversion mode. */
#define WSEN_PADS_CTRL_1_ODR_1HZ    (1 << 4) /**< 1Hz data rate. */
#define WSEN_PADS_CTRL_1_ODR_10HZ   (2 << 4) /**< 10Hz data rate. */
#define WSEN_PADS_CTRL_1_ODR_25HZ   (3 << 4) /**< 25Hz data rate. */
#define WSEN_PADS_CTRL_1_ODR_50HZ   (4 << 4) /**< 50Hz data rate. */
#define WSEN_PADS_CTRL_1_ODR_75HZ   (5 << 4) /**< 75Hz data rate. */
#define WSEN_PADS_CTRL_1_ODR_100HZ  (6 << 4) /**< 100Hz data rate. */
#define WSEN_PADS_CTRL_1_ODR_200HZ  (7 << 4) /**< 200Hz data rate. */
#define WSEN_PADS_CTRL_1_EN_LPFP    (1 << 3) /**< Enable low pass filter. */
#define WSEN_PADS_CTRL_1_LPFP_CFG   (1 << 2) /**< Configure low pass filter. */
#define WSEN_PADS_CTRL_1_BDU        (1 << 1) /**< Enable block data update. */
#define WSEN_PADS_CTRL_1_SIM        (1 << 0) /**< SPI 3-wire mode. */

/* CTRL_2 bits. */
#define WSEN_PADS_CTRL_2_BOOT           (1 << 7)    /**< Reload calibration content. */
#define WSEN_PADS_CTRL_2_INT_H_L        (1 << 6)    /**< INT pin active high / active low */
#define WSEN_PADS_CTRL_2_PP_OD          (1 << 5)    /**< INT pin push pull / open-drain */
#define WSEN_PADS_CTRL_2_IF_ADD_INC     (1 << 4)    /**< Auto increment address pointer */
#define WSEN_PADS_CTRL_2_SWRESET        (1 << 2)    /**< Reset configuration registers. */
#define WSEN_PADS_CTRL_2_LOW_NOISE_EN   (1 << 1)    /**< Enable low noise mode. */
#define WSEN_PADS_CTRL_2_ONE_SHOT       (1 << 0)    /**< Make one pressure and tempreature measurement. */

/* CTRL_3 bits. */
#define WSEN_PADS_CTRL_3_INT_F_FULL         (1 << 5)    /**< Enable FIFO full interrupt. */
#define WSEN_PADS_CTRL_3_INT_F_WTM          (1 << 4)    /**< Enable FIFO threshold interrupt. */
#define WSEN_PASD_CTRL_3_INT_F_OVR          (1 << 3)    /**< Enable FIFO overwrite interrupt. */
#define WSEN_PADS_CTRL_3_DRDY               (1 << 2)    /**< Enable data ready interrupt. */
#define WSEN_PADS_CTRL_S_POS                0           /**< Position of interrupt mux select bits. */
#define WSEN_PADS_CTRL_3_INT_S_MASK         (3 << 0)    /**< Mask for interrupt mux select bits. */
#define WSEN_PADS_CTRL_3_INT_S_DATA         (0 << 0)    /**< Interrupt mux select DRDY, INT_F_WTM, INT_F_OVER, INT_F_FULL (in priority order) */
#define WSEN_PADS_CTRL_3_INT_S_HIGH         (1 << 0)    /**< Interrupt mux select pressure high event. */
#define WSEN_PADS_CTRL_3_INT_S_LOW          (2 << 0)    /**< Interrupt mux select pressure low event. */
#define WSEN_PADS_CTRL_3_INT_S_HIGH_LOW     (3 << 0)    /**< Interrupt mux select pressure high and low event. */

/* FIFO_CTRL bits. */
#define WSEN_PADS_FIFO_CTRL_STOP_ON_WTM (1 << 3)    /**< Consider FIFO full when threshold reached. */
#define WSEN_PADS_FIFO_CTRL_TRIG_MODES  (1 << 2)    /**< Enabled triggered FIFO. */
#define WSEN_PADS_F_MODE_POS            0           /**< Position of FIFO mode select bits. */
#define WSEN_PASS_F_MODE_MASK           (3 << 0)    /**< Mask for FIFO mode select bits. */

/* INT_SOURCE bits. */
#define WSEN_PADS_INT_SRC_BOOT_ON   (1 << 7)    /**< Indicates the boot process is incomplete. */
#define WSEN_PADS_INT_SRC_IA        (1 << 2)    /**< Indicates an interrupt is active. */
#define WSEN_PADS_INT_SRC_PL        (1 << 1)    /**< Indicates a pressure low interrupt. */
#define WSEN_PADS_INT_SRC_PH        (1 << 0)    /**< Indicates a pressure high interrupt. */

#define WSEN_PADS_FIFO_STATUS_1_MAX_VALUE 128   /**< Maximum number of samples the FIFO can hold. */

/* FIFO_STAUS_2 bits.*/
#define WSEN_PADS_FIFO_STATUS_2_FIFO_WTM_IA     (1 << 7)    /**< Indicates a FIFO threshold interrupt. */
#define WSEN_PADS_FIFO_STATUS_2_FIFO_OVER_IA    (1 << 6)    /**< Indicates a FIFO overwritten interrupt. */
#define WSEN_PADS_FIFO_STATUS_2_FIFO_FULL_IA    (1 << 5)    /**< Indicates a FIFO full interrupt. */

/* STATUS bits. */
#define WSEN_PADS_STATUS_T_OR   (1 << 5)    /**< Indicates temperature data overrun. */
#define WSEN_PADS_STATUS_P_OR   (1 << 4)    /**< Indicates pressure data overrun. */
#define WSEN_PADS_STATUS_T_DA   (1 << 1)    /**< Indicates temperature data is ready. */
#define WSEN_PADS_STATUS_P_DA   (1 << 0)    /**< Indicates pressure data is ready. */

#endif
