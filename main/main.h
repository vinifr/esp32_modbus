//#include "packet.h"
#include "driver/i2c.h"

#define ADC_ADDR            0x6A

#define DEFAULT_CONFIG      0x90
#define CHANNEL_1           0x9F
#define CHANNEL_2           0xBF
#define CHANNEL_3           0xDF
#define CHANNEL_4           0xFF
#define CON_MODE            0xFF
#define ONE_SHOE_MODE       0xEF
#define SR_12BITS           0xF3
#define SR_14BITS           0xF7
#define SR_16BITS           0xFB
#define SR_18BITS           0xFF
#define PGA_1              0xFC
#define PGA_2              0xFD
#define PGA_4              0xFE
#define PGA_8              0xFF

#define CHANNEL_1_INDEX     0
#define CHANNEL_2_INDEX     1
#define CHANNEL_3_INDEX     2
#define CHANNEL_4_INDEX     3

#define REF_PT_1            1.024
#define REF_PT_1_CNT        (unsigned int)13391
#define REF_PT_2_CNT        (unsigned int)53573
#define REF_PT_2            4.096
#define REF_PT_DIFF         (ref_pt_2 - ref_pt_1)

#define ESP32_MIN_CONFIG
//#define ESP32_MAX_CONFIG

// RS485 pins
#define RS485_TX            22
#define RS485_RX            34
#define RS485_DE            19

#define RS485UART           UART_NUM_2

// Inputs
#define GPIO_VBAT           4
#define GPIO_DIN1           5
#define GPIO_DIN2           16
#define GPIO_DIN3           23
#define GPIO_DIN4           25
#define GPIO_MODEM_PWR_EN   12

// Outputs
#define GPIO_SENSOR_PWR     32
#define GPIO_SENMOD_PWR     27  // sensor/modem power enable
#define GPIO_OUTPUT1        0
#define GPIO_OUTPUT2        2
#define GPIO_MODEM_DTR      5
#define GPIO_MODEM_RTS      18
#define GPIO_MODEM_PWR      190
//#define GPIO_MODEM_RST      22  // modem reset
#define GPIO_MODEM_CTS      23
#define GPIO_OUTPUT_SEL(x)  ((1ULL << x))

#define I2C_EXAMPLE_MASTER_SCL_IO          25               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          26               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

typedef float sensor_volt;

//
//#define TEST
//#define HOME

void init_ota(void);
void init_wifi(void);
void initSDCARD(void);
void init_ethernet(void);

void i2c_example_master_init();
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t slave_addr,
                                        uint8_t* data_rd, size_t size);
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t slave_addr,
                                         uint8_t* data_wr, size_t size);
int iReadADC(uint8_t tuChan, char debug);
sensor_volt getCalibVolt(uint8_t chan_no, char debug);
