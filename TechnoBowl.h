#ifndef _TECHNOBOWL_H_
#define _TECHNOBOWL_H_
#include<system.h>

/************************************************************
* Header file created by Team Techno Bowl 2009
John Garro, Zach Mady, Joseph Bonath, Wenjie Jia, and Jon Thibeault 

*************************************************************/

//CCP4, CCP5 and TMR1 register variables
unsigned short ccp1_reg@CCPR1L; // CCP1 H & L registers now used as a single register
unsigned short ccp2_reg@CCPR2L; // CCP2 H & L registers now used as a single register
unsigned short ccp3_reg@CCPR3L; // CCP3 H & L registers now used as a single register
unsigned short ccp4_reg@CCPR4L; // CCP4 H & L registers now used as a single register
unsigned short ccp5_reg@CCPR5L; // CCP5 H & L registers now used as a single register
unsigned short tmr1_reg@TMR1L; // Timer1 H & L now used as a single register

//TMR1, CCP4 & CCP5 Interrupt Flag registers
volatile bit tmr1if@PIR1.0; // tmr1if = timer1 overflow interrupt flag
volatile bit comp1if@PIR1.2; // comp1if = compare interrupt flag
volatile bit comp2if@PIR2.0; // comp2if = compare interrupt flag
volatile bit comp3if@PIR3.0; // comp3if = compare interrupt flag
volatile bit comp4if@PIR3.1; // comp4if = compare interrupt flag
volatile bit comp5if@PIR3.2; // comp5if = compare interrupt flag

/*************************************************************************************************************
Function protoype declarations
-Servo/HBridge Functions
-Serial Comm Functions
-Zigbee Functions
-Accelerometer Functions

**************************************************************************************************************/

//Function prototypes for servo and hbridge functionality
void ccp_gen_init(void); //initialize interrupts
void tmr_init(void);     //initialize Timer1 and Timer2 for CCP use
void cmp1_init(void);   //initialize CCP4 for servo/compare
void cmp2_init(void);   //initialize CCP4 for servo/compare
void cmp3_init(void);   //initialize CCP4 for servo/compare
void cmp4_init(void);   //initialize CCP4 for servo/compare
void cmp5_init(void);   //initialize CCP5 for servo/compare
void pwm4_init(char hbridge); //initialize CCP4 for PWM
void pwm5_init(void);   //initialize CCP5 for PWM
unsigned short get_servo_val (unsigned short servo_num); //gets servo value used to create pulse boundary values
unsigned short get_hbridge_val (unsigned short hbridge_num); //gets hbridge values to determine duty cycle
unsigned short servo_upper(unsigned short servo); //calculates upper bounday servo value
unsigned short servo_lower(unsigned short servo); //calculates lower bounday servo value
unsigned short servo_width(unsigned short servo);


//Function prototypes for Serial(Hyperterminal) communication
void serial_init(void);  //initialize Serial communicaiton
void putc(char value);  //transmit character to Hyperterminal
char getc(void);		//retrieve character from Hyperterminal
void prints(const char* text);  //print static string to Hyperterminal
void print_dec(unsigned short dat);  //print decimal number to Hyperterminal
unsigned short convert(void);  //take character from Hyperterminal and convert to a number
char igetc(void);		//retrieve character from Hyperterminal
void iprints(const char* text);  //print static string to Hyperterminal
void iprint_dec(unsigned short dat); 

// Function protoypes for Zigbee
void spi_init(void); // Initialize the SPI interface
void spi_put_data(char data);  // read send data via SPI
char spi_get_data(void);  // receive data from SPI
char trx_reg_read(char addr); // read a register on ZigBee (uses SPI)
void trx_reg_write(char addr, char val);  // write to a register on ZigBee (uses SPI)
char trx_bit_read(char addr, char mask, char pos);  // read a bit on ZigBee (uses SPI)
void trx_bit_write(char addr, char mask, char pos, char val);  // write a bit on ZigBee (uses SPI)
void trx_frame_write(char length, char *frame);
char trx_frame_read(char  *frame);
void transceiver_pll_on(void);
void transceiver_reset(void);
void print_array(char *array, char L);
void transceiver_rx_aack_on(void);
void transceiver_rx_on(void);
void store_data(void);
char read_int(void);

//Function prototypes for Accelerometer
void spi2_write(char reg, char data);
char spi2_read(char reg);
void acc_setup(void);
void spi2_write_int(char reg, char data);
void spi2_init(void);
char spi2_read_int(char reg);

//Function prototypes for general functions
void init(void);
void interrupts_enable(void);
void interrupts_disable(void);
void interrupts_enable_int(void);
void interrupts_disable_int(void);

//Function prototypes for EEPROM
void eeprom_init(void);
void eeprom_write(unsigned short reg, char eeprom_val);
char eeprom_read(unsigned short reg);

//Function prototypes for Scaling - created by ME Blue Squad 2009
unsigned short stickScaler(unsigned short dig, unsigned short scaled_last);
unsigned short digScaler(unsigned short dig, unsigned short integerVal, unsigned short decOne, unsigned short decTwo, unsigned short decThree );
void stepit1(unsigned short servo1_val);

//Function prototypes for Logic Analyzer Port testing
void port_test(void);

/********************************************************************************************************
Definition of variables

*********************************************************************************************************/
//debug settings
#define TRUE             1
#define FALSE            0

//Servo naming
#define Servo1           1
#define Servo2           2
#define Servo3           3
#define Servo4           4
#define Servo5           5

//PWM/H-Bridge naming
#define HBridge1           1
#define HBridge2           2
#define HBridge3           3
#define HBridge4           4
#define HBridge5           5


// General ZigBee Defines 

#define Z_CS 	latf.0
#define Z_INT	pir1.3  //correct?
#define Z_WC	ssp1con1.7


//#define TRX_PIN_SLP_TR  latc.7  // ZigBee Control Pin*
#define TRX_PIN_IRQ     latb.0  // ZigBee Interrupt Pin*
#define TRX_RESET       latf.1  // ZigBee Reset Pin*

// ZigBee Timing Definitions (use delay_us();)
#define tTR1 380 //Used when set to P_ON and waiting for CLKM (microseconds)
#define tTR2 240 //Used when transitioning from SLEEP -> TRX_OFF (microseconds)
#define tTR4 110 //Used when transitioning from TRX_OFF -> PLL_ON (microseconds)
#define tTR6 110 //Used when transitioning from TRX_OFF -> RX_ON (microseconds)
#define tTR10 16 //Used when transitioning from PLL_ON -> BUSY_TX (microseconds)
#define tTR11 32 //Used when going from BUSY_TX -> PLL_ON (microseconds)
#define tTR13 37 //Used when going from RESET -> TRX_OFF (microseconds)


//Zigbee Chip Register definitions
#define 	RG_ANT_DIV   (0x0d)
#define 	RG_BATMON   (0x11)
#define 	RG_CCA_THRES   (0x09)
#define 	RG_CSMA_BE   (0x2f)
#define 	RG_CSMA_SEED_0   (0x2d)
#define 	RG_CSMA_SEED_1   (0x2e)
#define 	RG_FTN_CTRL   (0x18)
#define 	RG_IEEE_ADDR_0   (0x24)
#define 	RG_IEEE_ADDR_1   (0x25)
#define 	RG_IEEE_ADDR_2   (0x26)
#define 	RG_IEEE_ADDR_3   (0x27)
#define 	RG_IEEE_ADDR_4   (0x28)
#define 	RG_IEEE_ADDR_5   (0x29)
#define 	RG_IEEE_ADDR_6   (0x2a)
#define 	RG_IEEE_ADDR_7   (0x2b)
#define 	RG_IRQ_MASK   (0x0e)
#define 	RG_IRQ_STATUS   (0x0f)
#define 	RG_MAN_ID_0   (0x1e)
#define 	RG_MAN_ID_1   (0x1f)
#define 	RG_PAN_ID_0   (0x22)
#define 	RG_PAN_ID_1   (0x23)
#define 	RG_PART_NUM   (0x1c)
#define 	RG_PHY_CC_CCA   (0x08)
#define 	RG_PHY_ED_LEVEL   (0x07)
#define 	RG_PHY_RSSI   (0x06)
#define 	RG_PHY_TX_PWR   (0x05)
#define 	RG_PLL_CF   (0x1a)
#define 	RG_PLL_DCU   (0x1b)
#define 	RG_RX_CTRL   (0x0a)
#define 	RG_RX_SYN   (0x15)
#define 	RG_SFD_VALUE   (0x0b)
#define 	RG_SHORT_ADDR_0   (0x20)
#define 	RG_SHORT_ADDR_1   (0x21)
#define 	RG_TRX_CTRL_0   (0x03)
#define 	RG_TRX_CTRL_1   (0x04)
#define 	RG_TRX_CTRL_2   (0x0c)
#define 	RG_TRX_STATE   (0x02)
#define 	RG_TRX_STATUS   (0x01)
#define 	RG_VERSION_NUM   (0x1d)
#define 	RG_VREG_CTRL   (0x10)
#define 	RG_XAH_CTRL_0   (0x2c)
#define 	RG_XAH_CTRL_1   (0x17)
#define 	RG_XOSC_CTRL   (0x12)

//Subregister definitions
#define 	SR_AACK_ACK_TIME   0x17, 0x04, 2
#define 	SR_AACK_DIS_ACK   0x2e, 0x10, 4
#define 	SR_AACK_FLTR_RES_FT   0x17, 0x20, 5
#define 	SR_AACK_FVN_MODE   0x2e, 0xc0, 6
#define 	SR_AACK_I_AM_COORD   0x2e, 0x08, 3
#define 	SR_AACK_PROM_MODE   0x17, 0x02, 1
#define 	SR_AACK_SET_PD   0x2e, 0x20, 5
#define 	SR_AACK_UPLD_RES_FT   0x17, 0x10, 4
#define 	SR_ANT_CTRL   0x0d, 0x03, 0
#define 	SR_ANT_DIV_EN   0x0d, 0x08, 3
#define 	SR_ANT_EXT_SW_EN   0x0d, 0x04, 2
#define 	SR_ANT_SEL   0x0d, 0x80, 7
#define 	SR_AVDD_OK   0x10, 0x40, 6
#define 	SR_AVREG_EXT   0x10, 0x80, 7
#define 	SR_BATMON_HR   0x11, 0x10, 4
#define 	SR_BATMON_OK   0x11, 0x20, 5
#define 	SR_BATMON_VTH   0x11, 0x0f, 0
#define 	SR_CCA_DONE   0x01, 0x80, 7
#define 	SR_CCA_ED_THRES   0x09, 0x0f, 0
#define 	SR_CCA_MODE   0x08, 0x60, 5
#define 	SR_CCA_REQUEST   0x08, 0x80, 7
#define 	SR_CCA_STATUS   0x01, 0x40, 6
#define 	SR_CHANNEL   0x08, 0x1f, 0
#define 	SR_CLKM_CTRL   0x03, 0x07, 0
#define 	SR_CLKM_SHA_SEL   0x03, 0x08, 3
#define 	SR_CSMA_SEED_0   0x2d, 0xff, 0
#define 	SR_CSMA_SEED_1   0x2e, 0x07, 0
#define 	SR_DVDD_OK   0x10, 0x04, 2
#define 	SR_DVREG_EXT   0x10, 0x08, 3
#define 	SR_ED_LEVEL   0x07, 0xff, 0
#define 	SR_FTN_START   0x18, 0x80, 7
#define 	SR_IEEE_ADDR_0   0x24, 0xff, 0
#define 	SR_IEEE_ADDR_1   0x25, 0xff, 0
#define 	SR_IEEE_ADDR_2   0x26, 0xff, 0
#define 	SR_IEEE_ADDR_3   0x27, 0xff, 0
#define 	SR_IEEE_ADDR_4   0x28, 0xff, 0
#define 	SR_IEEE_ADDR_5   0x29, 0xff, 0
#define 	SR_IEEE_ADDR_6   0x2a, 0xff, 0
#define 	SR_IEEE_ADDR_7   0x2b, 0xff, 0
#define 	SR_IRQ_0_PLL_LOCK   0x0f, 0x01, 0
#define 	SR_IRQ_1_PLL_UNLOCK   0x0f, 0x02, 1
#define 	SR_IRQ_2_EXT_EN   0x04, 0x40, 6
#define 	SR_IRQ_2_RX_START   0x0f, 0x04, 2
#define 	SR_IRQ_3_TRX_END   0x0f, 0x08, 3
#define 	SR_IRQ_4_CCA_ED_READY   0x0f, 0x10, 4
#define 	SR_IRQ_5_AMI   0x0f, 0x20, 5
#define 	SR_IRQ_6_TRX_UR   0x0f, 0x40, 6
#define 	SR_IRQ_7_BAT_LOW   0x0f, 0x80, 7
#define 	SR_IRQ_MASK   0x0e, 0xff, 0
#define 	SR_IRQ_MASK_MODE   0x04, 0x02, 1
#define 	SR_IRQ_POLARITY   0x04, 0x01, 0
#define 	SR_MAN_ID_0   0x1e, 0xff, 0
#define 	SR_MAN_ID_1   0x1f, 0xff, 0
#define 	SR_MAX_BE   0x2f, 0xf0, 4
#define 	SR_MAX_CSMA_RETRIES   0x2c, 0x0e, 1
#define 	SR_MAX_FRAME_RETRIES   0x2c, 0xf0, 4
#define 	SR_MIN_BE   0x2f, 0x0f, 0
#define 	SR_OQPSK_DATA_RATE   0x0c, 0x03, 0
#define 	SR_PA_BUF_LT   0x05, 0xc0, 6
#define 	SR_PA_EXT_EN   0x04, 0x80, 7
#define 	SR_PA_LT   0x05, 0x30, 4
#define 	SR_PAD_IO   0x03, 0xc0, 6
#define 	SR_PAD_IO_CLKM   0x03, 0x30, 4
#define 	SR_PAN_ID_0   0x22, 0xff, 0
#define 	SR_PAN_ID_1   0x23, 0xff, 0
#define 	SR_PART_NUM   0x1c, 0xff, 0
#define 	SR_PDT_THRES   0x0a, 0x0f, 0
#define 	SR_PLL_CF_START   0x1a, 0x80, 7
#define 	SR_PLL_DCU_START   0x1b, 0x80, 7
#define 	SR_RND_VALUE   0x06, 0x60, 5
#define 	SR_RSSI   0x06, 0x1f, 0
#define 	SR_RX_BL_CTRL   0x04, 0x10, 4
#define 	SR_RX_CRC_VALID   0x06, 0x80, 7
#define 	SR_RX_PDT_DIS   0x15, 0x80, 7
#define 	SR_RX_PDT_LEVEL   0x15, 0x0f, 0
#define 	SR_RX_SAFE_MODE   0x0c, 0x80, 7
#define 	SR_SFD_VALUE   0x0b, 0xff, 0
#define 	SR_SHORT_ADDR_0   0x20, 0xff, 0
#define 	SR_SHORT_ADDR_1   0x21, 0xff, 0
#define 	SR_SLOTTED_OPERATION   0x2c, 0x01, 0
#define 	SR_SPI_CMD_MODE   0x04, 0x0c, 2
#define 	SR_TRAC_STATUS   0x02, 0xe0, 5
#define 	SR_TRX_CMD   0x02, 0x1f, 0
#define 	SR_TRX_STATUS   0x01, 0x1f, 0
#define 	SR_TX_AUTO_CRC_ON   0x04, 0x20, 5
#define 	SR_TX_PWR   0x05, 0x0f, 0
#define 	SR_VERSION_NUM   0x1d, 0xff, 0
#define 	SR_XTAL_MODE   0x12, 0xf0, 4
#define 	SR_XTAL_TRIM   0x12, 0x0f, 0

//Constants
#define 	AACK_ACK_TIME_12_symbols   (0)
#define 	AACK_ACK_TIME_2_symbols   (1)
#define 	AES_CON   (0x83)
#define 	AES_DIR_DECRYPT   (0x08)
#define 	AES_DIR_ENCRYPT   (0)
#define 	AES_MODE_CBC   (0x20)
#define 	AES_MODE_ECB   (0x0)
#define 	AES_MODE_KEY   (0x10)
#define 	AES_REQUEST   (0x80)
#define 	AES_RY_DONE   (1)
#define 	AES_RY_NOT_DONE   (0)
#define 	AES_ST   (0x82)
#define 	AES_STATE_KEY   (0x84)
#define 	ALTRATE_1Mbps   (2)
#define 	ALTRATE_250kbps   (0)
#define 	ALTRATE_2Mbps   (3)
#define 	ALTRATE_500kbps   (1)
#define 	ANT_DIV_disable   (0)
#define 	ANT_DIV_enable   (1)
#define 	ANT_EXT_SW_switch_disable   (0)
#define 	ANT_EXT_SW_switch_enable   (1)
#define 	ANT_SEL_antenna_0   (0)
#define 	ANT_SEL_antenna_1   (1)
#define 	BATMON_not_valid   (0)
#define 	BATMON_valid   (1)
#define 	BUSY_RX   (1)
#define 	BUSY_RX_AACK   (17)
#define 	BUSY_RX_AACK_NOCLK   (30)
#define 	BUSY_TX   (2)
#define 	BUSY_TX_ARET   (18)
#define 	CCA_calculation_done   (1)
#define 	CCA_calculation_not_finished   (0)
#define 	CCA_Mode_1   (1)
#define 	CCA_Mode_2   (2)
#define 	CCA_Mode_3   (3)
#define 	CCA_STATUS_channel_is_busy   (0)
#define 	CCA_STATUS_channel_is_idle   (1)
#define 	CLKM_16MHz   (5)
#define 	CLKM_1_16MHz   (7)
#define 	CLKM_1_4MHz   (6)
#define 	CLKM_1MHz   (1)
#define 	CLKM_2MHz   (2)
#define 	CLKM_4MHz   (3)
#define 	CLKM_8MHz   (4)
#define 	CLKM_no_clock   (0)
#define 	CLKM_SHA_disable   (0)
#define 	CLKM_SHA_enable   (1)
#define 	CMD_FORCE_PLL_ON   (4)
#define 	CMD_FORCE_TRX_OFF   (3)
#define 	CMD_NOP   (0)
#define 	CMD_PLL_ON   (9)
#define 	CMD_RX_AACK_ON   (22)
#define 	CMD_RX_ON   (6)
#define 	CMD_TRX_OFF   (8)
#define 	CMD_TX_ARET_ON   (25)
#define 	CMD_TX_START   (2)
#define 	CRC16_not_valid   (0)
#define 	CRC16_valid   (1)
#define 	IRQ_MASK_MODE_off   (0)
#define 	IRQ_MASK_MODE_on   (1)
#define 	IRQ_POL_high_active_IRQ   (0)
#define 	IRQ_POL_low_active_IRQ   (1)
#define 	P_ON   (0)
#define 	PA_BUF_LT_0s   (0)
#define 	PA_BUF_LT_2s   (1)
#define 	PA_BUF_LT_4s   (2)
#define 	PA_BUF_LT_6s   (3)
#define 	PA_LT_2s   (0)
#define 	PA_LT_4s   (1)
#define 	PA_LT_6s   (2)
#define 	PA_LT_8s   (3)
#define 	PAD_CLKM_2mA   (0)
#define 	PAD_CLKM_4mA   (1)
#define 	PAD_CLKM_6mA   (2)
#define 	PAD_CLKM_8mA   (3)
#define 	PAD_IO_2mA   (0)
#define 	PAD_IO_4mA   (1)
#define 	PAD_IO_6mA   (2)
#define 	PAD_IO_8mA   (3)
#define 	PLL_ON   (9)
#define 	RF231   (3)
#define 	RF231_RAM_SIZE   (0x80)
#define 	RG_AES_CON_MIRROR   (0x94)
#define 	RSSI_BASE_VAL   (-91)
#define 	RX_AACK_ON   (22)
#define 	RX_AACK_ON_NOCLK   (29)
#define 	RX_ON   (6)
#define 	RX_ON_NOCLK   (28)
#define 	SLEEP   (15)
#define 	SPI_CMD_MODE_default   (0)
#define 	SPI_CMD_MODE_monitor_IRQ_STATUS   (3)
#define 	SPI_CMD_MODE_monitor_PHY_RSSI   (2)
#define 	SPI_CMD_MODE_monitor_TRX_STATUS   (1)
#define 	STATE_TRANSITION_IN_PROGRESS   (31)
#define 	T_OCT   32
#define 	T_SYM   16
#define 	TRAC_CHANNEL_ACCESS_FAILURE   (3)
#define 	TRAC_CHANNEL_ACCESS_FAILURE   (3)
#define 	TRAC_INVALID   (7)
#define 	TRAC_NO_ACK   (5)
#define 	TRAC_NO_ACK   (5)
#define 	TRAC_SUCCESS   (0)
#define 	TRAC_SUCCESS   (0)
#define 	TRAC_SUCCESS_DATA_PENDING   (1)
#define 	TRAC_SUCCESS_WAIT_FOR_ACK   (2)
#define 	TRX_AES_BLOCK_SIZE   (16)
#define 	TRX_IRQ_AMI   (0x20)
#define 	TRX_IRQ_AWAKE_END   (0x10)
#define 	TRX_IRQ_BAT_LOW   (0x80)
#define 	TRX_IRQ_CCA_ED_READY   (0x10)
#define 	TRX_IRQ_PLL_LOCK   (0x01)
#define 	TRX_IRQ_PLL_UNLOCK   (0x02)
#define 	TRX_IRQ_RX_START   (0x04)
#define 	TRX_IRQ_TRX_END   (0x08)
#define 	TRX_IRQ_TRX_UR   (0x40)
#define 	TRX_OFF   (8)
#define 	TX_ARET_ON   (25)


struct controller //define struct to hold data from controller
	{
	
	char seq; //sequencing byte (3)
	char joy1x; //joystick 1 horizontal position
	char joy1y; //joystick 1 vertical position
	char joy2x; //joystick 2 horizontal position
	char joy2y; //joystick 2 vertical position 
	char buttons; //buttons data
	
	};

//SPI2 Definitions
#define A_CS	  lata.4
#define SPI2_INT  pir3.7
#define A_WC	ssp2con1.7

//Set mode registers with appropriate modes at 8g sensitivity
#define FOUR_WIRE_SB	0x16, 0x80
#define FOUR_WIRE_MEAS	0x16, 0x81
#define FOUR_WIRE_LEVEL	0x16, 0x82 
#define FOUR_WIRE_PULSE 0x16, 0x83


//Accelerometer definitions
volatile bit 	int1if@INTCON3.0; // int1if = interrupt pin1 flag
#define A_IRQ1	portb.1 //accelerometer interrupt input defined

//Accelerometer Defines 
#define XOUTL      (0x00)   // 10 bits output value X LSB 
#define XOUTH      (0x01)   // 10 bits output value X MSB 
#define YOUTL      (0x02)   // 10 bits output value Y LSB 
#define YOUTH      (0x03)   // 10 bits output value Y MSB 
#define ZOUTL      (0x04)   // 10 bits output value Z LSB 
#define ZOUTH      (0x05)   // 10 bits output value Z MSB 
#define   XOUT8    (0x06)   // 8 bits output value X 
#define   YOUT8    (0x07)   // 8 bits output value Y 
#define   ZOUT8    (0x08)   // 8 bits output value Z 
#define   ASTATUS   (0x09)   // Status registers 
#define   DETSRC   (0x0A)   // Detection source registers 
#define   TOUT     (0x0B)   // Temperture out value 
#define   RES_0C   (0x0C)   // Reserved 0x0C 
#define   I2CAD    (0x0D)   // I2C device address 
#define   USRINF   (0x0E)   // User information 
#define   WHOAMI   (0x0F)   // Who am I value 
#define   XOFFL    (0x10)   // Offset drift x value (LSB) 
#define   XOFFH    (0x11)   // Offset drift x value (MSB) 
#define   YOFFL    (0x12)   // Offset drift y value (LSB) 
#define   YOFFH    (0x13)   // Offset drift y value (MSB) 
#define   ZOFFL    (0x14)   // Offset drift z value (LSB) 
#define   ZOFFH    (0x15)   // Offset drift z value (MSB) 
#define   MCTL     (0x16)   // Mode control 
#define   INTRST   (0x17)   // Interrupt latch reset 
#define   CTL1     (0x18)   // Control 1 
#define   CTL2     (0x19)   // Control 2 
#define   LDTH     (0x1A)   // Level detection threshold limit value 
#define   PDTH     (0x1B)   // Pulse detection limit value 
#define   PW       (0x1C)   // Pulse duration value 
#define   LT       (0x1D)   // Latency time value 
#define   TW       (0x1E)   // Time window 2nd Pulse value. 
#define   RES_1F   (0x1F)   // Reserved 0x1F 

//EEPROM definitions
#define	eepromif  pir2.4
unsigned short eeprom_addr@EEADR; // EEADR & EEADRH now used as a single register

#define   ZIG_PAN    (0x001)   // Zigbee PAN used 
#define   ZIG_ADDR   (0x002)   // Zigbee address used 

#endif //_TECHNOBOWL_H_
