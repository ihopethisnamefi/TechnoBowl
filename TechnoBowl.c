/********************************************************************** 
Created by Team Techno Bowl 2009
John Garro, Zach Mady, Joseph Bonath, Wenjie Jia, and Jon Thibeault 

**********************************************************************/

#include <system.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "TechnoBowl.h" // necessary to use Awesome-O and Techno Bowl Functions, includes EESD.h

#define port_db TRUE  // turns logic analyzer mode on
#define zigbee_db TRUE  // turns zigbee hyperterminal mode on


#pragma CLOCK_FREQ 20000000
#pragma DATA _CONFIG1H, _OSC_HS_1H //20 mhz
#pragma DATA _CONFIG2H, _WDT_OFF_2H //Turns of watchdog timer 
#pragma DATA _CONFIG4L, _XINST_OFF_4L
#pragma DATA _CONFIG3H, _CCP2MX_PORTE_3H //CCP2 is E7 in microcontroller mode


//Declarations of needed external variables
extern struct controller pdata; //define pdata as an controller struct from external file (Header file)
extern unsigned short servo1_width; 
extern unsigned short servo2_width; 
extern unsigned short servo3_width; 
extern unsigned short servo4_width; 
extern unsigned short servo5_width; 
extern char robot;
extern char channel;
extern bool interrupt_flag_1;  
extern unsigned short trim;	
extern char leftJoy;
extern char rightJoy;
char acc_int_source;
bool packet_received = false;
bool updateSpeed;
bool threshold = false;
int direction;


//Main program	
void main()
{
    
	unsigned short leftScaled, rightScaled, leftScaled_last, rightScaled_last; //Declare scaling variables needed later

    // Entering Logic Analyzer mode if turned on
    // Port A, B ,C ,D, E, F, G
    #if port_db
        port_test();
    #endif

	//Set addressing scheme of transceiver to receive zigbee packets from desired remote control on the desired channel
	robot = 3;
	channel = 23;
	trim = 53020;
	
	trisf.7 = 0;//set pin F7 to output and set high, so that voltage translators are enabled
	latf.7 = 1;
	trisa.3 = 0;//set A3 to digital out, to be used to connect LED indicating tackle
	lata.3 = 0; //set A3 low initially
	adcon1 = 0x0f; //set default analog ins to digital i/o for 6722  
	
	init(); //initialize SPI1+2, Accel, Timers, CCP pins, reset transceiver and set to RX_ON_AACK mode  
	
	leftJoy = 128; //initial calibrated values for left and right joy sticks
	rightJoy = 128;
	leftScaled = 300; 
	rightScaled = 300; 
	int acount = 0; //declare counter for tackle detect LED duration
    
    
	//Start infinite loop where the robot runs and works until physically reset
	while(1){

		if(packet_received){ //check to see if zigbee packet received
			packet_received = false; //set packet receive flag to false
			store_data(); //grab and appropriately store packet received from remote
			leftJoy = pdata.joy2y; //set the value of left joystick to value received from remote
			rightJoy = pdata.joy1y;	//set the value of right joystick to value received from remote	
		}
		if (threshold){	//Check to see if accelerometer's level detect threshold flag is set
			threshold = false; //Set flag to false
			acount = 1; //Start Tackle LED duration count
			lata.3 = 1;//Set A# high to turn on LED signaling that a tackle has been made.
			intcon3.3 = 1;  //Enables interrupt on b1 (INT1)
		}
		if(acount > 0 & acount < 2000){  //sets duration of LED tackle indicator
			acount ++;
		}
		else{ //Reset counter and set LED to low
			acount = 0;
			lata.3 = 0;
		}
		//Set last stored joystick values to current ones 
		leftScaled_last = leftScaled;
		rightScaled_last = rightScaled;
		//Scale new joystick values
		leftScaled = stickScaler(leftJoy, leftScaled_last);
		rightScaled = stickScaler(rightJoy, rightScaled_last);
		//Set servos' signals widths according to received joystick data to control robot
		servo1_width =	servo_width(leftScaled);  
		servo5_width =	servo_width(rightScaled);
	}       
}

struct controller pdata; //define pdata as an controller struct from external file
char *frame;  //initialize pointer
char testarray[30]; //define a character array to store frame bytes
/****************************************************************************
General initialization functions

*****************************************************************************/

//Interrupt Service routine
void interrupt(void) 
{	
	interrupts_disable_int(); //Disable all interrupts except timer for servos
	char status = read_int(); //Check for zigbee interrupt
	if (status == 0x08) //If Zigbee receives a valid and correctly addressed packet
	{
		packet_received = 1; //Set Zigbee packet received flag
		intcon.1 = 0; //clear IRQ pin flag
		status = read_int();//clear status register
	}	
	if(!status & !int1if){ //If Zigbee and Accelerometer interrupt flags are not set
		//clear ccp interrupt flags if set
		if (comp1if == 1)
		{
			comp1if = 0; //clear flag bit  
		}
		else if (comp2if == 1)
		{
			comp2if = 0; //clear flag bit  
		}
		else if (comp3if == 1)
		{
			comp3if = 0; //clear flag bit  
		}
		else if (comp4if == 1)
		{
			comp4if = 0; //clear flag bit  
		}
	
		else if (comp5if == 1)
		{
			comp5if = 0; //clear flag bit  
		}
		else if (tmr1if == 1) //if timer flag set
		{
			interrupt_flag_1 = 1;  // for smooth servo operation
			tmr1_reg = 53000;	//20ms period ended, reset timer
			ccp1_reg = servo1_width;
			ccp1con = 0x09; //set CCP1 to Compare high-to-low mode, reinitializes pin to high
			ccp2_reg = servo2_width;
			ccp2con = 0x09; //set CCP2 to Compare high-to-low mode, reinitializes pin to high
			ccp3_reg = servo3_width;
			ccp3con = 0x09; //set CCP3 to Compare high-to-low mode, reinitializes pin to high
			ccp4_reg = servo4_width;
			ccp4con = 0x09; //set CCP4 to Compare high-to-low mode, reinitializes pin to high
			ccp5_reg = servo5_width;
			ccp5con = 0x09; //set CCP5 to Compare high-to-low mode, reinitializes pin to high
			tmr1if = 0; //clear flag bit 
          }	
	}
	else if (int1if == 1)  //check for threshold accelerometer interrupt
	{

		intcon3.3 = 0;  //Disables interrupt on b1 (INT1)
		acc_int_source = spi2_read_int(DETSRC);	//read what source caused level detect: x, y, or z   

		spi2_write_int(INTRST, 0x03);
		delay_ms(20);
		spi2_write_int(INTRST, 0x00);  //set then clear to complete clear of interrupt
		int1if = 0; //clear flag bit
		threshold = true;
        
	}			
	else //if unknown interrupt occured reset zigbee IRQ flag just in case
	{
		intcon.1 = 0; //clear IRQ pin flag
	}
	Z_INT = 1; //set SPI1 int flag high in case interrupted during read/write
	SPI2_INT = 1; //set SPI2 int flag high in case interrupted during read/write
	interrupts_enable_int(); //reenable interrupts needed
}




//Function to be used in the beginning of initialization or SPI read/write so that all used interrupts are disabled
void interrupts_disable(void)
{
	intcon.7 = 0; //Global Interrupt Disable
	intcon.6 = 0; //Peripheral Interrupt Disable
	intcon3.5 = 0;  //Disables interrupt on b3 (zigbee)
	intcon.3 = 0;  //Disables interrupt on b4 (accelerometer)
	pie1.0 = 0; //Disables timer1 overflow interrupt (servos)
	pie1.2 = 0; //Disables compare interrupt on CCP1
	pie2.0 = 0; //Disables compare interrupt on CCP2
	pie3.0 = 0; //Disables compare interrupt on CCP3
	pie3.1 = 0; //Disables compare interrupt on CCP4
	pie3.2 = 0; //Disables compare interrupt on CCP5
}

//Function to be used to enable all the used interrupts again after initialization and SPI read/writes
void interrupts_enable(void)
{

	intcon3.5 = 1;  //Enables interrupt on b3 (zigbee)
	pie1.0 = 1; //enables timer1 overflow interrupt (servos)
	pie1.2 = 1; //enables compare interrupt on CCP1
//	pie2.0 = 1; //enables compare interrupt on CCP2 - Uncomment when being used
//	pie3.0 = 1; //enables compare interrupt on CCP3 - Uncomment when being used
//	pie3.1 = 1; //enables compare interrupt on CCP4 - Uncomment when being used
	pie3.2 = 1; //enables compare interrupt on CCP5
	intcon.7 = 1; //Global Interrupt Enable
	intcon.6 = 1; //Peripheral Interrupt Enable
}

//Interrupt disable function to be used in the interrupt routine
void interrupts_disable_int(void)
{
	intcon.7 = 0; //Global Interrupt Disable
	intcon.6 = 0; //Peripheral Interrupt Disable
	intcon3.5 = 0;  //Disables interrupt on b3 (zigbee)
	intcon.3 = 0;  //Disables interrupt on b4 (accelerometer)
	pie1.2 = 0; //Disables compare interrupt on CCP1
//	pie2.0 = 0; //Disables compare interrupt on CCP2
//	pie3.0 = 0; //Disables compare interrupt on CCP3
//	pie3.1 = 0; //Disables compare interrupt on CCP4
	pie3.2 = 0; //Disables compare interrupt on CCP5
}

//Interrupt enable function to be used in the interrupt routine
void interrupts_enable_int(void)
{

	intcon3.5 = 1;  //Enables interrupt on b3* (zigbee)
//	intcon.3 = 1;  //Enables interrupt on b4 (accelerometer)
	pie1.0 = 1; //enables timer1 overflow interrupt (servos)
	pie1.2 = 1; //enables compare interrupt on CCP1
//	pie2.0 = 1; //enables compare interrupt on CCP2
//	pie3.0 = 1; //enables compare interrupt on CCP3
//	pie3.1 = 1; //enables compare interrupt on CCP4
	pie3.2 = 1; //enables compare interrupt on CCP5
	intcon.7 = 1; //Global Interrupt Enable*
	intcon.6 = 1; //Peripheral Interrupt Enable*
}

//Initialization function to be called in the beginning of main so that the used capabilities of the board are initialized and ready
void init(void){

	interrupts_disable();
	intcon.7 = 0; //Global Interrupt Disable
	delay_ms(100);
	tmr_init();//initialize timers
	spi_init(); // Initialze all values for Serial Port
	spi2_init(); // Initialze all values for Serial Port
	acc_setup();
	cmp1_init();//PWM5 on the board, designate as wheel 1
//	cmp2_init();
//	cmp3_init();
//	cmp4_init();  
	cmp5_init();//PWM3 on the board, designate as wheel 2
	transceiver_reset(); //reset connected Zigbee transceiver
	transceiver_rx_aack_on(); //change transceiver state to RX_AACK_ON to receive filtered packets
	interrupts_enable();

}	


/*****************************************************************************
Zigbee Functions

******************************************************************************/

void spi_init(void) 
{	
	ssp1con1.5 = 0;//Disable SPEN
	intcon.7 = 0; //Global Interrupt disable
	intcon.6 = 0; //Peripheral Interrupt disable
	
	// Enabling SPI1 I/O
	trisc.3 = 0; // set SCK1 to output 
	trisc.4 = 1; // set SDI1 in input
	trisc.5 = 0; // SDO1 to output
    // Configure ZigBee I/O 
    trisf.0 = 0; // set chip select to output Z_CS
    trisc.7 = 0; // set slp to output
    trisb.0 = 1; // Set IRQ to input    
    trisf.1 = 0; // Set set RESET to output
	// Configure SPI options
	ssp1stat.7 = 1; // data sampled at end
	ssp1stat.6 = 1; // rising edge triggered
	
	ssp1con1.4 = 0; //CKP idle state set low
	ssp1con1.3 = 0;  //SET 3-0 FOR TIMING OF SPI CLOCK (see below)
	ssp1con1.2 = 0;
	ssp1con1.1 = 1; 
	ssp1con1.0 = 0;
	ssp1con1.5 = 1; // Serial Port enable bit (SPEN)
	/*********************************************
	Choices for SSP1CON1.3-0:
	0011 = SPI Master mode, clock = TMR2 output/2 
	0010 = SPI Master mode, clock = FOSC/64 
	0001 = SPI Master mode, clock = FOSC/16 
	0000 = SPI Master mode, clock = FOSC/4
	**********************************************/  
	
	intcon.3 = 0; //Disable interrupt on change port
	intcon.4 = 1;  //Enables interrupt on b0
	intcon2.6 = 1;  //Enables interrupt on rising edge
	intcon.1 = 0; //Clear flag bit
	Z_WC = 0;
	Z_INT = 0;
	
	return;
}

//Performs read of zigbee chip's register at the address given it
char trx_reg_read(char addr)
{
	  interrupts_disable();
      char dataz;   // Initialize return data
      Z_CS = 0;  // Turn chip select on
      Z_INT = 0;  // Reset the interrupt flag
      Z_WC = 0; // Clear the Write Collision Detect bit  
      
      ssp1buf = 0b10000000 + addr;  //Put transmit bits (10), then the address into the buffer
      while(!Z_INT){} // Wait until transmission is complete
      Z_INT = 0;  // Reset the interrupt flag
      ssp1buf = 0x00;  // Send dummy data (allows for receive)
      
      while(!Z_INT){} // Wait until reception is complete
      dataz = ssp1buf;  // Load received data from buffer into variable
      
      Z_INT = 0;      // Reset interrupt flag
      Z_CS = 1;      //Turn chip select off
      interrupts_enable();
      return dataz;
}

//Performs write to zigbee chip's register to the address given it with the value given it
void trx_reg_write(char addr, char val)
{
	   interrupts_disable();
	   Z_INT = 0; 
       Z_CS = 0;  
       Z_WC = 0;
       ssp1buf = 0b11000000 + addr;  //Put transmit bits (10), then the address into the buffer
       while(!Z_INT){} // Wait until transmission is complete
       Z_INT = 0;  // Reset the interrupt flag
       ssp1buf = val;  // Put the data bits into the buffer
       while(!Z_INT){} // Wait until transmission is complete
       Z_INT = 0;  // Reset the interrupt flag
       Z_CS = 1;  //Turn chip select off
       interrupts_enable();
       return;
}

//Performs a read of the address of the bit given its address, mask, and position
char trx_bit_read(char addr, char mask, char pos)
{
	interrupts_disable();
	char regdata;  //Initialize regdata
    regdata = trx_reg_read(addr);  //Gather byte that subreg is from
    regdata &= mask;  //Mask the subregister
    regdata >>= pos; //Shift bit to subregister
    interrupts_enable();
	return regdata;
}

//Performs a write to the adress given at the position given and with the value given
void trx_bit_write(char addr, char mask, char pos, char val)	
{
    interrupts_disable();
    char regdata;
    //Read current register value and mask area outside the subregister.
    regdata = trx_reg_read(addr);
    regdata &= ~mask;
    //Start preparing the new subregister value. shift in place and mask.
    val <<= pos;
    val &= mask;
    val |= regdata; //Set the new subregister value.
    trx_reg_write(addr, val); //Write the modified register value.
    interrupts_enable();
    return;
}

//Performs a RESET of the zigbee transceiver 
void transceiver_reset(void)
{
	char state;
	delay_us(tTR1);  //Wait for CLKM to turn on
    TRX_RESET = 0;   //Throw /RST low (turns reset on)
    delay_us(1); //Wait reset pulse width (625 ns)
    TRX_RESET = 1;  //Throw /RST high again (completes reset)
    Z_INT = 0;  //Sets the IF low     
    trx_bit_write(SR_TRX_CMD, CMD_TRX_OFF);  //Send chip to TRX_OFF state
    delay_us(tTR13);  //Wait for chip to go from RESET -> TRX_OFF
    state = trx_bit_read(SR_TRX_STATUS);  //Check the status	
    while(state != TRX_OFF){}  //If the chip ends up in correct state (TRX_OFF), alternate LED's
    return;
}

//Sets the state of the transceiver to PLL_ON
void transceiver_pll_on(void)
{
	trx_bit_write(SR_TRX_CMD, CMD_PLL_ON);
    delay_us(tTR4);
    char trxstat = trx_bit_read(SR_TRX_STATUS);
    while(trxstat!=PLL_ON)
    {
		latc.1 = 0;
		latc.1 = 1;
    }
}

//global variables used to code robot number and channel of for zigbee filtering
char robot;
char channel;

//Sets transceiver into RX_AACK_ON mode and sets the appropriate registers for address filtering
void transceiver_rx_aack_on(void)
{
	Z_INT = 0;  //Sets the IF low 
	trx_reg_write(RG_IRQ_MASK, 0x08);//mask interrupts so only AMI is active
    trx_bit_write(SR_IRQ_MASK_MODE, 0);// disable polling of masked interrupts	
    trx_bit_write(SR_TRX_CMD, CMD_RX_AACK_ON); //write to control register to change state to RX_ON_AACK    
    delay_us(tTR4); //delay the necessary time to TX_OFF to RX_ON_AACK
    char trxstat = trx_bit_read(SR_TRX_STATUS);
    while(trxstat!=RX_AACK_ON){}
    trx_bit_write(SR_AACK_PROM_MODE, 0); //disable proomiscuous mode
	//Can be retreived by EEPROM read and write functions after external ZAP programmer sets appropriate registers and values in EEPROM
	trx_reg_write(RG_PHY_CC_CCA, channel); //this sets to receive channel 11 (default) 2405MHz 
	trx_reg_write(RG_PAN_ID_0, 0xbb);//Set up filtering parameters of signal
    trx_reg_write(RG_PAN_ID_1, 0xbb);
    trx_reg_write(RG_SHORT_ADDR_0, 0x00); //This means that short address is 0x0b00
    trx_reg_write(RG_SHORT_ADDR_1, robot); 
	

}

//definition of variables needed to store data in frame read and store_data function
char frame_length;//initialize variable to store and return the frame length
char lengthcount;//initialize variable used to count	 
char dummy = 0x00;//define variable with byte of dummy data
char FR = 0b00100000; //define frame receive command	 

//Grabs frame of received data from Zigbee via SPI and stores in pointed array and returns the length of the frame
char trx_frame_read(char  *frame) 
{
	 interrupts_disable();
	 Z_INT = 0; //Reset interrupt flag
	 Z_CS = 0;  // Turn chip select on
	 Z_WC = 0; // Clear the Write Collision Detect bit  
     
     ssp1buf = FR;  //read frame command to SPI
     while(!Z_INT){} // Wait until transmission is complete
     Z_INT = 0;  // Reset the interrupt flag
     ssp1buf = dummy;  // Send dummy data (allows for receive)
     while(!Z_INT){}  // Wait until reception is complete
     Z_INT = 0;      //Reset interrupt flag
	 lengthcount = ssp1buf; //grab 1st byte of frame and store as the frame length 
	 frame_length = lengthcount;//set permanent frame length variable
	 
     //loop to grab rest of data in frame and store bytes in the array
	 do{		
		ssp1buf = dummy;//send dummy data via SPI
		while(!Z_INT){}//wait for reception
		Z_INT = 0;  //reset interrupt flag
		*frame++ = ssp1buf; //grab data from frame buffer and point to frame array  
		--lengthcount;  //decrement the length counter
	   } while (lengthcount > 0);
		   
	Z_CS = 1; //Turn off chip select
	interrupts_enable();
	return frame_length; //returns the length of frame
}

//Prints to hyperterminal array of data with pointer and length as arguments
void print_array(char *array, char L){
	unsigned short x = 0; //initialize counter for do loop
	do{
		print_dec(array[x]);  //print to hyperterminal byte in testarray
		putc(10);//new line
		putc(13);//carriage return
		x++;
	}while(x < L);
}

//Use function in main to appropriately store data from remote controller into global struct variables 
void store_data(void){
	frame = &testarray[0];	//set beggining of array to point at address of frame variable	
	char L = trx_frame_read(testarray); //call frame read function to store frame in test array and return frame lenghth 	    
    pdata.seq = testarray[2];//store appropriate bytes into controller struct zig
	pdata.buttons = testarray[8];
	pdata.joy1y = testarray[9];
	pdata.joy1x = testarray[10];
	pdata.joy2y = testarray[11];
	pdata.joy2x = testarray[12];
    #if zigbee_db
      prints(pdata.seq);
      prints(pdata.buttons);
      prints(pdata.joy1y);
      prints(pdata.joy1x);
      prints(pdata.joy2y);
      prints(pdata.joy2x);
    #endif
}

//Function to be called in the interrupt routine to read and return the IRQ_STATUS register of Zigbee
char read_int(void){
	  interrupts_disable_int();
      char st;   // Initialize return data
      Z_CS = 0;  // Turn chip select on
      Z_INT = 0;  // Reset the interrupt flag
      Z_WC = 0; // Clear the Write Collision Detect bit  
      char addr = RG_IRQ_STATUS;
      ssp1buf = 0b10000000 + addr;  //Put transmit bits (10), then the address into the buffer
      while(!Z_INT){} // Wait until transmission is complete
      Z_INT = 0;  // Reset the interrupt flag
      ssp1buf = 0x00;  // Send dummy data (allows for receive)
      while(!Z_INT){} // Wait until reception is complete
      st = ssp1buf;  // Load received data from buffer into variable
      Z_INT = 0;      // Reset interrupt flag
      Z_CS = 1;      //Turn chip select off
      return st;
}

/*******************************************************************************
HBridge/Servo functions

********************************************************************************/

unsigned short servo1_width; //time value of pulse where it will switch from high to low
unsigned short servo2_width; //time value of pulse where it will switch from high to low
unsigned short servo3_width; //time value of pulse where it will switch from high to low
unsigned short servo4_width; //time value of pulse where it will switch from high to low
unsigned short servo5_width; //time value of pulse where it will switch from high to low
	
void tmr_init(void)
{
	//Setup Timer1
	wdtcon.4 = 0;
	t1con = 0xb1; //set Timer1 to run off internal oscillator Fosc/4 with prescaler of 8
	t3con.6 = 0;
	t3con.3 = 0; //Timer1 and Timer2 used for all CCP pins
	pie1.0 = 1; //enables timer1 overflow interrupt
	tmr1_reg = 53000; //53055;	//timer set 20ms before overflow interrupt 
	//Setup Timer2	
	pr2 = 255;  //set period 0.8192ms (5MHz) or 3.2768ms (20MHz)
	t2con = 0x07;  //set tmr2 prescaler to 16 and enable tmr2
}

unsigned short trim;//global variable needed for calibration of width of servo signals

//sets the width of the servo signal
unsigned short servo_width(unsigned short servo)
{
	unsigned short width;
	width = trim + 624 + 12 + servo;  //gives more resolution.  + 12 is so we have a range between 0 and 600
	return width;
}

//Setup CCP4 for compare
void cmp4_init(void)
{		
	unsigned short servo4_val = 300; //%value of period
	servo4_width = servo_width(servo4_val);
	ccp4con = 0x09; //set CCP4 to Compare high-to-low mode
	pie3.1 = 1; //enables compare interrupt on CCP4
	ccp4_reg = servo_width(servo4_val);  //set initial compare value
	trisg.3 = 0;  // port g output (CCP4 is PortG3)
	return;
}

//Setup CCP5 for compare
void cmp5_init(void)
{	
	unsigned short servo5_val = 300;
	servo5_width = servo_width(servo5_val);
	ccp5con = 0x09; //set CCP5 to Compare high-to-low mode	
	pie3.2 = 1; //enables compare interrupt on CCP5
	ccp5_reg = servo_width(servo5_val);  //set initial compare value
	trisg.4 = 0;  // port g output (CCP5 is PortG4)
	return;
}	

//Setup CCP3 for compare
void cmp3_init(void)
{	
	unsigned short servo3_val = 300; //%value of period
	servo3_width = servo_width(servo3_val);
	ccp3con = 0x09; //set CCP3 to Compare high-to-low mode
	pie3.0 = 1; //enables compare interrupt on CCP3
	trisg.0 = 0;  // port g output (CCP3 is PortG0)
	return;
}	

//Setup CCP1 for compare
void cmp1_init(void)
{	
	unsigned short servo1_val = 300; //%value of period
    servo1_width = servo_width(servo1_val);
    ccp1con = 0x09; //set CCP1 to Compare high-to-low mode
    pie1.2 = 1; //enables compare interrupt on CCP1
    trisc.2 = 0;  // port c output (CCP1 is PortC2)
    return;
}

//Setup CCP2 for compare
void cmp2_init(void)
{	
	unsigned short servo2_val = 300; //%value of period
	servo2_width = servo_width(servo2_val);
	ccp2con = 0x09; //set CCP2 to Compare high-to-low mode
	pie2.0 = 1; //enables compare interrupt on CCP2
	trise.7 = 0;  	// port e output (CCP2 is PortE7)  
	return;
}

/************************************************************************************
Serial Communication (Hyperterminal) functions

************************************************************************************/

///Initialize serial USART capability
void serial_init(void)
{
	intcon.7 = 1; //enables all interrupts
	intcon.6 = 1; //enables all peripheral interrupts
	rcon.7 = 0; //disable priority to interrupts
	trisc.7 = 1;
	trisc.6 = 0;
	baudcon1.3 = 1; //disables 16bit baud generator
	spbrg1 = 86; //sets baud rate at ~57600
	txsta1 = 00100100b; //enables transmission on serial port 2, makes it asynchronous, and only 8bit mode
	rcsta1 = 10010000b; //serial port 2 enabled, recevier enabled
	return;
}

//transmits character to Hyperterminal window
void putc(char value)  
{
	volatile bit check@PIR3.4; //check = tx2if
	while(!check); //wait for tx2if to go high (should be immediate)
	txreg2 = value;  //character in function call loaded into transmit register
}

//transmits character to Hyperterminal window
void iputc(char value)  
{
	volatile bit check@PIR3.4; //check = tx2if
	while(!check); //wait for tx2if to go high (should be immediate)
	txreg2 = value;  //character in function call loaded into transmit register
}

//takes character from Hyperterminal window 
char getc(void)  
{	
	char dat;
	volatile bit find@PIR3.5; //find = rc2if
	while(!find);  //wait for rc2if to go high
	dat = rcreg2;
	return dat;
}

//Prints a string of chars to the hyperterminal
void prints(const char* text)
	{
		char i = 0;
		while ( text[i] != '\0' ){
			putc( text[i++] );  //print to Hyperterminal
			}
		return;
	} 

//Prints a string of chars to the hyperterminal to be used in the interrupt routine
void iprints(const char* text)
	{
		char i = 0;
		while ( text[i] != '\0' ){
			iputc( text[i++] );  //print to Hyperterminal
			}
		return;
	} 

//prints short to the hyterterminal screen
void print_dec(unsigned short dat)
{
  unsigned short val;   // ascii results
  unsigned short temp;
  unsigned short div;
  unsigned short data;
  char i;
  char digit;
  data = dat;  // make it unsigned
  div = 10000;
  for(i=0; i <= 4; ++i)   // get all 5 digits
  {
    val = data/div;       // get most signif. digit
    putc(val + '0');    // print digit
    data -= val * div;   // what we've printed
    div=div/10;         // adjust divisor
  }
  return;
}

//prints short to the hyterterminal screen to be used in the interrupt service routine
void iprint_dec(unsigned short dat)
{
  unsigned short val;   // ascii results
  unsigned short temp;
  unsigned short div;
  unsigned short data;
  char i;
  char digit;
  data = dat;  // make it unsigned
  div = 10000;
  for(i=0; i <= 4; ++i)   // get all 5 digits
  {
    val = data/div;       // get most signif. digit
    iputc(val + '0');    // print digit
    data -= val * div;   // what we've printed
    div=div/10;         // adjust divisor
  }
  return;
}

//converts data to unsigned short
unsigned short convert(void)
{
	unsigned char data[5];
	int c = 5;
	for(int i = 0; i <= 5 ; i++){
		data[i] = getc();
		putc(data[i]);
		if(data[i] == 13)//13 = return in ASCII
		{
			c = i-1;
			break;
		}
	}
	unsigned short number = data[0] - 48;
	for(int j=1; j <= c; j++)
		number = number*10 + data[j] - 48;
	return number;
}

/***********************************************************************************
Accelerometer functions
*************************************************************************************/

//Initializes SPI2 for accelerometer communication to the micro
void spi2_init(void) {
	
	// Enabling SPI2 I/O
	ssp2con1.5 = 0; // Serial Port enable bit (SPEN) 
		//disable SPEN, set everything, then re-enable
	trisd.6 = 0; // set SCK2 to output 
	trisd.5 = 1; // set SDI2 in input
	trisd.4 = 0; // SDO2 to output
    // Configure ACC I/O 
    trisa.4 = 0;    // set chip select to output ACC_CS
	trisb.1 = 1;	//set b1 to input (INT1 from accelerometer)
	trisb.2 = 1;	//set b2 to (INT2) to input for encoders interrupt
	trisb.3 = 1;	//set b3 to (INT3) to input for encoders interrupt
	trisb.4 = 0;	//sets unused ports as outputs
	trisb.5 = 0;	//  "    "      "    "    "
	intcon.3 = 0;  //Disables interrupt on b4 and b5
	intcon.0 = 0; //clear RB interrupt on change flag bit
   
	// Configure SPI options
	ssp2stat.7 = 1; // data sampled at end if 1
	ssp2stat.6 = 1; // rising edge triggered if 0
	ssp2con1.4 = 0; // Idle clock is low
	ssp2con1.3 = 0;  //SET 3-0 FOR TIMING OF SPI CLOCK (see below)	
	ssp2con1.2 = 0;
	ssp2con1.1 = 1;
	ssp2con1.0 = 0;
	/*********************************************
	Choices for SSP2CON1.3-0:
	0011 = SPI Master mode, clock = TMR2 output/2 
	0010 = SPI Master mode, clock = FOSC/64 
	0001 = SPI Master mode, clock = FOSC/16 
	0000 = SPI Master mode, clock = FOSC/4
	**********************************************/  
	pie3.7 = 0;  //Disables MSSP2 interrupt
	rcon.7 = 0; //disables priority for interrupts
	intcon2.5 = 1; //enable rising edge interrupt for INT1 (b1)
	intcon3.0 = 0; //clear flag on INT1 (b1)
	intcon3.3 = 1; //enable INT1 (b1)
	ssp2con1.5 = 1; // Serial Port enable bit (SPEN) re-enabled
	A_WC = 0;
	SPI2_INT =  0;
    
	return;
}

//Performs a write to the accelerometer over SPI2
void spi2_write(char reg, char accdata){
    
	char dummy;
	A_CS = 0;
	SPI2_INT = 0;
	A_WC = 0;
	ssp2buf = (0b10000000 | (reg*2)); //need to x2 in order to shift over one bit
	while (!SPI2_INT){} //wait for transmission complete
	dummy = ssp2buf; //dummy read
	SPI2_INT = 0;
	A_WC = 0;
	ssp2buf = (0b00000000 | accdata);
	while (!SPI2_INT){} //wait for transmission complete
	dummy = ssp2buf;
	SPI2_INT = 0;	
	A_CS = 1;
}

//Performs a write to the accelerometer over SPI2 to be used in the interrupt service routine
void spi2_write_int(char reg, char accdata){
    
	char dummy;
	A_CS = 0;
	SPI2_INT = 0;
	A_WC = 0;
	ssp2buf = (0b10000000 | (reg*2)); //need to x2 in order to shift over one bit
	while (!SPI2_INT){} //wait for transmission complete
	dummy = ssp2buf; //dummy read
	SPI2_INT = 0;
	A_WC = 0;
	ssp2buf = (0b00000000 | accdata);
	while (!SPI2_INT){} //wait for transmission complete
	dummy = ssp2buf;
	SPI2_INT = 0;
	A_CS = 1;
}

//Performs a read of the accelerometer over SPI2

char spi2_read(char reg){
    
	char x;
	char dummy;
	A_CS = 0;
	A_WC = 0;
	SPI2_INT = 0;
	ssp2buf = (0b01111111 & (reg*2)); //need to x2 in order to shift over one bit
	while (!SPI2_INT){}
	dummy = ssp2buf; 
	SPI2_INT = 0;
	A_WC = 0;
	ssp2buf = 0x00; //load dummy data to send
	while (!SPI2_INT){}
	x = ssp2buf; 
	SPI2_INT = 0;
	A_CS = 1;
    
	return x;
}

//Performs a read of the accelerometer over SPI2 to be used in interrupt service routine
char spi2_read_int(char reg){
    
	char x;
	char dummy;
	A_CS = 0;
	A_WC = 0;
	SPI2_INT = 0;
	ssp2buf = (0b01111111 & (reg*2)); //need to x2 in order to shift over one bit
	while (!SPI2_INT){}
	dummy = ssp2buf; 
	SPI2_INT = 0;
	A_WC = 0;
	ssp2buf = 0x00; //load dummy data to send
	while (!SPI2_INT){}
	x = ssp2buf; 
	SPI2_INT = 0;
	A_CS = 1;
    
	return x;
}

//Sets up the Accelerometer into Level detect mode and sets the threshold value
void acc_setup(void){
    
	spi2_write(I2CAD, 0xFF); // I2C interface disabled	
	spi2_write(LDTH, 0x70); // 7g threshold 
	spi2_write(CTL1, 0x00); // put level detect interrupt onto pin INT1
	spi2_write(MCTL, 0x42); // put into level detection	mode
}

char leftJoy;
char rightJoy;
bool interrupt_flag_1;

//Created by ME Blue Team 2009
//Function stepit1() will take a desired servo value and step the servo
//motor in 10 discrete steps to the value.  Each step is separated by
//20ms, or the time it takes for each interrupt to trigger.
void stepit1(unsigned short servo1_val)
{
	unsigned short servo1_current_val, servo1_new_val;
	unsigned short diff, diff2, val, interval;
	servo1_current_val = servo1_width;
	servo1_new_val = servo_width(servo1_val);
	if(servo1_new_val > servo1_current_val) //Make sure get positive value for difference
		diff = servo1_new_val - servo1_current_val;
	else
		diff = servo1_current_val - servo1_new_val;
	interval = diff / 10;
	val = servo1_current_val;
	while(val < servo1_new_val)
	{
		//Find absolute value of the difference...
		if(servo1_new_val > val)
			diff2 = servo1_new_val - val;
		else
			diff2 = val - servo1_new_val;
		//Make sure we end on the correct value...
		if(diff2 <= interval)
			servo1_width = servo1_new_val;
		//Either increment or decrement based on what is necessary
		else
		{
			if(servo1_new_val > val)
				servo1_width += interval;
			else
				servo1_width -= interval;
		}
		//Wait for an interrupt to fire (every 20 ms)
		//When an interrupt fires, the value is loaded into the duty cycle
		interrupt_flag_1 = false;
		while(!interrupt_flag_1);  
	}
}

unsigned short digScaler(unsigned short dig, unsigned short integerVal, unsigned short decOne, unsigned short decTwo, unsigned short decThree )
//Created by ME Blue Team 2009
/* Scales the digital value given by multiplying it by the associated number.  This number
 * is to be determined by the user by dividing the maximum value of the output range by the maximum
 * value of the input range. integerVal is the whole number from this division.  decOne, decTwo, and
 * decThree are the three most significant decimal places.
 * scaled = dig*integerVal + (dig*decOne)/10 + (dig*decTwo)/100 + (dig*decThree)/1000
 */
{
  unsigned short scaled;
  scaled = 0;

  scaled = scaled + dig*integerVal;
  scaled = scaled + (dig*decOne)/10;
  scaled = scaled + (dig*decTwo)/100;
  scaled = scaled + (dig*decThree)/1000;
  
  return scaled;
}
//Created by ME Blue Team 2009 used for scaling joystick data appropriately
unsigned short stickScaler(unsigned short dig, unsigned short scaled_last)
{
  unsigned short scaled_again;
  scaled_again = 0;
  if(dig <= 27){                                   //dig values are ranges from the joystck pots
	scaled_again = digScaler(dig,7,4,0,7);
  }else if(dig > 27 && dig <= 228){
	scaled_again = digScaler(dig - 27,1,0,0,0) + 200;  //last value was 200
  }else if(dig > 228 && dig <= 255){
	scaled_again = digScaler(dig - 228,7,4,0,7) + 400; //last value was 400
  }   
  if( (dig >= 0 && dig <= 4) || (dig >= 251 & dig <= 255) ){
	return scaled_last;
  }else{
  	return scaled_again;
  }
}

/*****************************************************************************
EEPROM Functions
******************************************************************************/

void eeprom_init(void) {
	// Enabling EEPROM
	eecon1.7 = 0; // access data EEPROM, not flash program memory 
	eecon1.6 = 0; // accesses data EEPROM not configuration bits
	eecon1.2 = 0; // writes to data EEPROM enabled (WREN)	
	//eecon1.1 = 0; // initiates EEPROM write when set to 1 (WR)	
	//eecon1.0 = 0; // initiates EEPROM read when set to 1 (RD)
	eepromif = 0; // clear EEPROM interrupt flag bit

    // Configure Interrupts needed for EEPROM
	pie2.4 = 0;  //Disables eepromif interrupt
		
	return;
}

char eeprom_read(unsigned short reg){
	char x;
	eeprom_addr = reg; //10 bit address will be put in EEADR and EEADRH
	eecon1.7 = 0; // access data EEPROM, not flash program memory 
	eecon1.0 = 1; // initiates EEPROM read when set (RD)
	x = eedata; //read data from EEDATA register
	return x;
}

void eeprom_write(unsigned short reg, char eeprom_val){
	eeprom_addr = reg; //10 bit address will be put in EEADR and EEADRH
	eedata = eeprom_val; //data placed into EEDATA register

	eecon1.7 = 0; // access data EEPROM, not flash program memory 
	eecon1.6 = 0; // accesses data EEPROM not configuration bits
	eecon1.2 = 1; // Enabled writes to data EEPROM (WREN)	
	intcon.7 = 0; //Global Interrupt Disable
	
	eecon2 = 0x55;   //necessary sequence to execute write
	eecon2 = 0x0aa;  //necessary sequence to execute write
	eecon1.1 = 1; // initiates EEPROM write (WR)	
	intcon.7 = 1; //Global Interrupt Enable
	
	while (!eepromif);  //wait for write to complete
	eecon1.2 = 0; // Disabled writes to data EEPROM (WREN)		
}

/*****************************************************************************
Logic Analyzer Mode Functions
These routines set the port to output and 
toggles the value continually with a delay between increments.
******************************************************************************/
void port_test(void)
{

    int i=0;
    
    trisa = 0; trisb = 0; trisc = 0; trisd = 0; trise = 0; trisf = 0; trisg = 0;  // makeoutput

    while(1){

        lata = i; latb = i; latc = i; latd = i; late = i; latf = i; latg = i;  // increment   
        delay_ms(10);
        i++;

	}				
}

