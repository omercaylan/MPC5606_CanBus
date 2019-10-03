
#include "MPC5604B_M27V.h"

/*------------------------------------
	GLOBAL VARIABLE DECLARATIONS
-------------------------------------*/

/* State variables for the main program		*/
uint32_t LED_state = 0;
uint32_t App_state = 0;   

/* 1 byte data variable for Tx				*/
uint8_t myData;

/* Variables for holding RX					*/
uint32_t RxCODE;		 /* Received message buffer code */
uint32_t RxID;			 /* Received message ID */
uint32_t RxLENGTH;		 /* Recieved message number of data bytes */
uint8_t RxDATA[8];		 /* Received message data string*/
uint32_t RxTIMESTAMP;	 /* Received message time */
uint32_t validMessageRx; /* Rx message counter		*/

/*---------------------------------
	FUNCTION DECLARATIONS
---------------------------------*/

void PIT1_Interrupt(void);

void initModesAndClock(void)
{
	ME.MER.R = 0x0000001D; /* Enable DRUN, RUN0, SAFE, RESET modes */

	/* PLL is turning off, the bus clock is derived from Fast oscillator clock
	ext quartz = 8 MHz, no division*/
	CGM.FXOSC_CTL.R = 0x00800000;
	/* RUN0 cfg: voltage regu active, flash in mode normal, 
	FIRCON=0,OSC0ON=1,PLL0ON=0,syclk=Div ext xtal clock */
	ME.RUN[0].R = 0x001F0023;
	/* Peri. Cfg. 1 settings: only run in RUN0 mode
	Only RUNPC[1] mode configuration is defined. Only this configuration will be used.*/
	ME.RUNPC[1].R = 0x00000010;

	/*Selection of operation modes for active peripherals.*/
	ME.PCTL[68].R = 0x01; //SIUL use the configuration of RunPC[1]
	ME.PCTL[92].R = 0x01; //PIT1 use the configuration of RunPC[1]
	ME.PCTL[17].R = 0x01; //FlexCAN1 use the configuration of RunPC[1]
	ME.PCTL[5].R = 0x01;  //DSPI1 use the configuration of RunPC[1]

	/* Mode Transition to enter RUN0 mode: */
	ME.MCTL.R = 0x40005AF0; /* Enter RUN0 Mode & Key */
	ME.MCTL.R = 0x4000A50F; /* Enter RUN0 Mode & Inverted Key */

	while (ME.GS.B.S_MTRANS)
	{
	} /* Wait for mode transition to complete */
	while (ME.GS.B.S_CURRENTMODE != 4)
	{
	} /* Verify RUN0 is the current mode */
	  /* Note: This verification ensures a SAFE mode */
	  /*       tranistion did not occur. SW could instead */
	  /*       enable the safe mode tranision interupt */
}

void initDSPI_1(void)
{
	DSPI_1.MCR.R = 0x80010001;	 /* Set Master Mode, CS select at LOW, HALT=1  */
	DSPI_1.CTAR[0].R = 0x78024424; /* Set timing: Tcsc=Tasc=4�s, Tdt= 1�s, BR=100kbits/s  */

	DSPI_1.MCR.B.HALT = 0x0; /* Exit HALT mode: go from STOPPED to RUNNING state */

	SIU.PCR[113].R = 0x0A04; /* MPC56xxB: Config pad as DSPI_0 SOUT output - PH1 */
	SIU.PCR[112].R = 0x0103; /* MPC56xxB: Config pad as DSPI_0 SIN input - PH0 	 */
	SIU.PSMI[8].R = 2;		 /* MPC56xxB: Select PCR 112 for DSPI_1 SIN input   */
	SIU.PCR[114].R = 0x0A04; /* MPC56xxB: Config pad as DSPI_0 SCK output - PH2  */
	SIU.PCR[115].R = 0x0A04; /* MPC56xxB: Config pad as DSPI_0 PCS0 output - PH3  */
}
void ReadDataDSPI_1(void)
{
	while (DSPI_1.SR.B.RFDF != 1)
	{
	}						  /* Wait for Receive FIFO Drain Flag = 1  */
	DSPI_1.POPR.R;			  /* Read data received by slave SPI  */
	DSPI_1.SR.R = 0x80020000; /* Clear TCF, RDRF flags by writing 1 to them  */
}
void Init_SBC_DBG(void) /* Send SPI commands for activating CAN Transciever	 */
{
	vuint32_t i;
	DSPI_1.PUSHR.R = 0x0001DF80;
	ReadDataDSPI_1(); /* A dummy read after each command */
	for (i = 0; i < 200; i++)
	{
	} /* Wait a while for operations to be completed */

	DSPI_1.PUSHR.R = 0x00015A00;
	ReadDataDSPI_1();
	for (i = 0; i < 200; i++)
	{
	}

	DSPI_1.PUSHR.R = 0x00015E90;
	ReadDataDSPI_1();
	for (i = 0; i < 200; i++)
	{
	}

	DSPI_1.PUSHR.R = 0x000160C0;
	ReadDataDSPI_1();
	for (i = 0; i < 200; i++)
	{
	}
}

//Configuration of PORT E as GPIO
void config_PORT_E(void)
{
	/* Key inputs			*/
	SIU.PCR[64].R = 0x0100;
	SIU.PCR[65].R = 0x0100;
	/* LED Outputs			*/
	SIU.PCR[68].R = 0x0200;
	SIU.PCR[69].R = 0x0200;
	SIU.PCR[70].R = 0x0200;
	SIU.PCR[71].R = 0x0200;
}

//PIT 1 Timer initialisation: an interrupt every 200ms
void initPIT(void)
{
	PIT.PITMCR.R = 0x00000001;		 /* Enable PIT and configure to stop in debug mode */
	PIT.CH[1].LDVAL.R = 1600000;	 /* Timeout= 1600000 sysclks x 1sec/8M sysclks = 200 ms */
	PIT.CH[1].TCTRL.R = 0x000000003; /* Enable PIT1 interrupt & start PIT counting */
}

//FlexCAN1 initialisation
void initCAN1(void)
{
	uint8_t i;
	/* Put in Freeze Mode, local priority disable, enable only 8 message buffers, common ID filter */
	CAN_1.MCR.R = 0x50000007;
	/* Configure for 8MHz OSC, 100KHz bit time */
	CAN_1.CR.R = 0x04DB0006;
	/* Disactivate all 64 message buffers */
	for (i = 0; i < 64; i++)
	{
		CAN_1.BUF[i].CS.B.CODE = 0;
	}
	/* MB 0 will be the TX buffer, so initialised with TX INACTIVE		*/
	CAN_1.BUF[0].CS.B.CODE = 8; /* Message Buffer 0 set to TX INACTIVE */
	/* MB 1 will be RX buffer		*/
	CAN_1.BUF[1].CS.B.IDE = 0;		/* MB 1 will look for a standard ID (11 bits) */
	CAN_1.BUF[1].ID.B.STD_ID = 555; /* MB 1 will look for ID = 555 */
	CAN_1.BUF[1].CS.B.CODE = 4;		/* MB 1 set to RX EMPTY*/

	/*Common ID filtering: accept all bits if standard ID is used for matching*/
	CAN_1.RXGMASK.R = 0x07FF0000;

	/* Pin configuration		*/
	SIU.PCR[42].R = 0x0624; /* MPC56xxB: Config port C10 as CAN1TX, open drain */
	SIU.PCR[43].R = 0x0100; /* MPC56xxB: Configure port C11 as CAN1RX */
	SIU.PSMI[0].R = 0x01;   /* MPC56xxB: Select PCR 43 for CAN1RX Input */

	/* Leave Freeze mode			*/
	CAN_1.MCR.R = 0x00000007; /* Negate FlexCAN1 halt state for the 8 first message buffers */
}

//Enable masked interrupts
void enableIrq(void)
{
	INTC.CPR.B.PRI = 0; /* Single Core: Lower INTC's current priority */
	asm(" wrteei 1");   /* Enable external interrupts */
}

//Transmit a 8-bit data in standard frame format, with Transmit ID of 666
void TransmitMsg(uint8_t TxData, uint16_t MsgID)
{
	uint8_t i;
	/* Assumption:  Message buffer CODE is INACTIVE --> done in initCAN1 */
	CAN_1.BUF[0].CS.B.IDE = 0;		  /* Use standard ID length */
	CAN_1.BUF[0].ID.B.STD_ID = MsgID; /* Transmit ID */
	CAN_1.BUF[0].CS.B.RTR = 0;		  /* Data frame, not remote Tx request frame */
	CAN_1.BUF[0].CS.B.LENGTH = 1;
	CAN_1.BUF[0].DATA.B[0] = TxData; /* Data to be transmitted */
	for (i = 1; i < 8; i++)
	{
		CAN_1.BUF[0].DATA.B[i] = 0;
	}
	CAN_1.BUF[0].CS.B.SRR = 1;	/* Tx frame (not required for standard frame)*/
	CAN_1.BUF[0].CS.B.CODE = 0xC; /* Activate msg. buf. to transmit data frame */
}

//Receive a message on MB 1 with data ID 555
//Print 4 LSB bits of the first byte on PE4-PE7.
void ReceiveMsg(void)
{
	vuint8_t j;
	vuint32_t dummy;
	vuint32_t result32;

	//IFRL = IFLAG1 in Bolero datasheet.
	while (CAN_1.IFRL.B.BUF01I == 0)
	{
	}								 /* Wait for CAN 1 MB 1 flag */
	RxCODE = CAN_1.BUF[1].CS.B.CODE; /* Read CODE, ID, LENGTH, DATA, TIMESTAMP */
	RxID = CAN_1.BUF[1].ID.B.STD_ID;
	RxLENGTH = CAN_1.BUF[1].CS.B.LENGTH;
	for (j = 0; j < RxLENGTH; j++)
	{
		RxDATA[j] = CAN_1.BUF[1].DATA.B[j];
	}
	RxTIMESTAMP = CAN_1.BUF[1].CS.B.TIMESTAMP;
	dummy = CAN_1.TIMER.R;	 /* Read TIMER to unlock message buffers */
	CAN_1.IFRL.R = 0x00000002; /* Clear CAN 1 MB 0 flag */

	/*	Print LSB of the first byte to the four LEDs on the starter kit		*/
	result32 = (~(RxDATA[0]) & 0xF) << 24;
	SIU.PGPDO[2].R = result32;
	validMessageRx++;
}

void disableWatchdog(void)
{
	SWT.SR.R = 0x0000c520; /* Write keys to clear soft lock bit */
	SWT.SR.R = 0x0000d928;
	SWT.CR.R = 0x8000010A; /* Clear watchdog enable (WEN) */
}

int main(void)
{
	disableWatchdog();										 /* Disable SWT watchdog			*/
	initModesAndClock();									 /* Enable sysclock=8MHz and enable SIU, PIT, DSPI1, FlexCAN1 */
	initPIT();												 /* Init PIT timer 				*/
	initCAN1();												 /* Init CAN						*/
	config_PORT_E();										 /* Config SIU GPIO				*/
	initDSPI_1();											 /* Init DSPI1					*/
	Init_SBC_DBG();											 /* Init CAN Transciever using DSPI1	*/
	INTC_InstallINTCInterruptHandler(PIT1_Interrupt, 60, 2); /* Set PIT interrupt handler*/
	enableIrq();											 /* Ensure INTC current prority=0 & enable IRQ */

	App_state = 0;
	myData = 0;
	/* Loop forever:
	A state-machine with two states:
		- State 0: Tx 'myData' every 200ms on ID=666 with MB0, increment myData, change LEDs 
			at every Tx;press Key 1 to switch to State 1,
		- State 1: Rx frames on MB 1 with ID=555, print 4 least significant bits on LEDs;
			press Key 0 to switch to State 0,
	*/
	while (1)
	{
		if (App_state == 0)
		{
			if (!SIU.GPDI[65].B.PDI)
			{
				App_state = 1;
				PIT.CH[1].TCTRL.R = 0x000000000; /* Disable PIT1 interrupt & stop PIT counting */
			}
		}
		else
		{
			if (CAN_1.IFRL.B.BUF01I != 0) /* Wait for CAN 1 MB 1 flag */
				ReceiveMsg();
			if (!SIU.GPDI[64].B.PDI)
			{
				App_state = 0;
				myData = 0;
				PIT.CH[1].TCTRL.R = 0x000000003; /* Enable PIT1 interrupt & start PIT counting */
			}
		}
	}
}

/* PIT 1 timer ISR		*/
void PIT1_Interrupt(void)
{
	//Clear PIT1 flag
	PIT.CH[1].TFLG.B.TIF = 1;
	TransmitMsg(myData, 666);
	myData++;
	if (LED_state == 0)
	{
		LED_state = 1;
		SIU.PGPDO[2].R = 0x0A000000;
	}
	else
	{
		LED_state = 0;
		SIU.PGPDO[2].R = 0x05000000;
	}
}
