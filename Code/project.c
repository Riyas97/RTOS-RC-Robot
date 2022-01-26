#include "MKL25Z4.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
#include "cmsis_os.h"
#include "stdbool.h"

#define PTE20_Pin 20
#define PTB0_Pin 0
#define PTB1_Pin 1
#define PTB2_Pin 2
#define PTB3_Pin 3
#define PTD0_Pin 0

#define PTC7_Pin 7
#define PTC0_Pin 0
#define PTC3_Pin 3
#define PTC4_Pin 4
#define PTC5_Pin 5
#define PTC6_Pin 6
#define PTC10_Pin 10
#define PTC11_Pin 11
#define PTC12_Pin 12
   
   
#define RED_LED 18 // PortB Pin 18
#define MASK(x) (1 << (x))

#define FREQ 50
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART_INT_PRIO 128
#define Q_SIZE (32)
#define CLOCK 48000000
#define PRESCALER 128
#define TOTAL_NOTES_MAIN_TUNE 127
#define TOTAL_NOTES_END_TUNE 202

// Mutex for green LED resource
osMutexId_t ledMutex;

// To store data received
volatile uint8_t data = 0x00;
// To store state of the bot whether is it moving or stationary
volatile int state = 0;

// Enum to store the musical notes
typedef enum{
	rest = 0,
	As3 = 233,
	F3 = 175,
	Gs3 = 208,
	C4 = 262,
	Cs4 =277 ,
	D4 = 294,
	Ds4 = 311,
	E4 = 330 ,
	F4 = 349,
	Fs4 = 370 ,
	G4 = 392,
	Gs4 = 415,
	A4 = 440,
	As4 = 466,
	C5 = 523,
	Cs5 = 554,
	B4 = 494,
	D5 = 587,
	E5 = 659,
	F5 = 698,
	G5 = 784,
	A5 = 880,
	B5 = 988	 
}notes;

// Note Durations of end tune
int noteDuration_endTune[] = {         
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
 
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
 
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 125, 250, 125,

	125, 125, 250, 125, 125,
	250, 125, 250, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 375,

	250, 125,
	
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
 
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
 
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 125, 250, 125,

	125, 125, 250, 125, 125,
	250, 125, 250, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 375,
 
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 125, 125, 125, 375,
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 500,

	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 125, 125, 125, 375,
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 500
};

// Notes for main tune
int notes_mainTune[] = {
	As3, As3, As3, As3, As3, As3, Gs3, As3, As3, As3, As3, As3, As3, 
	Gs3, As3, As3, As3, As3, As3, As3, F3, F3, F3, F3, F3, F3, 
	F3, F3, F3, F3, As3, F3, As3, As3, C4, D4, Ds4, F4, F4, 
	F4, Fs4, Gs4, As4, As4, As4, As4, Gs4, Fs4, Gs4, Fs4,F4, F4, 
	Ds4, Ds4, F4, Fs4, F4, Ds4, Cs4, Cs4, Ds4, F4, Ds4, Cs4, C4, 
	C4, D4, E4, G4, F4, F3, F3, F3, F3, F3, F3, F3, F3, 
	F3, F3, F3, F3, F3, As3, F3, As3, As3, C4, D4, Ds4, F4, 
	F4, F4, Fs4, Gs4, As4, Cs5, C5, A4, F4, Fs4, As4, A4, F4, 
	F4,Fs4, As4, A4, F4, D4, Ds4, Fs4, F4, Cs4, As3, C4, C4, 
	D4, E4, G4, F4, F3, F3, F3, F3, F3, F3     
};

// Notes for end tune			 
int notes_endTune[] = {      
	 E4, G4, A4, A4, 0,
	 A4, B4, C5, C5, 0,
	 C5, D5, B4, B4, 0,
	 A4, G4, A4, 0,
	 
	 E4, G4, A4, A4, 0,
	 A4, B4, C5, C5, 0,
	 C5, D5, B4, B4, 0,
	 A4, G4, A4, 0,
	 
	 E4, G4, A4, A4, 0,
	 A4, C5, D5, D5, 0,
	 D5, E5, F5, F5, 0,
	 E5, D5, E5, A4, 0,
	 
	 A4, B4, C5, C5, 0,
	 D5, E5, A4, 0,
	 A4, C5, B4, B4, 0,
	 C5, A4, B4, 0,

	 A4, A4,
	 
	 A4, B4, C5, C5, 0,
	 C5, D5, B4, B4, 0,
	 A4, G4, A4, 0,

	 E4, G4, A4, A4, 0,
	 A4, B4, C5, C5, 0,
	 C5, D5, B4, B4, 0,
	 A4, G4, A4, 0,
	 
	 E4, G4, A4, A4, 0,
	 A4, C5, D5, D5, 0,
	 D5, E5, F5, F5, 0,
	 E5, D5, E5, A4, 0,
	 
	 A4, B4, C5, C5, 0,
	 D5, E5, A4, 0,
	 A4, C5, B4, B4, 0,
	 C5, A4, B4, 0,
	 //End of Repeat

	 E5, 0, 0, F5, 0, 0,
	 E5, E5, 0, G5, 0, E5, D5, 0, 0,
	 D5, 0, 0, C5, 0, 0,
	 B4, C5, 0, B4, 0, A4,

	 E5, 0, 0, F5, 0, 0,
	 E5, E5, 0, G5, 0, E5, D5, 0, 0,
	 D5, 0, 0, C5, 0, 0,
	 B4, C5, 0, B4, 0, A4
};
 
int duration_mainTune[] = {
	1000, 125, 125, 120, 125, 500, 125, 1000, 125, 125, 120, 125, 500,
	125, 1000, 125, 125, 120, 125, 250, 125, 125, 125, 125, 125, 125,
	125, 125, 125, 375, 500, 500, 125, 125, 125, 125, 125, 1000, 250,
	125, 125, 120, 1000, 125, 125, 250, 125, 125, 500, 125, 1000, 500,
	250, 125, 125, 1000, 250, 250, 250, 125, 125, 1000, 250, 250, 250,
	125, 125, 1000, 500, 250, 125, 125, 250, 125, 125, 250, 125, 125,
	250, 125, 125, 250, 250, 500, 1000, 250, 125, 125, 125, 125, 1000,
	250, 125, 125, 125, 1000, 500, 500, 1000, 500, 1500, 500, 500, 1000,
	500, 1000, 500, 500, 1000, 500, 1000, 500, 500, 1000, 500, 250, 125,
	125, 1000, 500, 250, 125, 125, 250, 125, 125, 2500
};     
 
int delay_mainTune[] = {  
	50, 25, 25, 25, 25, 25, 25, 50, 25, 25, 25, 25, 25,
	25, 200, 25, 25, 25, 25, 25, 25, 25, 75, 25, 25, 75,
	25, 25, 75, 100, 100, 100, 25, 25, 25, 25, 25, 250, 25,
	25, 25, 25, 500, 25, 25, 25, 25, 25, 25, 25, 50, 25,
	25, 25, 25, 100, 25, 25, 25, 25, 25, 50, 25, 25, 25,
	25, 25, 50, 50, 25, 25, 25, 25, 25, 25, 25, 25, 25,
	25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 500,
	25, 25, 25, 25, 500, 25, 25, 50, 50, 50, 50, 50, 50,
	50, 250, 50, 50, 50, 50, 50, 50, 50, 50, 50, 25, 25,
	25, 50, 50, 50, 25, 25, 50, 25, 25, 1000
};

// Get the mod value from a given frequency
int mod_value(int frequency){
	if (frequency == 0) {
		return 0;
	} else {
		return DEFAULT_SYSTEM_CLOCK / (frequency * 128)  - 1;
	}
}

// Initialisation for PWM
void initPWM(void){
	// Enable clock to PORTB, PORTD and PORTE
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	
	// clear the previous settings for the pins 
	// configure their MUX settings to enable the Timer/Pulse Width Module (TPM) functions
	PORTD->PCR[PTD0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_Pin] |= PORT_PCR_MUX(4);
	 
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	 
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	 
	PORTB->PCR[PTB2_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB2_Pin] |= PORT_PCR_MUX(3);
	 
	PORTB->PCR[PTB3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB3_Pin] |= PORT_PCR_MUX(3);
	 
	PORTE->PCR[PTE20_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PTE20_Pin] |= PORT_PCR_MUX(3);
	 
	//Enable the clock to TPM (Timer/PWM module) 
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM0_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	// Select the MCGFLLCLK clock or MCGPLLCLK/2 for TPM counter clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
  // Initialise values 	
	TPM1->MOD = mod_value(FREQ);
	TPM2->MOD = mod_value(FREQ);
	TPM1_C0V = (mod_value(FREQ) + 1 ) / 2;
	TPM1_C1V = (mod_value(FREQ) + 1 ) / 2;
	TPM2_C0V = 0;
	TPM2_C1V = 0;
	
  // Select prescalar of 128 and LPTPM clock	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	// Select CPWM mode
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
  // Configure High-true pulses (clear Output on match, set Output on reload) and the Edge-aligned PWM 	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
 	// Select prescalar of 128 and LPTPM clock
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	// Select CPWM mode
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
  // Configure High-true pulses (clear Output on match, set Output on reload) and the Edge-aligned PWM 
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1)); 
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
  TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Initialise values 
	TPM0->MOD = mod_value(FREQ);
	TPM0_C0V = (mod_value(FREQ) + 1 ) / 2;
	
  // Select prescalar of 128 and LPTPM clock	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	// Select CPWM mode
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	 
	// Configure High-true pulses (clear Output on match, set Output on reload) and the Edge-aligned PWM 
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

// Initalisation for LED
void initLED() {
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK) | (SIM_SCGC5_PORTC_MASK));
	
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
 
	PORTC->PCR[PTC7_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC7_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC0_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC3_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC3_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC4_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC5_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC6_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC6_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC10_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC10_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC11_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC11_Pin] |= PORT_PCR_MUX(1);
	 
	PORTC->PCR[PTC12_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC12_Pin] |= PORT_PCR_MUX(1);
	 
	// Set Data Direction Registers for PortB and PortC
	PTB->PDDR |= MASK(RED_LED);
	PTB->PDOR |= MASK(RED_LED);
	 
	PTC->PDDR |= MASK(PTC7_Pin);
	PTC->PDDR |= MASK(PTC0_Pin);
	PTC->PDDR |= MASK(PTC3_Pin);
	PTC->PDDR |= MASK(PTC4_Pin);
	PTC->PDDR |= MASK(PTC5_Pin);
	PTC->PDDR |= MASK(PTC6_Pin);
	PTC->PDDR |= MASK(PTC10_Pin);
	PTC->PDDR |= MASK(PTC11_Pin);
	PTC->PDDR |= MASK(PTC12_Pin);
}

// Initialisation for UART
void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	// Enable clock to UART & PORTE 
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
  // Connect UART to pins for PTE22, PTE23 
	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	 
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
  // Ensure tx and rx are disabled before configuration 	
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	 
	// Set baud rate  
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
  // Enable interrupts	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	
 	// No parity, 8 bits, two stop bits, other settings;
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
  // Enable transmitter and receiver 	
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | UART_C2_RIE_MASK);
}

// ON all front green LEDs
void onGreen() {
		PTC->PDOR |= MASK(PTC11_Pin);
		PTC->PDOR |= MASK(PTC10_Pin);
		PTC->PDOR |= MASK(PTC6_Pin);
		PTC->PDOR |= MASK(PTC5_Pin);
		PTC->PDOR |= MASK(PTC4_Pin);
		PTC->PDOR |= MASK(PTC3_Pin);
		PTC->PDOR |= MASK(PTC0_Pin);
		PTC->PDOR |= MASK(PTC7_Pin);
}

// OFF all front green LEDs
void offGreen() {
		PTC->PDOR &= ~MASK(PTC11_Pin);
		PTC->PDOR &= ~MASK(PTC10_Pin);
		PTC->PDOR &= ~MASK(PTC6_Pin);
		PTC->PDOR &= ~MASK(PTC5_Pin);
		PTC->PDOR &= ~MASK(PTC4_Pin);
		PTC->PDOR &= ~MASK(PTC3_Pin);
		PTC->PDOR &= ~MASK(PTC0_Pin);
		PTC->PDOR &= ~MASK(PTC7_Pin);
}

// UART Interrupt to receive data
void UART2_IRQHandler(void) {
	// Clear pending interrupts
	NVIC_ClearPendingIRQ(UART2_IRQn);
	if (UART2->S1 & UART_S1_RDRF_MASK) {	    
		// Store received data
		data = UART2->D;
		if (data == 0x02 || data == 0x03 || data == 0x04 || data == 0x05 || data == 0x06 || data == 0x07) {
			// Command for bot to move
			state = 1;
		} else {
			// Bot would be stationary
			state = 0;
		}
	} 
}

// Task to control rear red LEDs
void BackLEDControl(void *arg){
	for(;;) {
		// Toggle LED
		PTC->PDOR ^= MASK(PTC12_Pin);
		if(state){
			// Bot is moving
			osDelay(500);
		} else {
			// Bot is stationary
			osDelay(250);
		}
	}
}

// Thread/Task to control front green LEDs
void FrontLEDControl(void *arg){
	// Variable to keep count of which LED to be ON
	int counter = 0;
	for(;;) {
		if (state) {
		// Bot is moving
			switch (counter++){
				case 0 :
					// OFF all green LEDS
					offGreen();
					// ON first green LED
					PTC->PDOR |= MASK(PTC7_Pin);
					break;
				case 1 :
					offGreen();
					// ON second green LED
					PTC->PDOR |= MASK(PTC0_Pin);
					break;
				case 2 :
					offGreen();
					// ON third green LED
					PTC->PDOR |= MASK(PTC3_Pin);
					break;
				case 3 :
					offGreen();
					// ON fourth green LED
					PTC->PDOR |= MASK(PTC4_Pin);
					break;
				case 4 :
					offGreen();
					// ON fifth green LED 
					PTC->PDOR |= MASK(PTC5_Pin);
					break;
				case 5 :
					offGreen();
					// ON sixth green LED
					PTC->PDOR |= MASK(PTC6_Pin);
					break;
				case 6 :
					offGreen();
					// ON seventh green LED
					PTC->PDOR |= MASK(PTC10_Pin);
					break;
				case 7 :
					offGreen();
					// ON eigth green LED
					PTC->PDOR |= MASK(PTC11_Pin);
					break;
			}
			// Prevent counter from overshooting
			counter %= 8;
		} else {
				// Acquire LED Mutex
				osMutexAcquire(ledMutex, osWaitForever);
				// ON all the green LEDs
				PTC->PDOR |= MASK(PTC11_Pin);
				PTC->PDOR |= MASK(PTC10_Pin);
				PTC->PDOR |= MASK(PTC6_Pin);
				PTC->PDOR |= MASK(PTC5_Pin);
				PTC->PDOR |= MASK(PTC4_Pin);
				PTC->PDOR |= MASK(PTC3_Pin);
				PTC->PDOR |= MASK(PTC0_Pin);
				PTC->PDOR |= MASK(PTC7_Pin);
				// Release LED Mutex
				osMutexRelease(ledMutex);
		}
		osDelay(100);
	}
}

// Task to control directions and green LED blink when establish bluetooth
void MotorControl (void *args) {
	// Variable for power
	float power = 0.5;
	// Variable for curvature angle
	float curvature = 0.1;
	for(;;) {
		if (data == 0x08) {
			// Established bluetooth connection
			// Acquire LED Mutex
			osMutexAcquire(ledMutex, osWaitForever);
			// OFF all green LED
			offGreen();
			osDelay(1000);
			// ON all green LED
			onGreen();
			osDelay(1000);
			// OFF all green LED
			offGreen();
			osDelay(1000);
			// ON all green LED
			onGreen();
			osDelay(1000);
			// OFF all green LED
			offGreen();
			// Release LED Mutex
			osMutexRelease(ledMutex);
			// Prevent from bot from moving
			data = 0x09;
		}
		if (data == 0x02) {
			// Move forward
			TPM1_C0V = (mod_value(FREQ) + 1 ) * power;
			TPM1_C1V = 0;
			TPM2_C0V = (mod_value(FREQ) + 1 ) * power;
			TPM2_C1V = 0;
		} else if (data == 0x03) {
			// Move backward
			TPM1_C1V = (mod_value(FREQ) + 1 ) * power;
			TPM1_C0V = 0;
			TPM2_C1V = (mod_value(FREQ) + 1 ) * power;
			TPM2_C0V = 0;
		} else if (data == 0x04) {
			// Move left
			TPM1_C1V = (mod_value(FREQ) + 1 ) * power;
			TPM1_C0V = 0;
			TPM2_C1V = 0;
			TPM2_C0V = (mod_value(FREQ) + 1 ) * power;
		} else if (data == 0x05) {
			// Move right
			TPM1_C1V = 0;
			TPM1_C0V = (mod_value(FREQ) + 1 ) * power;
			TPM2_C1V = (mod_value(FREQ) + 1 ) * power;
			TPM2_C0V = 0;
		} else if (data == 0x06) {
			// Curl leftwards
			TPM1_C1V = 0;
			TPM1_C0V = (mod_value(FREQ) + 1 ) * power * curvature;
			TPM2_C0V = (mod_value(FREQ) + 1 ) * power;
			TPM2_C1V = 0;
		} else if (data == 0x07) {
			// Curl rightwards
			TPM1_C1V = 0;
			TPM1_C0V = (mod_value(FREQ) + 1 ) * power;
			TPM2_C1V = 0;
			TPM2_C0V = (mod_value(FREQ) + 1 ) * power * curvature;
		} else if (data == 0x09) {
			// Stop
			TPM1_C1V = 0;
			TPM1_C0V = 0;
			TPM2_C1V = 0;
			TPM2_C0V = 0;
		} else if (data >= 10 && data <= 110) {
		  // data to change speed settings
			power = ((float) data - 10) / 100.0;
			data = 0x09;
		} else {
			// data to change curvature settings
			curvature = (data % 200) / 100.0;
		}
		osDelay(1);
	}
}

// Task to control audio
void BuzzerControl(void *arg){
	// Variable to store number of notes played
	int count = -1;
	// Variable to stoe whether bot has finished the maze
	int end = 0x00;
	for(;;){
		count += 1;
		if (data == 0x08) {
			// Bluetooth connection established
			// Play unique tone
			TPM0->MOD = mod_value(G4);
			TPM0_C0V = (mod_value(G4 + 1) / 2);
			osDelay(500);
			TPM0->MOD = 0;
			osDelay(25);
			TPM0->MOD = mod_value(C4);
			TPM0_C0V = (mod_value(G4 + 1) / 2);
			osDelay(500);
			TPM0->MOD = 0;
			osDelay(25);
			TPM0->MOD = mod_value(Ds4);
			TPM0_C0V = (mod_value(G4 + 1) / 2);
			osDelay(250);
			TPM0->MOD = 0;
			osDelay(25);
			TPM0->MOD = mod_value(F4);
			TPM0_C0V = (mod_value(G4 + 1) / 2);
			osDelay(250);
			TPM0->MOD = 0;
			osDelay(25);
			count = -1;
		}
		if (end == 0x01) {
			// Bot has already ended the maze
			// Continue to play end tune
			count %= TOTAL_NOTES_END_TUNE;
			int mod = mod_value(notes_endTune[count]);
			if (mod == 0) {
				TPM0->MOD = 0;
			} else {
				TPM0->MOD = mod_value(notes_endTune[count]);
				TPM0_C0V = (mod_value(notes_endTune[count] + 1) / 2);
			}
			osDelay(noteDuration_endTune[count]);
			TPM0_C0V = 0;
			osDelay(25);
		} else if (data == 0x01) {
			// Bot just ended the maze
			// Start to play end tune
			end = 0x01;
			count = 0;
			TPM0->MOD = mod_value(notes_endTune[count]);
			TPM0_C0V = (mod_value(notes_endTune[count] + 1) / 2);
			osDelay(noteDuration_endTune[count]);
			TPM0_C0V = 0;
			osDelay(25);
		} else {
			// Bot still running the maze
			// Continue to play main tune
				if (count == TOTAL_NOTES_MAIN_TUNE) {
					count = 0;
				}
				TPM0->MOD = mod_value(notes_mainTune[count]);
				TPM0_C0V = (mod_value(notes_mainTune[count] + 1) / 2);
				osDelay(duration_mainTune[count]);
				TPM0_C0V = 0;
				osDelay(delay_mainTune[count]);
			}
		}
}
							 			 
int main(){
	// Initalise PWM, LED & UART
	initPWM();
	initLED();
	initUART2(BAUD_RATE);  
	// Initalise CMSIS RTOS
	osKernelInitialize();
	// Create LED Mutex
	ledMutex = osMutexNew(NULL);
	 
	// Create BuzzerControl Task 
	const osThreadAttr_t BuzzerControlAttr = {
		.priority = osPriorityNormal2
	};
	osThreadNew(BuzzerControl, NULL, &BuzzerControlAttr);
	 
	// Create MotorControl Task 
	const osThreadAttr_t MotorControlAttr = {
		.priority = osPriorityHigh
	};
	osThreadNew(MotorControl, NULL, &MotorControlAttr);
	 
	// Create BackLEDControl Task 
	const osThreadAttr_t BackLEDControlAttr = {
		.priority = osPriorityLow
	};
	osThreadNew(BackLEDControl, NULL, &BackLEDControlAttr);

	// Create FrontLEDControl Task
	const osThreadAttr_t FrontLEDControlAttr = {
		.priority = osPriorityLow1
	};
	osThreadNew(FrontLEDControl, NULL, &FrontLEDControlAttr);

	// Start thread execution
	osKernelStart();
	 
	for(;;) {
		
		}
		
}
