#include "MKL25Z4.h"
#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
#include "cmsis_os.h"

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

osMutexId_t myMutex;

volatile uint8_t data = 0x00;
volatile float power = 0.5;
volatile float curvature = 0.1;
volatile uint8_t end = 0x00;

typedef enum{rest = 0,
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
	 
		NOTE_C4 = 262,
	NOTE_D4 = 294,
	NOTE_E4 = 330,
	NOTE_F4 = 349,
	NOTE_G4 = 392,
	NOTE_A4 = 440,
	NOTE_B4 = 494,
	NOTE_C5 = 523,
	NOTE_D5 = 587,
	NOTE_E5 = 659,
	NOTE_F5 = 698,
	NOTE_G5 = 784,
	NOTE_A5 = 880,
	NOTE_B5 = 988
	 
}notes;

int duration[] = {         //duration of each note (in ms) Quarter Note is set to 250 ms
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
	//Rpeat of First Part
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
	//End of Repeat
 
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 125, 125, 125, 375,
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 500,

	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 125, 125, 125, 375,
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 500
};


int beep_notes[] = {
		G4,C4, Ds4, F4
};

int beep_duration[] = {
		500, 500, 250, 250
};


//127 Notes
int firstSongNotes[] = {
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
			 
int finalSongNotes[] = {       //Note of the song, 0 is a rest/pulse
	 NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	 NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	 NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	 NOTE_A4, NOTE_G4, NOTE_A4, 0,
	 
	 NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	 NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	 NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	 NOTE_A4, NOTE_G4, NOTE_A4, 0,
	 
	 NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	 NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
	 NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
	 NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
	 
	 NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	 NOTE_D5, NOTE_E5, NOTE_A4, 0,
	 NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
	 NOTE_C5, NOTE_A4, NOTE_B4, 0,

	 NOTE_A4, NOTE_A4,
	 //Repeat of first part
	 NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	 NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	 NOTE_A4, NOTE_G4, NOTE_A4, 0,

	 NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	 NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	 NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	 NOTE_A4, NOTE_G4, NOTE_A4, 0,
	 
	 NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	 NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
	 NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
	 NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
	 
	 NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	 NOTE_D5, NOTE_E5, NOTE_A4, 0,
	 NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
	 NOTE_C5, NOTE_A4, NOTE_B4, 0,
	 //End of Repeat

	 NOTE_E5, 0, 0, NOTE_F5, 0, 0,
	 NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
	 NOTE_D5, 0, 0, NOTE_C5, 0, 0,
	 NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

	 NOTE_E5, 0, 0, NOTE_F5, 0, 0,
	 NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
	 NOTE_D5, 0, 0, NOTE_C5, 0, 0,
	 NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};
 
 int duration_first[] = {
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
				 
				int delay_first[] = {  
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
				 
 

volatile int check = 0;
typedef struct{
unsigned char Data[Q_SIZE];
unsigned int Head; // points to oldest data element
unsigned int Tail; // points to next free space
unsigned int Size; // quantity of elements in queue
} Q_T;

Q_T TxQ, RxQ;

void Q_Init(Q_T * q) {
unsigned int i;
for (i=0; i<Q_SIZE; i++)
q->Data[i] = 0; // to simplify our lives when debugging
q->Head = 0;
q->Tail = 0;
q->Size = 0;
}

 int Q_Empty(Q_T * q) {
return q->Size == 0;
}
int Q_Full(Q_T * q) {
return q->Size == Q_SIZE;
}


int Q_Enqueue(Q_T * q, unsigned char d) {
		// What if queue is full?
		if (!Q_Full(q)) {
				q->Data[q->Tail++] = d;
				q->Tail %= Q_SIZE;
				q->Size++;
				return 1; // success
		} else
				return 0; // failure
}
unsigned char Q_Dequeue(Q_T * q) {
		// Must check to see if queue is empty before dequeueing
		unsigned char t=0;
		if (!Q_Empty(q)) {
				t = q->Data[q->Head];
				q->Data[q->Head++] = 0; // to simplify debugging
				q->Head %= Q_SIZE;
				q->Size--;
		}
		return t;
}

int mod_value(int frequency){
	if (frequency == 0) {
				return 0;
		} else {   
				return DEFAULT_SYSTEM_CLOCK / (frequency * 128)  - 1;
		}
}

/* initPWM() */
void initPWM(void){
		SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	 
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
	 
	 
		SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM0_MASK;
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	 
		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
			 
		TPM1->MOD = mod_value(FREQ);
		TPM2->MOD = mod_value(FREQ);
		TPM1_C0V = (mod_value(FREQ) + 1 ) / 2;
		TPM1_C1V = (mod_value(FREQ) + 1 ) / 2;
		TPM2_C0V = 0;
		TPM2_C1V = 0;
	 
		TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	 
		TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
		TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
		TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
		TPM1_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
		TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	 
		TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
		TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
		TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
		TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	 
		TPM0->MOD = mod_value(FREQ);
		TPM0_C0V = (mod_value(FREQ) + 1 ) / 2;
	 
		TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	 
		TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSA_MASK) | (TPM_CnSC_MSB_MASK));
		TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void UART2_IRQHandler(void) {
		//osSignalSet(tsk3,isrSignal);
		NVIC_ClearPendingIRQ(UART2_IRQn);
		if (UART2->S1 & UART_S1_RDRF_MASK) {
				data = UART2->D;
				// received a character
				/*if (!Q_Full(&RxQ)) {
				Q_Enqueue(&RxQ, UART2->D);
				} else {
				// error -queue full.
				}*/
		}
	 
}

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
		//PTB->PDOR |= MASK(RED_LED);
}

void initUART2(uint32_t baud_rate) {
		uint32_t divisor, bus_clock;
	 
		SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	 
		PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	 
		PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
			 
		UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	 
		bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
		divisor = bus_clock / (baud_rate*16);
		UART2->BDH = UART_BDH_SBR(divisor >> 8);
		UART2->BDL = UART_BDL_SBR(divisor);
	 
		NVIC_SetPriority(UART2_IRQn, 128);
		NVIC_ClearPendingIRQ(UART2_IRQn);
		NVIC_EnableIRQ(UART2_IRQn);
	 
		UART2->C1 = 0;
		UART2->S2 = 0;
		UART2->C3 = 0;
	 
		//UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
		UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK) | UART_C2_RIE_MASK);
		Q_Init(&RxQ);
}


void BACKLEDControl(void *arg){
		for(;;) {
			 
				if (check){
						PTC->PDOR &= ~MASK(PTC12_Pin);
						osDelay(500);

						PTC->PDOR |= MASK(PTC12_Pin);
						osDelay(500);
				} else {
						PTC->PDOR &= ~MASK(PTC12_Pin);
						osDelay(250);
						PTC->PDOR |= MASK(PTC12_Pin);
						osDelay(250);
				}
			 
		}
}

void FRONTLEDControl(void *arg){
		int counter = 0;
		for(;;) {
				if (check) {
						switch (counter++){
								case 0 :
										PTC->PDOR = MASK(PTC7_Pin);
										//PTC->PDOR = MASK(PTC11_Pin);
										break;
								case 1 :
										PTC->PDOR = MASK(PTC0_Pin);
										//PTC->PDOR = MASK(PTC7_Pin);
										break;
								case 2 :
										PTC->PDOR = MASK(PTC3_Pin);
										//PTC->PDOR = MASK(PTC0_Pin);
										break;
								case 3 :
										PTC->PDOR = MASK(PTC4_Pin);
										//PTC->PDOR = MASK(PTC3_Pin);
										break;
								case 4 :
										PTC->PDOR = MASK(PTC5_Pin);
										//PTC->PDOR = MASK(PTC4_Pin);
										break;
								case 5 :
										PTC->PDOR = MASK(PTC6_Pin);
										//PTC->PDOR = MASK(PTC5_Pin);
										break;
								case 6 :
										PTC->PDOR = MASK(PTC10_Pin);
										//PTC->PDOR = MASK(PTC6_Pin);
										break;
								case 7 :
										PTC->PDOR = MASK(PTC11_Pin);
										//PTC->PDOR = MASK(PTC10_Pin);
										break;
						}
						counter %= 8;
				} else {
					osMutexAcquire(myMutex, osWaitForever);
						PTC->PDOR |= MASK(PTC11_Pin);
						PTC->PDOR |= MASK(PTC10_Pin);
						PTC->PDOR |= MASK(PTC6_Pin);
						PTC->PDOR |= MASK(PTC5_Pin);
						PTC->PDOR |= MASK(PTC4_Pin);
						PTC->PDOR |= MASK(PTC3_Pin);
						PTC->PDOR |= MASK(PTC0_Pin);
						PTC->PDOR |= MASK(PTC7_Pin);
						osMutexRelease(myMutex);
			 
				}
				osDelay(100);
		}
}
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
void motorControl (void *args) {
		for(;;) {
						if (data == 0x08) {
						osMutexAcquire(myMutex, osWaitForever);
						offGreen();
						osDelay(1000);
						onGreen();
						osDelay(1000);
						offGreen();
						osDelay(1000);
						onGreen();
						osDelay(1000);
						offGreen();
						osMutexRelease(myMutex);
						data = 0x09;
						}
						if (data == 0x02) {
								check = 1;
								TPM1_C0V = (mod_value(FREQ) + 1 ) * power;
								TPM1_C1V = 0;
								TPM2_C0V = (mod_value(FREQ) + 1 ) * power;
								TPM2_C1V = 0;
						} else if (data == 0x03) {
								check = 1;
								TPM1_C1V = (mod_value(FREQ) + 1 ) * power;
								TPM1_C0V = 0;
								TPM2_C1V = (mod_value(FREQ) + 1 ) * power;
								TPM2_C0V = 0;
						} else if (data == 0x09) {
								check = 0;
								TPM1_C1V = 0;
								TPM1_C0V = 0;
								TPM2_C1V = 0;
								TPM2_C0V = 0;
						} else if (data == 0x05) {
								check = 1;
								TPM1_C1V = 0;
								TPM1_C0V = (mod_value(FREQ) + 1 ) * power;
								TPM2_C1V = (mod_value(FREQ) + 1 ) * power;
								TPM2_C0V = 0;
						} else if (data == 0x04) {
								check = 1;
								TPM1_C1V = (mod_value(FREQ) + 1 ) * power;
								TPM1_C0V = 0;
								TPM2_C1V = 0;
								TPM2_C0V = (mod_value(FREQ) + 1 ) * power;
						} else if (data == 0x06) {
								check = 1;
								TPM1_C1V = 0;
								TPM1_C0V = (mod_value(FREQ) + 1 ) * power * curvature;
								TPM2_C0V = (mod_value(FREQ) + 1 ) * power;
								TPM2_C1V = 0;
						} else if (data == 0x07) {
								check = 1;
								TPM1_C1V = 0;
								TPM1_C0V = (mod_value(FREQ) + 1 ) * power;
								TPM2_C1V = 0;
								TPM2_C0V = (mod_value(FREQ) + 1 ) * power * curvature;
						} else if ( data == 0x01) {
							 
						} else if (data >= 10 && data <= 110) {
								power = ((float) data - 10) / 100.0;
								data = 0x09;
						} else {
								curvature = (data % 200) / 100.0;
						}
						osDelay(1);
				}
}


	 
void BuzzerControl(void *arg){
		int i = -1;
		for(;;){
						i += 1;
						if (data == 0x08) {
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
								i = -1;
								}
						if (end == 0x01) {
								i %= 202;
								int mod = mod_value(finalSongNotes[i]);
								if (mod == 0) {
								TPM0->MOD = 0;
								} else {
								TPM0->MOD = mod_value(finalSongNotes[i]);
								TPM0_C0V = (mod_value(finalSongNotes[i] + 1) / 2);
								}
								osDelay(1 * duration[i]);
								TPM0_C0V = 0;
								osDelay(25);
						} else if (data == 0x01) {
								end = 0x01;
								i = 0;
								TPM0->MOD = mod_value(finalSongNotes[i]);
								TPM0_C0V = (mod_value(finalSongNotes[i] + 1) / 2);
								osDelay(1 * duration[i]);
								TPM0_C0V = 0;
								osDelay(25);
						} else {
							if (i == 127) {
								//osDelay(10000);
								i = 0;
								}
								TPM0->MOD = mod_value(firstSongNotes[i]);
								TPM0_C0V = (mod_value(firstSongNotes[i] + 1) / 2);
								osDelay(duration_first[i]);
								TPM0_C0V = 0;
								osDelay(delay_first[i]);
						}
				}
		}
							 
	 
				 

int main(){
		initPWM();
		initLED();
		initUART2(BAUD_RATE);  
		osKernelInitialize();
		myMutex = osMutexNew(NULL);
	 
		const osThreadAttr_t BuzzerControlAttr = {
				.priority = osPriorityNormal2
		};
		osThreadNew(BuzzerControl, NULL, &BuzzerControlAttr);
	 
		const osThreadAttr_t motorControlAttr = {
				.priority = osPriorityHigh
		};
		osThreadNew(motorControl, NULL, &motorControlAttr);
	 
		// need to work out LED
		const osThreadAttr_t BACKLEDControlAttr = {
				.priority = osPriorityLow
		};
		osThreadNew(BACKLEDControl, NULL, &BACKLEDControlAttr);

		const osThreadAttr_t FRONTLEDControlAttr = {
				.priority = osPriorityLow1
		};
		osThreadNew(FRONTLEDControl, NULL, &FRONTLEDControlAttr);

		osKernelStart();
	 
		for(;;) {
		
		}
		
}

