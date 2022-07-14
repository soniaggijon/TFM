/*************************************************************
 * http://drazzy.com/package_drazzy.com_index.json
 * Usando MegatinyCore version 2.2.3
 * El procesador Attiny 1616 
 * Clock: 20 MHz
 * millis()/micros(): TCB1(3216/1616 only)
 * Voltage for UART baud (ignored for 2K/4K parts): "Closer to 5V"
 * Support SerialEvent: "No (saves space)"
 * Startup Time : 8 ms
 * BOD level if enabled (burn bootloeader req'd): "4.2V (20 MHZ or less)"
 * BOD Mode when active/sleeping (nurm bootloader req'd): "Disabled/Disabled"
 * Save EEPROM (burn bootloeader req'd): "EEPROM retained"
 * UPDI Pin, READ DOCS or pick UPDI! (burn bootloeader req'd): "UPDI (pick this unless you have an HV UPDI program..."
 */

#define PWM_TOP 1000
#define PWM_MED PWM_TOP/2
#define TCA_FREQ_MIN 2685 //2685=10.997760Mhz //2197 //2441 //2700
#define TCA_FREQ_MAX 3540 //3540=14.499840Mhz  3662 //3750
#define TCA_FREQ_STEP 10
#define FREQ_INDEX_START  0//24
#define VGET_MAX 30+20
#define VGET_MIN 30-25
//#define debug
// Global Var declaration
float NUM_FREQ = (TCA_FREQ_MAX - TCA_FREQ_MIN) / TCA_FREQ_STEP;
uint8_t* OffsetFreqs = malloc(NUM_FREQ+1);
//uint8_t* StrData=malloc((NUM_FREQ+1)*10+1);
uint16_t* RawData=malloc((NUM_FREQ+1)*4);
volatile uint16_t MSByte;
volatile uint32_t Counter;
volatile boolean Ready = false;                     // New reading ready?
volatile uint16_t Pwm_top=PWM_TOP;
volatile uint16_t Pwm_med=Pwm_top/2;
uint16_t tcaFreq;

unsigned int Period=0xFFFF;

// End global var declaration

void swapu16(uint16_t* A, uint16_t* B) {
  uint16_t temp;
  temp=*A; *A=*B; *B=temp;
}
uint16_t getVRead() {
  uint16_t freturn=0;
  for (uint8_t i=0;i<32;i++) {
    freturn+=analogRead(PIN_PA7);
    delayMicroseconds(10);
  }
  return freturn>>5;
}

void TCD0_enableOutputChannels(void) {
    /* enable write protected register */
    CPU_CCP = CCP_IOREG_gc;        
    TCD0.FAULTCTRL = TCD_CMPAEN_bm             /* enable channel A */
                   | TCD_CMPBEN_bm;            /* enable channel B */
}

void TCDSetup(){
  TCD0_enableOutputChannels();
  TCD0.CTRLB = TCD_WGMODE_DS_gc;
  TCD0.CMPBCLR = PWM_TOP;
  //TCD0.CMPBSET = PWM_TOP/3;
  //TCD0.CMPASET = PWM_MED;
  while(!(TCD0.STATUS & TCD_ENRDY_bm));
//  TCD0.CTRLA = TCD_CLKSEL_20MHZ_gc | TCD_CNTPRES_DIV1_gc | TCD_ENABLE_bm;
  TCD0.CTRLA = TCD_CLKSEL_EXTCLK_gc | TCD_CNTPRES_DIV1_gc | TCD_ENABLE_bm;
  PORTA.DIR |= PIN4_bm | PIN5_bm;
  TCD0.INTCTRL = TCD_OVF_bm; 
}

ISR (TCD0_OVF_vect) {
  PORTC.IN = PIN1_bm;  
  TCD0.INTFLAGS = TCD_OVF_bm;                       // Clear overflow interrupt flag 
}

#define FINE_FREQ_AJUST 70
void setFrequency(uint16_t freqdiv4096) {
  uint16_t tempperiod = (F_CPU / freqdiv4096);
  Period            = tempperiod+FINE_FREQ_AJUST;
  while(!(TCA0.SINGLE.INTFLAGS && TCA_SINGLE_CMP0_bm));
  TCA0.SINGLE.PER   = Period;
  TCA0.SINGLE.CMP0 = Period>>1;                         //map(128, 0, 255, 0, Period);
}

void TCA0_init(void)
{
  TCA0.SPLIT.CTRLA=0; //disable TCA0 and set divider to 1
  TCA0.SPLIT.CTRLESET=TCA_SPLIT_CMD_RESET_gc | TCA_SINGLE_LUPD_bm | TCA_SINGLE_DIR_DOWN_gc; //set CMD to RESET, and enable on both pins. 
  //TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;                                // enable overflow interrupt
  TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc); // enable compare 0 & wgmode single slope
  TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);                           // disable event counting
  TCA0.SINGLE.PER = TCA_FREQ_MIN;                                                // set the period (1/20000000)*64*31250 = 0,1Seg
  TCA0.SINGLE.CMP0  = TCA_FREQ_MIN>>1;   //FFFF/2
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;  // set clocksource (sys_clk/64) & start timer
}


uint8_t getOffset(uint16_t freq, uint16_t VGet_Min, uint16_t VGet_Max) {
  uint16_t vread=1024;
  uint8_t vdac=0;
  uint8_t sumval=128;
  setFrequency(freq);
  delay(100);
  while (vread<VGet_Min || vread>VGet_Max) {
    analogWrite(PIN_PA6,vdac);
    delayMicroseconds(100);
    vread=getVRead();
    if(vread>VGet_Max)
      vdac+=sumval;
    else if(vread<VGet_Min) vdac-=sumval;
    
    if (sumval>1) sumval=sumval>>1;  
  }
  analogWrite(PIN_PA6,vdac);  //100
  return vdac;
}

void doCalibration(uint16_t freqmin, uint16_t freqmax, uint16_t freqstep){
  uint16_t i=0;
  while (freqmin<=freqmax) {
      OffsetFreqs[i]=getOffset(freqmin,VGET_MIN,VGET_MAX);
      Serial.printf("Offset of Freq. %d calibrated with %d ...\n\r",freqmin*4096,OffsetFreqs[i]);
      freqmin+=freqstep;
      i++;
  }
}


#ifdef debug          // Used for debug only
uint16_t  vmax=1024;
uint16_t  vmin=0;
#endif


uint16_t  vread;
char Data[10];
void setup() {
  Serial.swap(1);
  Serial.begin(115200);
  //USART0.CTRLB = USART_ODME_bm | USART_RXEN_bm | USART_TXEN_bm;
  #ifdef debug
    Serial.printf("Dynamic Memory used: %d\n\r",(uint32_t)((NUM_FREQ+1)+((NUM_FREQ+1)*10+1)));
  #endif
  PORTC.DIRSET = PIN2_bm | PIN1_bm | PIN0_bm;
  pinMode(PIN_PB0, OUTPUT);   // PB0 - TCA0 WO0, pin7 on 14-pin parts
  TCDSetup ();
  TCA0_init();
  
  analogReference(INTERNAL4V3);
  DACReference(INTERNAL4V3);
  Serial.println("Calibrando...");
  doCalibration(TCA_FREQ_MIN,TCA_FREQ_MAX,TCA_FREQ_STEP);
}


void loop() {
  uint16_t freqIndex;
  while (1) {
    freqIndex=0;
    #ifdef debug
      swapu16(&vmax,&vmin);
    #endif
    setFrequency(TCA_FREQ_MIN + TCA_FREQ_STEP*(freqIndex+FREQ_INDEX_START));
    delay(100);
    memset(RawData,0,((NUM_FREQ+1)*4) );
    while (freqIndex < NUM_FREQ) {
      tcaFreq= TCA_FREQ_MIN + TCA_FREQ_STEP*(freqIndex+FREQ_INDEX_START);
      setFrequency(tcaFreq);
      analogWrite(PIN_PA6,OffsetFreqs[freqIndex]);
      delay(20);
      RawData[freqIndex*2]=tcaFreq;
      RawData[freqIndex*2+1]=getVRead();

#ifdef debug                    // Used for debug only
      Serial.print(vmax);
      Serial.print(",");
      Serial.print(vmin);
      Serial.print(",");
      Serial.print(OffsetFreqs[freqIndex]);
      Serial.print(",");
      Serial.println(RawData[freqIndex*2+1]);
#endif
      freqIndex++;
    }

#ifndef debug
      Serial.printf("Init:%0#6X",freqIndex*4);
      Serial.write((uint8_t*)RawData,freqIndex * 4);
#endif
  }
}
