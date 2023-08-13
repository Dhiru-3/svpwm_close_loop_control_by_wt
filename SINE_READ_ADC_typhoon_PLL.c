#include "DSP28x_Project.h" // Device Headerfile and Examples Include File
#include "math.h"
#include "float.h"
#define sample 100
#define ADC_CKPS 0x1
#define PI 3.14159265358979323846

#define EPWM1_TIMER_TBPRD 15000 // 5kHz PWM Period register
#define EPWM_CMP_UP 1
#define EPWM_CMP_DOWN 0
#define Amplitude 1 //Amplitude of grid simulated voltage


typedef struct
{
volatile struct EPWM_REGS *EPwmRegHandle;
Uint16 EPwm_CMPA_Direction;
Uint16 EPwm_CMPB_Direction;
Uint16 EPwmTimerIntCount;
Uint16 EPwmMaxCMPA;
Uint16 EPwmMinCMPA;
Uint16 EPwmMaxCMPB;
Uint16 EPwmMinCMPB;

}EPWM_INFO;
//ADC pins Declerations
int LoopCount;
int ConversionCount;
int Voltage1[100];
int Voltage2[100];
int Voltage3[100];

int w[100];


float a=0,a0=0,a1=0,k=0,w1;

int i;int m=0;


float Va=0,Vb=0,Vc=0;
float V_alpha=0,V_beta=0,Angle=0,delta=0,angle=0;

float V_ref=0,m0=0,m1=0,m2=0;
int n=0;//n is sector
float Udc=1200,phi=0;
int R=0;

float ta=0,tb=0,tc=0;
//bool K=false;

float Ts=1.0/5000;
float s1a=0,s2a=0,s1b=0,s2b=0,s1c=0,s2c=0;
float ps1a=0,ps2a=0,ps1b=0,ps2b=0,ps1c=0,ps2c=0;

int m_g=0;
float angle_g=0;
int Va_g,Vb_g,Vc_g;float V_alpha_g=0,V_beta_g=0;
//float w[]={0,0,0},Vq[]={0,0,0},Vd=0;
float temp_q1=0,temp_w1=0;
// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void InitEPwm4Example(void);
void InitEPwm5Example(void);
void InitEPwm6Example(void);
//__interrupt void epwm1_isr(void);
//__interrupt void epwm2_isr(void);
//__interrupt void epwm3_isr(void);
//__interrupt void epwm4_isr(void);
//__interrupt void epwm5_isr(void);
//__interrupt void epwm6_isr(void);
__interrupt void adc_isr(void);


void clarke_g(int,int,int);
void dq_transformation(float,float);
void clarke(float,float,float);
void V_ref_angle(float,float);
void sector(float);
void phie(int,float);
void region(float,float,float);
void dwell(float,int,float,float);
void switchtiming(int,float,float,float,float);
void PWM(int,float,float,float,float,float,float,float);

//void update_compare(EPWM_INFO*);*/


// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;
EPWM_INFO epwm4_info;
EPWM_INFO epwm5_info;
EPWM_INFO epwm6_info;

// To keep track of which way the compare value is moving

void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
InitSysCtrl();
EALLOW;
#if (CPU_FRQ_150MHZ) // Default - 150 MHz SYSCLKOUT
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3) = 25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
#define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2) = 25.0 MHz
#endif
EDIS;

EALLOW;
SysCtrlRegs.HISPCP.all = ADC_MODCLK;
EDIS;



// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2833x_EPwm.c file
InitEPwm1Gpio();
InitEPwm2Gpio();
InitEPwm3Gpio();
InitEPwm4Gpio();
InitEPwm5Gpio();
InitEPwm6Gpio();


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
IER = 0x0000;
IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example. This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
EALLOW; // This is needed to write to EALLOW protected registers
//PieVectTable.EPWM1_INT = &epwm1_isr;
//PieVectTable.EPWM2_INT = &epwm2_isr;
//PieVectTable.EPWM3_INT = &epwm3_isr;
//PieVectTable.EPWM4_INT = &epwm4_isr;
//PieVectTable.EPWM5_INT = &epwm5_isr;
//PieVectTable.EPWM6_INT = &epwm6_isr;
PieVectTable.ADCINT = &adc_isr;
EDIS; // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example

// For this example, only initialize the ePWM

EALLOW;
SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
EDIS;

InitEPwm1Example();
InitEPwm2Example();
InitEPwm3Example();
InitEPwm4Example();
InitEPwm5Example();
InitEPwm6Example();
InitAdc();

EALLOW;
SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
EDIS;

// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWM1-3 INT:
//IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
//PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
//PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
//PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
//PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
//PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
//PieCtrlRegs.PIEIER3.bit.INTx6 = 1;
PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
IER |= M_INT1;
// Enable global Interrupts and higher priority real-time debug events:
EINT; // Enable Global interrupt INTM
ERTM; // Enable Global realtime interrupt DBGM

 LoopCount = 0;
 ConversionCount = 0;


 // Configure ADC
    AdcRegs.ADCMAXCONV.all = 0x0002;       // Setup 2 conv's on SEQ1
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA3 as 1st SEQ1 conv.
   // AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA2 as 2nd SEQ1 conv.
   // AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup ADCINA2 as 2nd SEQ1 conv.
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Enable SOCA from ePWM to start SEQ1
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

 // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;       // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event
//    EPwm1Regs.CMPA.half.CMPA = 0x0080;     // Set compare A value
//    EPwm1Regs.TBPRD = 15000;              // Set period for ePWM1
//    EPwm1Regs.TBCTL.bit.CTRMODE = 0;       // count up and start

 // Wait for ADC interrupt
    for(;;)
    {
       LoopCount++;
    }
 }



__interrupt void adc_isr (void)
{
    //PLL

w[ConversionCount] = AdcRegs.ADCRESULT0 >>4;

angle=(w[ConversionCount]*6.28*3/4096)*0.5;

a=sin(angle-0.3316);//phase ref=-19.5 degree initial was 18 degree
a0=sin(angle-0.3316-(2*PI)/3);
a1=sin(angle-0.3316-(4*PI)/3);



Va=407.08*sqrt(2)*a;

Vb=407.08*sqrt(2)*a0;

Vc=407.08*sqrt(2)*a1;

clarke(Va,Vb,Vc);

V_ref_angle(V_alpha,V_beta);

sector(Angle);

phie(n,Angle);

region(V_ref,phi,Udc);

dwell(m0,R,phi,Ts);

switchtiming(R,ta,tb,tc,Ts);

PWM(n,s1a,s2a,s1b,s2b,s1c,s2c,Ts);



EPwm1Regs.CMPA.half.CMPA = ps1a*EPWM1_TIMER_TBPRD;
EPwm1Regs.CMPB = ps1a*EPWM1_TIMER_TBPRD;

EPwm1Regs.AQCTLA.bit.ZRO =AQ_SET;
EPwm1Regs.AQCTLA.bit.CAU =AQ_CLEAR;
EPwm1Regs.AQCTLA.bit.CAD =AQ_SET;

EPwm1Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
EPwm1Regs.AQCTLB.bit.CAU =AQ_SET;
EPwm1Regs.AQCTLB.bit.CAD =AQ_CLEAR;

EPwm2Regs.CMPA.half.CMPA = ps2a*EPWM1_TIMER_TBPRD;
EPwm2Regs.CMPB = ps2a*EPWM1_TIMER_TBPRD;

EPwm2Regs.AQCTLA.bit.ZRO =AQ_SET;
EPwm2Regs.AQCTLA.bit.CAU =AQ_CLEAR;
EPwm2Regs.AQCTLA.bit.CAD =AQ_SET;

EPwm2Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
EPwm2Regs.AQCTLB.bit.CAU =AQ_SET;
EPwm2Regs.AQCTLB.bit.CAD =AQ_CLEAR;

EPwm3Regs.CMPA.half.CMPA = ps1b*EPWM1_TIMER_TBPRD;
EPwm3Regs.CMPB = ps1b*EPWM1_TIMER_TBPRD;

EPwm3Regs.AQCTLA.bit.ZRO =AQ_SET;
EPwm3Regs.AQCTLA.bit.CAU =AQ_CLEAR;
EPwm3Regs.AQCTLA.bit.CAD =AQ_SET;

EPwm3Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
EPwm3Regs.AQCTLB.bit.CAU =AQ_SET;
EPwm3Regs.AQCTLB.bit.CAD =AQ_CLEAR;

EPwm4Regs.CMPA.half.CMPA = ps2b*EPWM1_TIMER_TBPRD;
EPwm4Regs.CMPB = ps2b*EPWM1_TIMER_TBPRD;

EPwm4Regs.AQCTLA.bit.ZRO =AQ_SET;
EPwm4Regs.AQCTLA.bit.CAU =AQ_CLEAR;
EPwm4Regs.AQCTLA.bit.CAD =AQ_SET;

EPwm4Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
EPwm4Regs.AQCTLB.bit.CAU =AQ_SET;
EPwm4Regs.AQCTLB.bit.CAD =AQ_CLEAR;

EPwm5Regs.CMPA.half.CMPA = ps1c*EPWM1_TIMER_TBPRD;
EPwm5Regs.CMPB = ps1c*EPWM1_TIMER_TBPRD;

EPwm5Regs.AQCTLA.bit.ZRO =AQ_SET;
EPwm5Regs.AQCTLA.bit.CAU =AQ_CLEAR;
EPwm5Regs.AQCTLA.bit.CAD =AQ_SET;

EPwm5Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
EPwm5Regs.AQCTLB.bit.CAU =AQ_SET;
EPwm5Regs.AQCTLB.bit.CAD =AQ_CLEAR;


EPwm6Regs.CMPA.half.CMPA = ps2c*EPWM1_TIMER_TBPRD;
EPwm6Regs.CMPB = ps2c*EPWM1_TIMER_TBPRD;

EPwm6Regs.AQCTLA.bit.ZRO =AQ_SET;
EPwm6Regs.AQCTLA.bit.CAU =AQ_CLEAR;
EPwm6Regs.AQCTLA.bit.CAD =AQ_SET;

EPwm6Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
EPwm6Regs.AQCTLB.bit.CAU =AQ_SET;
EPwm6Regs.AQCTLB.bit.CAD =AQ_CLEAR;





if(ConversionCount == 99)
{
   ConversionCount = 0;
}
else
{
    ConversionCount++;
}
// Reinitialize for next ADC sequence

AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // Reset SEQ1
AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // Clear INT SEQ1 bit
PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE

return;

}


void InitEPwm1Example()
{
    // Setup TBCLK
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD; // Set timer period 801 TBCLKs
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
    EPwm1Regs.TBCTR = 0x0000; // Clear counter

    // Set Compare values
    //EPwm1Regs.CMPA.half.CMPA = 150; // Set compare A value
    //EPwm1Regs.CMPB = ps1a*15000; // Set Compare B value

    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    //EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading. It means "EPwm1Regs.TBCTL.bit.PHSEN = 0x01;"
       EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  // EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  //  It means "EPwm1Regs.TBCTL.bit.SYNCOSEL = 0x01;"
       EPwm1Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1;       // Clock ratio to SYSCLKOUT
       EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
       EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
       EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
       EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
       EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
//    EPwm1Regs.AQCTLA.bit.ZRO =AQ_SET;
//    EPwm1Regs.AQCTLA.bit.CAU =AQ_CLEAR;
//    EPwm1Regs.AQCTLA.bit.CAD =AQ_SET;
//
//    EPwm1Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
//    EPwm1Regs.AQCTLB.bit.CAU =AQ_SET;
//    EPwm1Regs.AQCTLB.bit.CAD =AQ_CLEAR;
    //**********************
//    EPwm1Regs.ETSEL.bit.SOCAEN = 1; // Enable SOC on A group
//    //EPwm3Regs.ETSEL.bit.SOCASEL = 4; // Select SOC from from CPMA on upcount
//    EPwm1Regs.ETSEL.bit.SOCASEL = 1; // Select SOC from TBCTR equal to zero
//    EPwm1Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event
    //**********************
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = 500*2;
    EPwm1Regs.DBFED = 500*2;
    // Interrupt where we will change the Compare Values
//    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD; // Select INT on PERIOD event*****
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event

    EPwm1Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event
    //EPwm1Regs.ETPS.bit.INTPRD = ET_2ND; // Generate INT on 2ND event*****


    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
     epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
    epwm1_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
    epwm1_info.EPwmRegHandle = &EPwm1Regs; // Set the pointer to the ePWM module
    /*epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;*/
}

void InitEPwm2Example()
{
    // Setup TBCLK
    EPwm2Regs.TBPRD = EPWM1_TIMER_TBPRD; // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
    EPwm2Regs.TBCTR = 0x0000; // Clear counter

    // Set Compare values
    //EPwm1Regs.CMPA.half.CMPA = ps1a*15000; // Set compare A value
   // EPwm1Regs.CMPB = EPWM1_MAX_CMPB; // Set Compare B value

    // Setup counter mode
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm2Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1; // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
//    EPwm2Regs.AQCTLA.bit.ZRO =AQ_SET;
//    EPwm2Regs.AQCTLA.bit.CAU =AQ_CLEAR;
//    EPwm2Regs.AQCTLA.bit.CAD =AQ_SET;
//
//    EPwm2Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
//    EPwm2Regs.AQCTLB.bit.CAU =AQ_SET;
//    EPwm2Regs.AQCTLB.bit.CAD =AQ_CLEAR;

    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = 500*2;
    EPwm2Regs.DBFED = 500*2;

    // Interrupt where we will change the Compare Values
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
    epwm2_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
    epwm2_info.EPwmRegHandle = &EPwm2Regs; // Set the pointer to the ePWM module
    /*epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;*/
}

void InitEPwm3Example()
{
    // Setup TBCLK
    EPwm3Regs.TBPRD = EPWM1_TIMER_TBPRD; // Set timer period 801 TBCLKs
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
    EPwm3Regs.TBCTR = 0x0000; // Clear counter

    // Set Compare values
    //EPwm1Regs.CMPA.half.CMPA = ps1a*15000; // Set compare A value
    //EPwm1Regs.CMPB = EPWM1_MAX_CMPB; // Set Compare B value

    // Setup counter mode
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1; // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
//    EPwm3Regs.AQCTLA.bit.ZRO =AQ_SET;
//    EPwm3Regs.AQCTLA.bit.CAU =AQ_CLEAR;
//    EPwm3Regs.AQCTLA.bit.CAD =AQ_SET;
//
//    EPwm3Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
//    EPwm3Regs.AQCTLB.bit.CAU =AQ_SET;
//    EPwm3Regs.AQCTLB.bit.CAD =AQ_CLEAR;

    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = 500*2;
    EPwm3Regs.DBFED = 500*2;

    // Interrupt where we will change the Compare Values
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
    epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
    epwm3_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
    epwm3_info.EPwmRegHandle = &EPwm3Regs; // Set the pointer to the ePWM module
    /*epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;*/
}

void InitEPwm4Example()
{
    // Setup TBCLK
    EPwm4Regs.TBPRD = EPWM1_TIMER_TBPRD; // Set timer period 801 TBCLKs
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTR = 0x0000; // Clear counter

    // Set Compare values
    //EPwm1Regs.CMPA.half.CMPA = ps1a*15000; // Set compare A value
    //EPwm1Regs.CMPB = EPWM1_MAX_CMPB; // Set Compare B value

    // Setup counter mode
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm4Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1; // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

//    EPwm4Regs.AQCTLA.bit.ZRO =AQ_SET;
//    EPwm4Regs.AQCTLA.bit.CAU =AQ_CLEAR;
//    EPwm4Regs.AQCTLA.bit.CAD =AQ_SET;
//
//    EPwm4Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
//    EPwm4Regs.AQCTLB.bit.CAU =AQ_SET;
//    EPwm4Regs.AQCTLB.bit.CAD =AQ_CLEAR;

    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBRED = 500*2;
    EPwm4Regs.DBFED = 500*2;
    // Interrupt where we will change the Compare Values
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm4_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
     epwm4_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
    epwm4_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
    epwm4_info.EPwmRegHandle = &EPwm4Regs; // Set the pointer to the ePWM module
    /*epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;*/
}
void InitEPwm5Example()
{
    // Setup TBCLK
    EPwm5Regs.TBPRD = EPWM1_TIMER_TBPRD; // Set timer period 801 TBCLKs
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTR = 0x0000; // Clear counter

    // Set Compare values
    //EPwm1Regs.CMPA.half.CMPA = ps1a*15000; // Set compare A value
    //EPwm1Regs.CMPB = EPWM1_MAX_CMPB; // Set Compare B value

    // Setup counter mode
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm5Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1; // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
//    EPwm5Regs.AQCTLA.bit.ZRO =AQ_SET;
//    EPwm5Regs.AQCTLA.bit.CAU =AQ_CLEAR;
//    EPwm5Regs.AQCTLA.bit.CAD =AQ_SET;
//
//    EPwm5Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
//    EPwm5Regs.AQCTLB.bit.CAU =AQ_SET;
//    EPwm5Regs.AQCTLB.bit.CAD =AQ_CLEAR;

    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = 500*2;
    EPwm5Regs.DBFED = 500*2;

    // Interrupt where we will change the Compare Values
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm5Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm5Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm5_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
     epwm5_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
    epwm5_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
    epwm5_info.EPwmRegHandle = &EPwm5Regs; // Set the pointer to the ePWM module
//    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
//    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
//    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
//    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}

void InitEPwm6Example()
{
    // Setup TBCLK
    EPwm6Regs.TBPRD = EPWM1_TIMER_TBPRD; // Set timer period 801 TBCLKs
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
    EPwm6Regs.TBCTR = 0x0000; // Clear counter

    // Set Compare values
    //EPwm1Regs.CMPA.half.CMPA = ps1a*15000; // Set compare A value
    //EPwm1Regs.CMPB = EPWM1_MAX_CMPB; // Set Compare B value

    // Setup counter mode
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
    EPwm6Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1; // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

//    EPwm6Regs.AQCTLA.bit.ZRO =AQ_SET;
//    EPwm6Regs.AQCTLA.bit.CAU =AQ_CLEAR;
//    EPwm6Regs.AQCTLA.bit.CAD =AQ_SET;
//
//    EPwm6Regs.AQCTLB.bit.ZRO =AQ_CLEAR;
//    EPwm6Regs.AQCTLB.bit.CAU =AQ_SET;
//    EPwm6Regs.AQCTLB.bit.CAD =AQ_CLEAR;

    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED = 500*2;
    EPwm6Regs.DBFED = 500*2;
    // Interrupt where we will change the Compare Values
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm6Regs.ETSEL.bit.INTEN = 1; // Enable INT
    EPwm6Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event

    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    epwm6_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
     epwm6_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
    epwm6_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
    epwm6_info.EPwmRegHandle = &EPwm6Regs; // Set the pointer to the ePWM module
//    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
//    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
//    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
//    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}


void clarke(float Va,float Vb,float Vc)
{
V_alpha=(2.0/3)*(Va-Vb*0.5-Vc*0.5);
V_beta=(2.0/3)*(sqrt(3)*0.5*Vb-sqrt(3)*0.5*Vc);
}


//V_REF AND ANGLE CALCULATION
void V_ref_angle(float V_alpha,float V_beta)
{
V_ref=sqrt((V_alpha*V_alpha)+(V_beta*V_beta));
Angle=2*(atan(V_beta/V_alpha));

}


//ANGLE TO SECTOR CALCULATION
void sector(float Angle)
{
if(Angle>=0 && Angle <= PI/3)
{
  n=1;
}
if(Angle >PI/3 && Angle <=(2*PI)/3 )
{
  n=2;
}
if(Angle>(2*PI)/3 && Angle <=PI)
{
  n=3;
}
if(Angle>(-PI) && Angle<=(-2*PI)/3)
{
  n=4;
}
if(Angle>(-2*PI)/3 && Angle<=(-PI)/3)
{
  n=5;
}
if(Angle>(-PI)/3 && Angle<=0)
{
  n=6;
}
}

//phie
void phie(int n,float Angle)
{
delta=Angle;
if (n==1)
  phi=delta;

if (n==2)
  phi=delta-PI/3;

if (n==3)
  phi=delta-2*PI/3;

if (n==6)
  phi=PI/3+delta;

if (n==5)
  phi=2*PI/3+delta;

if (n==4)
  phi=PI+delta;
}

//REGION SELECTION
void region(float V_ref,float phi,float Udc)
{
m0=V_ref/((2.0/3)*Udc);
m1=m0*(2/sqrt(3))*sin(PI/3-phi);
m2=m0*(2/sqrt(3))*sin(phi);
//R=0;

if((m1<0.5)&&(m2<0.5)&&(m1+m2)<0.5)
{
  R=1;
}
if((m1<0.5)&&(m2<0.5)&&(m1+m2)>0.5)
{
  R=2;
}
if((m1>0.5))
{
  R=4;
}
if(m2>0.5)
{
  R=3;
}
}

//DWELL TIMES CALCULATION
void dwell(float m0,int R,float phi,float Ts)
{
k=2/sqrt(3)*m0;
//    ta=0;
//    tb=0;
//    tc=0;

if(R==1)
{
ta=Ts*2*k*sin(PI/3-phi);
tb=(1-2*k*sin(PI/3+phi))*Ts;
tc=2*k*sin(phi)*Ts;
}
if(R==2)
{
ta=(1-2*k*sin(phi))*Ts;
tb=(2*k*sin(PI/3+phi)-1)*Ts;
tc=(1-2*k*sin(PI/3-phi))*Ts;
}
if(R==3)
{
ta=(2*k*sin(phi)-1)*Ts;
tb=2*k*sin(PI/3-phi)*Ts;
tc=(2-2*k*sin(PI/3+phi))*Ts;
}
if(R==4)
{
ta=(2-2*k*sin(PI/3+phi))*Ts;
tb=2*k*sin(phi)*Ts;
tc=(2*k*sin(PI/3-phi)-1)*Ts;
}
}

//SWITCH TIMINGS CALCULATION

//    s1a=0;
//    s2a=0;
//    s1b=0;
//    s2b=0;
//    s1c=0;
//    s2c=0;
void switchtiming(int R,float ta,float tb,float tc,float Ts)
{
if(R==1)
{
s1a=tc/2+ta/2;
s2a=Ts;
s1b=tc/2;
s2b=Ts-ta/2;
s1c=0;
s2c=Ts-ta/2-tc/2;
}
if (R==2)
{
s1a=tc/2+ta/2+tb;
s2a=Ts;
s1b=tc/2;
s2b=Ts-ta/2;
s1c=0;
s2c=ta/2+tc/2;
}
if (R==3)
{
s1a=Ts-tc/2;
s2a=Ts;
s1b=tc/2+ta;
s2b=Ts;
s1c=0;
s2c=tc/2;
}
if (R==4)
{
s1a=Ts-ta/2;
s2a=Ts;
s1b=0;
s2b=ta/2+tb;
s1c=0;
s2c=ta/2;
}
}

//PWM GENERATION

//    ps1a=0;
//    ps2a=0;
//    ps1b=0;
//    ps2b=0;
//    ps1c=0;
//    ps2c=0;
void PWM(int n,float s1a,float s2a,float s1b,float s2b,float s1c,float s2c,float Ts)
{

if (n==1)
{
ps1a=s1a/Ts;
ps2a=s2a/Ts;
ps1b=s1b/Ts;
ps2b=s2b/Ts;
ps1c=s1c/Ts;
ps2c=s2c/Ts;
}
if (n==2)
{
ps1a=(Ts-s2b)/Ts;
ps2a=(Ts-s1b)/Ts;
ps1b=(Ts-s2c)/Ts;
ps2b=(Ts-s1c)/Ts;
ps1c=(Ts-s2a)/Ts;
ps2c=(Ts-s1a)/Ts;
}
if (n==3)
{
ps1a=s1c/Ts;
ps2a=s2c/Ts;
ps1b=s1a/Ts;
ps2b=s2a/Ts;
ps1c=s1b/Ts;
ps2c=s2b/Ts;
}
if (n==4)
{
ps1a=(Ts-s2a)/Ts;
ps2a=(Ts-s1a)/Ts;
ps1b=(Ts-s2b)/Ts;
ps2b=(Ts-s1b)/Ts;
ps1c=(Ts-s2c)/Ts;
ps2c=(Ts-s1c)/Ts;
}
if (n==5)
{
ps1a=s1b/Ts;
ps2a=s2b/Ts;
ps1b=s1c/Ts;
ps2b=s2c/Ts;
ps1c=s1a/Ts;
ps2c=s2a/Ts;
}
if (n==6)
{
ps1a=(Ts-s2c)/Ts;
ps2a=(Ts-s1c)/Ts;
ps1b=(Ts-s2a)/Ts;
ps2b=(Ts-s1a)/Ts;
ps1c=(Ts-s2b)/Ts;
ps2c=(Ts-s1b)/Ts;
}
}

