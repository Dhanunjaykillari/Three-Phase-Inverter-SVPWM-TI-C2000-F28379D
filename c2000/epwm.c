#include "F28x_Project.h"
#include <math.h>


#define PI              3.14159265359f       // Pi
#define TWO_PI          6.28318530718f       // 2*pi

#define BUFFER_SIZE     200                   // Data buffer size

#define TBPRD_VALUE     2500                  // ePWM period value
#define PWM_FREQ        20000.0f              // PWM frequency = 20 kHz
#define FUND_FREQ       50.0f                 // Fundamental frequency = 50 Hz

#define Ts              100e-6f               // SVPWM sampling time = 100 us = 10 kHz

#define ADC_VREF        3.3f                  // ADC reference voltage
#define ADC_RESOLUTION  4095.0f               // 12-bit ADC maximum value


volatile float theta = 0.0f;                  // Electrical angle
volatile float M = 0.7f;                      // Modulation index

volatile float duty = 0.5f;                   // Phase-A duty ratio

volatile float vam = 0.5f;                    // Phase-A modulation value
volatile float vbm = 0.5f;                    // Phase-B modulation value
volatile float vcm = 0.5f;                    // Phase-C modulation value


volatile Uint16 vadc = 0;                   // Raw ADC value
volatile Uint16 iadc = 0;                   // Raw ADC value
volatile float vadcVoltage = 0.0f;             // ADC voltage
volatile float iadccurrent = 0.0f;             // ADC current 
volatile Uint16 bufferIndex = 0;              // Buffer index

              // Store theta values
float vamBuffer[BUFFER_SIZE];                 // Store phase-A duty values
float vadcBuffer[BUFFER_SIZE];                 // Store ADC values
float iadcBuffer[BUFFER_SIZE];                 // store the values 

void Setup_ePWM1(void);                       // ePWM1 initialization
void Setup_ADC(void);                         // ADC-B initialization

void SVPWM(void);                             // SVPWM calculation

__interrupt void epwm1ISR(void);              // ePWM1 interrupt ISR


void main(void)
{
    InitSysCtrl();                            // Initialize system clock

    DINT;                                     // Disable CPU interrupts

    InitPieCtrl();                            // Initialize PIE controller

    IER = 0;                                  // Clear CPU interrupt enable register
    IFR = 0;                                  // Clear CPU interrupt flags

    InitPieVectTable();                       // Initialize PIE vector table


    /*=======================================================
     * GPIO CONFIGURATION
     *=======================================================*/

    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;        // Disable GPIO0 pull-up
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;      // Select GPIO mux option
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;       // GPIO0 = EPWM1A

    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;        // Disable GPIO1 pull-up
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;      // Select GPIO mux option
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;       // GPIO1 = EPWM1B

    EDIS;


    /*=======================================================
     * EPWM1 INTERRUPT VECTOR
     *=======================================================*/

    EALLOW;

    PieVectTable.EPWM1_INT = &epwm1ISR;       // EPWM1 interrupt -> epwm1ISR

    EDIS;


    /*=======================================================
     * PERIPHERAL INITIALIZATION
     *=======================================================*/

    Setup_ePWM1();                            // Configure ePWM1

    Setup_ADC();                              // Configure ADC-B


    /*=======================================================
     * EPWM1 INTERRUPT ENABLE
     *
     * EPWM1_INT belongs to:
     *
     * PIE Group 3
     * PIE INTx1
     *=======================================================*/

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;        // Enable EPWM1 interrupt

    IER |= M_INT3;                            // Enable CPU interrupt group 3


    /*=======================================================
     * INITIAL SVPWM
     *=======================================================*/

    SVPWM();                                  // Calculate initial duty

    EPwm1Regs.CMPA.bit.CMPA =
            (Uint16)(duty * TBPRD_VALUE);     // Load initial duty

    EINT;                                     // Enable global interrupts

    ERTM;                                     // Enable real-time debug interrupts


    /*=======================================================
     * MAIN LOOP
     *=======================================================*/

    for(;;)
    {
        asm(" NOP");                          // Main loop does nothing
    }
}


/*=============================================================
 * EPWM1 INTERRUPT SERVICE ROUTINE
 *
 * NOTE:
 *
 * This is now the ONLY interrupt in the system. ADC result
 * reading, buffer storage, SVPWM calculation, and duty
 * (CMPA) update all happen here, synchronized to the ePWM1
 * zero event. The stand-alone ADC-B4 ISR has been removed.
 *=============================================================*/

__interrupt void epwm1ISR(void)
{
    /*---------------------------------------------------------
     * Read ADC-B SOC0 result
     *---------------------------------------------------------*/

    vadc = AdcbResultRegs.ADCRESULT0;       // Read ADC-B channel
    iadc = AdccResultRegs.ADCRESULT1;       // read adc- c channel

    vadcVoltage =((float)vadc / ADC_RESOLUTION) * ADC_VREF;

    iadccurrent = ((float)iadc / ADC_RESOLUTION) * ADC_VREF;

    vadcBuffer[bufferIndex] = vadcVoltage;       // Store ADC voltage
    iadcBuffer[bufferIndex] = iadccurrent;

    /*---------------------------------------------------------
     * SVPWM calculation and duty update
     *---------------------------------------------------------*/

    SVPWM();                                  // Calculate new SVPWM duty

    EPwm1Regs.CMPA.bit.CMPA =
            (Uint16)(duty * TBPRD_VALUE);     // Load new PWM duty

    bufferIndex++;
    if(bufferIndex >= BUFFER_SIZE)
    {
        bufferIndex = 0;                       // Restart buffer
    }

    /*---------------------------------------------------------
     * Clear EPWM1 interrupt flag and acknowledge PIE
     *---------------------------------------------------------*/

    EPwm1Regs.ETCLR.bit.INT = 1;              // Clear EPWM1 interrupt flag

    PieCtrlRegs.PIEACK.bit.ACK3 = 1;          // Acknowledge PIE group 3
}



void Setup_ePWM1(void)
{
    EALLOW;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Select up-down counting mode
    EPwm1Regs.TBPRD = TBPRD_VALUE;                    // Set PWM period
    EPwm1Regs.TBCTR = 0; 
    EPwm1Regs.CMPA.bit.CMPA = 2500;            // Set initial compare value
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Set high-speed clock divider
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;      // Set time-base clock divider
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;         // Set EPWM1A on up-count compare
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;       // Clear EPWM1A on down-count compare
    EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;       // Clear EPWM1B on up-count compare
    EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;         // Set EPWM1B on down-count compare

    /*---------------------------------------------------------
     * ePWM1 SOCA
     *
     * This is still used to TRIGGER ADC-B.
     *---------------------------------------------------------*/

    EPwm1Regs.ETSEL.bit.SOCASEL =ET_CTR_ZERO;                           // SOCA at counter zero
    EPwm1Regs.ETSEL.bit.SOCAEN =1;                                     // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;                                // SOCA every event
                                                    
    /*---------------------------------------------------------
     * ePWM1 interrupt
     *
     * Generates EPWM1_INT at counter zero, used to trigger
     * SVPWM calculation / duty update in epwm1ISR().
     *---------------------------------------------------------*/

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                           // Interrupt at counter zero
    EPwm1Regs.ETSEL.bit.INTEN = 1;                                      // Enable EPWM1 interrupt
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;                                 // Interrupt every event

    EDIS;
}
/*=============================================================
 * ADC-B CONFIGURATION
 *=============================================================*/

void Setup_ADC(void)
{
    EALLOW;
    AdcbRegs.ADCCTL2.bit.PRESCALE =6;                                     // ADC clock prescaler
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS =1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ =1;

    ////////////////////////////////
    AdccRegs.ADCCTL2.bit.PRESCALE =6;                                     // ADC clock prescaler
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdccRegs.ADCCTL1.bit.INTPULSEPOS =1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ =1;
    EDIS;

    DELAY_US(1000);

    EALLOW;

    /*---------------------------------------------------------
     * ADC-B SOC0 configuration
     *---------------------------------------------------------*/

    AdcbRegs.ADCSOC0CTL.bit.CHSEL =4;                                     // ADCINB4
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 15;                                    // Acquisition window
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;                                     // ePWM1 SOCA trigger
    /*---------------------------------------------------------
     * ADC-C SOC1 configuration
     *---------------------------------------------------------*/
    AdccRegs.ADCSOC1CTL.bit.CHSEL =5;                                     // ADCINC5
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 15;                                    // Acquisition window
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5;     

    /*---------------------------------------------------------
     * NOTE: ADCINT4 (ADC-B interrupt) configuration removed.
     *
     * ADC-B is still triggered by ePWM1 SOCA every period and
     * its result registers are read directly in epwm1ISR(),
     * but the ADC no longer generates its own interrupt since
     * only the EPWM1 interrupt is used in this design.
     *---------------------------------------------------------*/

    EDIS;
}



/*=============================================================
 * SVPWM FUNCTION
 *=============================================================*/

void SVPWM(void)
{
    float Va;                                  // Phase-A reference
    float Vb;                                  // Phase-B reference
    float Vc;                                  // Phase-C reference

    float Vmax;                                // Maximum reference
    float Vmin;                                // Minimum reference

    float Voffset;                             // Zero-sequence voltage

    float Va_ref;                              // Offset corrected A
    float Vb_ref;                              // Offset corrected B
    float Vc_ref;                              // Offset corrected C



    Va = M * sinf(theta);                      // Phase-A
    Vb = M * sinf(theta - (2.0f * PI / 3.0f));         // Phase-B
    Vc = M * sinf(theta + (2.0f * PI / 3.0f));         // Phase-C

    /*---------------------------------------------------------
     * Find maximum value
     *---------------------------------------------------------*/

    Vmax = Va;                                // Assume Va maximum
    if(Vb > Vmax)
    {
        Vmax = Vb;                             // Vb is maximum
    }

    if(Vc > Vmax)
    {
        Vmax = Vc;                             // Vc is maximum
    }

    Vmin = Va;                                // Assume Va minimum

    if(Vb < Vmin)
    {
        Vmin = Vb;                             // Vb is minimum
    }

    if(Vc < Vmin)
    {
        Vmin = Vc;                             // Vc is minimum
    }

    Voffset = -0.5f * (Vmax + Vmin);      // offset

    /*---------------------------------------------------------
     * Add zero-sequence component
     *---------------------------------------------------------*/

    Va_ref = Va + Voffset;                     // Corrected A

    Vb_ref = Vb + Voffset;                     // Corrected B

    Vc_ref = Vc + Voffset;                     // Corrected C


    vam = 0.5f * (Va_ref + 1.0f);              // A duty

    vbm = 0.5f * (Vb_ref + 1.0f);              // B duty

    vcm = 0.5f * (Vc_ref + 1.0f);              // C duty


    /*---------------------------------------------------------
     * Limit phase-A duty
     *---------------------------------------------------------*/

    if(vam > 1.0f)
    {
        vam = 1.0f;                            // Maximum duty
    }


    if(vam < 0.0f)
    {
        vam = 0.0f;                            // Minimum duty
    }


    /*---------------------------------------------------------
     * Assign phase-A duty
     *---------------------------------------------------------*/

    duty = vam;


    /*---------------------------------------------------------
     * Store data
     *---------------------------------------------------------*/

         // Store angle

    vamBuffer[bufferIndex] = vam;              // Store duty


    /*---------------------------------------------------------
     * Update electrical angle
     *
     * NOTE:
     *
     * Ts = 100 us
     *
     * Therefore:
     *
     * Sampling frequency = 10 kHz
     *---------------------------------------------------------*/

    theta += TWO_PI * FUND_FREQ * Ts;


    /*---------------------------------------------------------
     * Keep theta between 0 and 2*pi
     *---------------------------------------------------------*/

    if(theta >= TWO_PI)
    {
        theta -= TWO_PI;
    }
}

// end of the  file.
