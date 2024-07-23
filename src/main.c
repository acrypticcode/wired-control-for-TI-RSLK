/* ENGR-2350 Lab 4 Template
//Nicholas Danas and Curran Flanders
 * 662055547 and 662017081
 *
// README!!!!!
// README!!!!!
// README!!!!!
//
// This template project has all initializations required to both control the motors
// via PWM and measure the speed of the motors. The PWM is configured using a 24 kHz
// period (1000 counts). The motors are initialized to be DISABLED and in FORWARD mode.
// The encoders measurements are stored within the variables Tach_R and Tach_L for the
// right and left motors, respectively. A maximum value for Tach_R and Tach_L is
// enforced to be 1e6 such that when the wheel stops, a reasonable value for the
// encoders exists: a very large number that can be assumed to be stopped.
// Finally, a third timer is added to measure a 100 ms period for control system
// timing. The variable run_control is set to 1 each period and then reset in the main.
*/

#include "engr2350_msp432.h"

void GPIOInit();
void TimerInit();
void ADCInit();
void Encoder_ISR();
void T2_100ms_ISR();

Timer_A_UpModeConfig TA0cfg; // PWM timer
Timer_A_UpModeConfig TA2cfg; // 100 ms timer
Timer_A_ContinuousModeConfig TA3cfg; // Encoder timer
Timer_A_CompareModeConfig TA0_ccr3; // PWM Right
Timer_A_CompareModeConfig TA0_ccr4; // PWM Left
Timer_A_CaptureModeConfig TA3_ccr0; // Encoder Right
Timer_A_CaptureModeConfig TA3_ccr1; // Encoder Left


// Encoder total events
uint32_t enc_total_L,enc_total_R;
// Speed measurement variables
// Note that "Tach" stands for "Tachometer," or a device used to measure rotational speed
int32_t Tach_L_count,Tach_L,Tach_L_sum,Tach_L_sum_count,Tach_L_avg; // Left wheel
int32_t Tach_R_count,Tach_R,Tach_R_sum,Tach_R_sum_count,Tach_R_avg; // Right wheel
    // Tach_L,Tach_R are equivalent to enc_counts from Activity 10/Lab 3
    // Tach_L/R_avg is the averaged Tach_L/R value after every 12 encoder measurements
    // The rest are the intermediate variables used to assemble Tach_L/R_avg

uint8_t run_control = 0; // Flag to denote that 100ms has passed and control should be run.
uint16_t adcSpeed;
uint16_t adcTurn;
uint16_t CompareValueLeft = 0;
uint16_t CompareValueRight = 0;
float desiredSpeed = 0;
float desiredLeft = 0;
float desiredRight = 0;
float totalErrorLeft = 0;
float totalErrorRight = 0;
float errorLeft = 0;
float errorRight = 0;
float kI = 0.01;
float measuredSpeedLeft = 0;
float measuredSpeedRight = 0;
float controlling = 0;
float differentialSpeed = 0;
float correctedSpeedRight = 0;
float correctedSpeedLeft = 0;
float vmax = 7.45;
float absDesiredLeftPWM = 0;
float absDesiredRightPWM = 0;
float theta = 0;
float absTheta = 0;
int main(void)
{
    SysInit();
    GPIOInit();
    ADCInit();
    TimerInit();

    __delay_cycles(24e6);

    while(1){

        if(run_control){    // If 100 ms has passed

            run_control = 0;    // Reset the 100 ms flag
            // Control routine ... Explicitly follow pseudocode from Lab document
            // Trigger ADC to read the potentiometers
            ADC14_toggleConversionTrigger();


            while(ADC14_isBusy());
            adcSpeed = ADC14_getResult(ADC_MEM0); // Speed potentiometer result
            adcTurn = ADC14_getResult(ADC_MEM1); //Q above or below is busy?



            // Mapping ADC value to desired speed
            // Assuming the ADC range is 0 to 16383 (14-bit) and the speed is mapped between -50% to +50% PWM duty cycle
            desiredSpeed = ((float)adcSpeed * 100 / 16383) - 50; // Convert to -50 to 50 range
            theta = ((float)adcTurn * 180 / 16383) - 90;
            if (theta < 0){
                absTheta = (-1*theta);
            }
            else{
                absTheta = theta;
            }

            if (absTheta < 15){
                differentialSpeed = 0;
            }
            else{
                differentialSpeed = vmax*(absTheta-15)/75;
            }


            //Starting wheel speed for the left wheel


            if (theta < 0){
                desiredLeft = desiredSpeed - differentialSpeed;
                desiredRight = desiredSpeed + differentialSpeed;
            }else{
                desiredLeft = desiredSpeed + differentialSpeed;
                desiredRight = desiredSpeed - differentialSpeed;
            }


            //Left Motor
            if (desiredLeft < 0){
            absDesiredLeftPWM = (-1*desiredLeft);
            }
            else{
                absDesiredLeftPWM = desiredLeft;
            }

            if (desiredRight < 0){
                absDesiredRightPWM = (-1*desiredRight);
            }
            else{
            absDesiredRightPWM = desiredRight;
                        }

            if (absDesiredLeftPWM  < 5){
                CompareValueLeft = 0;

            }else {

                if (desiredLeft < 0){
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN4);

                }
                else{
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);

                }
                measuredSpeedLeft = 1500000/Tach_L_avg;
                errorLeft = (absDesiredLeftPWM) - measuredSpeedLeft ;    //???
                totalErrorLeft += errorLeft;
                correctedSpeedLeft = (absDesiredLeftPWM) +(kI * totalErrorLeft);//?


                CompareValueLeft = correctedSpeedLeft * 999/100;
                if (CompareValueLeft < 100){
                    CompareValueLeft = 100;

               }else if(CompareValueLeft > 900){
                    CompareValueLeft = 900;
                }

            }

            TA0_ccr4.compareValue = CompareValueLeft;
            Timer_A_initCompare(TIMER_A0_BASE, &TA0_ccr4);



            //Right Motor
            if (absDesiredRightPWM  < 5){
                CompareValueRight = 0;

            }else {


                if (desiredRight < 0){
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN5);

                }
                else{
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);

                }
                measuredSpeedRight = 1500000/Tach_R_avg;
                errorRight = (absDesiredRightPWM) - measuredSpeedRight ;
                totalErrorRight += errorRight;
                correctedSpeedRight = (absDesiredRightPWM) +(kI * totalErrorRight);


                CompareValueRight = correctedSpeedRight * 999/100;
                if (CompareValueRight < 100){
                    CompareValueRight = 100;

                }else if(CompareValueRight > 900){
                    CompareValueRight = 900;
                }

            }

            TA0_ccr3.compareValue = CompareValueRight;
            Timer_A_initCompare(TIMER_A0_BASE, &TA0_ccr3);


            //printf("%1.3f-----------%u\r\n", desiredLeft, CompareValueLeft*100/999);
            //printf("%1.3f-----------%1.3f\r\n", desiredLeft, correctedSpeedLeft);
            //printf("%1.3f \r\n", measuredSpeedLeft);
            printf("%1.3f         %1.3f         %1.3u          %1.3u\r\n", desiredLeft, desiredRight, CompareValueLeft, CompareValueRight);
            //printf("%1.3f           %1.3f           %1.3f\r\n", desiredSpeed, theta, differentialSpeed);

        }
    }
}

void ADCInit(){
    // Add your ADC initialization code here.
    // Assuming 14-bit resolution and 3.3V reference
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);

    // Configure the GPIO pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    // Configure ADC memory for the potentiometer inputs (4.1)
    //ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A12, false);

    // Potentiometer on P4.4
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9, false);
    ADC14_configureMultiSequenceMode(ADC_MEM0,ADC_MEM1, true);//should mem 1 have been added?


    ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    ADC14_enableConversion();
    //  Don't forget the GPIO, either here or in GPIOInit()!!
}

void GPIOInit(){
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motor direction pins
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motor enable pins
        // Motor PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6|GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
        // Motor Encoder pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10,GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motors set to forward
       // Motors are OFF
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
    //while(1);
}

void TimerInit(){
    // Configure PWM timer for 30 kHz
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 999;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA0cfg);
    // Configure TA0.CCR3 for PWM output, Right Motor
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr3);
    // Configure TA0.CCR4 for PWM output, Left Motor
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr4);
    // Configure Encoder timer in continuous mode
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE,&TA3cfg);
    // Configure TA3.CCR0 for Encoder measurement, Right Encoder
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr0);
    // Configure TA3.CCR1 for Encoder measurement, Left Encoder
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr1);
    // Register the Encoder interrupt
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Encoder_ISR);
    // Configure 10 Hz timer
    TA2cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA2cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA2cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA2cfg.timerPeriod = 37499;
    Timer_A_configureUpMode(TIMER_A2_BASE,&TA2cfg);
    Timer_A_registerInterrupt(TIMER_A2_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,T2_100ms_ISR);
    // Start all the timers
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A2_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);
}


void Encoder_ISR(){
    // If encoder timer has overflowed...
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        Tach_R_count += 65536;
        if(Tach_R_count >= 1e6){ // Enforce a maximum count to Tach_R so stopped can be detected
            Tach_R_count = 1e6;
            Tach_R = 1e6;
        }
        Tach_L_count += 65536;
        if(Tach_L_count >= 1e6){ // Enforce a maximum count to Tach_L so stopped can be detected
            Tach_L_count = 1e6;
            Tach_L = 1e6;
        }
    // Otherwise if the Left Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_total_R++;   // Increment the total number of encoder events for the left encoder
        // Calculate and track the encoder count values
        Tach_R = Tach_R_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Tach_R_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        // Sum values for averaging
        Tach_R_sum_count++;
        Tach_R_sum += Tach_R;
        // If 6 values have been received, average them.
        if(Tach_R_sum_count == 12){
            Tach_R_avg = Tach_R_sum/12;
            Tach_R_sum_count = 0;
            Tach_R_sum = 0;
        }
    // Otherwise if the Right Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_total_L++;
        Tach_L = Tach_L_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_sum_count++;
        Tach_L_sum += Tach_L;
        if(Tach_L_sum_count == 12){
            Tach_L_avg = Tach_L_sum/12;
            Tach_L_sum_count = 0;
            Tach_L_sum = 0;
        }
    }
}

void T2_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);
    run_control = 1;
}



