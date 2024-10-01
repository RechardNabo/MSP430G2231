
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>

// Digital Function BIT Operators List
#define SET_BIT(ADDRESS, BIT)                       (ADDRESS) |= (BIT)
#define SET_PIN(PORT, BIT)                          (PORT) |= (BIT)
#define SET_PORT(PORT, BITS)                        (PORT) = (BITS)

#define CLEAR_BIT(ADDRESS, BIT)                     (ADDRESS) &= ~(BIT)
#define CLEAR_PIN(PORT, BIT)                        (PORT) &= ~(BIT)

#define TOGGLE_BIT(ADDRESS, BIT)                    (ADDRESS) ^= (BIT)
#define TOGGLE_PIN(PORT, BIT)                       (PORT) ^= (BIT)
#define TOGGLE_PORT(PORT)                           (PORT) ^= (0xFF)

#define READ_BIT(ADDRESS, BIT)                      (((ADDRESS) >> (BIT)) & 1)
#define READ_PIN(PORT, BIT)                         (PORT & BIT)

#define SET_PIN_DIRECTION_OUTPUT(PORT, BIT)         (SET_BIT(PORT, BIT))    // Set pin as output
#define SET_PIN_DIRECTION_INPUT(PORT, BIT)          (CLEAR_BIT(PORT, BIT))  // Set pin as input
#define SET_PIN_PULL_UP(PORT,BIT)                   (SET_PIN(PORT, BIT))


#define CLEAR_PORT(PORT)                            ((PORT) = (0x00))


#define CHECK_PORT(PORT)                            (PORT) != 0x00)
#define STOP_WATCHDOG_TIMER()                       (WDTCTL = WDTPW | WDTHOLD)
#define LOW_POWER_MODE()                            (__bis_SR_register(LPM4_bits + GIE))
#define GLOBAL_INTERRUPT_ENABLE()                   (__bis_SR_register(LPM0_bits + GIE))
#define PORT1_INPUT                                 (P1IN)
#define PORT1_OUTPUT                                (P1OUT)
#define PORT1_DIRECTION                             (P1DIR)
#define PORT1_FUCTION_SELECT                        (P1SEL)
#define TRUE                                        (0x01)
#define RUN  while(TRUE)
#define LED_01                                      (BIT0)
#define LED_02                                      (BIT6)
#define BUTTON                                      (BIT3)


#define CALBC1_1MHZ_ADDR                            0x10FE  // Address for BCSCTL1 value for 1 MHz
#define CALDCO_1MHZ_ADDR                            0x10FE  // Address for DCOCTL value for 1 MHz (same word as BCSCTL1)

#define CALBC1_8MHZ_ADDR                            0x10FC  // Address for BCSCTL1 value for 8 MHz
#define CALDCO_8MHZ_ADDR                            0x10FC  // Address for DCOCTL value for 8 MHz (same word as BCSCTL1)

#define CALBC1_12MHZ_ADDR                           0x10FA  // Address for BCSCTL1 value for 12 MHz
#define CALDCO_12MHZ_ADDR                           0x10FA  // Address for DCOCTL value for 12 MHz (same word as BCSCTL1)

#define CALBC1_16MHZ_ADDR                           0x10F8  // Address for BCSCTL1 value for 16 MHz
#define CALDCO_16MHZ_ADDR                           0x10F8  // Address for DCOCTL value for 16 MHz (same word as BCSCTL1)

#define DCO_FREQUENCY_0_10MHz                        (0x00)
#define DCO_FREQUENCY_0_50MHz                        (0x01)
#define DCO_FREQUENCY_1_00MHz                        (0x02)
#define DCO_FREQUENCY_2_00MHz                        (0x03)
#define DCO_FREQUENCY_4_00MHz                        (0x04)
#define DCO_FREQUENCY_8_00MHz                        (0x05)
#define DCO_FREQUENCY_12_0MHz                        (0x06)
#define DCO_FREQUENCY_16_0MHz                        (0x07)


#define DCO_1MHz                                    (0x01)
#define DCO_8MHz                                    (0x08)
#define DCO_12MHz                                   (0x0C)
#define DCO_16MHz                                   (0x10)

#define ANALOG_INPUT_A0                             (BIT0)
#define ANALOG_INPUT_A1                             (BIT1)
#define ANALOG_INPUT_A2                             (BIT2)
#define ANALOG_INPUT_A3                             (BIT3)
#define ANALOG_INPUT_A4                             (BIT4)
#define ANALOG_INPUT_A5                             (BIT5)
#define ANALOG_INPUT_A6                             (BIT6)
#define INPUT_CAPTURE_PIN_P1_1                      (BIT1)
#define INPUT_CAPTURE_PIN_P1_2                      (BIT2)
#define CAPTURE_NO_SIGNALS                          (CM_0)
#define CAPTURE_POSITIVE_EDGE                       (CM_1)
#define CAPTURE_NEGATIVE_EDGE                       (CM_2)
#define CAPTURE_POSITIVE_AND_NEGATIVE_EDGES         (CM_3)


#define SDA                                          BIT7  // P1.7 as SDA
#define SCL                                          BIT6  // P1.6 as SCL
#define I2C_DELAY                                   __delay_cycles(10) // Small delay for I2C timing

#define TXD                                         BIT1 // TXD on P1.1
#define RXD                                         BIT2 // RXD on P1.2

#define UART_TBIT_DIV_2                             (1000000 / (9600 * 2))
#define UART_TBIT                                   (1000000 / 9600)

unsigned int txData; // UART internal variable for TX
unsigned char rxBuffer; // Received UART character

//=====================================[FUNCTIONS PROTOTYPES]=======================================
void I2C_INITIALIZE();
void I2C_START();
void I2C_STOP();
unsigned char I2C_WRITE(unsigned char data);
unsigned char I2C_READ(unsigned char ack);
void I2C_SCAN();
void I2C_SEND_DATA(unsigned char slaveAddress, unsigned char data);

//==================================================================================================
volatile unsigned int captured_value = 0;

int DCO_INITIALIZE(unsigned int FREQUENCY) {
    unsigned char calBC1, calDCO;
    unsigned int freq;
    switch (FREQUENCY) {
        case DCO_1MHz:
            calBC1 = *((unsigned char *)CALBC1_1MHZ_ADDR);
            calDCO = *((unsigned char *)CALDCO_1MHZ_ADDR);
            freq = (unsigned int)1000000;
            break;
        case DCO_8MHz:
            calBC1 = *((unsigned char *)CALBC1_8MHZ_ADDR);
            calDCO = *((unsigned char *)CALDCO_8MHZ_ADDR);
            freq = (unsigned int)8000000;
            break;
        case DCO_12MHz:
            calBC1 = *((unsigned char *)CALBC1_12MHZ_ADDR);
            calDCO = *((unsigned char *)CALDCO_12MHZ_ADDR);
            freq = (unsigned int)12000000;
            break;
        case DCO_16MHz:
            calBC1 = *((unsigned char *)CALBC1_16MHZ_ADDR);
            calDCO = *((unsigned char *)CALDCO_16MHZ_ADDR);
            freq = (unsigned int)16000000;
            break;
        default:
            // Set default frequency or handle error
            calBC1 = *((unsigned char *)CALBC1_16MHZ_ADDR); // Default to 16 MHz
            calDCO = *((unsigned char *)CALDCO_16MHZ_ADDR);
            break;
    }

    BCSCTL1 = calBC1;  // Set the BCSCTL1 register
    DCOCTL = calDCO;   // Set the DCOCTL register
    return (freq);
}

void ADC_INITIALIZE(unsigned char PIN) {
    switch(PIN){
        case BIT0:
            ADC10CTL1 = INCH_0;
            break;
        case BIT1:
            ADC10CTL1 = INCH_1;
            break;
        case BIT2:
            ADC10CTL1 = INCH_2;
            break;
        case BIT3:
            ADC10CTL1 = INCH_3;
            break;
        case BIT4:
            ADC10CTL1 = INCH_4;
            break;
        case BIT5:
            ADC10CTL1 = INCH_5;
            break;
        case BIT6:
            ADC10CTL1 = INCH_6;
            break;
        case BIT7:
            ADC10CTL1 = INCH_7;
            break;

    }
    ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON + ENC; // Vcc & Vss as reference, ADC on
    SET_PIN(ADC10AE0, PIN);
}


unsigned int ADC_READ(unsigned char BIT) {
    // Start conversion
    SET_BIT(ADC10CTL0 ,ADC10SC);
    while (ADC10CTL1 & ADC10BUSY);
    return ADC10MEM;                           // Return the ADC value
}

void TIMER0_A_INITIALIZE_STOP_MODE(void) {
    TA0CTL = TASSEL_2 | ID_3 | MC_0 | TACLR; // SMCLK, divide by 8, up mode, clear TAR
    __bis_SR_register(GIE);
}

void TIMER0_A_INITIALIZE_UP_COUNTER_MODE(void) {
    TA0CCTL0 = CCIE;              // Enable interrupt for TimerA_0 CCR0
    TA0CCR0 = 0xFFFF;          // Set CCR0 for 1-second interrupt (16MHz / 1000)
    TA0CTL = TASSEL_2 | ID_3 | MC_1 | TACLR;  // SMCLK, divide by 8, up mode, clear TAR
    __bis_SR_register(GIE);
}

void TIMER0_A_INITIALIZE_CONTINUOUS_COUNTER_MODE(void){
    TA0CCTL0 = CCIE;              // Enable interrupt for TimerA_0 CCR0
    TA0CTL = TASSEL_2 | ID_3| MC_2 | TACLR; // SMCLK, divide by 8, up mode, clear TAR
    __bis_SR_register(GIE);
}

void TIMER0_A_INITIALIZE_DOWN_COUNTER_MODE(void) {
    TA0CCTL0 = CCIE;              // Enable interrupt for TimerA_0 CCR0
    TA0CCR0 = 0xFFFF;          // Set CCR0 for 1-second interrupt (16MHz / 1000)
    TA0CTL = TASSEL_2 | ID_3 | MC_3 | TACLR;  // SMCLK, divide by 8, up mode, clear TAR
    __bis_SR_register(GIE);
}

void TIMER0_A_INITIALIZE_CAPTURE_INPUT_MODE(unsigned int PIN , unsigned int EDGE){
    switch(PIN){
     case BIT1:
         SET_PIN_DIRECTION_INPUT(P1DIR,PIN);
         SET_PIN(P1SEL,PIN);
         TACCTL0 = CCIS_0 + SCS + CAP + CCIE; // Capture on rising edge, CCIxA, synchronous capture, enable capture mode, enable interrupt for P1.1
         switch(EDGE){
             case CM_0:
                 SET_BIT(TACCTL0, EDGE);
                 break;
             case CM_1:
                 SET_BIT(TACCTL0, EDGE);
                 break;
             case CM_2:
                 SET_BIT(TACCTL0, EDGE);
                 break;
             case CM_3:
                 SET_BIT(TACCTL0, EDGE);
                 break;
             default:
                 SET_BIT(TACCTL0,CAPTURE_NO_SIGNALS);
                 break;
         }
         break;
     case BIT2:
         SET_PIN_DIRECTION_INPUT(P1DIR,PIN);
         SET_PIN(P1SEL,PIN);
         TACCTL1 = CCIS_0 + SCS + CAP + CCIE; // Capture on rising edge, CCIxA, synchronous capture, enable capture mode, enable interrupt for P1.2
         switch(EDGE){
             case CM_0:
                 SET_BIT(TACCTL1 , EDGE);
                 break;
             case CM_1:
                 SET_BIT(TACCTL1 , EDGE);
                 break;
             case CM_2:
                 SET_BIT(TACCTL1 , EDGE);
                 break;
             case CM_3:
                 SET_BIT(TACCTL1 , EDGE);
                 break;
             default:
                 SET_BIT(TACCTL1 , CAPTURE_NO_SIGNALS);
                 break;
         }
         break;
    }


    TACTL = TASSEL_2 + MC_2 + TACLR;            // SMCLK, continuous mode, clear TAR
    __bis_SR_register(GIE);     // Enable global interrupts
}

void PWM_INITIALIZE(unsigned int PIN, unsigned int DUTY_CYCLE) {

    SET_PIN_DIRECTION_OUTPUT(P1DIR,PIN);
    SET_PIN_PULL_UP(P1SEL,PIN);

    // Check if Timer_A is already configured
    if ((TACTL & MC_3) == 0) { // If Timer_A is not in use
        // Configure Timer_A
        TACCR0 = 1000 - 1; // PWM Period
        TACCTL1 = OUTMOD_7; // CCR1 reset/set
        TACCR1 = DUTY_CYCLE; // CCR1 PWM duty cycle
        TACTL = TASSEL_2 + MC_1 + TAIE; // SMCLK, up mode
    } else {
        // Timer_A is already configured, adjust only the necessary registers
        TACCTL1 = OUTMOD_7; // CCR1 reset/set
        TACCR1 = DUTY_CYCLE; // CCR1 PWM duty cycle
    }
}

//================================[INTERRUPTERS-VECTORS]===============================


//================================[i2c communications]===============================


void I2C_INITIALIZE(){
    SET_PIN_DIRECTION_OUTPUT(PORT1_OUTPUT, SCL);
    SET_PIN_DIRECTION_OUTPUT(PORT1_OUTPUT ,SDA);
    SET_PIN(PORT1_OUTPUT, SCL);
    SET_PIN(PORT1_OUTPUT,SDA);
}

void I2C_START(){
    SET_PIN(PORT1_OUTPUT, SDA);
    SET_PIN(PORT1_OUTPUT,SCL);
    I2C_DELAY;
    CLEAR_PIN(PORT1_OUTPUT, SDA);
    I2C_DELAY;
    CLEAR_PIN(PORT1_OUTPUT, SCL);
    I2C_DELAY;
}

void I2C_STOP(){
    CLEAR_PIN(P1OUT,SDA);
    I2C_DELAY;
    SET_PIN(P1OUT, SCL);
    I2C_DELAY;
    SET_PIN(P1OUT, SDA);
    I2C_DELAY;
}

unsigned char I2C_WRITE(unsigned char data)
{
    unsigned char i;

    for (i = 0; i < 8; i++){
        if (data & 0x80)
            SET_PIN(P1OUT , SDA);
        else
            CLEAR_PIN(P1OUT, SDA);
        data <<= 1;

        SET_PIN(P1OUT, SCL);
        I2C_DELAY;
        CLEAR_PIN(P1OUT, SCL);
    }

    // Check for ACK
    SET_PIN_DIRECTION_INPUT(P1DIR, SDA);
    SET_PIN(P1OUT, SCL);
    I2C_DELAY;

    unsigned char ack =READ_PIN(P1IN, SDA); // Read ACK
    CLEAR_PIN(P1OUT, SCL);
    SET_PIN_DIRECTION_OUTPUT(P1DIR, SDA);
    return ack;
}

unsigned char I2C_READ(unsigned char ack){
    unsigned char data = 0;
    unsigned char i;
    SET_PIN_DIRECTION_INPUT(P1DIR, SDA);

    for (i = 0; i < 8; i++) {
        SET_PIN(P1OUT, SCL);
        I2C_DELAY;
        data = (data << 1) | READ_PIN(P1IN , SDA) ? 1 : 0;
        CLEAR_PIN(P1OUT, SCL);
        I2C_DELAY;
    }

    // Send ACK or NACK
    SET_PIN_DIRECTION_OUTPUT(P1DIR,SDA);
    if (ack)
        CLEAR_PIN(P1OUT, SDA);
    else
        SET_PIN(P1OUT, SDA);

    SET_PIN(P1OUT, SCL);
    I2C_DELAY;
    CLEAR_PIN(P1OUT, SCL);
    CLEAR_PIN(P1DIR,SDA);

    return data;
}

void I2C_SCAN(){
    unsigned char address, ack;
    for (address = 0x00; address < 0x80; address++){
        I2C_START();
        ack = I2C_WRITE((address << 1) | 0); // Send address + Write bit
        if (ack == 0) {
            // Indicate that a device was found at the current address
            // Toggle LED on P1.0 to show device found
        }
        I2C_STOP();
    }
}

void I2C_SEND_DATA(unsigned char slaveAddress, unsigned char data){
    I2C_START();
    // Send slave address with Write bit (0)
    if (I2C_WRITE((slaveAddress << 1) | 0) == 0){
        I2C_WRITE(data); // Send the data byte
    }
    I2C_STOP();
}

void I2C_SendString(unsigned char slaveAddress, const char *str) {
    I2C_START();
    // Send slave address with Write bit (0)
    if (I2C_WRITE((slaveAddress << 1) | 0) == 0) {  // If ACK received
        while (*str) {
            I2C_WRITE(*str++);  // Send each character
        }
    }
    I2C_STOP();
}

//===========================================[SPI COMMUNICATION]=============================
void SPI_INITIALIZE() {
    // Set USI in SPI master mode
    USICTL0 = USISWRST; // Put USI in reset state
    USICTL0 |= USIPE5 | USIPE6 | USIPE7 | USIMST | USIOE; // Enable SPI pins and set as master
    USICTL1 = USICKPH; // Data is captured on the first edge
    USICKCTL = USIDIV_4 | USISSEL_2; // Clock source: SMCLK/16
    USICTL0 &= ~USISWRST; // Release USI for operation
    USICNT = 8; // Load counter with 8 bits
}

void SPI_SEND_DATA(unsigned char data) {
    USISRL = data; // Load data to be sent
    USICNT = 8; // Load counter with 8 bits
    while (!(USICTL1 & USIIFG)); // Wait for transmission to complete
    USICTL1 &= ~USIIFG; // Clear interrupt flag
}

unsigned char SPI_RECEIVE_DATA(void) {
    USICNT = 8; // Load counter with 8 bits (expecting to receive 8 bits)
    while (!(USICTL1 & USIIFG)); // Wait for reception to complete
    USICTL1 &= ~USIIFG; // Clear interrupt flag
    return USISRL; // Return the received data
}


void SPI_SEND_STRING(const char *str) {
    while (*str) { // Iterate through each character in the string until the null terminator
        SPI_SEND_DATA((unsigned char)*str); // Send each character
        str++; // Move to the next character
    }
}

//=========================================[INTERRUPTERS]==========================================


// Timer_A0 interrupt service routine for P1.1
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0(void) {
    //TOGGLE_PIN(P1OUT, BIT7);
    CLEAR_BIT(TACCTL0 ,CCIFG);
}

// Timer_A1 interrupt service routine for P1.2
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_A1(void) {
    if (TAIV == TA0IV_TACCR1) { // Check if the interrupt is from TACCR1
        //TOGGLE_PIN(P1OUT, BIT7);
        CLEAR_BIT(TACCTL1,CCIFG );
    }

}

