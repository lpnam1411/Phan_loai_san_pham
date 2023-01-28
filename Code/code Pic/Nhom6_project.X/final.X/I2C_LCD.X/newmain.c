#include <xc.h>
#include "config.h"
#include "I2C_LCD.h"

/*------------------------------------------------[ define ]-----------------------------------------------------*/
// servo
#define WritePosOpenServo RB6
#define WritePosOpenServo1 RB5

// Motor DC
#define IN1 RC2
#define IN2 RB1
#define ENA RB2

// Bien tro//
#define POT 0 // RA0

// PWM
#define Fpwm 3950 // tan so bam xung
#define ps 1      // prescaler = 1:1

/*------------------------------------------------[ khai bao ham ]-----------------------------------------------*/
// servo
void Servo_Init(); // 20M->5M->2.5M-> 2500 trong 1ms -> 250 0.1ms
void Servo_Init(); // 20M->5M->2.5M-> 2500 trong 1ms -> 250 0.1ms

// LCD
void LCD_Write(uint8_t count_sp, uint8_t x, uint8_t y);

// PWM
void ADC_Init();                        // config chan cho ADC
uint16_t ADC_Read(uint8_t ADC_channel); // doc du lieu ADC tu kenh
void init_PWM();                        // config chan PWM
void setPWM_DutyCycle(uint16_t DC);

// IR_Sensor
void IR_Init();

// UART
void UART_send_char(char bt);
void UART_send_string(char *st_pt);
char UART_get_char();
void config_UART();

// EEPROM
uint8_t EEPROM_Read(uint8_t Address);
void EEPROM_Write(uint8_t Address, uint8_t Data);

/*------------------------------------------------[ khai bien global ]-------------------------------------------*/
// Servo
uint8_t OpenServo = 0;
uint8_t SelectServo = 0;
uint8_t cnt_pos = 0; // dem timer 0
uint16_t cnt_time_delay = 0;
uint8_t pos = 2;         // vi tri dong mo servo
uint16_t time_delay = 2; // vi tri dong mo servo

// LCD
uint8_t LCD_SWAP = 1;

// Bien tro Ngat dong co -> quet QR
uint8_t flag_dc = 1;

// UART
uint16_t B_rate = 9600;
uint8_t flag_trans = 0;
char result = '\0';

// EPROM
uint8_t count_sp1;
uint8_t count_sp2;
uint8_t count_sp3;

/*------------------------------------------------[ MAIN ]-------------------------------------------------------*/
void main(void)
{
    /*---------------------[ SETUP ]-------------------------------*/
    // I2C
    I2C_Master_Init();

    // LCD
    LCD_Init(0x4E); // dia chi i2c 0x4E
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("    VI XU LY    ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("   TAN MAI NAM  ");
    LCD_Clear();

    // servo
    Servo_Init();

    // PWM_Motor_DC
    ADC_Init();
    init_PWM();

    // IR_Sensor
    IR_Init();

    // UART
    config_UART();

    /*---------------------[ khai bao bien local ]------------------*/

    /*---------------------[ khaibao chan I/O ]---------------------*/
    // servo
    TRISB6 = 0; // cap xung pos servo1   - OUTPUT
    TRISB5 = 0; // cap xung pos servo2   - OUTPUT
    RB6 = 1;
    RB5 = 1;

    // PWM_Motor_DC 0 la out , 1 la in
    TRISA0 = 1; // chan doc ADC tu bien tro
    TRISB1 = 0; // chan dat dc
    TRISB2 = 0; // chan d/c enable on off
    IN2 = 0;

    // IR_Sensor
    TRISB0 = 1; // chan ngat

    // EEPROM
    uint8_t tmp;
    count_sp1 = EEPROM_Read(0x00); // doc gia tri tu eeprom
    count_sp2 = EEPROM_Read(0x01);
    count_sp3 = EEPROM_Read(0x02);

    /*---------------------[ LOOP ]---------------------------------*/
    while (1)
    {
        // LCD
        if (LCD_SWAP == 1)
        {
            LCD_Clear();
            LCD_Set_Cursor(1, 1);
            LCD_Write_String(" DFS  I2C  CRC ");
            //                 234  789  234

            LCD_Write(count_sp1, 2, 2);
            LCD_Write(count_sp2, 2, 7);
            LCD_Write(count_sp3, 2, 12);

            LCD_SWAP = 0;
        }

        // servo
        if (OpenServo == 1)
        {
            pos = 8; // CLOSE
        }
        else
        {
            pos = 14; // OPEN
        }

        // PWM_Motor_DC
        setPWM_DutyCycle(ADC_Read(POT));

        // Ngat dong co -> quet QR
        ENA = flag_dc;
    }
    return;
}

/*------------------------------------------------[ NGAT ]-------------------------------------------------------*/
void __interrupt() isr1()
{
    // Servo
    if (T0IF == 1) // phat hien co bao ngat
    {
        TMR0 = 5;
        T0IF = 0; // dua co bao ngat ve gia tri 0
        cnt_pos++;

        if (cnt_pos == pos) // khi timer dem du time
        {
            if (SelectServo == 0)
            {
                WritePosOpenServo = ~WritePosOpenServo;
            }
            else
            {
                WritePosOpenServo1 = ~WritePosOpenServo1;
            }
        }

        if (cnt_pos == 200 - pos) // khi timer dem du time 200 cnt = 20ms
        {
            if (SelectServo == 0)
            {
                WritePosOpenServo = ~WritePosOpenServo;
            }
            else
            {
                WritePosOpenServo1 = ~WritePosOpenServo1;
            }
            cnt_pos = 0; // reset gia tri cnt
        }
    }

    // PWM
    if (INTF == 1)
    { // Check the flag bit neu co ngat thi dung dong co
        OpenServo = 0;
        setPWM_DutyCycle(0);
        flag_dc = 0;
        UART_send_string("check\n");
        INTF = 0;
    }

    // UART receive
    if (RCIF == 1)
    { //

        result = RCREG;
        if (result == 'a')
        {
            if (count_sp1 == 255)
            {
                count_sp1 = 0;
            }
            count_sp1++;
            EEPROM_Write(0x00, count_sp1);

            SelectServo = 0;
            OpenServo = 1;
        }
        else if (result == 'b')
        {
            if (count_sp2 == 255)
            {
                count_sp2 = 0;
            }
            count_sp2++;
            EEPROM_Write(0x01, count_sp2);
            SelectServo = 1;
            OpenServo = 1;
        }
        else if (result == 'c')
        {
            if (count_sp3 == 255)
            {
                count_sp3 = 0;
            }
            count_sp3++;
            EEPROM_Write(0x02, count_sp3);
            OpenServo = 0;
        }
        flag_dc = 1;
        setPWM_DutyCycle(ADC_Read(0)); // chay dong co
        LCD_SWAP = 1;
        result = '\0'; // khoi tao lai result

        RCIF = 0;
    }
}

/*------------------------------------------------[ detail function ]---------------------------------------------*/
//--------------------[ Servo ]--------------------
void Servo_Init()
{
    // config timer 0
    PSA = 0; // chon bo chia truoc cho timer 0

    PS2 = 0;
    PS1 = 0;
    PS0 = 0; // chon bo chia truoc 2

    T0CS = 0; // chon nguon xung clock noi

    GIE = 1;  // cho phep ngat
    T0IE = 1; // cho phep ngat timer 0
    T0IF = 0; // ghi gia tri co ngat = 0
}

// LCD
void LCD_Write(uint8_t count_sp, uint8_t x, uint8_t y)
{
    char c;
    uint8_t y_ = y + 2;
    uint8_t i;
    for (i = 2; i >= 0; i--)
    {
        c = count_sp % 10 + 48;

        LCD_Set_Cursor(x, y_);
        LCD_Write_Char(c);
        y_--;

        count_sp /= 10;
        if (count_sp <= 0)
        {
            break;
        }
    }

    while (y_ >= y)
    {
        LCD_Write_Char(' ');
        y_--;
    }
}

// ADC
void ADC_Init()
{
    //------[There are 2 registers to configure ADCON0 and ADCON1]---------
    // ADCON0 = 0x41
    // Select clock option Fosc/8
    ADCS0 = 1;
    ADCS1 = 0;
    // Turn ADC on
    ADON = 1;

    // ADCON1 = 0x80
    //  Result mode: Right justified
    ADFM = 1;
    // Select clock option Fosc/8
    ADCS2 = 1;
    // Configure all 8 channels are analog
    PCFG0 = 0;
    PCFG1 = 0;
    PCFG2 = 0;
    PCFG3 = 0;
}

uint16_t ADC_Read(uint8_t ADC_channel)
{
    // Check channel number
    if (ADC_channel < 0 || ADC_channel > 7)
        return 0;

    // Write ADC__channel into register ADCON0
    CHS0 = (ADC_channel & 1) >> 0;
    CHS1 = (ADC_channel & 2) >> 1;
    CHS2 = (ADC_channel & 4) >> 2;

    // Wait the Acquisition time
    __delay_us(25);

    // Start A/D conversion
    GO_DONE = 1;

    // (Polling) Wait for the conversion to complete
    while (GO_DONE)
        ;

    // Read the ADC result ("right justified" mode)
    uint16_t result = ((ADRESH << 8) + ADRESL);
    return result;
}

// PWM
void init_PWM()
{
    // Configure the CCP1 module for PWM operation
    CCP1M2 = 1;
    CCP1M3 = 1;
    // Set CCP1 pin as output
    TRISC2 = 0;
    // Set the Timer2 prescaler value and enable Timer2
    switch (ps)
    {
    case 1:
    {
        T2CKPS0 = 0;
        T2CKPS1 = 0;
        break;
    }
    case 4:
    {
        T2CKPS0 = 1;
        T2CKPS1 = 0;
        break;
    }
    case 16:
        T2CKPS1 = 1;
    }
    TMR2ON = 1;
    // Set the PWM period
    PR2 = ((float)(_XTAL_FREQ / Fpwm)) / (4 * ps) - 1;
    // --------[Warning: Check if PR2 value fits in 8-bit register (0-255)]---------]
}

void setPWM_DutyCycle(uint16_t DC)
{
    // DC means % (Ex: DC = 50 means Duty Cycle = 50%)
    // uint16_t val = ((float)_XTAL_FREQ/(float)Fpwm)*((float)DC/(float)100)/ps;
    // Write to CCP1CON<5:4>
    CCP1Y = DC & (1 << 0);
    CCP1X = DC & (1 << 1);
    // Write to CCPR1L register
    CCPR1L = DC >> 2;
}

// IR_Sensor
void IR_Init()
{
    INTEDG = 0; // Interrupt edge config bit (HIGH value means interrupt occurs every rising edge)
    INTE = 1;   // IRQ (Interrupt request pin RB0) Interrupt Enable bit
    GIE = 1;
}

// UART
void UART_send_char(char bt)
{
    while (!TRMT)
        ;       // hold the program till TX buffer is free
    TXREG = bt; // Load the transmitter buffer with the received value
}

void UART_send_string(char *st_pt)
{
    while (*st_pt)                // if there is a char
        UART_send_char(*st_pt++); // process it as a byte data
}

void config_UART()
{
    // Baud rate configuration
    BRGH = 1;    // highspeed baundrate
    SPBRG = 129; //( ( _XTAL_FREQ/16 ) / B_rate) - 1 ;
    // Enable Asynchronous Serial Port
    SYNC = 0;
    SPEN = 1;
    // Configure Rx-Tx pin for UART
    TRISC6 = 1;
    TRISC7 = 1;
    // Enable UART Transmission
    TXEN = 1;
    CREN = 1;
    // select 8-bit mode
    // Enable Interrupt Rx
    RCIE = 1;
    PEIE = 1;
    GIE = 1;

    TX9 = 0;
    RX9 = 0;
}

// EEPROM
uint8_t EEPROM_Read(uint8_t Address)
{
    EEADR = Address;
    EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}
void EEPROM_Write(uint8_t Address, uint8_t Data)
{
    while (WR)
        ; // cho chuong trinh nap truoc do neu co
    EEADR = Address;
    EEDATA = Data;
    EEPGD = 0;
    EECON1bits.WREN = 1;
    GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    GIE = 1;
    EECON1bits.WREN = 0;
    EECON1bits.WR = 0;
}