#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "inc\hw_uart.h"
#include <math.h>

//#include "uart.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"

//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
tI2CMInstance g_sI2CMSimpleInst;

//
//Device frequency
//
int clockFreq;

//
uint32_t g_ui32Flags;
uint8_t cont0 = 0, cont1 = 0;

float fAccel[3], fGyro[3];
tMPU6050 sMPU6050;
float x = 0;
float y = 0, z = 0;
float x2 = 0;
float y2 = 0, z2 = 0;
float gyrox = 0;
float gyroy = 0, gyroz = 0;
float angulo1 = 0;

int lvl = 0;
float kp1 = 0;
float ki1 = 1;
float kd1 = 1;
float kp2 = 5;
float ki2 = 4;
float kd2 = 7;

uint8_t ban = 0;
uint8_t mensaje[];
uint8_t vacio[];
uint32_t ind = 0;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char cThisChar;
    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //

        cThisChar = UARTCharGet(UART0_BASE);
        if (cThisChar == '@') {
            ban = 1;
        //ROM_UARTCharPutNonBlocking(UART0_BASE, cThisChar);
        }
        if (cThisChar == '!') {
            ban = 0;
        }
        if (ban == 1) {
            mensaje[ind] = cThisChar;
            ind++;
        }

        if (ban == 0 && ind >= 1 ) {
            uint8_t* p = mensaje +1;
           // UARTSend((uint8_t *)p, ind-1);
            ind = 0;
            //mensaje = vacio;
        }

    }
}


//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
   // UARTprintf("\n\n adios0 \n");

    // Enciende el LED si cont0 es impar y apaga el LED si cont0 es par.
    if(cont0%2 == 0)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

    // Update the interrupt status on the display.
    IntMasterDisable();
   // UARTprintf("\rT0: %02d,  T1: %02d", cont0, cont1);

    cont0 = (cont0+1)%100;

    IntMasterEnable();
}

void
Timer1IntHandler(void)
{
    //uint8_t C = 1;
    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    x = -(atan2(fAccel[0], sqrt (fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2]))*180.0)/3.14;

    y = (atan2(fAccel[1], sqrt (fAccel[0] * fAccel[0] + fAccel[2] * fAccel[2]))*180.0)/3.14;

    z = (atan2(fAccel[0], fAccel[1])*180.0)/3.14;


    //y = fGyro[1];
    gyrox = (fGyro[1]+0.047)*60;
    x2 += gyrox/100;

    gyroy = (fGyro[0]+0.042)*60;
    y2 += gyroy/100;

    gyroz = (fGyro[2]-0.00)*60;
    z2 += gyroz/100;
   // x = fGyro[0];
   // y = fGyro[1];
   // z = fGyro[2];

    // filtro complementario para el eje x
    angulo1 = 0.98*(angulo1)+0.02*x;
    // verdadero complementario
    x2 = 0.98*x2+0.02*x;
    y2 = 0.98*y2+0.02*y;
    z2 = 0.98*z2+0.02*z;

   // UARTprintf("Ang. X: %d | Ang. Y: %d | Ang. Z: %d\n", (int)x, (int)y, (int)z);
   // UARTprintf("Comp. X: %d | Comp. Y: %d | Comp. Z: %d\n", (int)x2, (int)y2, (int)z2);
  //  UARTprintf("Acc. X: %d | Gyro. X: %d | Complementario: %d\n", (int)x, (int)x2, (int)angulo1);
  //  UARTprintf("Gyro. X: %d | Gyro. X: %d \n", (int)x, (int)x2);
   // delayMS(100);
  //  UARTprintf("$\n");
    UARTprintf("%d&%d&%d&%d&%d.%d&%d.%d&%d.%d\n",(int)x2,(int)y2,(int)z,(int)lvl,(int)kp1,(int)kp2,(int)ki1,(int)ki2,(int)kd1,(int)kd2);
    // Update the interrupt status on the display.
    IntMasterDisable();

    IntMasterEnable();
}

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

void ConfigureUART(void) { // Função retirada do exemplo hello.c
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    IntMasterEnable();
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));

    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void delayMS(int ms) {
    //ROM_SysCtlDelay( (ROM_SysCtlClockGet()/(3*1000))*ms ) ;  // more accurate
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

//
// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // See if an error occurred.
    //
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        //
        // An error occurred, so handle it here if required.
        //
    }
    //
    // Indicate that the MPU6050 transaction has completed.
    //
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

//
// The MPU6050 example.
//
void MPU6050Example(void)
{
/*    float fAccel[3], fGyro[3];
    tMPU6050 sMPU6050;
    float x = 0;
    float y = 0, z = 0;
    float x2 = 0;
    float y2 = 0, z2 = 0;
    float gyrox = 0;
    float gyroy = 0, gyroz = 0;
    float angulo1 = 0;
*/
    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Configure the MPU6050 for +/- 4 g accelerometer range.
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }


    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //

    while (1)
    {
        //
        // Request another reading from the MPU6050.
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1],
            &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        // Do something with the new accelerometer and gyroscope readings.
        //

    }
}

int main()
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
   // instructions to be used within interrupt handlers, but at the expense of
   // extra stack usage.
    FPULazyStackingEnable();
   //ROM_FPULazyStackingEnable();

    //clockFreq = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 16000000);
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    ConfigureUART();
    InitI2C0();
    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF1 & PF2).
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);

    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enable processor interrupts.
    IntMasterEnable();

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    // Esto configura el timer0 a 1 Hz
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    // Esto configura el timer1 a 100 Hz
    TimerLoadSet(TIMER1_BASE, TIMER_A, (uint32_t)(SysCtlClockGet() / 100));

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

    MPU6050Example();
    return(0);
}
