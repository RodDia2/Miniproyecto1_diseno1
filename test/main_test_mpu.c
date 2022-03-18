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
#include "driverlib/adc.h"
#include "driverlib/ssi.h" //del spi

#include "inc\hw_uart.h"
#include <math.h>

//#include "uart.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
//******************************************************************************************************
#define NUM_SPI_DATA    1  // Número de palabras que se envían cada vez
#define SPI_FREC  4000000  // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16  // Número de bits que se envían cada vez, entre 4 y 16

uint16_t dato;  // Para lo que se envía por SPI.
uint16_t valor;
// se crean variables de tipo float para que puedan tener decimales
float v0, v1;   // para ilustrar el uso de variables tipo float
float e_k_1;
float E_k;
float e_k;
float eD;
float u_k;
float kP;
float kI;
float kD;

float u_k2;

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
uint16_t kp1 = 1;
uint16_t ki1 = 0;
uint16_t kd1 = 0;
uint16_t kp2 = 0;
uint16_t ki2 = 0;
uint16_t kd2 = 0;
float kp;
float ki;
float kd;

uint8_t ban = 0;
uint8_t mensaje[];
uint8_t mensaje2[];
uint8_t vacio[];
uint32_t ind = 0;
uint8_t ban1 = 0;
uint8_t ban2 = 0;
uint8_t ban3 = 0;
uint8_t ban4 = 0;
uint8_t length = 0;
uint16_t length2 = 0;
int mensajes = 0;

char buf[8];

uint8_t c = 0;
char ceros[4];


uint16_t lengthkp2 = 10;
uint16_t lengthki2 = 10;
uint16_t lengthkd2 = 10;

int abc = 0;

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
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
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
            if (ind >= 1) {
                mensaje[ind] = cThisChar;
                if (ind >= 2) {
                    mensaje2[ind-1]=cThisChar;
                }
            }
            ind++;
        }

        if (ban == 0 && ind >= 1 ) {
            mensaje[ind] = '_';
            int i = 0;
            c = 0;
            for (i=2;i<ind;i++) {
                if (mensaje[i] == '0') {
                    ceros[i-2] = '0';
                    c = c + 1;
                }
            }
            uint8_t* p = mensaje +2;
            mensajes = 0;
            mensajes = atoi(p);
            //length = sizeof(p)/sizeof(p[1]);
            length = ind-2;
            length2 = pow(10,length);
            //  mensaje2 = (char*)mensaje;
            //UARTSend((uint8_t *)p, ind-2);
            //UARTprintf("%d\n",mensajes);
            if (mensaje[1] == 'a') {
                ban1 = 1;
            }
            else if (mensaje[1] == 'b') {
                ban1 = 2;
            }
            else if (mensaje[1] == 'c') {
                ban1 = 3;
            }
            else if (mensaje[1] == 'd') {
                ban1 = 4;
            }
            else if (mensaje[1] == 'e') {
                ban1 = 5;
            }
            else if (mensaje[1] == 'f') {
                ban1 = 6;
            }
            else if (mensaje[1] == 'g') {
                ban1 = 7;
            }
            else {
                ban1 = 0;
            }
            //UARTSend((uint8_t *)mensaje, ind+1);
            ind = 0;
           // mensaje[1] = 0;
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
    // Notar que ahora necesitamos dos espacios para las conversiones.
    //uint32_t pui32ADC0Value[2];

    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;

    // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    /*
    // Trigger the ADC conversion.
    ADCProcessorTrigger(ADC0_BASE, 2);  // Notar el cambio de "secuencia" (2 en lugar de 3).

    // Wait for conversion to be completed.
    while(!ADCIntStatus(ADC0_BASE, 2, false))  // Notar el cambio de "secuencia".
    {
    }

    // Clear the ADC interrupt flag.
    ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // Ahora se leen dos valores, no sólo uno, según la configuración.
    // En pui32ADC0Value[0] se tendrá el valor del puerto AIN0 y
    // en pui32ADC0Value[1] se tendrá el valor del puerto AIN1, porque así se
    // configuró en el main.
    ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Value);  // Notar el cambio de "secuencia".

    v0 = pui32ADC0Value[0]*3.3/4095;  // Convertir a voltios
    v1 = pui32ADC0Value[1]*3.3/4095;  // Convertir a voltios

    // Se muestra el valor convertido de AIN0 (PE3) y AIN1 (PE2), que son enteros
    // entre 0 y 4095, y los valores en voltios, entre 0.00 y 3.30.
    // Nota: UARTprintf no soporta floats (%f). Acá se hace un truco para mostrar
    // la parte entera y dos posiciones decimales, como enteros.
    // Notar que el "casting" trunca, no redondea.
    // Si no es indispensable desplegar floats, mejor evitar hacerlo.


    UARTprintf("A0: %04d, v0: %d.%02d,   A1: %04d, v1: %d.%02d\n",
               pui32ADC0Value[0], (uint8_t)v0, (uint8_t)(100*(v0-(uint8_t)v0)),
               pui32ADC0Value[1], (uint8_t)v1, (uint8_t)(100*(v1-(uint8_t)v1)));
    */

    /* v0 es la referencia que viene del generador de funciones AIN0, PE3
     * v1 es la salida de la planta AIN1, PE2
     * Las demás constantes siguen el modelo planteado en la lecture 8
     */

    v0 = 0;
    v1 = x2;

    e_k = v0 - v1;
    eD = e_k - e_k_1;
    E_k = E_k + e_k;
    u_k = kp*e_k + ki*E_k + kd*eD;
    e_k_1 = e_k;

 //  se revisa si no existe un overflow, y se corrige si existe

    if (u_k>6)
        u_k = 6;
    else if (u_k<-6)
        u_k = -6;

    // se realiza el mapeo  de -6 a 6 -> 0-4095
    u_k2 = u_k*341.25 + 2047.5;

// se convirte el dato a int para ser mandado por spi
    valor = (uint16_t) (u_k2);
    // se realiza un or para tener los 4 bits de configuración y 12 de dato
    dato = 0b0111000000000000 | valor;
    // Colocar el dato de SPI_ANCHO bits en pui32DataTx
    pui32DataTx[0] = (uint32_t)(dato);

    // Send data
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    while(SSIBusy(SSI0_BASE))
    {
    }
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
    //sprintf(buf,"%g",kp);
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
    abc = 2;
}

void ConfigureUART(void) { // Função retirada do exemplo hello.c
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    IntMasterEnable();
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    /*UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));
    */
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
    abc = 1;
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
        switch(ban1){
            case 1:
                ban2 = 1;
                /*
                if ((int)mensajes>(int)0) {
                    lvl = -mensajes;
                } else {
                    lvl = -mensajes;
                }
                */
                lvl = mensajes;
                break;
            case 2:
                ban2 = 2;
                kp1 = mensajes;
                break;
            case 3:
                ban2 = 3;
                kp2 = mensajes;
                lengthkp2 = length2;
                break;
            case 4:
                ban2 = 4;
                ki1 = mensajes;
                break;
            case 5:
                ban2 = 4;
                ki2 = mensajes;
                lengthki2 = length2;
                break;
            case 6:
                ban2 = 4;
                kd1 = mensajes;
                break;
            case 7:
                ban2 = 4;
                kd2 = mensajes;
                lengthkd2 = length2;
                break;
            default:
                ban2 = 0;
                break;
            }
            kp = (float)kp2/(lengthkp2)+(float)kp1;
            ki = (float)ki2/(lengthki2)+(float)ki1;
            kd = (float)kd2/(lengthkd2)+(float)kd1;

    }
}

int main()
{
    uint32_t pui32residual[NUM_SPI_DATA];
    uint16_t freq_muestreo = 10000;    // En Hz
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
   // instructions to be used within interrupt handlers, but at the expense of
   // extra stack usage.
    FPULazyStackingEnable();
   //ROM_FPULazyStackingEnable();

    //clockFreq = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 16000000);
    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ); // 80 MHz

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
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet() / 10000));
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
    //abc = 4;
    //******************************************************************************
    //uint32_t pui32residual[NUM_SPI_DATA];
    //uint16_t freq_muestreo = 10000;    // En Hz

    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // For this example ADC0 is used with AIN0 and AIN1.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.  GPIO port E needs to be enabled
    // so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);  // Configura el pin PE3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);  // Configura el pin PE2

    // Se configura la secuencia 2, que permitiría hasta cuatro muestras (aunque
    // se usarán dos).
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

    // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);

    // Step 1 en la secuencia 2: Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
    // y configura la bandera de interrupción (ADC_CTL_IE) para "setearse"
    // cuando se tenga esta muestra. También se indica que esta es la última
    // conversión en la secuencia 2 (ADC_CTL_END).
    // Para más detalles del módulo ADC, consultar el datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

    // Since sample sequence 2 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    ADCIntClear(ADC0_BASE, 2);  // Notar el cambio de "secuencia".

    // ---------------------------------------------------------
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 4MHz SSI frequency, and 16-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    abc = 4;
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
    abc = 5;
    // Enable the SSI0 module.
    SSIEnable(SSI0_BASE);
    abc = 6;
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32residual[0]))
    {
    }
    abc = 3;
    // se estalecen los valores iniciales
    dato = 0b0111000000000000;
    valor = 0b0000000000000000;

    e_k_1 = 0;
    E_k = 0;

    MPU6050Example();
    return(0);
}
