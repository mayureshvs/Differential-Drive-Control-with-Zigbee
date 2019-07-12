#include "stdio.h"
#include "math.h"
# include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
 // #include "inc/tm4c123gh6pm.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

#include "encoder_init.h"
#include "timer_init.h"
#include "uart_init.h"
#include "send_float.h"
#include "pwm_init.h"




#define WIDTH 12.00                //distance between the two wheels
#define CIRCUM 8.00                //circumference of wheel
#define PPR 400.00                 //pulses per revoluton
#define L(x) ((x*CIRCUM)/PPR)     //expression for distance covered by wheel


float v1,v2,m1=20000,m2=0,e1,e2, r1=40,r2=40,sum1,sum2,t=0.1,kp1=1,ki1=0.000001,kp2=1,ki2=0.000001,d1,d2,dc1,dc2;
char a;

float lL=0,lR=0,dtheta=0,theta=0,disp=0,x[400],y[400];

uint32_t store_ptr=0,read_ptr=0,send_ptr=0;
uint32_t timestamp[400];
encoder_parameters wh1,wh2;
bool state1;

int j[3]={1,1,247};
int i,t1,t2;
char a;
void Tim_ISR(void)
{

    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);

    wh1.pos[store_ptr]=QEIPositionGet(QEI0_BASE);
    wh2.pos[store_ptr]=QEIPositionGet(QEI1_BASE);
    //wh1.direction[store_ptr]=QEIDirectionGet(QEI0_BASE);
   // wh2.direction[store_ptr]=QEIDirectionGet(QEI1_BASE);
    wh1.velocity[store_ptr]=QEIVelocityGet(QEI0_BASE);
    wh2.velocity[store_ptr]=QEIVelocityGet(QEI1_BASE);
    state1=1;
   timestamp[store_ptr]=12500*(store_ptr+1);
    store_ptr++;
    if(store_ptr>300 )
    {
    store_ptr=0;
    }
    QEIPositionSet(QEI1_BASE,0);
    QEIPositionSet(QEI0_BASE,0);
   // GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);


    TimerIntClear(TIMER5_BASE,TIMER_TIMA_TIMEOUT);

}
void PID_ISR(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);

    v1=QEIVelocityGet(QEI0_BASE);
    v2=QEIVelocityGet(QEI1_BASE);
            e1=r1-v1;
    sum1 =sum1+e1;
    d1=(kp1*e1)+(t*ki1*sum1);
    dc1 = (dc1 + d1);
    t1=(dc1/m1)+1;
    e2=r2-v2;
        sum2 =sum2+e2;
        d2=(kp2*e2)+(t*ki2*sum2);
        dc2 = dc2 + d2;
        t2=(dc2/m1)+1;
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,t1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,t2);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,1);


       TimerIntClear(TIMER4_BASE,TIMER_TIMA_TIMEOUT);

}
void Uart_ISR(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);

a=UARTCharGetNonBlocking(UART0_BASE);

UARTCharPut(UART0_BASE,v1);
UARTCharPut(UART0_BASE,v2);
    switch(a)
    {
    case'0':
    {
        while(send_ptr!=read_ptr)
        {

            send_float(x[send_ptr]);
            send_float(y[send_ptr]);

            send_ptr++;
        }
        break;
    }
    case'U':
    {
        r1=15;
        r2=15;
       // m1=1;
        //m2=0;
        break;
    }
    case'L':
    {
        r1=10;
        r2=30;
        //m1=1;
        //m2=0;
        break;
    }
    case'R':
    {
        r1=30;
        r2=10;
        //m1=1;
        //m2=0;
        break;
    }
    case'D':
    {
        r1=0;
        r2=0;
        //m1=0;
        //m2=1;
        break;
    }
        //for(i=0;i<3;i++)
        //UARTCharPutNonBlocking(UART5_BASE,j[i]);
  //  UARTIntClear(UART0_BASE, UART_INT_RX | UART_INT_RT);

    }
    UARTIntClear(UART0_BASE, UART_INT_RX);
}
int main(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);

    pwm_init();
    encoder_init();
    uart_init();
    Timer_init();


    while(1){

        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);
        SysCtlDelay(1000);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0x00);
        SysCtlDelay(1000);




          if(read_ptr != store_ptr)
        {
              GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);
                     SysCtlDelay(1000);
                     GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0x00);
                     SysCtlDelay(1000);

       lL = L(wh1.pos[read_ptr]);
            //lL=;
              lR = L(wh2.pos[read_ptr]);
              //Serial.println(lL);
              dtheta = (lL-lR)/(2*WIDTH);
              theta = theta + dtheta; // update of theta to calculate the x and y displacement

                if(!((abs(lL)>abs(lR))^(dtheta>0)))
                {
                  disp= -abs(lL+lR)/2;    // only magnitude of displacement of center of bot
                }
                else
                {
                  disp= abs(lL+lR)/2;
                }

                /*
              if(theta>6.28)              //reset the theta
               {
                theta = theta- 6.28;
                }
                else if(theta<(-6.28))
                {
                  theta = theta + 6.28;
                  }
*/
              x[read_ptr]=x[read_ptr-1]+(disp*sin(theta));
              y[read_ptr]=y[read_ptr-1]+(disp*cos(theta));
              read_ptr++;
              if(read_ptr>300)
              {
               read_ptr=0;
              }


              state1=0;
        }



          //GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);
                 //SysCtlDelay(1000000);

                // SysCtlDelay(1000000);

    }

    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0xFF);

}
