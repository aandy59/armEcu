#include "delay10us.h"



#define DelayCOUNT	20

static struct
            {
                TaskHandle_t        Message;
                uint32_t            Time;
			}
            delays100us[DelayCOUNT];

			
void TIM3_IRQHandler()
{
    if(TIM3->SR & TIM_SR_UIF)           // Проверяем, что это нас переполнение вызывало
    {
        TIM3->SR &= ~TIM_SR_UIF;        //сбросить флаг


        uint8_t index;
        BaseType_t xHigherPriorityTaskWoken = pdTRUE;



        TimeMs++;


        for(index=0; index < DelayCOUNT ;index++)
        {
            if(delays100us[index].Message == NULL) continue;

            if(delays100us[index].Time != 0)
            {
                delays100us[index].Time --;
            }
            else
            {
                vTaskNotifyGiveFromISR(delays100us[index].Message,&xHigherPriorityTaskWoken);
                delays100us[index].Message = NULL;
								portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}



uint32_t TimerSet(const uint32_t AddTimeMs)
{
    int32_t TimeMs_;

    taskENTER_CRITICAL();
    TimeMs_ = TimeMs;
    taskEXIT_CRITICAL();

    return TimeMs_ + AddTimeMs;
}


bool TimerIsExpired(const uint32_t Timer)
{
    int32_t TimeMs_;

    taskENTER_CRITICAL();
    TimeMs_ = TimeMs;
    taskEXIT_CRITICAL();


    return ((TimeMs_ - Timer) < (1UL << 31));
}


uint32_t TimerRemainingMs(const uint32_t Timer)
{
    int32_t TimeMs_;

    taskENTER_CRITICAL();
    TimeMs_ = TimeMs;
    taskEXIT_CRITICAL();


    if ((TimeMs_ - Timer) > (1UL << 31))
    {
        return (Timer - TimeMs_);
    }
    else
    {
        return 0;
    }
}




uint32_t TimerPassMs(const uint32_t Timer)
{
    int32_t TimeMs_;

    taskENTER_CRITICAL();
    TimeMs_ = TimeMs;
    taskEXIT_CRITICAL();

    return (TimeMs_ - Timer);
}




void Delay_100usInit(void)
{

// Flush all Delays
    uint8_t index;
    for(index=0; index < DelayCOUNT; index++)
    {
        delays100us[index].Message = NULL;
        delays100us[index].Time = 0;
    }

}



bool Delay_100us(uint16_t Time)

{
uint8_t		index = 0;
bool        out=false;
TaskHandle_t xMessage;

xMessage = xTaskGetCurrentTaskHandle();
    taskENTER_CRITICAL();
    for(index=0; index < DelayCOUNT; ++index)
        {
        if (delays100us[index].Message == NULL)
            {
            delays100us[index].Message = xMessage;
            delays100us[index].Time = Time;
            out = true;
            break;
            }
        }
    taskEXIT_CRITICAL();


    if(out)
    {
				return ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    else
    {
        return out;
    }
}
