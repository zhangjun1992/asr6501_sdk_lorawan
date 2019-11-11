/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "hw.h"
#include "low_power.h"
#include "linkwan_ica_at.h"
#include "linkwan.h"
#include "timeServer.h"
//#include "version.h"
#include "radio.h"
//#include "sx1276Regs-Fsk.h"
//#include "sx1276Regs-LoRa.h"
#include "board_test.h"
#include "delay.h"
#include "board.h"

#include "LoRaMac.h"
#include "Region.h"
#include "commissioning.h"

#include <k_api.h>

#define APP_TX_DUTYCYCLE 30000  
#define LORAWAN_ADR_ON 1        //是否打开ADR(自动速率控制)  0不打开，1打开
#define LORAWAN_APP_PORT 100    //应用层端口，默认100，可以设置为其他值
#define JOINREQ_NBTRIALS 3      //加入请求最大重试次数

static void LoraTxData(lora_AppData_t *AppData);
static void LoraRxData(lora_AppData_t *AppData);
uint8_t BoardGetBatteryLevel()
{
    return 100;
}

/**
 * \brief 获取唯一ID
 * \param ID
 *  
 */
void BoardGetUniqueId(uint8_t *id)
{
}

/**
 * \brief 获取随机数因子
 * \param ID 
 */
uint32_t BoardGetRandomSeed()
{
    return 0;
}

/**
 * \brief 主函数回调函数
 */
static LoRaMainCallback_t LoRaMainCallbacks = {
    BoardGetBatteryLevel,   //获取电池电量    
    BoardGetUniqueId,       //获取唯一ID
    BoardGetRandomSeed,     //获取随机数因子
    LoraTxData,             //Lora发送数据
    LoraRxData              //Lora接收数据
};

/**
 * \brief alios种lora主线程入口函数
 */
static void lora_task_entry(void *arg)
{
    BoardInitMcu();//初始化板子MCU
    lora_init(&LoRaMainCallbacks);  //Lora初始化
    lora_fsm( );//Lora状态机
}

/**
 * \brief Lora应用程序启动
 */
int application_start( void )
{
    lora_task_entry(NULL);
    return 0;
}

/**
 * \brief Lora发送数据
 * \param lora_AppData_t 类型的数据
 */
static void LoraTxData( lora_AppData_t *AppData)
{
    AppData->BuffSize = sprintf( (char *) AppData->Buff, "linklora asr data++");
    PRINTF_RAW("tx: %s\r\n", AppData->Buff );
    AppData->Port = LORAWAN_APP_PORT;
}

/**
 * \brief Lora接收数据
 * \param lora_AppData_t 类型的数据
 */
static void LoraRxData( lora_AppData_t *AppData )
{
    AppData->Buff[AppData->BuffSize] = '\0';
    PRINTF_RAW( "rx: port = %d, len = %d\r\n", AppData->Port, AppData->BuffSize);
    int i;
    for (i = 0; i < AppData->BuffSize; i++) {
        PRINTF_RAW("0x%x ", AppData->Buff[i] );
    }
    PRINTF_RAW("\r\n");
}
