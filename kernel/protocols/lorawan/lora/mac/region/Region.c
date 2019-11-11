/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC region implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ) and Daniel Jaeckle ( STACKFORCE )
*/
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

//#include "timer.h"
#include "timeServer.h"
#include "LoRaMac.h"



// Regional includes
#include "Region.h"



// Setup regions

#ifdef REGION_CN470
#include "RegionCN470.h"
#define CN470_CASE                                 case LORAMAC_REGION_CN470:
#define CN470_IS_ACTIVE( )                         CN470_CASE { return true; }
#define CN470_GET_PHY_PARAM( )                     CN470_CASE { return RegionCN470GetPhyParam( getPhy ); }
#define CN470_SET_BAND_TX_DONE( )                  CN470_CASE { RegionCN470SetBandTxDone( txDone ); break; }
#define CN470_INIT_DEFAULTS( )                     CN470_CASE { RegionCN470InitDefaults( type ); break; }
#define CN470_VERIFY( )                            CN470_CASE { return RegionCN470Verify( verify, phyAttribute ); }
#define CN470_APPLY_CF_LIST( )                     CN470_CASE { RegionCN470ApplyCFList( applyCFList ); break; }
#define CN470_CHAN_MASK_SET( )                     CN470_CASE { return RegionCN470ChanMaskSet( chanMaskSet ); }
#define CN470_ADR_NEXT( )                          CN470_CASE { return RegionCN470AdrNext( adrNext, drOut, txPowOut, adrAckCounter ); }
#define CN470_COMPUTE_RX_WINDOW_PARAMETERS( )      CN470_CASE { RegionCN470ComputeRxWindowParameters( datarate, minRxSymbols, rxError, rxConfigParams ); break; }
#define CN470_RX_CONFIG( )                         CN470_CASE { return RegionCN470RxConfig( rxConfig, datarate ); }
#define CN470_TX_CONFIG( )                         CN470_CASE { return RegionCN470TxConfig( txConfig, txPower, txTimeOnAir ); }
#define CN470_LINK_ADR_REQ( )                      CN470_CASE { return RegionCN470LinkAdrReq( linkAdrReq, drOut, txPowOut, nbRepOut, nbBytesParsed ); }
#define CN470_RX_PARAM_SETUP_REQ( )                CN470_CASE { return RegionCN470RxParamSetupReq( rxParamSetupReq ); }
#define CN470_NEW_CHANNEL_REQ( )                   CN470_CASE { return RegionCN470NewChannelReq( newChannelReq ); }
#define CN470_TX_PARAM_SETUP_REQ( )                CN470_CASE { return RegionCN470TxParamSetupReq( txParamSetupReq ); }
#define CN470_DL_CHANNEL_REQ( )                    CN470_CASE { return RegionCN470DlChannelReq( dlChannelReq ); }
#define CN470_ALTERNATE_DR( )                      CN470_CASE { return RegionCN470AlternateDr( alternateDr ); }
#define CN470_CALC_BACKOFF( )                      CN470_CASE { RegionCN470CalcBackOff( calcBackOff ); break; }
#define CN470_NEXT_CHANNEL( )                      CN470_CASE { return RegionCN470NextChannel( nextChanParams, channel, time, aggregatedTimeOff ); }
#define CN470_CHANNEL_ADD( )                       CN470_CASE { return RegionCN470ChannelAdd( channelAdd ); }
#define CN470_CHANNEL_REMOVE( )                    CN470_CASE { return RegionCN470ChannelsRemove( channelRemove ); }
#define CN470_SET_CONTINUOUS_WAVE( )               CN470_CASE { RegionCN470SetContinuousWave( continuousWave ); break; }
#define CN470_APPLY_DR_OFFSET( )                   CN470_CASE { return RegionCN470ApplyDrOffset( downlinkDwellTime, dr, drOffset ); }
#define CN470_RX_BEACON_SETUP( )                   CN470_CASE { RegionCN470RxBeaconSetup( rxBeaconSetup, outDr ); }
#else
#define CN470_IS_ACTIVE( )
#define CN470_GET_PHY_PARAM( )
#define CN470_SET_BAND_TX_DONE( )
#define CN470_INIT_DEFAULTS( )
#define CN470_VERIFY( )
#define CN470_APPLY_CF_LIST( )
#define CN470_CHAN_MASK_SET( )
#define CN470_ADR_NEXT( )
#define CN470_COMPUTE_RX_WINDOW_PARAMETERS( )
#define CN470_RX_CONFIG( )
#define CN470_TX_CONFIG( )
#define CN470_LINK_ADR_REQ( )
#define CN470_RX_PARAM_SETUP_REQ( )
#define CN470_NEW_CHANNEL_REQ( )
#define CN470_TX_PARAM_SETUP_REQ( )
#define CN470_DL_CHANNEL_REQ( )
#define CN470_ALTERNATE_DR( )
#define CN470_CALC_BACKOFF( )
#define CN470_NEXT_CHANNEL( )
#define CN470_CHANNEL_ADD( )
#define CN470_CHANNEL_REMOVE( )
#define CN470_SET_CONTINUOUS_WAVE( )
#define CN470_APPLY_DR_OFFSET( )
#define CN470_RX_BEACON_SETUP( )
#endif

#ifdef REGION_CN470A
#include "RegionCN470A.h"
#define CN470A_CASE                                 case LORAMAC_REGION_CN470A:
#define CN470A_IS_ACTIVE( )                         CN470A_CASE { return true; }
#define CN470A_GET_PHY_PARAM( )                     CN470A_CASE { return RegionCN470AGetPhyParam( getPhy ); }
#define CN470A_SET_BAND_TX_DONE( )                  CN470A_CASE { RegionCN470ASetBandTxDone( txDone ); break; }
#define CN470A_INIT_DEFAULTS( )                     CN470A_CASE { RegionCN470AInitDefaults( type ); break; }
#define CN470A_VERIFY( )                            CN470A_CASE { return RegionCN470AVerify( verify, phyAttribute ); }
#define CN470A_APPLY_CF_LIST( )                     CN470A_CASE { RegionCN470AApplyCFList( applyCFList ); break; }
#define CN470A_CHAN_MASK_SET( )                     CN470A_CASE { return RegionCN470AChanMaskSet( chanMaskSet ); }
#define CN470A_ADR_NEXT( )                          CN470A_CASE { return RegionCN470AAdrNext( adrNext, drOut, txPowOut, adrAckCounter ); }
#define CN470A_COMPUTE_RX_WINDOW_PARAMETERS( )      CN470A_CASE { RegionCN470AComputeRxWindowParameters( datarate, minRxSymbols, rxError, rxConfigParams ); break; }
#define CN470A_RX_CONFIG( )                         CN470A_CASE { return RegionCN470ARxConfig( rxConfig, datarate ); }
#define CN470A_TX_CONFIG( )                         CN470A_CASE { return RegionCN470ATxConfig( txConfig, txPower, txTimeOnAir ); }
#define CN470A_LINK_ADR_REQ( )                      CN470A_CASE { return RegionCN470ALinkAdrReq( linkAdrReq, drOut, txPowOut, nbRepOut, nbBytesParsed ); }
#define CN470A_RX_PARAM_SETUP_REQ( )                CN470A_CASE { return RegionCN470ARxParamSetupReq( rxParamSetupReq ); }
#define CN470A_NEW_CHANNEL_REQ( )                   CN470A_CASE { return RegionCN470ANewChannelReq( newChannelReq ); }
#define CN470A_TX_PARAM_SETUP_REQ( )                CN470A_CASE { return RegionCN470ATxParamSetupReq( txParamSetupReq ); }
#define CN470A_DL_CHANNEL_REQ( )                    CN470A_CASE { return RegionCN470ADlChannelReq( dlChannelReq ); }
#define CN470A_ALTERNATE_DR( )                      CN470A_CASE { return RegionCN470AAlternateDr( alternateDr ); }
#define CN470A_CALC_BACKOFF( )                      CN470A_CASE { RegionCN470ACalcBackOff( calcBackOff ); break; }
#define CN470A_NEXT_CHANNEL( )                      CN470A_CASE { return RegionCN470ANextChannel( nextChanParams, channel, time, aggregatedTimeOff ); }
#define CN470A_CHANNEL_ADD( )                       CN470A_CASE { return RegionCN470AChannelAdd( channelAdd ); }
#define CN470A_CHANNEL_REMOVE( )                    CN470A_CASE { return RegionCN470AChannelsRemove( channelRemove ); }
#define CN470A_SET_CONTINUOUS_WAVE( )               CN470A_CASE { RegionCN470ASetContinuousWave( continuousWave ); break; }
#define CN470A_APPLY_DR_OFFSET( )                   CN470A_CASE { return RegionCN470AApplyDrOffset( downlinkDwellTime, dr, drOffset ); }
#define CN470A_RX_BEACON_SETUP( )                   CN470A_CASE { RegionCN470ARxBeaconSetup( rxBeaconSetup, outDr ); }
#else
#define CN470A_IS_ACTIVE( )
#define CN470A_GET_PHY_PARAM( )
#define CN470A_SET_BAND_TX_DONE( )
#define CN470A_INIT_DEFAULTS( )
#define CN470A_VERIFY( )
#define CN470A_APPLY_CF_LIST( )
#define CN470A_CHAN_MASK_SET( )
#define CN470A_ADR_NEXT( )
#define CN470A_COMPUTE_RX_WINDOW_PARAMETERS( )
#define CN470A_RX_CONFIG( )
#define CN470A_TX_CONFIG( )
#define CN470A_LINK_ADR_REQ( )
#define CN470A_RX_PARAM_SETUP_REQ( )
#define CN470A_NEW_CHANNEL_REQ( )
#define CN470A_TX_PARAM_SETUP_REQ( )
#define CN470A_DL_CHANNEL_REQ( )
#define CN470A_ALTERNATE_DR( )
#define CN470A_CALC_BACKOFF( )
#define CN470A_NEXT_CHANNEL( )
#define CN470A_CHANNEL_ADD( )
#define CN470A_CHANNEL_REMOVE( )
#define CN470A_SET_CONTINUOUS_WAVE( )
#define CN470A_APPLY_DR_OFFSET( )
#define CN470A_RX_BEACON_SETUP( )
#endif


/**
 * \brief 判断地区参数是否使能
 * \param 地区参数枚举，当前使用的是CAN_470
 */
bool RegionIsActive( LoRaMacRegion_t region )
{
    switch( region )
    {
        AS923_IS_ACTIVE( );
        AU915_IS_ACTIVE( );
        CN470_IS_ACTIVE( );
        CN470A_IS_ACTIVE( );
        CN779_IS_ACTIVE( );
        EU433_IS_ACTIVE( );
        EU868_IS_ACTIVE( );
        KR920_IS_ACTIVE( );
        IN865_IS_ACTIVE( );
        US915_IS_ACTIVE( );
        US915_HYBRID_IS_ACTIVE( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 获取地区物理参数
 * \param 地区参数枚举，返回物理参数的指针
 */
PhyParam_t RegionGetPhyParam( LoRaMacRegion_t region, GetPhyParams_t* getPhy )
{
    PhyParam_t phyParam = { 0 };
    switch( region )
    {
        AS923_GET_PHY_PARAM( );
        AU915_GET_PHY_PARAM( );
        CN470_GET_PHY_PARAM( );
        CN470A_GET_PHY_PARAM( );
        CN779_GET_PHY_PARAM( );
        EU433_GET_PHY_PARAM( );
        EU868_GET_PHY_PARAM( );
        KR920_GET_PHY_PARAM( );
        IN865_GET_PHY_PARAM( );
        US915_GET_PHY_PARAM( );
        US915_HYBRID_GET_PHY_PARAM( );
        default:
        {
            return phyParam;
        }
    }
}

/**
 * \brief 设置地区频段发送完成函数
 * \param 地区参数、发送完成函数
 */
void RegionSetBandTxDone( LoRaMacRegion_t region, SetBandTxDoneParams_t* txDone )
{
    switch( region )
    {
        AS923_SET_BAND_TX_DONE( );
        AU915_SET_BAND_TX_DONE( );
        CN470_SET_BAND_TX_DONE( );
        CN470A_SET_BAND_TX_DONE( );
        CN779_SET_BAND_TX_DONE( );
        EU433_SET_BAND_TX_DONE( );
        EU868_SET_BAND_TX_DONE( );
        KR920_SET_BAND_TX_DONE( );
        IN865_SET_BAND_TX_DONE( );
        US915_SET_BAND_TX_DONE( );
        US915_HYBRID_SET_BAND_TX_DONE( );
        default:
        {
            return;
        }
    }
}

/**
 * \brief 地区默认初始化
 * \param 地区、初始化类型
 */
void RegionInitDefaults( LoRaMacRegion_t region, InitType_t type )
{
    switch( region )
    {
        AS923_INIT_DEFAULTS( );
        AU915_INIT_DEFAULTS( );
        CN470_INIT_DEFAULTS( );
        CN470A_INIT_DEFAULTS( );
        CN779_INIT_DEFAULTS( );
        EU433_INIT_DEFAULTS( );
        EU868_INIT_DEFAULTS( );
        KR920_INIT_DEFAULTS( );
        IN865_INIT_DEFAULTS( );
        US915_INIT_DEFAULTS( );
        US915_HYBRID_INIT_DEFAULTS( );
        default:
        {
            break;
        }
    }
}

/**
 * \brief 地区参数验证？
 * \param 地区、验证参数、
 */
bool RegionVerify( LoRaMacRegion_t region, VerifyParams_t* verify, PhyAttribute_t phyAttribute )
{
    switch( region )
    {
        AS923_VERIFY( );
        AU915_VERIFY( );
        CN470_VERIFY( );
        CN470A_VERIFY( );
        CN779_VERIFY( );
        EU433_VERIFY( );
        EU868_VERIFY( );
        KR920_VERIFY( );
        IN865_VERIFY( );
        US915_VERIFY( );
        US915_HYBRID_VERIFY( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 地区应用 可选频道列表(CFList)参数
 * \param 地区、应用的可选频段列表(CFList）
 */
void RegionApplyCFList( LoRaMacRegion_t region, ApplyCFListParams_t* applyCFList )
{
    switch( region )
    {
        AS923_APPLY_CF_LIST( );
        AU915_APPLY_CF_LIST( );
        CN470_APPLY_CF_LIST( );
        CN470A_APPLY_CF_LIST( );
        CN779_APPLY_CF_LIST( );
        EU433_APPLY_CF_LIST( );
        EU868_APPLY_CF_LIST( );
        KR920_APPLY_CF_LIST( );
        IN865_APPLY_CF_LIST( );
        US915_APPLY_CF_LIST( );
        US915_HYBRID_APPLY_CF_LIST( );
        default:
        {
            break;
        }
    }
}

/**
 * \brief 地区信道掩码设置
 * \param 地区、信道掩码
 */
bool RegionChanMaskSet( LoRaMacRegion_t region, ChanMaskSetParams_t* chanMaskSet )
{
    switch( region )
    {
        AS923_CHAN_MASK_SET( );
        AU915_CHAN_MASK_SET( );
        CN470_CHAN_MASK_SET( );
        CN470A_CHAN_MASK_SET( );
        CN779_CHAN_MASK_SET( );
        EU433_CHAN_MASK_SET( );
        EU868_CHAN_MASK_SET( );
        KR920_CHAN_MASK_SET( );
        IN865_CHAN_MASK_SET( );
        US915_CHAN_MASK_SET( );
        US915_HYBRID_CHAN_MASK_SET( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 自动速率根据地区参数自动获取下一个速率
 * \param 地区、asr参数
 */
bool RegionAdrNext( LoRaMacRegion_t region, AdrNextParams_t* adrNext, int8_t* drOut, int8_t* txPowOut, uint32_t* adrAckCounter )
{
    switch( region )
    {
        AS923_ADR_NEXT( );
        AU915_ADR_NEXT( );
        CN470_ADR_NEXT( );
        CN470A_ADR_NEXT( );
        CN779_ADR_NEXT( );
        EU433_ADR_NEXT( );
        EU868_ADR_NEXT( );
        KR920_ADR_NEXT( );
        IN865_ADR_NEXT( );
        US915_ADR_NEXT( );
        US915_HYBRID_ADR_NEXT( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 预估接收窗口参数
 * \param 地区、速率、 接收参数等
 */
void RegionComputeRxWindowParameters( LoRaMacRegion_t region, int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams )
{
    switch( region )
    {
        AS923_COMPUTE_RX_WINDOW_PARAMETERS( );
        AU915_COMPUTE_RX_WINDOW_PARAMETERS( );
        CN470_COMPUTE_RX_WINDOW_PARAMETERS( );
        CN470A_COMPUTE_RX_WINDOW_PARAMETERS( );
        CN779_COMPUTE_RX_WINDOW_PARAMETERS( );
        EU433_COMPUTE_RX_WINDOW_PARAMETERS( );
        EU868_COMPUTE_RX_WINDOW_PARAMETERS( );
        KR920_COMPUTE_RX_WINDOW_PARAMETERS( );
        IN865_COMPUTE_RX_WINDOW_PARAMETERS( );
        US915_COMPUTE_RX_WINDOW_PARAMETERS( );
        US915_HYBRID_COMPUTE_RX_WINDOW_PARAMETERS( );
        default:
        {
            break;
        }
    }
}

/**
 * \brief 地区接收配置
 * \param 地区、接收配置、 速率
 */
bool RegionRxConfig( LoRaMacRegion_t region, RxConfigParams_t* rxConfig, int8_t* datarate )
{
    switch( region )
    {
        AS923_RX_CONFIG( );
        AU915_RX_CONFIG( );
        CN470_RX_CONFIG( );
        CN470A_RX_CONFIG( );
        CN779_RX_CONFIG( );
        EU433_RX_CONFIG( );
        EU868_RX_CONFIG( );
        KR920_RX_CONFIG( );
        IN865_RX_CONFIG( );
        US915_RX_CONFIG( );
        US915_HYBRID_RX_CONFIG( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 地区发送配置
 * \param 地区、发送配置、发送功率、发送时间
 */
bool RegionTxConfig( LoRaMacRegion_t region, TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    switch( region )
    {
        AS923_TX_CONFIG( );
        AU915_TX_CONFIG( );
        CN470_TX_CONFIG( );
        CN470A_TX_CONFIG( );
        CN779_TX_CONFIG( );
        EU433_TX_CONFIG( );
        EU868_TX_CONFIG( );
        KR920_TX_CONFIG( );
        IN865_TX_CONFIG( );
        US915_TX_CONFIG( );
        US915_HYBRID_TX_CONFIG( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief ADR请求
 * \param 地区
 */
uint8_t RegionLinkAdrReq( LoRaMacRegion_t region, LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed )
{
    switch( region )
    {
        AS923_LINK_ADR_REQ( );
        AU915_LINK_ADR_REQ( );
        CN470_LINK_ADR_REQ( );
        CN470A_LINK_ADR_REQ( );
        CN779_LINK_ADR_REQ( );
        EU433_LINK_ADR_REQ( );
        EU868_LINK_ADR_REQ( );
        KR920_LINK_ADR_REQ( );
        IN865_LINK_ADR_REQ( );
        US915_LINK_ADR_REQ( );
        US915_HYBRID_LINK_ADR_REQ( );
        default:
        {
            return 0;
        }
    }
}

/**
 * \brief 接收参数设置请求
 * \param 地区、接收参数
 */
uint8_t RegionRxParamSetupReq( LoRaMacRegion_t region, RxParamSetupReqParams_t* rxParamSetupReq )
{
    switch( region )
    {
        AS923_RX_PARAM_SETUP_REQ( );
        AU915_RX_PARAM_SETUP_REQ( );
        CN470_RX_PARAM_SETUP_REQ( );
        CN470A_RX_PARAM_SETUP_REQ( );
        CN779_RX_PARAM_SETUP_REQ( );
        EU433_RX_PARAM_SETUP_REQ( );
        EU868_RX_PARAM_SETUP_REQ( );
        KR920_RX_PARAM_SETUP_REQ( );
        IN865_RX_PARAM_SETUP_REQ( );
        US915_RX_PARAM_SETUP_REQ( );
        US915_HYBRID_RX_PARAM_SETUP_REQ( );
        default:
        {
            return 0;
        }
    }
}

/**
 * \brief 新信道请求
 * \param 地区、新的信道
 */
uint8_t RegionNewChannelReq( LoRaMacRegion_t region, NewChannelReqParams_t* newChannelReq )
{
    switch( region )
    {
        AS923_NEW_CHANNEL_REQ( );
        AU915_NEW_CHANNEL_REQ( );
        CN470_NEW_CHANNEL_REQ( );
        CN470A_NEW_CHANNEL_REQ( );
        CN779_NEW_CHANNEL_REQ( );
        EU433_NEW_CHANNEL_REQ( );
        EU868_NEW_CHANNEL_REQ( );
        KR920_NEW_CHANNEL_REQ( );
        IN865_NEW_CHANNEL_REQ( );
        US915_NEW_CHANNEL_REQ( );
        US915_HYBRID_NEW_CHANNEL_REQ( );
        default:
        {
            return 0;
        }
    }
}

/**
 * \brief 发送参数设置请求
 * \param 地区、发送参数
 */
int8_t RegionTxParamSetupReq( LoRaMacRegion_t region, TxParamSetupReqParams_t* txParamSetupReq )
{
    switch( region )
    {
        AS923_TX_PARAM_SETUP_REQ( );
        AU915_TX_PARAM_SETUP_REQ( );
        CN470_TX_PARAM_SETUP_REQ( );
        CN470A_TX_PARAM_SETUP_REQ( );
        CN779_TX_PARAM_SETUP_REQ( );
        EU433_TX_PARAM_SETUP_REQ( );
        EU868_TX_PARAM_SETUP_REQ( );
        KR920_TX_PARAM_SETUP_REQ( );
        IN865_TX_PARAM_SETUP_REQ( );
        US915_TX_PARAM_SETUP_REQ( );
        US915_HYBRID_TX_PARAM_SETUP_REQ( );
        default:
        {
            return 0;
        }
    }
}

/**
 * \brief 下行信道计算？
 * \param 地区、
 */
uint8_t RegionDlChannelReq( LoRaMacRegion_t region, DlChannelReqParams_t* dlChannelReq )
{
    switch( region )
    {
        AS923_DL_CHANNEL_REQ( );
        AU915_DL_CHANNEL_REQ( );
        CN470_DL_CHANNEL_REQ( );
        CN470A_DL_CHANNEL_REQ( );
        CN779_DL_CHANNEL_REQ( );
        EU433_DL_CHANNEL_REQ( );
        EU868_DL_CHANNEL_REQ( );
        KR920_DL_CHANNEL_REQ( );
        IN865_DL_CHANNEL_REQ( );
        US915_DL_CHANNEL_REQ( );
        US915_HYBRID_DL_CHANNEL_REQ( );
        default:
        {
            return 0;
        }
    }
}

/**
 * \brief 备用速率？
 * \param 地区、备用速率
 */
int8_t RegionAlternateDr( LoRaMacRegion_t region, AlternateDrParams_t* alternateDr )
{
    switch( region )
    {
        AS923_ALTERNATE_DR( );
        AU915_ALTERNATE_DR( );
        CN470_ALTERNATE_DR( );
        CN470A_ALTERNATE_DR( );
        CN779_ALTERNATE_DR( );
        EU433_ALTERNATE_DR( );
        EU868_ALTERNATE_DR( );
        KR920_ALTERNATE_DR( );
        IN865_ALTERNATE_DR( );
        US915_ALTERNATE_DR( );
        US915_HYBRID_ALTERNATE_DR( );
        default:
        {
            return 0;
        }
    }
}

/**
 * \brief Calc Backoff(计算退避)
 * \param 地区、计算退避参数
 */
void RegionCalcBackOff( LoRaMacRegion_t region, CalcBackOffParams_t* calcBackOff )
{
    switch( region )
    {
        AS923_CALC_BACKOFF( );
        AU915_CALC_BACKOFF( );
        CN470_CALC_BACKOFF( );
        CN470A_CALC_BACKOFF( );
        CN779_CALC_BACKOFF( );
        EU433_CALC_BACKOFF( );
        EU868_CALC_BACKOFF( );
        KR920_CALC_BACKOFF( );
        IN865_CALC_BACKOFF( );
        US915_CALC_BACKOFF( );
        US915_HYBRID_CALC_BACKOFF( );
        default:
        {
            break;
        }
    }
}

/**
 * \brief 计算下一个信道
 * \param 地区、计算退避参数
 */
bool RegionNextChannel( LoRaMacRegion_t region, NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff )
{
    switch( region )
    {
        AS923_NEXT_CHANNEL( );
        AU915_NEXT_CHANNEL( );
        CN470_NEXT_CHANNEL( );
        CN470A_NEXT_CHANNEL( );
        CN779_NEXT_CHANNEL( );
        EU433_NEXT_CHANNEL( );
        EU868_NEXT_CHANNEL( );
        KR920_NEXT_CHANNEL( );
        IN865_NEXT_CHANNEL( );
        US915_NEXT_CHANNEL( );
        US915_HYBRID_NEXT_CHANNEL( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 信道增加
 * \param 地区、增加的信道参数
 */
LoRaMacStatus_t RegionChannelAdd( LoRaMacRegion_t region, ChannelAddParams_t* channelAdd )
{
    switch( region )
    {
        AS923_CHANNEL_ADD( );
        AU915_CHANNEL_ADD( );
        CN470_CHANNEL_ADD( );
        CN470A_CHANNEL_ADD( );
        CN779_CHANNEL_ADD( );
        EU433_CHANNEL_ADD( );
        EU868_CHANNEL_ADD( );
        KR920_CHANNEL_ADD( );
        IN865_CHANNEL_ADD( );
        US915_CHANNEL_ADD( );
        US915_HYBRID_CHANNEL_ADD( );
        default:
        {
            return LORAMAC_STATUS_PARAMETER_INVALID;
        }
    }
}

/**
 * \brief 信道移除
 * \param 地区、移除的信道参数
 */
bool RegionChannelsRemove( LoRaMacRegion_t region, ChannelRemoveParams_t* channelRemove )
{
    switch( region )
    {
        AS923_CHANNEL_REMOVE( );
        AU915_CHANNEL_REMOVE( );
        CN470_CHANNEL_REMOVE( );
        CN470A_CHANNEL_REMOVE( );
        CN779_CHANNEL_REMOVE( );
        EU433_CHANNEL_REMOVE( );
        EU868_CHANNEL_REMOVE( );
        KR920_CHANNEL_REMOVE( );
        IN865_CHANNEL_REMOVE( );
        US915_CHANNEL_REMOVE( );
        US915_HYBRID_CHANNEL_REMOVE( );
        default:
        {
            return false;
        }
    }
}

/**
 * \brief 设置连续波？
 * \param 地区、连续波参数
 */
void RegionSetContinuousWave( LoRaMacRegion_t region, ContinuousWaveParams_t* continuousWave )
{
    switch( region )
    {
        AS923_SET_CONTINUOUS_WAVE( );
        AU915_SET_CONTINUOUS_WAVE( );
        CN470_SET_CONTINUOUS_WAVE( );
        CN470A_SET_CONTINUOUS_WAVE( );
        CN779_SET_CONTINUOUS_WAVE( );
        EU433_SET_CONTINUOUS_WAVE( );
        EU868_SET_CONTINUOUS_WAVE( );
        KR920_SET_CONTINUOUS_WAVE( );
        IN865_SET_CONTINUOUS_WAVE( );
        US915_SET_CONTINUOUS_WAVE( );
        US915_HYBRID_SET_CONTINUOUS_WAVE( );
        default:
        {
            break;
        }
    }
}

/**
 * \brief 应用速率便宜
 * \param 地区、下行持续时间、速率、偏移
 */
uint8_t RegionApplyDrOffset( LoRaMacRegion_t region, uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset )
{
    switch( region )
    {
        AS923_APPLY_DR_OFFSET( );
        AU915_APPLY_DR_OFFSET( );
        CN470_APPLY_DR_OFFSET( );
        CN470A_APPLY_DR_OFFSET( );
        CN779_APPLY_DR_OFFSET( );
        EU433_APPLY_DR_OFFSET( );
        EU868_APPLY_DR_OFFSET( );
        KR920_APPLY_DR_OFFSET( );
        IN865_APPLY_DR_OFFSET( );
        US915_APPLY_DR_OFFSET( );
        US915_HYBRID_APPLY_DR_OFFSET( );
        default:
        {
            return dr;
        }
    }
}

/**
 * \brief 接收信标设置
 * \param 地区、接收信标设置参数
 */
void RegionRxBeaconSetup( LoRaMacRegion_t region, RxBeaconSetup_t* rxBeaconSetup, uint8_t* outDr )
{
    switch( region )
    {
        //AS923_RX_BEACON_SETUP( );
       // AU915_RX_BEACON_SETUP( );
        CN470_RX_BEACON_SETUP( );
        CN470A_RX_BEACON_SETUP( );
        //CN779_RX_BEACON_SETUP( );
        //EU433_RX_BEACON_SETUP( );
       // EU868_RX_BEACON_SETUP( );
       // KR920_RX_BEACON_SETUP( );
        //IN865_RX_BEACON_SETUP( );
        //US915_RX_BEACON_SETUP( );
       // US915_HYBRID_RX_BEACON_SETUP( );
        default:
        {
            break;
        }
    }
}
