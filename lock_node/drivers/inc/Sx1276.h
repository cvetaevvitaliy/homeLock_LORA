#ifndef __SX1276__H__
#define __SX1276__H__

#include "spi.h"
#include "GlobalVar.h"

#define   DATARATETYPE				 2 
//DATARATE rate
// 0       0.3kbps,
// 1       1.2kbps, //send max data about 2100ms 
// 2       2.4kbps, //respond 1050ms
// 3       4.8kbps,
// 4       9.6kbps,
// 5       19.2kbps,
#if (DATARATETYPE == 0)
	#define LORA_SEND_MAX_TIMEOUT 85000
#elif (DATARATETYPE == 1)
	#define LORA_SEND_MAX_TIMEOUT 2500
#elif (DATARATETYPE == 2)
	#define LORA_SEND_MAX_TIMEOUT 1200
#elif (DATARATETYPE == 3)
	#define LORA_SEND_MAX_TIMEOUT 800
#elif (DATARATETYPE == 4)
	#define LORA_SEND_MAX_TIMEOUT 500
#elif (DATARATETYPE == 5)
	#define LORA_SEND_MAX_TIMEOUT 300
#endif
	
#define REG_LR_FIFO                                  0x00 
 // Common settings
#define REG_LR_OPMODE                                0x01 
#define REG_LR_BANDSETTING                           0x04
#define REG_LR_FRFMSB                                0x06 
#define REG_LR_FRFMID                                0x07
#define REG_LR_FRFLSB                                0x08 
 // Tx settings
#define REG_LR_PACONFIG                              0x09 
#define REG_LR_PARAMP                                0x0A 
#define REG_LR_OCP                                   0x0B 
 // Rx settings
#define REG_LR_LNA                                   0x0C 
 // LoRa registers
#define REG_LR_FIFOADDRPTR                           0x0D 
#define REG_LR_FIFOTXBASEADDR                        0x0E 
#define REG_LR_FIFORXBASEADDR                        0x0F 
#define REG_LR_FIFORXCURRENTADDR                     0x10 
#define REG_LR_IRQFLAGSMASK                          0x11 
#define REG_LR_IRQFLAGS                              0x12 
#define REG_LR_NBRXBYTES                             0x13 
#define REG_LR_RXHEADERCNTVALUEMSB                   0x14 
#define REG_LR_RXHEADERCNTVALUELSB                   0x15 
#define REG_LR_RXPACKETCNTVALUEMSB                   0x16 
#define REG_LR_RXPACKETCNTVALUELSB                   0x17 
#define REG_LR_MODEMSTAT                             0x18 
#define REG_LR_PKTSNRVALUE                           0x19 
#define REG_LR_PKTRSSIVALUE                          0x1A 
#define REG_LR_RSSIVALUE                             0x1B 
#define REG_LR_HOPCHANNEL                            0x1C 
#define REG_LR_MODEMCONFIG1                          0x1D 
#define REG_LR_MODEMCONFIG2                          0x1E 
#define REG_LR_SYMBTIMEOUTLSB                        0x1F 
#define REG_LR_PREAMBLEMSB                           0x20 
#define REG_LR_PREAMBLELSB                           0x21 
#define REG_LR_PAYLOADLENGTH                         0x22 
#define REG_LR_PAYLOADMAXLENGTH                      0x23 
#define REG_LR_HOPPERIOD                             0x24 
#define REG_LR_FIFORXBYTEADDR                        0x25
#define REG_LR_MODEMCONFIG3                          0x26
 // end of documented register in datasheet
 // I/O settings
#define REG_LR_DIOMAPPING1                           0x40
#define REG_LR_DIOMAPPING2                           0x41
 // Version
#define REG_LR_VERSION                               0x42
 // Additional settings
#define REG_LR_PLLHOP                                0x44
#define REG_LR_TCXO                                  0x4B
#define REG_LR_PADAC                                 0x4D
#define REG_LR_FORMERTEMP                            0x5B
#define REG_LR_BITRATEFRAC                           0x5D
#define REG_LR_AGCREF                                0x61
#define REG_LR_AGCTHRESH1                            0x62
#define REG_LR_AGCTHRESH2                            0x63
#define REG_LR_AGCTHRESH3                            0x64


#define GPIO_VARE_1                                  0X00
#define GPIO_VARE_2                                  0X00

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK            0xF1 
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK          0xFB 
#define RFLR_MODEMCONFIG1_BW_MASK                    0x0F 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK        0xFE 

#define RFLR_MODEMCONFIG3_MOBILE_NODE_MASK           0xF7 

#define TIME_OUT_INT                                 0x80 
#define PACKET_RECVER_INT                            0x40 
#define CRC_ERROR_INT                                0x20 
#define RECVER_HEAR_INT                              0x10 
#define FIFO_SEND_OVER                               0x08 
#define RFLR_IRQFLAGS_CAD                            0x04 
#define RFLR_IRQFLAGS_FHSS                           0x02 
#define RFLR_IRQFLAGS_CADD                           0x01 

#define IRQN_TXD_Value                               0xF7
#define IRQN_RXD_Value                               0x9F
#define IRQN_CAD_Value                               0xFA
#define IRQN_SLEEP_Value                             0xFF
#define PACKET_MIAX_Value                            0xff

/*******************************************************************
** SX1276 initialisation register values definition               **
*******************************************************************/
#define DEF_LR_FIFO                                 0x00
// Common settings
#define DEF_LR_OPMODE                               0x09
#define DEF_LR_BANDSETTING                          0x00
#define DEF_LR_FRFMSB                               0x6C
#define DEF_LR_FRFMID                               0x80
#define DEF_LR_FRFLSB                               0x00
// Tx settings
#define DEF_LR_PACONFIG                             0x4F
#define DEF_LR_PARAMP                               0x09
#define DEF_LR_OCP                                  0x2B
// Rx settings
#define DEF_LR_LNA                                  0x20
// LoRa registers
#define DEF_LR_FIFOADDRPTR                          0x00
#define DEF_LR_FIFOTXBASEADDR                       0x80
#define DEF_LR_FIFORXBASEADDR                       0x00

#define DEF_LR_IRQFLAGSMASK                         0x00

#define DEF_LR_MODEMCONFIG1                         0x72
#define DEF_LR_MODEMCONFIG2                         0x70
#define DEF_LR_SYMBTIMEOUTLSB                       0x64
#define DEF_LR_PREAMBLEMSB                          0x00
#define DEF_LR_PREAMBLELSB                          0x08
#define DEF_LR_PAYLOADLENGTH                        0x01
#define DEF_LR_PAYLOADMAXLENGTH                     0xFF
#define DEF_LR_HOPPERIOD                            0x00

#define DEF_LR_MODEMCONFIG3                         0x00
// end of documented register in datasheet
// I/O settings
#define DEF_LR_DIOMAPPING1                          0x00
#define DEF_LR_DIOMAPPING2                          0x00
// Version

// Additional settings
#define DEF_LR_PLLHOP                               0x2D
#define DEF_LR_TCXO                                 0x09
#define DEF_LR_PADAC                                0x84


#define DEF_LR_AGCREF                               0x13
#define DEF_LR_AGCTHRESH1                           0x0E
#define DEF_LR_AGCTHRESH2                           0x5B
#define DEF_LR_AGCTHRESH3                           0xDB

/*******************************************************************
** SX1276 bit control definition                                  **
*******************************************************************/

/*!
 * RegFifo
 */

/*!
 * RegOpMode
 */
#define RFLR_OPMODE_LONGRANGEMODE_MASK              0x7F
#define RFLR_OPMODE_LONGRANGEMODE_OFF               0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON                0x80

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK            0xBF
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE          0x40
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE         0x00 // Default

#define RFLR_OPMODE_FREQMODE_ACCESS_MASK            0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF              0x08 // Default
#define RFLR_OPMODE_FREQMODE_ACCESS_HF              0x00

#define RFLR_OPMODE_MASK                            0xF8
#define RFLR_OPMODE_SLEEP                           0x00
#define RFLR_OPMODE_STANDBY                         0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX                  0x02
#define RFLR_OPMODE_TRANSMITTER                     0x03
#define RFLR_OPMODE_SYNTHESIZER_RX                  0x04
#define RFLR_OPMODE_RECEIVER                        0x05
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE                 0x06
#define RFLR_OPMODE_CAD                             0x07

/*!
 * RegBandSetting
 */
#define RFLR_BANDSETTING_MASK                    0x3F
#define RFLR_BANDSETTING_AUTO                    0x00 // Default
#define RFLR_BANDSETTING_DIV_BY_1                0x40
#define RFLR_BANDSETTING_DIV_BY_2                0x80
#define RFLR_BANDSETTING_DIV_BY_6                0xC0

/*!
 * RegFrf (MHz)
 */

#define RFLR_FRFMSB_434_MHZ                         0x6C // Default
#define RFLR_FRFMID_434_MHZ                         0x80 // Default
#define RFLR_FRFLSB_434_MHZ                         0x00 // Default

#define RFLR_FRFMSB_863_MHZ                         0xD7
#define RFLR_FRFMID_863_MHZ                         0xC0
#define RFLR_FRFLSB_863_MHZ                         0x00
#define RFLR_FRFMSB_864_MHZ                         0xD8
#define RFLR_FRFMID_864_MHZ                         0x00
#define RFLR_FRFLSB_864_MHZ                         0x00
#define RFLR_FRFMSB_865_MHZ                         0xD8
#define RFLR_FRFMID_865_MHZ                         0x40
#define RFLR_FRFLSB_865_MHZ                         0x00
#define RFLR_FRFMSB_866_MHZ                         0xD8
#define RFLR_FRFMID_866_MHZ                         0x80
#define RFLR_FRFLSB_866_MHZ                         0x00
#define RFLR_FRFMSB_867_MHZ                         0xD8
#define RFLR_FRFMID_867_MHZ                         0xC0
#define RFLR_FRFLSB_867_MHZ                         0x00
#define RFLR_FRFMSB_868_MHZ                         0xD9
#define RFLR_FRFMID_868_MHZ                         0x00
#define RFLR_FRFLSB_868_MHZ                         0x00
#define RFLR_FRFMSB_869_MHZ                         0xD9
#define RFLR_FRFMID_869_MHZ                         0x40
#define RFLR_FRFLSB_869_MHZ                         0x00
#define RFLR_FRFMSB_870_MHZ                         0xD9
#define RFLR_FRFMID_870_MHZ                         0x80
#define RFLR_FRFLSB_870_MHZ                         0x00

#define RFLR_FRFMSB_902_MHZ                         0xE1
#define RFLR_FRFMID_902_MHZ                         0x80
#define RFLR_FRFLSB_902_MHZ                         0x00
#define RFLR_FRFMSB_903_MHZ                         0xE1
#define RFLR_FRFMID_903_MHZ                         0xC0
#define RFLR_FRFLSB_903_MHZ                         0x00
#define RFLR_FRFMSB_904_MHZ                         0xE2
#define RFLR_FRFMID_904_MHZ                         0x00
#define RFLR_FRFLSB_904_MHZ                         0x00
#define RFLR_FRFMSB_905_MHZ                         0xE2
#define RFLR_FRFMID_905_MHZ                         0x40
#define RFLR_FRFLSB_905_MHZ                         0x00
#define RFLR_FRFMSB_906_MHZ                         0xE2
#define RFLR_FRFMID_906_MHZ                         0x80
#define RFLR_FRFLSB_906_MHZ                         0x00
#define RFLR_FRFMSB_907_MHZ                         0xE2
#define RFLR_FRFMID_907_MHZ                         0xC0
#define RFLR_FRFLSB_907_MHZ                         0x00
#define RFLR_FRFMSB_908_MHZ                         0xE3
#define RFLR_FRFMID_908_MHZ                         0x00
#define RFLR_FRFLSB_908_MHZ                         0x00
#define RFLR_FRFMSB_909_MHZ                         0xE3
#define RFLR_FRFMID_909_MHZ                         0x40
#define RFLR_FRFLSB_909_MHZ                         0x00
#define RFLR_FRFMSB_910_MHZ                         0xE3
#define RFLR_FRFMID_910_MHZ                         0x80
#define RFLR_FRFLSB_910_MHZ                         0x00
#define RFLR_FRFMSB_911_MHZ                         0xE3
#define RFLR_FRFMID_911_MHZ                         0xC0
#define RFLR_FRFLSB_911_MHZ                         0x00
#define RFLR_FRFMSB_912_MHZ                         0xE4
#define RFLR_FRFMID_912_MHZ                         0x00
#define RFLR_FRFLSB_912_MHZ                         0x00
#define RFLR_FRFMSB_913_MHZ                         0xE4
#define RFLR_FRFMID_913_MHZ                         0x40
#define RFLR_FRFLSB_913_MHZ                         0x00
#define RFLR_FRFMSB_914_MHZ                         0xE4
#define RFLR_FRFMID_914_MHZ                         0x80
#define RFLR_FRFLSB_914_MHZ                         0x00
#define RFLR_FRFMSB_915_MHZ                         0xE4  // Default
#define RFLR_FRFMID_915_MHZ                         0xC0  // Default
#define RFLR_FRFLSB_915_MHZ                         0x00  // Default
#define RFLR_FRFMSB_916_MHZ                         0xE5
#define RFLR_FRFMID_916_MHZ                         0x00
#define RFLR_FRFLSB_916_MHZ                         0x00
#define RFLR_FRFMSB_917_MHZ                         0xE5
#define RFLR_FRFMID_917_MHZ                         0x40
#define RFLR_FRFLSB_917_MHZ                         0x00
#define RFLR_FRFMSB_918_MHZ                         0xE5
#define RFLR_FRFMID_918_MHZ                         0x80
#define RFLR_FRFLSB_918_MHZ                         0x00
#define RFLR_FRFMSB_919_MHZ                         0xE5
#define RFLR_FRFMID_919_MHZ                         0xC0
#define RFLR_FRFLSB_919_MHZ                         0x00
#define RFLR_FRFMSB_920_MHZ                         0xE6
#define RFLR_FRFMID_920_MHZ                         0x00
#define RFLR_FRFLSB_920_MHZ                         0x00
#define RFLR_FRFMSB_921_MHZ                         0xE6
#define RFLR_FRFMID_921_MHZ                         0x40
#define RFLR_FRFLSB_921_MHZ                         0x00
#define RFLR_FRFMSB_922_MHZ                         0xE6
#define RFLR_FRFMID_922_MHZ                         0x80
#define RFLR_FRFLSB_922_MHZ                         0x00
#define RFLR_FRFMSB_923_MHZ                         0xE6
#define RFLR_FRFMID_923_MHZ                         0xC0
#define RFLR_FRFLSB_923_MHZ                         0x00
#define RFLR_FRFMSB_924_MHZ                         0xE7
#define RFLR_FRFMID_924_MHZ                         0x00
#define RFLR_FRFLSB_924_MHZ                         0x00
#define RFLR_FRFMSB_925_MHZ                         0xE7
#define RFLR_FRFMID_925_MHZ                         0x40
#define RFLR_FRFLSB_925_MHZ                         0x00
#define RFLR_FRFMSB_926_MHZ                         0xE7
#define RFLR_FRFMID_926_MHZ                         0x80
#define RFLR_FRFLSB_926_MHZ                         0x00
#define RFLR_FRFMSB_927_MHZ                         0xE7
#define RFLR_FRFMID_927_MHZ                         0xC0
#define RFLR_FRFLSB_927_MHZ                         0x00
#define RFLR_FRFMSB_928_MHZ                         0xE8
#define RFLR_FRFMID_928_MHZ                         0x00
#define RFLR_FRFLSB_928_MHZ                         0x00

/*!
 * RegPaConfig
 */
#define RFLR_PACONFIG_PASELECT_MASK                 0x7F
#define RFLR_PACONFIG_PASELECT_PABOOST              0x80
#define RFLR_PACONFIG_PASELECT_RFO                  0x00 // Default

#define RFLR_PACONFIG_MAX_POWER_MASK                0x8F

#define RFLR_PACONFIG_OUTPUTPOWER_MASK              0xF0

/*!
 * RegPaRamp
 */
#define RFLR_PARAMP_TXBANDFORCE_MASK                0xEF
#define RFLR_PARAMP_TXBANDFORCE_BAND_SEL            0x10
#define RFLR_PARAMP_TXBANDFORCE_AUTO                0x00 // Default

#define RFLR_PARAMP_MASK                            0xF0
#define RFLR_PARAMP_3400_US                         0x00
#define RFLR_PARAMP_2000_US                         0x01
#define RFLR_PARAMP_1000_US                         0x02
#define RFLR_PARAMP_0500_US                         0x03
#define RFLR_PARAMP_0250_US                         0x04
#define RFLR_PARAMP_0125_US                         0x05
#define RFLR_PARAMP_0100_US                         0x06
#define RFLR_PARAMP_0062_US                         0x07
#define RFLR_PARAMP_0050_US                         0x08
#define RFLR_PARAMP_0040_US                         0x09 // Default
#define RFLR_PARAMP_0031_US                         0x0A
#define RFLR_PARAMP_0025_US                         0x0B
#define RFLR_PARAMP_0020_US                         0x0C
#define RFLR_PARAMP_0015_US                         0x0D
#define RFLR_PARAMP_0012_US                         0x0E
#define RFLR_PARAMP_0010_US                         0x0F

/*!
 * RegOcp
 */
#define RFLR_OCP_MASK                               0xDF
#define RFLR_OCP_ON                                 0x20 // Default
#define RFLR_OCP_OFF                                0x00

#define RFLR_OCP_TRIM_MASK                          0xE0
#define RFLR_OCP_TRIM_045_MA                        0x00
#define RFLR_OCP_TRIM_050_MA                        0x01
#define RFLR_OCP_TRIM_055_MA                        0x02
#define RFLR_OCP_TRIM_060_MA                        0x03
#define RFLR_OCP_TRIM_065_MA                        0x04
#define RFLR_OCP_TRIM_070_MA                        0x05
#define RFLR_OCP_TRIM_075_MA                        0x06
#define RFLR_OCP_TRIM_080_MA                        0x07
#define RFLR_OCP_TRIM_085_MA                        0x08
#define RFLR_OCP_TRIM_090_MA                        0x09
#define RFLR_OCP_TRIM_095_MA                        0x0A
#define RFLR_OCP_TRIM_100_MA                        0x0B  // Default
#define RFLR_OCP_TRIM_105_MA                        0x0C
#define RFLR_OCP_TRIM_110_MA                        0x0D
#define RFLR_OCP_TRIM_115_MA                        0x0E
#define RFLR_OCP_TRIM_120_MA                        0x0F
#define RFLR_OCP_TRIM_130_MA                        0x10
#define RFLR_OCP_TRIM_140_MA                        0x11
#define RFLR_OCP_TRIM_150_MA                        0x12
#define RFLR_OCP_TRIM_160_MA                        0x13
#define RFLR_OCP_TRIM_170_MA                        0x14
#define RFLR_OCP_TRIM_180_MA                        0x15
#define RFLR_OCP_TRIM_190_MA                        0x16
#define RFLR_OCP_TRIM_200_MA                        0x17
#define RFLR_OCP_TRIM_210_MA                        0x18
#define RFLR_OCP_TRIM_220_MA                        0x19
#define RFLR_OCP_TRIM_230_MA                        0x1A
#define RFLR_OCP_TRIM_240_MA                        0x1B

/*!
 * RegLna
 */
#define RFLR_LNA_GAIN_MASK                          0x1F
#define RFLR_LNA_GAIN_G1                            0x20 // Default
#define RFLR_LNA_GAIN_G2                            0x40
#define RFLR_LNA_GAIN_G3                            0x60
#define RFLR_LNA_GAIN_G4                            0x80
#define RFLR_LNA_GAIN_G5                            0xA0
#define RFLR_LNA_GAIN_G6                            0xC0

#define RFLR_LNA_BOOST_LF_MASK                      0xE7
#define RFLR_LNA_BOOST_LF_DEFAULT                   0x00 // Default
#define RFLR_LNA_BOOST_LF_GAIN                      0x08
#define RFLR_LNA_BOOST_LF_IP3                       0x10
#define RFLR_LNA_BOOST_LF_BOOST                     0x18

#define RFLR_LNA_RXBANDFORCE_MASK                   0xFB
#define RFLR_LNA_RXBANDFORCE_BAND_SEL               0x04
#define RFLR_LNA_RXBANDFORCE_AUTO                   0x00 // Default

#define RFLR_LNA_BOOST_HF_MASK                      0xFC
#define RFLR_LNA_BOOST_HF_OFF                       0x00 // Default
#define RFLR_LNA_BOOST_HF_ON                        0x03

/*!
 * RegFifoAddrPtr
 */
#define RFLR_FIFOADDRPTR                            0x00 // Default

/*!
 * RegFifoTxBaseAddr
 */
#define RFLR_FIFOTXBASEADDR                         0x80 // Default

/*!
 * RegFifoTxBaseAddr
 */
#define RFLR_FIFORXBASEADDR                         0x00 // Default

/*!
 * RegFifoRxCurrentAddr (Read Only)
 */

/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01

/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80
#define RFLR_IRQFLAGS_RXDONE                        0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10
#define RFLR_IRQFLAGS_TXDONE                        0x08
#define RFLR_IRQFLAGS_CADDONE                       0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02
#define RFLR_IRQFLAGS_CADDETECTED                   0x01



/*!
 * RegFifoRxNbBytes (Read Only)    //
 */


 /*!
 * RegRxHeaderCntValueMsb (Read Only)    //
 */


  /*!
 * RegRxHeaderCntValueLsb (Read Only)    //
 */


/*!
 * RegRxPacketCntValueMsb (Read Only)    //
 */


 /*!
 * RegRxPacketCntValueLsb (Read Only)    //
 */


 /*!
 * RegModemStat (Read Only)    //
 */
#define RFLR_MODEMSTAT_RX_CR_MASK                   0x1F
#define RFLR_MODEMSTAT_MODEM_STATUS_MASK            0xE0

/*!
 * RegPktSnrValue (Read Only)    //
 */


 /*!
 * RegPktRssiValue (Read Only)    //
 */


/*!
 * RegRssiValue (Read Only)    //
 */


 /*!
 * RegModemConfig1
 */
#define RFLR_MODEMCONFIG1_BW_MASK                   0x0F

#define RFLR_MODEMCONFIG1_BW_7_81_KHZ               0x00
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ              0x10
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ              0x20
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ              0x30
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ              0x40
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ              0x50
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ              0x60
#define RFLR_MODEMCONFIG1_BW_125_KHZ                0x70 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                0x80
#define RFLR_MODEMCONFIG1_BW_500_KHZ                0x90

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK           0xF1
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5            0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6            0x04 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7            0x06
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8            0x08

#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK       0xFE
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON         0x01
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF        0x00 // Default

 /*!
 * RegModemConfig2
 */
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F
#define RFLR_MODEMCONFIG2_SF_6                      0x60
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80
#define RFLR_MODEMCONFIG2_SF_9                      0x90
#define RFLR_MODEMCONFIG2_SF_10                     0xA0
#define RFLR_MODEMCONFIG2_SF_11                     0xB0
#define RFLR_MODEMCONFIG2_SF_12                     0xC0

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK     0xF7
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON       0x08
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF      0x00

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK         0xFB
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON           0x04
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF          0x00 // Default

#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK       0xFC
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB            0x00 // Default


/*!
 * RegHopChannel (Read Only)
 */
#define RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK       0x7F
#define RFLR_HOPCHANNEL_PLL_LOCK_FAIL               0x80
#define RFLR_HOPCHANNEL_PLL_LOCK_SUCCEED            0x00 // Default

#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_MASK          0xBF
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_ON            0x40
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_OFF           0x00 // Default

#define RFLR_HOPCHANNEL_CHANNEL_MASK                0x3F


/*!
 * RegSymbTimeoutLsb
 */
#define RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT             0x64 // Default

/*!
 * RegPreambleLengthMsb
 */
#define RFLR_PREAMBLELENGTHMSB                      0x00 // Default

/*!
 * RegPreambleLengthLsb
 */
#define RFLR_PREAMBLELENGTHLSB                      0x08 // Default

/*!
 * RegPayloadLength
 */
#define RFLR_PAYLOADLENGTH                          0x0E // Default

/*!
 * RegPayloadMaxLength
 */
#define RFLR_PAYLOADMAXLENGTH                       0xFF // Default

/*!
 * RegHopPeriod
 */
#define RFLR_HOPPERIOD_FREQFOPPINGPERIOD            0x00 // Default


/*!
 * RegDioMapping1
 */
#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK                  0xFC
#define RFLR_DIOMAPPING1_DIO3_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO3_01                    0x01
#define RFLR_DIOMAPPING1_DIO3_10                    0x02
#define RFLR_DIOMAPPING1_DIO3_11                    0x03

/*!
 * RegDioMapping2
 */
#define RFLR_DIOMAPPING2_DIO4_MASK                  0x3F
#define RFLR_DIOMAPPING2_DIO4_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO4_01                    0x40
#define RFLR_DIOMAPPING2_DIO4_10                    0x80
#define RFLR_DIOMAPPING2_DIO4_11                    0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK                  0xCF
#define RFLR_DIOMAPPING2_DIO5_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO5_01                    0x10
#define RFLR_DIOMAPPING2_DIO5_10                    0x20
#define RFLR_DIOMAPPING2_DIO5_11                    0x30

#define RFLR_DIOMAPPING2_MAP_MASK                   0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT         0x01
#define RFLR_DIOMAPPING2_MAP_RSSI                   0x00  // Default

/*!
 * RegVersion (Read Only)
 */

/*!
 * RegAgcRef
 */

/*!
 * RegAgcThresh1
 */

/*!
 * RegAgcThresh2
 */

/*!
 * RegAgcThresh3
 */

/*!
 * RegFifoRxByteAddr (Read Only)
 */

/*!
 * RegPllHop
 */
#define RFLR_PLLHOP_FASTHOP_MASK                    0x7F
#define RFLR_PLLHOP_FASTHOP_ON                      0x80
#define RFLR_PLLHOP_FASTHOP_OFF                     0x00 // Default

/*!
 * RegTcxo
 */
#define RFLR_TCXO_TCXOINPUT_MASK                    0xEF
#define RFLR_TCXO_TCXOINPUT_ON                      0x10
#define RFLR_TCXO_TCXOINPUT_OFF                     0x00  // Default

/*!
 * RegPaDac
 */
#define RFLR_PADAC_20DBM_MASK                       0xF8
#define RFLR_PADAC_20DBM_ON                         0x07
#define RFLR_PADAC_20DBM_OFF                        0x04  // Default

/*!
 * RegPll
 */
#define RFLR_PLL_BANDWIDTH_MASK                     0x3F
#define RFLR_PLL_BANDWIDTH_75                       0x00
#define RFLR_PLL_BANDWIDTH_150                      0x40
#define RFLR_PLL_BANDWIDTH_225                      0x80
#define RFLR_PLL_BANDWIDTH_300                      0xC0  // Default

/*!
 * RegPllLowPn
 */
#define RFLR_PLLLOWPN_BANDWIDTH_MASK                0x3F
#define RFLR_PLLLOWPN_BANDWIDTH_75                  0x00
#define RFLR_PLLLOWPN_BANDWIDTH_150                 0x40
#define RFLR_PLLLOWPN_BANDWIDTH_225                 0x80
#define RFLR_PLLLOWPN_BANDWIDTH_300                 0xC0  // Default

/*!
 * RegModemConfig3
 */
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK  0xF7
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON    0x08
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF   0x00 // Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK              0xFB
#define RFLR_MODEMCONFIG3_AGCAUTO_ON                0x04 // Default
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF               0x00

/*!
 * RegFormerTemp
 */

typedef enum 
{
   Sleep_mode	        = (unsigned char)0x00, 
   Stdby_mode	        = (unsigned char)0x01, 
   TX_mode 	          = (unsigned char)0x02,
   Transmitter_mode	  = (unsigned char)0x03,
   RF_mode 		        = (unsigned char)0x04,
   Receiver_mode	    = (unsigned char)0x05,
   receive_single	    = (unsigned char)0x06,
   CAD_mode		        = (unsigned char)0x07,
}RFMode_SET;

typedef enum 
{
   FSK_mode             = (unsigned char)0x00, 
   LORA_mode            = (unsigned char)0x80, 
}  Debugging_fsk_ook;

//typedef enum { False=0, True=1 }BOOL;

typedef enum { enOpen, enClose } cmdEntype_t;

typedef enum { rxOpen, txOpen } cmdpaType_t;

typedef struct{
   unsigned char (*lpByteWritefunc)(unsigned char src);
   unsigned char (*lpByteReadfunc)(unsigned char src);
   void (*lpSwitchEnStatus)(cmdEntype_t cmd);
   void (*paSwitchCmdfunc)(cmdpaType_t cmd);
}lpCtrlTypefunc_t;



void SX1276LoRaSetOpMode(RFMode_SET opMode);
void SX1276LoRaFsk(Debugging_fsk_ook opMode);
void SX1276LoRaSetRFFrequency(void);
void SX1276LoRaSetRFPower(unsigned char power);
void SX1276LoRaSetSpreadingFactor(unsigned char factor);
void SX1276LoRaSetNbTrigPeaks(unsigned char value);
void SX1276LoRaSetErrorCoding(unsigned char value);
void SX1276LoRaSetPacketCrcOn(BOOL enable);
void SX1276LoRaSetSignalBandwidth(unsigned char bw);
void SX1276LoRaSetImplicitHeaderOn(BOOL enable);
void SX1276LoRaSetSymbTimeout(unsigned int value);
void SX1276LoRaSetPayloadLength(unsigned char value);
void SX1276LoRaSetPreamLength(unsigned int value);
void SX1276LoRaSetMobileNode(BOOL enable);
void FUN_RF_SENDPACKET(unsigned char *RF_TRAN_P, uint16_t LEN);
void SX1276_LoRa_SetDataRate( uint8_t rate );
void RF_RECEIVE (void);
void RF_CAD_RECEIVE (void);

/*常用外部调用函数*/
void SX1278_Interupt(void);
void Lora_Init(void);


#endif
