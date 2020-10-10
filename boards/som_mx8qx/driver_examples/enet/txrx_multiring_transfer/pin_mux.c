/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.1
processor: MIMX8QX6xxxFZ
package_id: MIMX8QX6AVLFZ
mcu_data: ksdk2_0
processor_version: 0.0.0
board: VAR-SOM-MX8X-STK
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "pin_mux.h"
#include "fsl_common.h"
#include "main/imx8qx_pads.h"
#include "svc/pad/pad_api.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'false', coreID: m4}
- pin_list:
  - {pin_num: L31, peripheral: ADMA__UART1, signal: uart_rx, pin_signal: UART1_RX, sw_config: sw_config_0}
  - {pin_num: H34, peripheral: ADMA__UART1, signal: uart_tx, pin_signal: UART1_TX, sw_config: sw_config_0}
  - {pin_num: D30, peripheral: CONN__ENET0, signal: enet_mdc, pin_signal: ENET0_MDC, sw_config: sw_config_0}
  - {pin_num: B32, peripheral: CONN__ENET0, signal: enet_mdio, pin_signal: ENET0_MDIO, sw_config: sw_config_0}
  - {pin_num: F28, peripheral: CONN__ENET0, signal: enet_refclk_125m_25m, pin_signal: ENET0_REFCLK_125M_25M, sw_config: sw_config_0}
  - {pin_num: D28, peripheral: CONN__ENET0, signal: enet_rgmii_rxc, pin_signal: ENET0_RGMII_RXC, sw_config: sw_config_0}
  - {pin_num: A31, peripheral: CONN__ENET0, signal: 'enet_rgmii_rxd, 0', pin_signal: ENET0_RGMII_RXD0, sw_config: sw_config_0}
  - {pin_num: C29, peripheral: CONN__ENET0, signal: 'enet_rgmii_rxd, 1', pin_signal: ENET0_RGMII_RXD1, sw_config: sw_config_0}
  - {pin_num: G27, peripheral: CONN__ENET0, signal: 'enet_rgmii_rxd, 2', pin_signal: ENET0_RGMII_RXD2, sw_config: sw_config_0}
  - {pin_num: H26, peripheral: CONN__ENET0, signal: 'enet_rgmii_rxd, 3', pin_signal: ENET0_RGMII_RXD3, sw_config: sw_config_0}
  - {pin_num: B30, peripheral: CONN__ENET0, signal: enet_rgmii_rx_ctl, pin_signal: ENET0_RGMII_RX_CTL, sw_config: sw_config_0}
  - {pin_num: H24, peripheral: CONN__ENET0, signal: enet_rgmii_txc, pin_signal: ENET0_RGMII_TXC, sw_config: sw_config_0}
  - {pin_num: G25, peripheral: CONN__ENET0, signal: 'enet_rgmii_txd, 0', pin_signal: ENET0_RGMII_TXD0, sw_config: sw_config_0}
  - {pin_num: B28, peripheral: CONN__ENET0, signal: 'enet_rgmii_txd, 1', pin_signal: ENET0_RGMII_TXD1, sw_config: sw_config_0}
  - {pin_num: E27, peripheral: CONN__ENET0, signal: 'enet_rgmii_txd, 2', pin_signal: ENET0_RGMII_TXD2, sw_config: sw_config_0}
  - {pin_num: F26, peripheral: CONN__ENET0, signal: 'enet_rgmii_txd, 3', pin_signal: ENET0_RGMII_TXD3, sw_config: sw_config_0}
  - {pin_num: A29, peripheral: CONN__ENET0, signal: enet_rgmii_tx_ctl, pin_signal: ENET0_RGMII_TX_CTL, sw_config: sw_config_0}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(sc_ipc_t ipc)                          /*!< Function assigned for the core: Cortex-M4F[m4] */
{
  sc_err_t err = SC_ERR_NONE;

  err = sc_pad_set_all(ipc, BOARD_INITPINS_DEBUG_UART_RX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_UART1_RX register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_DEBUG_UART_TX_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_UART1_TX register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_MDC_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_MDC register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_MDIO_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_MDIO register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_USB_TYPEC_CROSS_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_REFCLK_125M_25M register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_RXC_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_RXC register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_RXD0_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_RXD0 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_RXD1_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_RXD1 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_RXD2_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_RXD2 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_RXD3_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_RXD3 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_RX_CTL_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_RX_CTL register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_TXC_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_TXC register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_TXD0_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_TXD0 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_TXD1_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_TXD1 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_TXD2_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_TXD2 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_TXD3_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_TXD3 register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
  err = sc_pad_set_all(ipc, BOARD_INITPINS_ENET0_RGMII_TX_CTL_PIN_FUNCTION_ID, 0U, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, 0x0 ,SC_PAD_WAKEUP_OFF);/* IOMUXD_ENET0_RGMII_TX_CTL register modification value */
  if (SC_ERR_NONE != err)
  {
      assert(false);
  }
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
