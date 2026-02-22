/********************************************************************************************************
 * @file    main.c
 *
 * @brief   This is the source file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2020
 *
 * @par     Copyright (c) 2020, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "gpio.h"
#include "timer.h"
#include "application/uartinterface/uart_interface.h"


/**
 * @brief      LinkLayer RX & TX FIFO configuration
 */
/* CAL_LL_ACL_RX_BUF_SIZE(maxRxOct): maxRxOct + 22, then 16 byte align */
#define RX_FIFO_SIZE	64
/* must be: 2^n, (power of 2);at least 4; recommended value: 4, 8, 16 */
#define RX_FIFO_NUM		8


/* CAL_LL_ACL_TX_BUF_SIZE(maxTxOct):  maxTxOct + 10, then 4 byte align */
#define TX_FIFO_SIZE	40
/* must be: (2^n), (power of 2); at least 8; recommended value: 8, 16, 32, other value not allowed. */
#define TX_FIFO_NUM		16


_attribute_data_retention_  u8 		 	blt_rxfifo_b[RX_FIFO_SIZE * RX_FIFO_NUM] = {0};
_attribute_data_retention_	my_fifo_t	blt_rxfifo = {
												RX_FIFO_SIZE,
												RX_FIFO_NUM,
												0,
												0,
												blt_rxfifo_b,};


_attribute_data_retention_  u8 		 	blt_txfifo_b[TX_FIFO_SIZE * TX_FIFO_NUM] = {0};
_attribute_data_retention_	my_fifo_t	blt_txfifo = {
												TX_FIFO_SIZE,
												TX_FIFO_NUM,
												0,
												0,
												blt_txfifo_b,};


static u32 tick250ms = 0;
static u8	tbl_advData[] = {
 7,  DT_COMPLETE_LOCAL_NAME, 				'H', 'H', 'H', 'H', 'H', 'H',
 2,	 DT_FLAGS, 								0x01, 					// BLE limited discoverable mode and BR/EDR not supported
 3,  DT_APPEARANCE, 						0x66, 0x66, 			// 384, Generic Remote Control, Generic category
 5,  DT_INCOMPLETE_LIST_16BIT_SERVICE_UUID,	0xAA, 0xBB, 0xCC, 0xDD,	// incomplete list of service class UUIDs (0x1812, 0x180F)
};

int controller_event_callback (u32 h, u8 *p, int n);

static void controllerInitialization(void)
{
	u8  mac_public[6];
	u8  mac_random_static[6];
	u8 adv_param_status = BLE_SUCCESS;

	// const static u8	tbl_advData[] = {
	//  7,  DT_COMPLETE_LOCAL_NAME, 				'H', 'H', 'H', 'H', 'H', 'H',
	//  2,	 DT_FLAGS, 								0x01, 					// BLE limited discoverable mode and BR/EDR not supported
	//  3,  DT_APPEARANCE, 						0x66, 0x66, 			// 384, Generic Remote Control, Generic category
	//  5,  DT_INCOMPLETE_LIST_16BIT_SERVICE_UUID,	0xAA, 0xBB, 0xCC, 0xDD,	// incomplete list of service class UUIDs (0x1812, 0x180F)
	// };

	/* for 512K Flash, flash_sector_mac_address equals to 0x76000, for 1M  Flash, flash_sector_mac_address equals to 0xFF000 */
	blc_initMacAddress(flash_sector_mac_address, mac_public, mac_random_static);
	// u_printf("Public Address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac_public[5], mac_public[4], mac_public[3], mac_public[2], mac_public[1], mac_public[0]);

	blc_ll_initBasicMCU();                      //mandatory
	blc_ll_initStandby_module(mac_public);		//mandatory
	blc_ll_initAdvertising_module(mac_public); 	//legacy advertising module: mandatory for BLE slave
	blc_ll_initScanning_module(mac_public);

	adv_param_status = bls_ll_setAdvParam(  ADV_INTERVAL_30MS,
											ADV_INTERVAL_50MS,
											ADV_TYPE_NONCONNECTABLE_UNDIRECTED,
											OWN_ADDRESS_PUBLIC,
											0,
											NULL,
											BLT_ENABLE_ADV_ALL,
											ADV_FP_NONE);

	// u_printf("ADV parameters set status: 0x%x\n", adv_param_status);

	if (adv_param_status != BLE_SUCCESS) 
	{
		u_printf("ADV parameters set status: 0x%x\n", adv_param_status);
		return;
	}

	blc_ll_clearResolvingList();
	bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );

	bls_ll_setAdvEnable(BLC_ADV_ENABLE);  //must: set ADV enable

	blc_ll_addScanningInAdvState();  //add scan in ADV state
	blc_ll_setScanParameter(SCAN_TYPE_PASSIVE,
							SCAN_INTERVAL_50MS,
							SCAN_INTERVAL_50MS,
							OWN_ADDRESS_PUBLIC,
							SCAN_FP_ALLOW_ADV_ANY);

	blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_ADVERTISING_REPORT);
	
	blc_hci_registerControllerEventHandler(controller_event_callback);

	blc_ll_setScanEnable (BLC_SCAN_ENABLE, DUP_FILTER_ENABLE);

}


static void task250ms(void)
{
	static u32 counter = 0;
	static u8 sleepCounter = 0;
	// u_printf("counter is %d\n", counter++);
	if (counter % 4 == 0) {
		// u_printf("===1===\n");
		gpio_write(GPIO_PD2, 0);
		gpio_write(GPIO_PD3, 1);
		gpio_write(GPIO_PD4, 0);
		gpio_write(GPIO_PD5, 0);

		u_printf("Current ll state is %d\n", blc_ll_getCurrentState());
	}
	else if (counter % 4 == 1) {
		// u_printf("===2===\n");
		gpio_write(GPIO_PD2, 0);
		gpio_write(GPIO_PD3, 0);
		gpio_write(GPIO_PD4, 1);
		gpio_write(GPIO_PD5, 0);
	}
	else if (counter % 4 == 2) {
		// u_printf("===3===\n");
		gpio_write(GPIO_PD2, 0);
		gpio_write(GPIO_PD3, 0);
		gpio_write(GPIO_PD4, 0);
		gpio_write(GPIO_PD5, 1);
	}
	else if (counter % 4 == 3) {
		// u_printf("===4===\n");
		gpio_write(GPIO_PD2, 1);
		gpio_write(GPIO_PD3, 0);
		gpio_write(GPIO_PD4, 0);
		gpio_write(GPIO_PD5, 0);
		// u_printf("= PD1 is %d =\n", gpio_read(GPIO_PD1));

		if (sleepCounter <= 60) 
		{
			// u_printf("sleepCounter is %d\n", sleepCounter);
			tbl_advData[20u] = sleepCounter;
			 bls_ll_setAdvData( (u8 *)tbl_advData, sizeof(tbl_advData) );
			sleepCounter++;
		}
		else 
		{
			u_printf("Go to sleep !\n");
			sleep_us(500);
			sleepCounter = 0;
			cpu_set_gpio_wakeup (GPIO_PD1, Level_High,1);  //button pin pad low wakeUp suspend/deepSleep
			cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  //deepSleep
		}
	}
	counter++;
}

int controller_event_callback (u32 h, u8 *p, int n)
{
    (void)h;(void)p;(void)n;
	if (h &HCI_FLAG_EVENT_BT_STD)		//ble controller hci event
	{
		u8 evtCode = h & 0xff;
		if(evtCode == HCI_EVT_LE_META)
		{
			u8 subEvt_code = p[0];
			//--------hci le event: le ADV report event ----------------------------------------
			if (subEvt_code == HCI_SUB_EVT_LE_ADVERTISING_REPORT)	// ADV packet
			{
				// blm_le_adv_report_event_handle(p);
				u_printf("le ADV report event !\n");

			}

		}
	}
	return 0;
}

/**
 * @brief   IRQ handler
 * @param   none.
 * @return  none.
 */
_attribute_ram_code_ void irq_handler(void)
{

	irq_blt_sdk_handler();

}


/**
 * @brief		This is main function
 * @param[in]	none
 * @return      none
 */
_attribute_ram_code_ int main (void)    //must run in ramcode
{

	// DBG_CHN0_LOW;   //debug

	blc_pm_select_external_32k_crystal();

	// #if(MCU_CORE_TYPE == MCU_CORE_825x)
		cpu_wakeup_init();
	// #else
	// 	cpu_wakeup_init(LDO_MODE,INTERNAL_CAP_XTAL24M);
	// #endif

	// int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp

	rf_drv_ble_init();

	// gpio_init(!deepRetWakeUp);  //analog resistance will keep available in deepSleep mode, so no need initialize again
	gpio_init(1);
	clock_init(SYS_CLK_TYPE);

	// #if (MODULE_WATCHDOG_ENABLE)
	// 	wd_set_interval_ms(WATCHDOG_INIT_TIMEOUT,CLOCK_SYS_CLOCK_1MS);
	// 	wd_start();
	// #endif

	// if( deepRetWakeUp ){
	// 	user_init_deepRetn();
	// }
	// else{
		// user_init_normal();
	// }
	controllerInitialization();

    irq_enable();
	UARTIF_uartinit();
	gpio_set_output_en(GPIO_PD0, 1);
	gpio_set_input_en(GPIO_PD1, 1);
	gpio_set_output_en(GPIO_PD2, 1);
	gpio_set_output_en(GPIO_PD3, 1);
	gpio_set_output_en(GPIO_PD4, 1);
	gpio_set_output_en(GPIO_PD5, 1);

	gpio_write(GPIO_PD2, 1);
	gpio_write(GPIO_PD3, 0);
	gpio_write(GPIO_PD4, 0);
	gpio_write(GPIO_PD5, 0);

	gpio_write(GPIO_PD0, 1);

	u_printf("hello world\n");
	tick250ms = clock_time();
	while (1)
	{
		// #if (MODULE_WATCHDOG_ENABLE)
		// 	#if (MCU_CORE_TYPE == MCU_CORE_TC321X)
		// 		if (g_chip_version != CHIP_VERSION_A0)
		// 	#endif
		// 		{
		// 			wd_clear(); //clear watch dog
		// 		}
		// #endif
		// 	main_loop();
		blt_sdk_main_loop();

		if(clock_time_exceed(tick250ms, 250000))
		{
			tick250ms = clock_time();
			task250ms();
		}
	}
}

