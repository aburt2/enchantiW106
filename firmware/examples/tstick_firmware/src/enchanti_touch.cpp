#include "enchanti_touch.h"
#include <cmath>
#include <algorithm>

// headers needed for Zephyr I2C
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#define I2C_MASTER DT_ALIAS(i2c3)

// Use HAL NXP drivers
#include "fsl_i2c.h"
#include "fsl_i2c_dma.h"

// Zephyr setup
// I2C master functionality
static const struct device *const com_dev = DEVICE_DT_GET(I2C_MASTER);

// Get DMA 

// Define MCUS flexcomm config so I can get I2c base
struct mcux_flexcomm_config {
	I2C_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
	uint32_t bitrate;
	const struct pinctrl_dev_config *pincfg;
	const struct reset_dt_spec reset;
};

// DMA settings
/*******************************************************************************
 * Definitions needed for HAL I2C
 ******************************************************************************/
#define HAL_I2C_MASTER              I2C2
#define DMA_CHANNEL                DMA0
#define HAL_I2C_MASTER_CHANNEL         5
#define I2C_BAUDRATE               (1000000) /* 1Mhz */
#define I2C_DATA_LENGTH            (240)     /* MAX is 256 */

i2c_master_dma_handle_t g_m_dma_handle;
static dma_handle_t dmaHandle;
volatile bool g_MasterCompletionFlag = false;

// i2c Buffers
uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];

// callback
static void i2c_master_dma_callback(I2C_Type *base, i2c_master_dma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Class methods
 ******************************************************************************/
// Initialise touch board, 
uint8_t EnchantiTouch::initTouch(touch_config enchanti_config) {
    float num = enchanti_config.touchsize / ENCHANTI_BASETOUCHSIZE;
    // Clip number of touch boards to 2
    if (num > 2) {
        num = 2;
        touchSize = 2 * ENCHANTI_BASETOUCHSIZE;
    } else if (num < 0) {
        num = 1;
        touchSize = ENCHANTI_BASETOUCHSIZE;
    } else {
        // set touchsize
        num_boards = num;
        touchSize = floor(num_boards * ENCHANTI_BASETOUCHSIZE);
    }

    // Save properties to class
    num_boards = num;
    noise_threshold = enchanti_config.touch_threshold;
    boardMode = enchanti_config.touch_mode;
    comMode = enchanti_config.comm_mode;

    // Setup I2C device
    if(!device_is_ready(com_dev)) {
        return 0;
    }

    // Initialise DMA Channel
    DMA_Init(DMA_CHANNEL);

    /* Initialise tx and rx buffer */
    memset(g_master_txBuff, 0 , I2C_DATA_LENGTH);
    memset(g_master_rxBuff, 0 , I2C_DATA_LENGTH);

    /* Initialise DMA handle and transfer config*/
    memset(&g_m_dma_handle, 0, sizeof(g_m_dma_handle));

    /* Initialise DMA Handle and buffer */
    DMA_EnableChannel(DMA_CHANNEL, HAL_I2C_MASTER_CHANNEL);
    DMA_CreateHandle(&dmaHandle, DMA_CHANNEL, HAL_I2C_MASTER_CHANNEL);
    I2C_MasterTransferCreateHandleDMA(HAL_I2C_MASTER, &g_m_dma_handle, i2c_master_dma_callback, NULL, &dmaHandle);


    running = true;
    return 1;
}

void EnchantiTouch::readTouch(){
    // Read data from all touch buttons
    if (comMode == COMMS::I2C_MODE) {
        // Compute buffer length
        int length = 0;
        if (touchSize < ENCHANTI_BASETOUCHSIZE) {
            // Each sensor needs 2 bytes == 120 Byte read
            length = touchSize * 2;
        } else {
            length = ENCHANTI_BASETOUCHSIZE * 2;
        }

        // Read touch data from I2C buffer
        readI2CBuffer(main_i2c_addr, baseReg, length);

        // // Read auxillary touch board data
        // if (num_boards > 1) {
        //     length = (touchSize - ENCHANTI_BASETOUCHSIZE) * 2;
        //     readI2CBuffer(aux_i2c_addr, baseReg, length, ENCHANTI_BASETOUCHSIZE); // offset data index to not overwrite main touch board data
        // }
    }
}

void EnchantiTouch::cookData() {
    // Get max value, but also use it to check if any touch is pressed
    int instant_maxTouchValue = *std::max_element(data, data+touchSize);

    // Touching the T-Stick or not?
    if (instant_maxTouchValue == 0) {
        touchStatus = 0;
    } else {
        touchStatus = 1;
    }

    // We need a updated maxTouchValue to normalize touch
    maxTouchValue = std::max(maxTouchValue,instant_maxTouchValue);

    // Touch discretize and normalize
    for (int i=0; i < touchSize; i++) {
        if (data[i] < noise_threshold) {
            touch[i] = 0;
        } else {
            touch[i] = data[i] - noise_threshold;
        }

        // Set other data vectors
        if (touch[i] != 0) {
            discreteTouch[i] = 1;
            normTouch[i] = (data[i] *100) / maxTouchValue;
        } else {
            normTouch[i] = 0;
            discreteTouch[i] = 0;
        }
    }
}

void EnchantiTouch::readI2CBuffer(uint8_t i2c_addr, uint8_t reg, uint8_t length, int offset)
{
    // If previous transfer complete read available data
    if (g_MasterCompletionFlag) {
        memcpy(data, g_master_rxBuff, sizeof(uint8_t)*length);
        g_MasterCompletionFlag = false;
        newData = true;
    }

    // Update transfer settings
    i2c_master_transfer_t masterXfer;
    masterXfer.slaveAddress   = i2c_addr;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = (uint32_t)reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = g_master_rxBuff;
    masterXfer.dataSize       = length;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    // Start new transfer
    I2C_MasterTransferDMA(HAL_I2C_MASTER, &g_m_dma_handle, &masterXfer);
}

bool EnchantiTouch::ready() {
    bool ret = newData;
    newData = false;
    return ret;
}

// I2C transfer functions for non blocking I2C call
static void i2c_master_dma_callback(I2C_Type *base, i2c_master_dma_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

