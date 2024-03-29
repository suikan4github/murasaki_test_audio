/**
 * @file murasaki_platform.cpp
 *
 * @date 2018/05/20
 * @author Seiichi "Suikan" Horie
 * @brief A glue file between the user application and HAL/RTOS.
 */

// Include the definition created by CubeIDE.
#include <murasaki_platform.hpp>
#include "main.h"

// Include the murasaki class library.
#include "murasaki.hpp"
#include "../drivers/codec/adau1361.hpp"

// Include the prototype  of functions of this file.

/* -------------------- PLATFORM Macros -------------------------- */
#define AUDIO_CHANNEL_LEN 64
#define CODEC_ADDRESS 0x38

/* -------------------- PLATFORM Type and classes -------------------------- */

/* -------------------- PLATFORM Variables-------------------------- */

// Essential definition.
// Do not delete
murasaki::Platform murasaki::platform;
murasaki::Debugger *murasaki::debugger;

/* ------------------------ STM32 Peripherals ----------------------------- */

/*
 * Platform dependent peripheral declaration.
 *
 * The variables here are defined at the top of the main.c.
 * Only the variable needed by the InitPlatform() are declared here
 * as external symbols.
 *
 * The declaration here is user project dependent.
 */
// Following block is just sample.
// Original declaration is in the top of main.c.
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;

/* -------------------- PLATFORM Prototypes ------------------------- */

void TaskBodyFunction(const void *ptr);

/* -------------------- PLATFORM Implementation ------------------------- */

void InitPlatform()
{
#if ! MURASAKI_CONFIG_NOCYCCNT
    // Start the cycle counter to measure the cycle in MURASAKI_SYSLOG.
    murasaki::InitCycleCounter();
#endif
    // UART device setting for console interface.
    // On Nucleo, the port connected to the USB port of ST-Link is
    // referred here.
    murasaki::platform.uart_console = new murasaki::DebuggerUart(&huart3);
    while (nullptr == murasaki::platform.uart_console)
        ;  // stop here on the memory allocation failure.

    // UART is used for logging port.
    // At least one logger is needed to run the debugger class.
    murasaki::platform.logger = new murasaki::UartLogger(murasaki::platform.uart_console);
    while (nullptr == murasaki::platform.logger)
        ;  // stop here on the memory allocation failure.

    // Setting the debugger
    murasaki::debugger = new murasaki::Debugger(murasaki::platform.logger);
    while (nullptr == murasaki::debugger)
        ;  // stop here on the memory allocation failure.

    // Set the debugger as AutoRePrint mode, for the easy operation.
    murasaki::debugger->AutoRePrint();  // type any key to show history.

    // The port and pin names are fined by CubeIDE.
    murasaki::platform.led_st0 = new murasaki::BitOut(ST0_GPIO_Port, ST0_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led_st0)

    murasaki::platform.led_st1 = new murasaki::BitOut(ST1_GPIO_Port, ST1_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led_st1)

    murasaki::platform.i2c_master = new murasaki::I2cMaster(&hi2c1);

    // Initialize audio peripheral
    // By Akashi02 board design, SAI1_B and SAI1_A are assigned to RX and TX, respectively.
    murasaki::platform.sai = new murasaki::SaiPortAdapter(
                                                          &hsai_BlockB1, /* TX */
                                                          &hsai_BlockA1); /* RX */
    MURASAKI_ASSERT(nullptr != murasaki::platform.sai)
    // CODEC initialization.
    // Fs is up to application. MCLOCK must follow the board design.
    murasaki::platform.codec = new murasaki::Adau1361(
                                                      48000, /* Fs Sample/s */
                                                      12000000, /* MCLOCK frequency [Hz] */
                                                      murasaki::platform.i2c_master, /* I2C driver */
                                                      CODEC_ADDRESS); /* I2C address of ADAU1361 on Akashi02 */
    MURASAKI_ASSERT(nullptr != murasaki::platform.codec)

    // Initializing audio.
    murasaki::platform.audio = new murasaki::DuplexAudio(
                                                         murasaki::platform.sai, /* Peripheral adapter */
                                                         AUDIO_CHANNEL_LEN); /* Channel Block length by one DMA  */
    MURASAKI_ASSERT(nullptr != murasaki::platform.audio)

    // For demonstration of audio task.
    murasaki::platform.audio_task = new murasaki::SimpleTask(
                                                             "audio_task",
                                                             256,
                                                             murasaki::ktpRealtime,
                                                             nullptr,
                                                             &TaskBodyFunction
                                                             );
    MURASAKI_ASSERT(nullptr != murasaki::platform.audio_task)
}

void ExecPlatform()
{
    // counter for the demonstration.
    int count = 0;

    murasaki::platform.audio_task->Start();

    // Loop forever
    while (true) {

        // print a message with counter value to the console.
        murasaki::debugger->Printf("Hello %d \n", count);

        // update the counter value.
        count++;

        // wait for a while
        murasaki::Sleep(5000);
    }
}


/* ------------------ User Functions -------------------------- */
/**
 * @brief Demonstration task.
 * @param ptr Pointer to the parameter block
 * @details
 * Task body function as demonstration of the @ref murasaki::SimpleTask.
 *
 * You can delete this function if you don't use.
 */
void TaskBodyFunction(const void *ptr) {
    // phase of the oscillator.
    // Let's make phase [0,36000) representing [0,360).

    // audio buffer for left and right, tx and rx.
    float *l_tx = new float[AUDIO_CHANNEL_LEN];
    float *r_tx = new float[AUDIO_CHANNEL_LEN];
    float *l_rx = new float[AUDIO_CHANNEL_LEN];
    float *r_rx = new float[AUDIO_CHANNEL_LEN];

    murasaki::platform.led_st0->Set();
    murasaki::platform.led_st1->Clear();

    murasaki::platform.codec->Start();

    murasaki::SetSyslogFacilityMask(murasaki::kfaAudioCodec);
    murasaki::SetSyslogSeverityThreshold(murasaki::kseDebug);

    murasaki::platform.codec->SetGain(
                                      murasaki::kccLineInput,
                                      0.0,
                                      0.0);  // right gain in dB, left gain in dB
    murasaki::platform.codec->SetGain(
                                      murasaki::kccHeadphoneOutput,
                                      0.0,
                                      0.0);  // right gain in dB, left gain in dB
    murasaki::platform.codec->Mute(murasaki::kccLineInput, false);
    murasaki::platform.codec->Mute(murasaki::kccHeadphoneOutput, false);

    // Loop forever
    while (true) {

        // Talk through : copy received data to transmit.
        for (int i = 0; i < AUDIO_CHANNEL_LEN; i++) {
            l_tx[i] = l_rx[i] * 0.9;
            r_tx[i] = r_rx[i] * 0.9;
        }

        // Wait last TX/RX. Then, copy TX data to DMA TX buffer and copy DMA RX buffer to RX data.
        murasaki::platform.audio->TransmitAndReceive(
                                                     l_tx,
                                                     r_tx,
                                                     l_rx,
                                                     r_rx);
        murasaki::platform.led_st0->Toggle();
        murasaki::platform.led_st1->Toggle();
    }

}

