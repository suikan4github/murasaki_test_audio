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

// Include the prototype  of functions of this file.

/* -------------------- PLATFORM Macros -------------------------- */
#define AUDIO_CHANNEL_LEN 48
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
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s1;
extern I2S_HandleTypeDef hi2s2;

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
    MURASAKI_ASSERT(nullptr != murasaki::platform.i2c_master)

    // CODEC initialization.
    // Fs is up to application. MCLOCK must follow the board design.
    murasaki::platform.codec = new murasaki::Adau1361(
                                                      48000, /* Fs Sample/s */
                                                      12000000, /* MCLOCK frequency [Hz] */
                                                      murasaki::platform.i2c_master, /* I2C driver */
                                                      CODEC_ADDRESS); /* I2C address of ADAU1361 on Akashi02 */
    MURASAKI_ASSERT(nullptr != murasaki::platform.codec)

    // Initialize audio peripheral
    // By Akashi02 board design, I2S1 and I2S2 are assigned to RX and TX, respectively.
    murasaki::platform.audio_port = new murasaki::I2sPortAdapter(&hi2s1, &hi2s2);
    MURASAKI_ASSERT(nullptr != murasaki::platform.audio_port)

    // Initializing audio.
    murasaki::platform.audio = new murasaki::DuplexAudio(
                                                         murasaki::platform.audio_port, /* Peripheral adapter */
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
    static int count = 0;

    I2cSearch(murasaki::platform.i2c_master);

    murasaki::platform.audio_task->Start();

    // Loop forever
    while (true) {

        // print a message with counter value to the console.
        murasaki::debugger->Printf("Hello %d \n", count);

        // update the counter value.
        count++;

        // wait for a while
        murasaki::Sleep(500);
    }
}

/* ------------------------- UART ---------------------------------- */

/**
 * @brief Essential to sync up with UART.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param huart
 * @details
 * This is called from inside of HAL when an UART transmission done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::TransmissionCompleteCallback() function.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
                             {
    // Poll all uart tx related interrupt receivers.
    // If hit, return. If not hit,check next.
    if (murasaki::platform.uart_console->TransmitCompleteCallback(huart))
        return;

}

/**
 * @brief Essential to sync up with UART.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param huart
 * @details
 * This is called from inside of HAL when an UART receive done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default RX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::ReceiveCompleteCallback() function.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
                             {
    // Poll all uart rx related interrupt receivers.
    // If hit, return. If not hit,check next.
    if (murasaki::platform.uart_console->ReceiveCompleteCallback(huart))
        return;

}

/**
 * @brief Optional error handling of UART
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param huart
 * @details
 * This is called from inside of HAL when an UART error interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::HandleError() function.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Poll all uart error related interrupt receivers.
    // If hit, return. If not hit,check next.
    if (murasaki::platform.uart_console->HandleError(huart))
        return;

}

/* -------------------------- SPI ---------------------------------- */

#ifdef HAL_SPI_MODULE_ENABLED

/**
 * @brief Essential to sync up with SPI.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hspi
 * @details
 * This is called from inside of HAL when an SPI transfer done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX/RX interrupt call back.
 *
 * In this call back, the SPI device handle have to be passed to the
 * murasaki::Spi::TransmitAndReceiveCompleteCallback () function.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Poll all SPI TX RX related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
     if ( murasaki::platform.spi1->TransmitAndReceiveCompleteCallback(hspi) )
     return;
#endif
}

/**
 * @brief Optional error handling of SPI
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hspi
 * @details
 * This is called from inside of HAL when an SPI error interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::HandleError() function.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi) {
    // Poll all SPI error interrupt related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
     if ( murasaki::platform.spi1->HandleError(hspi) )
     return;
#endif
}

#endif

/* -------------------------- I2C ---------------------------------- */

#ifdef HAL_I2C_MODULE_ENABLED

/**
 * @brief Essential to sync up with I2C.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C transmission done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::I2c::TransmitCompleteCallback() function.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
                                  {
    // Poll all I2C master tx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 1
    if (murasaki::platform.i2c_master->TransmitCompleteCallback(hi2c))
        return;
#endif
}

/**
 * @brief Essential to sync up with I2C.
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C receive done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default RX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::ReceiveCompleteCallback() function.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Poll all I2C master rx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 1
    if (murasaki::platform.i2c_master->ReceiveCompleteCallback(hi2c))
        return;
#endif
}
/**
 * @brief Essential to sync up with I2C.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C transmission done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX interrupt call back.
 *
 * In this call back, the I2C slave device handle have to be passed to the
 * murasaki::I2cSlave::TransmitCompleteCallback() function.
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
                                 {
    // Poll all I2C master tx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
    if (murasaki::platform.i2c_slave->TransmitCompleteCallback(hi2c))
    return;
#endif
}

/**
 * @brief Essential to sync up with I2C.
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C receive done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default RX interrupt call back.
 *
 * In this call back, the I2C slave device handle have to be passed to the
 * murasaki::I2cSlave::ReceiveCompleteCallback() function.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Poll all I2C master rx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
    if (murasaki::platform.i2c_slave->ReceiveCompleteCallback(hi2c))
    return;
#endif
}

/**
 * @brief Optional error handling of I2C
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C error interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::I2c::HandleError() function.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    // Poll all I2C master error related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 1
    if (murasaki::platform.i2c_master->HandleError(hi2c))
        return;
#endif
}

#endif

/* ------------------ SAI  -------------------------- */
#ifdef HAL_SAI_MODULE_ENABLED
/**
 * @brief Optional SAI interrupt handler at buffer transfer halfway.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the SAI device.
 * @details
 * Invoked after SAI RX DMA complete interrupt is at halfway.
 * This interrupt have to be forwarded to the  murasaki::DuplexAudio::ReceiveCallback().
 * The second parameter of the ReceiveCallback() have to be 0 which mean the halfway interrupt.
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (murasaki::platform.audio->DmaCallback(hsai, 0)) {
        murasaki::platform.led_st0->Clear();
        murasaki::platform.led_st1->Set();
        return;
    }
}

/**
 * @brief Optional SAI interrupt handler at buffer transfer complete.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the SAI device.
 * @details
 * Invoked after SAI RX DMA complete interrupt is at halfway.
 * This interrupt have to be forwarded to the  murasaki::DuplexAudio::ReceiveCallback().
 * The second parameter of the ReceiveCallback() have to be 1 which mean the complete interrupt.
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (murasaki::platform.audio->DmaCallback(hsai, 1)) {
        murasaki::platform.led_st0->Set();
        murasaki::platform.led_st1->Clear();
        return;
    }
}

/**
 * @brief Optional SAI error interrupt handler.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the SAI device.
 * @details
 * The error have to be forwarded to murasaki::DuplexAudio::HandleError().
 * Note that DuplexAudio::HandleError() trigger a hard fault.
 * So, never return.
 */

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    if (murasaki::platform.audio->HandleError(hsai))
        return;
}

#endif

/* ------------------ I2S  -------------------------- */
#ifdef HAL_I2S_MODULE_ENABLED
/**
 * @brief Optional I2S interrupt handler at buffer transfer halfway.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the I2S device.
 * @details
 * Invoked after I2S RX DMA complete interrupt is at halfway.
 * This interrupt have to be forwarded to the  murasaki::DuplexAudio::ReceiveCallback().
 * The second parameter of the ReceiveCallback() have to be 0 which mean the halfway interrupt.
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (murasaki::platform.audio->DmaCallback(hi2s, 0)) {
        murasaki::platform.led_st0->Clear();
        murasaki::platform.led_st1->Set();
        return;
    }
}

/**
 * @brief Optional I2S interrupt handler at buffer transfer complete.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the I2S device.
 * @details
 * Invoked after I2S RX DMA complete interrupt is at halfway.
 * This interrupt have to be forwarded to the  murasaki::DuplexAudio::ReceiveCallback().
 * The second parameter of the ReceiveCallback() have to be 1 which mean the complete interrupt.
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (murasaki::platform.audio->DmaCallback(hi2s, 1)) {
        murasaki::platform.led_st0->Set();
        murasaki::platform.led_st1->Clear();
        return;
    }
}

/**
 * @brief Optional I2S error interrupt handler.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the I2S device.
 * @details
 * The error have to be forwarded to murasaki::DuplexAudio::HandleError().
 * Note that DuplexAudio::HandleError() trigger a hard fault.
 * So, never return.
 */

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    if (murasaki::platform.audio->HandleError(hi2s))
        return;
}

#endif

/* -------------------------- GPIO ---------------------------------- */

/**
 * @brief Optional interrupt handling of EXTI
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param GPIO_Pin Pin number from 0 to 31
 * @details
 * This is called from inside of HAL when an EXTI is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * The GPIO_Pin is the number of Pin. For example, if a programmer set the pin name by CubeIDE as FOO, the
 * macro to identify that EXTI is FOO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
                            {
#if 0
    // Sample of the EXTI call back.
    // USER_Btn is a standard name of the user push button switch of the Nucleo F722.
    // This switch can be configured as EXTI interrupt srouce.
    // In this sample, releasing the waiting task if interrupt comes.

    // Check whether appropriate interrupt or not
    if ( USER_Btn_Pin == GPIO_Pin) {
        // Check whether sync object is ready or not.
        // This check is essential from the interrupt before the platform variable is ready
        if (murasaki::platform.sync_with_button != nullptr)
        // release the waiting task
            murasaki::platform.sync_with_button->Release();
    }
#endif
}

/* ------------------ ASSERTION AND ERROR -------------------------- */

/**
 * @brief Hook for the assert_failure() in main.c
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param file Name of the source file where assertion happen
 * @param line Number of the line where assertion happen
 * @details
 * This routine provides a custom hook for the assertion inside STM32Cube HAL.
 * All assertion raised in HAL will be redirected here.
 *
 * @code
 * void assert_failed(uint8_t* file, uint32_t line)
 * {
 *     CustomAssertFailed(file, line);
 * }
 * @endcode
 * By default, this routine output a message with location informaiton
 * to the debugger console.
 */
void CustomAssertFailed(uint8_t *file, uint32_t line)
                        {
    murasaki::debugger->Printf("Wrong parameters value: file %s on line %d\n",
                               file,
                               line);
    // To stop the execusion, raise assert.
    MURASAKI_ASSERT(false);
}

/*
 * CustmDefaultHanlder :
 *
 * An entry of the exception. Especialy for the Hard Fault exception.
 * In this function, the Stack pointer just before exception is retrieved
 * and pass as the first parameter of the PrintFaultResult().
 *
 * Note : To print the correct information, this function have to be
 * Jumped in from the exception entry without any data push to the stack.
 * To avoid the pushing extra data to stack or making stack frame,
 * Compile the program without debug information and with certain
 * optimization leve, when you investigate the Hard Fault.
 */
__asm volatile (
        ".global CustomDefaultHandler \n"
        "CustomDefaultHandler: \n"
        " movs r0,#4       \n"
        " movs r1, lr      \n"
        " tst r0, r1       \n"
        " beq _MSP         \n"
        " mrs r0, psp      \n"
        " b _HALT          \n"
        "_MSP:               \n"
        " mrs r0, msp      \n"
        "_HALT:              \n"
        " ldr r1,[r0,#20]  \n"
        " b PrintFaultResult \n"
        " bkpt #0          \n"
);

/**
 * @brief Printing out the context information.
 * @param stack_pointer retrieved stack pointer before interrupt / exception.
 * @details
 * Do not call from application. This is murasaki_internal_only.
 *
 */
void PrintFaultResult(unsigned int *stack_pointer) {

    murasaki::debugger->Printf("\nSpurious exception or hardfault occured.  \n");
    murasaki::debugger->Printf("Stacked R0  : 0x%08X \n", stack_pointer[0]);
    murasaki::debugger->Printf("Stacked R1  : 0x%08X \n", stack_pointer[1]);
    murasaki::debugger->Printf("Stacked R2  : 0x%08X \n", stack_pointer[2]);
    murasaki::debugger->Printf("Stacked R3  : 0x%08X \n", stack_pointer[3]);
    murasaki::debugger->Printf("Stacked R12 : 0x%08X \n", stack_pointer[4]);
    murasaki::debugger->Printf("Stacked LR  : 0x%08X \n", stack_pointer[5]);
    murasaki::debugger->Printf("Stacked PC  : 0x%08X \n", stack_pointer[6]);
    murasaki::debugger->Printf("Stacked PSR : 0x%08X \n", stack_pointer[7]);

    murasaki::debugger->Printf("       CFSR : 0x%08X \n", *(unsigned int*) 0xE000ED28);
    murasaki::debugger->Printf("       HFSR : 0x%08X \n", *(unsigned int*) 0xE000ED2C);
    murasaki::debugger->Printf("       DFSR : 0x%08X \n", *(unsigned int*) 0xE000ED30);
    murasaki::debugger->Printf("       AFSR : 0x%08X \n", *(unsigned int*) 0xE000ED3C);

    murasaki::debugger->Printf("       MMAR : 0x%08X \n", *(unsigned int*) 0xE000ED34);
    murasaki::debugger->Printf("       BFAR : 0x%08X \n", *(unsigned int*) 0xE000ED38);

    murasaki::debugger->Printf("(Note : To avoid the stacking by C compiler, use release build to investigate the fault. ) \n");

    murasaki::debugger->DoPostMortem();
}

/**
 * @brief StackOverflow hook for FreeRTOS
 * @param xTask Task ID which causes stack overflow.
 * @param pcTaskName Name of the task which cuases stack overflow.
 * @fn vApplicationStackOverflowHook
 * @details
 * This function will be called from FreeRTOS when some task causes overflow.
 * See TaskStrategy::getStackMinHeadroom() for details.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName) {
    murasaki::debugger->Printf("Stack overflow at task : %s \n", pcTaskName);
    MURASAKI_ASSERT(false);
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

    murasaki::platform.codec->Start();

    murasaki::SetSyslogFacilityMask(murasaki::kfaI2s);
    murasaki::SetSyslogSererityThreshold(murasaki::kseDebug);

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

    int count = 1;
    // Loop forever
    while (true) {

        if (count == 5)
                {
            murasaki::SetSyslogSererityThreshold(murasaki::kseError);
        }
        else
        {
            count++;
        }
        // Talk through : copy received data to transmit.
        for (int i = 0; i < AUDIO_CHANNEL_LEN; i++) {
#if 0
            l_tx[i] = l_rx[i];
            r_tx[i] = r_rx[i];
#else
            l_tx[i] = l_rx[i] * 0.9;
            r_tx[i] = r_rx[i] * 0.9;
#endif
        }

        // Wait last TX/RX. Then, copy TX data to DMA TX buffer and copy DMA RX buffer to RX data.
        murasaki::platform.audio->TransmitAndReceive(
                                                     l_tx,
                                                     r_tx,
                                                     l_rx,
                                                     r_rx);

    }

}