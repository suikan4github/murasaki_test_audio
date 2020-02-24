/**
 * \file platform_defs.hpp
 *
 * \date 2018/01/16
 * \author Seiichi "Suikan" Horie
 * \brief Murasaki platform customize file.
 * \details
 * This file contains user defined struct murasaki::Platform.
 *
 * This file will be included by \ref murasaki.hpp.
 *
 */

#ifndef PLATFORM_DEFS_HPP_
#define PLATFORM_DEFS_HPP_

namespace murasaki {
/**
 * @ingroup MURASAKI_PLATFORM_GROUP
 * \{
 */
/**
 * \brief Custom aggregation struct for user platform.
 * \details
 * A collection of the peripheral / MPU control variable.
 *
 * This is a custom struct. Programmer can change this struct as suitable to the hardware and software.
 * But debugger_ member variable have to be left untouched.
 *
 * In the run time, the debugger_ variable have to be initialized by appropriate \ref murasaki::Debugger class instance.
 *
 * See \ref murasaki::platform
 */
struct Platform
{
    // Platform dependent Custom variables.

    UartStrategy *uart_console;    ///< UART wrapping class object for debugging
    LoggerStrategy *logger;        ///< logging class object for debugger

    BitOutStrategy *led_st0;           ///< GP out
    BitOutStrategy *led_st1;           ///< GP out
    TaskStrategy *audio_task;      ///< Driving Audio

    I2CMasterStrategy *i2c_master;  ///< I2C Master

    AudioCodecStrategy *codec;      ///< adau1361
    AudioPortAdapterStrategy *sai;      ///< SAI peripheral adapter
    DuplexAudio *audio;             ///< duplex audio class.
};

/**
 * \brief Grobal variable to provide the access to the platform component.
 * \details
 * This variable is declared by murasaki platform. But not instantiated. To make it happen,
 * programmer have to make an variable and initilize it explicitly.
 *
 * Note that the instantiation of this variable is optional. This is provided just of
 * ease of read.
 *
 */
extern Platform platform;

/**
 * \}
 * end of defgroup PLATFORM_GROUP
 */

} /* namespace murasaki */

#endif /* PLATFORM_DEFS_HPP_ */
