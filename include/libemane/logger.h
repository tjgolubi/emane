/*
 * Copyright (c) 2011 - DRS CenGen, LLC, Columbia, Maryland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of DRS CenGen, LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The copyright holder hereby grants to the U.S. Government a copyright
 * license to use this computer software/computer software documentation
 * that is of the same scope as the rights set forth in the definition of
 * "unlimited rights" found in DFARS 252.227-7014(a)(15)(June 1995).
 */

#ifndef EMANELOGGER_HEADER_
#define EMANELOGGER_HEADER_

#include "emane/emanetypes.h"
#include "emane/emanelogserviceprovider.h"

namespace EMANE
{
  /**
   * @class Logger
   *
   * @brief An instance of the EMANE logger. Provides methods for logger
   *        configuration and logging.
   *
   */
  class Logger
  {
  public:
    Logger();
    ~Logger();

    /**
     * Set Log Level
     *
     * @param level highest logging level
     */
    void setLogLevel(const LogLevel level);

    /**
     * Redirect logging to syslog
     *
     * @param program the tag name of logged statements in syslog
     */
    void redirectLogsToSysLog(const char *program);

    /**
     * Redirect logging to file
     *
     * @param filename the name of the destination file
     */
    void redirectLogsToFile(const char* filename); 

    /**
     * Redirect logging to remoet logger
     *
     * @param program the tag name of logged statements in syslog
     * @param address the address of the remote logger
     */
    void redirectLogsToRemoteLogger(const char *program, 
                                    const char* address);

    /**
     * Output a log message
     *
     * @param level Log level of the message
     * @param fmt format string (see printf)
     * @param ... Variable data (see printf)
     */
    void log(LogLevel level, const char *fmt, ...)  __attribute__ ((format (printf, 3, 4)));

    /**
     * open the logging control port
     *
     * @param LoggerPort the port number to accept logging control 
     */
    void open(const unsigned short LoggerPort);
  };
}

#endif // EMANELOGGER_HEADER_
