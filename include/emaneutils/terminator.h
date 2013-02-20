/*
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSTERMINATOR_HEADER_
#define EMANEUTILSTERMINATOR_HEADER_

#include <ace/Event_Handler.h>
#include <ace/Reactor.h>

namespace EMANEUtils
{
  /**
   *
   * @class Terminator
   *
   * @brief Class to handle termination signals SIGINT and SIGQUIT.
   *
   */
  class Terminator : public ACE_Event_Handler
  {
  public:
    /**
     *
     * @brief constructor.
     *
     */
    Terminator()
    {
      ACE_Reactor::instance()->register_handler(SIGINT,this);
      ACE_Reactor::instance()->register_handler(SIGQUIT,this);
    }
    
    
    /**
     *
     * @brief destructor.
     *
     */
    ~Terminator()
    {
      ACE_Reactor::instance()->remove_handler(SIGINT,NULL,NULL,-1);
      ACE_Reactor::instance()->remove_handler(SIGQUIT,NULL,NULL,-1);
    }
    
    /**
     *
     * @brief callback method to handle termination signal.
     * 
     * @param sig signal number.
     * @param info siginfo.
     * @param context context info.
     *
     * @retval 0
     *
     */
    int handle_signal(int sig, siginfo_t *info, ucontext_t *context)
    {
      ACE_UNUSED_ARG(sig);
      ACE_UNUSED_ARG(info);
      ACE_UNUSED_ARG(context);

      ACE_Reactor::instance()->end_reactor_event_loop();
      return 0;
    }
  };
}

#endif //EMANEUTILSTERMINATOR_HEADER_
