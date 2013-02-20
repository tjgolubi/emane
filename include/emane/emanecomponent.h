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

#ifndef EMANECOMPONENT_HEADER_
#define EMANECOMPONENT_HEADER_

#include "emane/emaneconfigurationitem.h"
#include "emane/emaneconfigureexception.h"
#include "emane/emanestartexception.h"
#include "emane/emaneconfigurationrequirement.h"

#include <list>

namespace EMANE
{
  /**
   * @class InitializeException
   *
   * @brief  Exception thrown during component initialization
   */
  class InitializeException{};


  /**
   * @class StopException
   *
   * @brief  Exception thrown during a component stop
   */
  class StopException{};
  
  typedef std::list<ConfigurationItem> ConfigurationItems;

  /**
   * @class Component
   *
   * @brief Generic interface used to configure and control all components.
   *
   * @details The component interface is used to transition all components
   * through the component state machine.
   *
   * -# initialize
   * -# configure
   * -# start
   * -# stop
   * -# destroy
   */
  class Component
  {
  public:
    virtual ~Component(){}
   
    /**
     * Initialize the component.
     *
     * @exception InitializeException Thrown when an error is encountered during
     * initialization
     */
    virtual void initialize()
      throw(InitializeException) = 0;
    
    /**
     * Configure the component.  Called by the NEMBuilder object during Platform
     * construction.  Default implementation compares configuration items to 
     * the specified configuration requirments.
     *
     * @param items List of configuration items
     *
     * @exception ConfigureException Thrown when a unexpected configuration item is
     * encountered
     */
    virtual void configure(const ConfigurationItems & items)
      throw(ConfigureException);

    /**
     * Start the component.  Called during Platform start for all the Platform
     * components.
     *
     * @exception StartException Thrown when an error is encountered during
     * start. One reason for this would be missing configuration.
     *
     */
    // start the component
    virtual void start()
      throw(StartException) = 0;


    /**
     * Hook to run any post start functionaililty.  Called after all the Platform
     * components have been started.
     */
    virtual void postStart(){};

    /**
     * Stop the component.
     *
     * @exception StopException Thrown when an error is encountered during
     * stop
     */
    virtual void stop()
      throw(StopException) = 0;
    
    /**
     * Destroy the component.
     *
     * @exception StopException Thrown when an error is encountered during
     * destroy
     */
    virtual void destroy()
      throw() = 0;
    
  protected:
    Component(){}
    
    ConfigurationRequirements configRequirements_;
  };
}

#include "emane/emanecomponent.inl"

#endif //EMANECOMPONENT_HEADER_

