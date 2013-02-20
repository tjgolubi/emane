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

#include "deploymentmanager.h"

#include <sstream>

#include <ace/Guard_T.h>

EMANE::DeploymentManager::DeploymentManager(){}

EMANE::DeploymentManager::~DeploymentManager(){}

void EMANE::DeploymentManager::addNEMPlatformEntry(EMANEUtils::NEMId nemId, EMANEUtils::PlatformId platformId)
  throw(DuplicateNEMException)
{
  ACE_Write_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);

  if(nemSet_.insert(nemId).second)
    {
      nemPlatformMapping_.insert(std::make_pair(nemId,platformId));
      
      std::pair<PlatformNEMMembership::iterator,bool> ret =
        platformNEMMembership_.insert(std::make_pair(platformId,NEMSet()));
      
      ret.first->second.insert(nemId);
    }
  else
    {
      std::stringstream msg;
      msg << "Duplicate node insert attempted: " 
          << platformId << " :: " << nemId;

      throw DuplicateNEMException(msg.str());
    }
}

EMANE::DeploymentManager::NEMSet EMANE::DeploymentManager::getNEMSet() const
{
  ACE_Read_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);
  return nemSet_;
}

EMANE::DeploymentManager::NEMPlatformMapping EMANE::DeploymentManager::getNEMPlatformMapping() const
{
  ACE_Read_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);
  return nemPlatformMapping_;
}

EMANE::DeploymentManager::PlatformNEMMembership EMANE::DeploymentManager::getPlatformNEMMembership() const
{
  ACE_Read_Guard<ACE_RW_Thread_Mutex> guard(rwmutex_);
  return platformNEMMembership_;
}
