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

#include <cstdio>

inline
void EMANE::Component::configure(const ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  ConfigurationItems::const_iterator iter = items.begin();
  
  // iterate through all configuration items
  for(;iter != items.end(); ++iter)
    {
      ConfigurationRequirements::iterator reqIter;
      
      // test to see if the configuration item is a corresponding
      // requirement
      if((reqIter = configRequirements_.find(iter->getName())) !=
         configRequirements_.end())
        {
          // get distance between the iterators (effective count of entries)
          unsigned uiIterDist = 0;
          
          ConfigurationItems::const_iterator countIter = iter;

          // for exception verbocity, only
          ConfigurationItems::const_iterator problemIter = iter;

          for (; countIter != items.end(); ++countIter)
            {
              if (countIter->getName() == iter->getName())
                {
                  ++uiIterDist;
                  
                  problemIter = countIter;
                }
            }

          /* Verify count does not exceed requirement */
          if ( (uiIterDist > 1) && 
               (uiIterDist > reqIter->second.uiCount_) && 
               (reqIter->second.uiCount_ != 0) )
            {
              // too many configuration items with this name were received
              // throw an expectio
              throw ConfigureException("configuration item count exceeds requirement",*problemIter);
            }
          else
            {
              // if item has the default flag set, it's the first entry
              // no need to add another 
              if (reqIter->second.bPresent_ && reqIter->second.bDefault_)
                {
                  reqIter->second.bDefault_ = false;
                }
              // if item is already present, we have multiple params with same name
              else if (reqIter->second.bPresent_)
                {
                  // add another
                  reqIter = configRequirements_.insert(std::make_pair(iter->getName(), 
                                                                      ConfigurationRequirement(reqIter->second.bRequired_,
                                                                                               true,
                                                                                               false,
                                                                                               reqIter->second.uiCount_,
                                                                                               *iter)));
                }
              else
                {
                  // mark item as present
                  reqIter->second.bPresent_ = true;
                }

              // store the configuration items
              reqIter->second.item_ = *iter;
            }
        }
      else
        {
          // an unknown (unexpected) configuration item was receieved
          // throw an expection
          throw ConfigureException("unexpected configuration item",*iter);
        }
    }
}
