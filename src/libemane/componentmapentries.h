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
#include "emane/emanetypes.h"
#include "emane/emanecomponenttypes.h"
#include <vector>

namespace EMANE
{
  /**
   * @class LeafEntry
   *
   * @brief ComponentMap registration type used to describe registration
   *        information for leaf objects (those that don't contain
   *        other objects).
   */
  template <class L>
  class LeafEntry
  {
  public:
    /**
     * Constructor
     *
     * @param bid the BuildId of the Leaf object
     * @param type the ComponentType of the Leaf object
     * @param pInstance a pointer to the Leaf object instance
     */    
    LeafEntry(BuildId bid, ComponentType type, L * pInstance) :  
      bid_(bid), 
      type_(type),
      pInstance_(pInstance) {}
    
    /**
     * getBuildId
     *
     * @return The BuildId of the Leaf object
     */    
    BuildId getBuildId() const { return bid_; }

    /**
     * getType
     *
     * @return The ComponentType of the Leaf object
     */    
    ComponentType getType() const { return type_; }

    /**
     * getInstance
     *
     * @return A pointer to the instance of the leaf object
     */    
    L * getInstance() const { return pInstance_; }

  private:
    BuildId bid_;
    ComponentType type_;
    L * pInstance_;
  };

  /**
   * @class ContainerEntry
   *
   * @brief ComponentMap registration type used to describe registration
   *        information for container objects (those that contain other
   *        objects).
   */
  template <class C, class L, typename IDTYPE>
  class ContainerEntry
  {
  public:
    /**
     * Constructor
     *
     * @param bid the BuildId of the Leaf object
     * @param pInstance a pointer to the Leaf object instance
     * @param id the native id of this container (example NEMId)
     */    
    ContainerEntry(BuildId bid, C * pInstance, IDTYPE id) :  
      bid_(bid), 
      pInstance_(pInstance),
      id_(id) {}
    
    /**
     * getBuildId
     *
     * @return The BuildId of this Container Object
     */    
    BuildId getBuildId() const { return bid_; }

    /**
     * getInstance
     *
     * @return A pointer to the instance of the Container object
     */    
    C * getInstance() const { return pInstance_; }

    /**
     * getId
     *
     * @return The native ID of this Container object (example NEMId)
     */    
    IDTYPE getId() const { return id_; }

    /**
     * addChild
     *
     * @param childentry A child entry object to add to this ContainerEntry.
     *        Children may be Leaf or other Container entries
     */    
    void addChild(L * childentry) { childentries_.push_back(childentry); }

    /**
     * getChildEntries
     *
     * @return A vector of the child entires of this container
     */    
    std::vector<L*> getChildEntries() const { return childentries_; }

  private:
    BuildId bid_;
    C * pInstance_;
    IDTYPE id_;

    std::vector<L*> childentries_;
  };
}
