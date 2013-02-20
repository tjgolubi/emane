/*
 * Copyright (c) 2008-2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANECONFIGURATIONITEM_HEADER_
#define EMANECONFIGURATIONITEM_HEADER_

#include <string>

/**
 * @class EMANE::ConfigurationItem emaneconfigurationitem.h "include/emaneconfigurationitem.h"
 *
 * @brief Container for a component configuration item.
 *
 * The ConfigurationItem class describes an item being passed into a component
 * during configuration (either initial or runtime). Data populating this item
 * is retrieved from the ConfigParser - usually in a std::list container.
 *
 */

namespace EMANE
{
  class ConfigurationItem
  {
  public:
    /**
     * Default Constructor
     */
    ConfigurationItem();

    /**
     * Parameterized Constructor
     *
     * @param name Const reference to a string with the name of the item
     * @param value Const reference to a string with the value of the item
     * @param desc Const reference to a string with the description of the item
     * @param type Const reference to a string with the type of the item
     */
    ConfigurationItem(const std::string &name, const std::string &value,
                      const std::string &desc, const std::string &type);

    /**
     * Parameterized Constructor
     *
     * @param name Const reference to a string with the name of the item
     * @param value Const reference to a string with the value of the item
     */
    ConfigurationItem(const std::string &name, const std::string &value);

    /**
     * Destructor
     */
    virtual ~ConfigurationItem(){}

    /**
     * Gets the value of the item
     *
     * @return copy of the string with the value of this item
     */
    std::string getValue() const;

    /**
     * Gets the name of the item
     *
     * @return copy of the string with the name of this item
     */
    std::string getName() const;

    /**
     * Gets the description of the item
     *
     * @return copy of the string with the description of this item
     */
    std::string getDescription() const;
   
    /**
     * Gets the type of the item
     *
     * @return copy of the string with the type of this item
     */
    std::string getType() const;

    /**
     * Sets the value of the item
     *
     * @param sValue const reference to the string with the value to be set
     */
    void setValue(const std::string & sValue);

    /**
     * Sets the name of the item
     *
     * @param sName const reference to the string with the name to be set
     */
    void setName(const std::string  & sName);

    /**
     * Sets the description of the item
     *
     * @param sDescription const reference to the string with the 
     *        description to be set
     */
    void setDescritpion(const std::string & sDescription);
   
    /**
     * Sets the type of the item
     *
     * @param sType const reference to the string with the type to be set
     */
    void setType(const std::string & sType);

  private:
    /**
     * Container for name of the item
     */
    std::string sName_;

    /**
     * Container for value of the item
     */
    std::string sValue_;

    /**
     * Container for description of the item
     */
    std::string sDescription_;

    /**
     * Container for type of the item
     */
    std::string sType_;
  };
}

#include "emane/emaneconfigurationitem.inl"

#endif //EMANECONFIGURATIONITEM_HEADER_
