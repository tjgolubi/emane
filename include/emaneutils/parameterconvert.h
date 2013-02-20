/*
 * Copyright (c) 2009 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANEUTILSPARAMETERCONVERT_HEADER_
#define EMANEUTILSPARAMETERCONVERT_HEADER_

#include "emane/emaneexception.h"

#include <string>

#include <ace/Basic_Types.h>
#include <ace/INET_Addr.h>

namespace EMANEUtils
{
  /**
   * @class ParameterConvert
   *
   * @brief Parameter conversion class with range checks
   *
   */
  class ParameterConvert
  {
  public:

    /**
     * @class ConversionException
     *
     * @brief Parameter conversion exception class
     *
     */
    class ConversionException : public EMANE::EMANEException
    {
    public:
      ConversionException(const std::string & sDescription):
        EMANEException("ConvertionException",sDescription){}
      
      ~ConversionException() throw(){}
    };

    ParameterConvert(const std::string & sParameter);
    
    ~ParameterConvert();
    
    /**
     * Convert paraemter string to an ACE_INT64
     *
     * @param i64Min Minimum value in range
     * @param i64Max Maximum value in range
     *
     * @return ACE_INT64 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_INT64 toINT64(ACE_INT64 i64Min = ACE_INT64_MIN , ACE_INT64 i64Max = ACE_INT64_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_UINT64
     *
     * @param u64Min Minimum value in range
     * @param u64Max Maximum value in range
     *
     * @return ACE_UINT64 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_UINT64 toUINT64(ACE_UINT64 u64Min = 0 , ACE_UINT64 u64Max = ACE_UINT64_MAX) const
      throw(ConversionException);

    
    /**
     * Convert paraemter string to an ACE_INT32
     *
     * @param i32Min Minimum value in range
     * @param i32Max Maximum value in range
     *
     * @return ACE_INT32 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_INT32 toINT32(ACE_INT32 i32Min = ACE_INT32_MIN , ACE_INT32 i32Max = ACE_INT32_MAX) const
    throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_UINT32
     *
     * @param u32Min Minimum value in range
     * @param u32Max Maximum value in range
     *
     * @return ACE_UINT32 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_UINT32 toUINT32(ACE_UINT32 u32Min = 0 , ACE_UINT32 u32Max = ACE_UINT32_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_INT16
     *
     * @param i16Min Minimum value in range
     * @param i16Max Maximum value in range
     *
     * @return ACE_INT16 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_INT16 toINT16(ACE_INT16 i16Min = ACE_INT16_MIN , ACE_INT16 i16Max = ACE_INT16_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_UINT16
     *
     * @param u16Min Minimum value in range
     * @param u16Max Maximum value in range
     *
     * @return ACE_UINT16 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_UINT16 toUINT16(ACE_UINT16 u16Min = 0 , ACE_UINT16 u16Max = ACE_UINT16_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_INT8
     *
     * @param i8Min Minimum value in range
     * @param i8Max Maximum value in range
     *
     * @return ACE_INT8 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_INT8 toINT8(ACE_INT8 i8Min = ACE_CHAR_MIN , ACE_INT8 i8Max = ACE_CHAR_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_UINT8
     *
     * @param u8Min Minimum value in range
     * @param u8Max Maximum value in range
     *
     * @return ACE_UINT8 value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_UINT8 toUINT8(ACE_UINT8 u8Min = 0 , ACE_UINT8 u8Max = ACE_OCTET_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to a float
     *
     * @param fMin Minimum value in range
     * @param fMax Maximum value in range
     *
     * @return float value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    float toFloat(float fMin = -ACE_FLT_MAX, float fMax = ACE_FLT_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to a double
     *
     * @param dMin Minimum value in range
     * @param dMax Maximum value in range
     *
     * @return double value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    double toDouble(double dMin = -ACE_DBL_MAX, double dMax = ACE_DBL_MAX) const
      throw(ConversionException);

    /**
     * Convert paraemter string to an ACE_INET_Addr
     *
     * @return ACE_INET_Addr value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    ACE_INET_Addr toINETAddr() const
      throw(ConversionException);

    /**
     * Convert paraemter string to an bool
     *
     * @return bool value
     *
     * @exception ConversionException Thrown when an error is encountered during
     * convertion either to to input format or out of range value.
     */
    bool toBool() const
      throw(ConversionException);

  private:
    std::string sParameter_;
  };
}

#include "parameterconvert.inl"

#endif //EMANEUTILSPARAMETERCONVERT_HEADER_
