/*
 * Copyright (c) 2009,2012 - DRS CenGen, LLC, Columbia, Maryland
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

#include <sstream>

#include <ace/OS_NS_errno.h>
#include <ace/OS_NS_stdlib.h>

namespace
{
  std::string scaleNumericalStringRepresentation(const std::string & sValue)
  {
    std::string sTmpParameter(sValue);
    
    ACE_UINT8 u8PowerOf10 = 0;
    
    switch(*(sValue.end() - 1))
        {
        case 'G':
          sTmpParameter.assign(sValue,0,sValue.size() - 1);
          u8PowerOf10 = 9;
          break;

        case 'M':
          sTmpParameter.assign(sValue,0,sValue.size() - 1);
          u8PowerOf10 = 6;
          break;

        case 'K':
          sTmpParameter.assign(sValue,0,sValue.size() - 1);
          u8PowerOf10 = 3;
          break;
        }
    
    if(u8PowerOf10 != 0)
      {
        // location of decimal point, if exists
        std::string::size_type indexPoint =  sTmpParameter.find(".",0);
        
        if(indexPoint != std::string::npos)
          {
             std::string::size_type numberOfDigitsAfterPoint = 
               sTmpParameter.size() - indexPoint - 1;
            
            if(numberOfDigitsAfterPoint > u8PowerOf10)
              {
                // need to move the decimal point, enough digits are present
                sTmpParameter.insert(sTmpParameter.size() - (numberOfDigitsAfterPoint - u8PowerOf10),
                                     ".");
              }
            else
              {
                // need to append 0s
                sTmpParameter.append(u8PowerOf10 - numberOfDigitsAfterPoint,'0');
              }
            
            // remove original decimal point
            sTmpParameter.erase(indexPoint,1);
          }
        else
          {
            // need to append 0s
            sTmpParameter.append(u8PowerOf10,'0');
          }
      }

    return sTmpParameter;
  }
}

inline
EMANEUtils::ParameterConvert::ParameterConvert(const std::string & sParameter):
  sParameter_(sParameter){}

inline
EMANEUtils::ParameterConvert::~ParameterConvert(){}

inline
ACE_INT64 EMANEUtils::ParameterConvert::toINT64(ACE_INT64 i64Min, ACE_INT64 i64Max) const
  throw(ConversionException)
{
  long long llValue = 0;

  if(sParameter_.empty())
    {
      throw ConversionException("Empty string in numeric conversion");
    }
  else
    {
      std::string sTmpParameter(scaleNumericalStringRepresentation(sParameter_));

      char * pEnd = 0;

      //Clear errno before making call because Ubuntu does not
      //clear it when a call is made
      errno = 0;

      llValue =  ACE_OS::strtoll(sTmpParameter.c_str(),&pEnd,0);

      if(errno == ERANGE ||
         llValue < i64Min ||
         llValue > i64Max)
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" out of range ["<<i64Min<<","<<i64Max<<"]"<<std::ends;
          throw ConversionException(sstream.str());
        }
      else if(pEnd != 0 && *pEnd !='\0')
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" invalid character in numeric: '"<<*pEnd<<"'"<<std::ends;
          throw ConversionException(sstream.str());
        }
    }
  
  return llValue;
}

inline
ACE_UINT64 EMANEUtils::ParameterConvert::toUINT64(ACE_UINT64 u64Min, ACE_UINT64 u64Max) const
 throw(ConversionException)
{
  unsigned long long ullValue = 0;
  
  if(sParameter_.empty())
    {
      throw ConversionException("Empty string in numeric conversion");
    }
  else
    {
      std::string sTmpParameter(scaleNumericalStringRepresentation(sParameter_));

      char * pEnd = 0;
      
      //Clear errno before making call because Ubuntu does not
      //clear it when a call is made
      errno = 0;

      ullValue =  ACE_OS::strtoull(sTmpParameter.c_str(),&pEnd,0);

      if(errno == ERANGE ||
         ullValue < u64Min ||
         ullValue > u64Max)
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" out of range ["<<u64Min<<","<<u64Max<<"]"<<std::ends;
          throw ConversionException(sstream.str());
        }
      else if(pEnd != 0 && *pEnd !='\0')
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" invalid character in numeric: '"<<*pEnd<<"'"<<std::ends;
          throw ConversionException(sstream.str());
        }
    }
  
  return ullValue;
}

inline
ACE_INT32 EMANEUtils::ParameterConvert::toINT32(ACE_INT32 i32Min, ACE_INT32 i32Max) const
  throw(ConversionException)
{
  return static_cast<ACE_INT32>(toINT64(i32Min,i32Max));
}

inline
ACE_UINT32 EMANEUtils::ParameterConvert::toUINT32(ACE_UINT32 u32Min , ACE_UINT32 u32Max) const
  throw(ConversionException)
{
  return static_cast<ACE_UINT32>(toUINT64(u32Min,u32Max));
}

inline
ACE_INT16 EMANEUtils::ParameterConvert::toINT16(ACE_INT16 i16Min, ACE_INT16 i16Max) const
  throw(ConversionException)
{
  return static_cast<ACE_INT16>(toINT64(i16Min,i16Max));
}

inline
ACE_UINT16 EMANEUtils::ParameterConvert::toUINT16(ACE_UINT16 u16Min , ACE_UINT16 u16Max) const
  throw(ConversionException)
{
  return static_cast<ACE_UINT16>(toUINT64(u16Min,u16Max));
}

inline
ACE_INT8 EMANEUtils::ParameterConvert::toINT8(ACE_INT8 i8Min, ACE_INT8 i8Max) const
  throw(ConversionException)
{
  return static_cast<ACE_INT8>(toINT64(i8Min,i8Max));
}

inline
ACE_UINT8 EMANEUtils::ParameterConvert::toUINT8(ACE_UINT8 u8Min , ACE_UINT8 u8Max) const
  throw(ConversionException)
{
  return static_cast<ACE_UINT8>(toUINT64(u8Min,u8Max));
}

inline
bool EMANEUtils::ParameterConvert::toBool() const
  throw(ConversionException)
{
  if(!strcasecmp(sParameter_.c_str(),"on")   ||
     !strcasecmp(sParameter_.c_str(),"yes")  ||
     !strcasecmp(sParameter_.c_str(),"true") ||
     !strcasecmp(sParameter_.c_str(),"1"))
    {
      return true;
    }
  else if(!strcasecmp(sParameter_.c_str(),"off")   ||
          !strcasecmp(sParameter_.c_str(),"no")  ||
          !strcasecmp(sParameter_.c_str(),"false") ||
          !strcasecmp(sParameter_.c_str(),"0"))
    {
      return false;
    }
  else
    {
      std::stringstream sstream;
      sstream<<"'"<<sParameter_<<"' invalid boolean conversion"<<std::ends;
      throw ConversionException(sstream.str());
    }
}

inline
ACE_INET_Addr EMANEUtils::ParameterConvert::toINETAddr() const
  throw(ConversionException)
{
  
  size_t pos = sParameter_.rfind("/");

  if(pos == std::string::npos)
    {
     pos  = sParameter_.rfind(":");
    }


  if(pos != std::string::npos)
    {
      std::string sAddress = sParameter_.substr(0,pos);
      std::string  sPort    = sParameter_.substr(pos+1);
      ACE_INET_Addr addr;
 
      if(addr.set(ParameterConvert(sPort).toUINT16(),sAddress.c_str()) == -1)
      {
         std::stringstream sstream;
         sstream<<"'"<<sParameter_<<"' Invalid IP Address"<<std::ends;
         throw ConversionException(sstream.str());
      }
      else
      {
         return addr;
      }
    }
  else
    {
      std::stringstream sstream;
      sstream<<"'"<<sParameter_<<"' bad endpoint format"<<std::ends;
      throw ConversionException(sstream.str());
    }
}

inline
double EMANEUtils::ParameterConvert::toDouble(double dMin, double dMax) const
  throw(ConversionException)
{
  double dValue = 0;

  if(sParameter_.empty())
    {
      throw ConversionException("Empty string in numeric conversion");
    }
  else
    {
      std::string sTmpParameter(scaleNumericalStringRepresentation(sParameter_));
      
      char * pEnd = 0;
      
      //Clear errno before making call because Ubuntu does not
      //clear it when a call is made
      errno = 0;

      dValue =  ACE_OS::strtod(sTmpParameter.c_str(),&pEnd);

      if(errno == ERANGE ||
         dValue < dMin ||
         dValue > dMax)
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" out of range ["<<dMin<<","<<dMax<<"]"<<std::ends;
          throw ConversionException(sstream.str());
        }
      else if(pEnd != 0 && *pEnd !='\0')
        {
          std::stringstream sstream;
          sstream<<sParameter_<<" invalid character in numeric: '"<<*pEnd<<"'"<<std::ends;
          throw ConversionException(sstream.str());
        }
    }
  
  return dValue;
}

inline
float EMANEUtils::ParameterConvert::toFloat(float fMin, float fMax) const
  throw(ConversionException)
{
  return static_cast<float>(toDouble(fMin,fMax));
}
