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


#include "macconfig.h"

#include "emaneutils/parameterconvert.h"

#include <ace/OS_NS_string.h>

namespace
{
  const ACE_UINT32 CWMIN_CATEGORY_00 = 32;
  const ACE_UINT32 CWMIN_CATEGORY_01 = 32;
  const ACE_UINT32 CWMIN_CATEGORY_02 = 16;
  const ACE_UINT32 CWMIN_CATEGORY_03 = 8;

  const ACE_UINT32 CWMAX_CATEGORY_00 = 1024;
  const ACE_UINT32 CWMAX_CATEGORY_01 = 1024;
  const ACE_UINT32 CWMAX_CATEGORY_02 = 64;
  const ACE_UINT32 CWMAX_CATEGORY_03 = 16;

  const ACE_UINT32 AIFS_CATEGORY_00 = 2;
  const ACE_UINT32 AIFS_CATEGORY_01 = 2;
  const ACE_UINT32 AIFS_CATEGORY_02 = 2;
  const ACE_UINT32 AIFS_CATEGORY_03 = 1;

  ACE_UINT32 UnicastDataRateIndexTable[IEEE80211ABG::UNICAST_DATARATE_INDEX_MAX + 1] =
// index   0     1     2     3     4      5     6     7      8      9      10     11     12
         { 0,    1000, 2000, 5500, 11000, 6000, 9000, 12000, 18000, 24000, 36000, 48000, 54000 };


  ACE_UINT32 MulticastDataRateIndexTable[IEEE80211ABG::MULTICAST_DATARATE_INDEX_MAX + 1] =
// index   0     1     2     3     4      5     6     7      8      9      10     11     12
         { 0,    1000, 2000, 5500, 11000, 6000, 9000, 12000, 18000, 24000, 36000, 48000, 54000 };
}

/**
*
* @brief ieee80211abg mac configuration initializer.
*
*/
IEEE80211ABG::ConfigItems::ConfigItems ():
 u8ModeIndex_                    (IEEE80211ABG::MODULATION_TYPE_INDEX_DEFAULT),
 bPromiscousEnable_              (IEEE80211ABG::PROMISCOUS_ENABLE_DEFAULT),
 u32MaxP2pDistance_              (IEEE80211ABG::MAX_DISTANCE_DEFAULT),
 u16UnicastDataRateIndex_        (IEEE80211ABG::UNICAST_DATARATE_INDEX_DEFAULT),
 u16MulticastDataRateIndex_      (IEEE80211ABG::MULTICAST_DATARATE_INDEX_DEFAULT),
 u16RtsThreshold_                (IEEE80211ABG::RTS_THRESHOLD_DEFAULT),
 bWmmEnable_                     (IEEE80211ABG::WMM_ENABLE_DEFAULT),
 u8NumAccessCategories_          (NUM_ACCESS_CATEGORIES_DEFAULT), 
 bFlowControlEnable_             (IEEE80211ABG::FLOW_CONTROL_ENABLE_DEFAULT),
 u16FlowControlTokens_           (IEEE80211ABG::FLOW_CONTROL_TOKENS_DEFAULT),
 tvNeighborTimeout_              (ACE_Time_Value(NEIGHBOR_TIMEOUT_SEC_DEFAULT, 0)),
 tvChannelActivityTimerInterval_ (ACE_Time_Value(0, CHANNEL_ACTIVITY_TIMER_MSEC_DEFAULT * 1000)),
 queueConfigVec_                 (IEEE80211ABG::NUM_ACCESS_CATEGORIES_MAX)
{

  // for each possible category
  for (ACE_UINT8 idx = 0; idx < queueConfigVec_.size (); ++idx)
    {
      // common for all categories
      queueConfigVec_.at (idx).tvTxOpTime_   = ACE_Time_Value (0, IEEE80211ABG::TXOP_TIME_USEC_DEFAULT);
      queueConfigVec_.at (idx).u16QueueSize_ = IEEE80211ABG::QUEUE_SIZE_DEFAULT;
      queueConfigVec_.at (idx).u8RetryLimit_ = IEEE80211ABG::RETRY_LIMIT_DEFAULT;

      // specific to a category
      switch (idx)
        {
        case 0:                // best effort
          queueConfigVec_.at (idx).u16CwMin_    = CWMIN_CATEGORY_00;
          queueConfigVec_.at (idx).u16CwMax_    = CWMAX_CATEGORY_00;
          queueConfigVec_.at (idx).u32AifsUsec_ = AIFS_CATEGORY_00;
          break;

        case 1:                // background
          queueConfigVec_.at (idx).u16CwMin_    = CWMIN_CATEGORY_01;
          queueConfigVec_.at (idx).u16CwMax_    = CWMAX_CATEGORY_01;
          queueConfigVec_.at (idx).u32AifsUsec_ = AIFS_CATEGORY_01;
          break;

        case 2:                // video
          queueConfigVec_.at (idx).u16CwMin_    = CWMIN_CATEGORY_02;
          queueConfigVec_.at (idx).u16CwMax_    = CWMAX_CATEGORY_02;
          queueConfigVec_.at (idx).u32AifsUsec_ = AIFS_CATEGORY_02;
          break;

        case 3:                // voice
          queueConfigVec_.at (idx).u16CwMin_    = CWMIN_CATEGORY_03;
          queueConfigVec_.at (idx).u16CwMax_    = CWMAX_CATEGORY_03;
          queueConfigVec_.at (idx).u32AifsUsec_ = AIFS_CATEGORY_03;
          break;

        default:               // default
          queueConfigVec_.at (idx).u16CwMin_    = CWMIN_DEFAULT;
          queueConfigVec_.at (idx).u16CwMax_    = CWMAX_DEFAULT;
          queueConfigVec_.at (idx).u32AifsUsec_ = AIFS_USEC_DEFAULT;
          break;
        }
    }
}


/**
*
* @brief constructor
*
*/
IEEE80211ABG::MACConfig::MACConfig ()
{ } 

/**
*
* @brief destructor
*
*/
IEEE80211ABG::MACConfig::~MACConfig ()
{ }




/**
*
* @brief get the promiscous mode
*
*
* @retval true if on, false if off
*
*/
bool 
IEEE80211ABG::MACConfig::getPromiscuosEnable () const
{
  return configItems_.bPromiscousEnable_;
}



/**
*
* @brief get the wmm mode
*
* @retval true if on, false if off
*
*/
bool 
IEEE80211ABG::MACConfig::getWmmEnable () const
{
  return configItems_.bWmmEnable_;
}


/**
*
* @brief get the modulation type
*
* @retval modulation type
*
*/
IEEE80211ABG::MODULATION_TYPE 
IEEE80211ABG::MACConfig::getModulationType () const
{
  switch (configItems_.u8ModeIndex_)
    {
    case 0:                    // 80211B
      return MODULATION_TYPE_80211B;

    case 1:                    // 80211A
      return MODULATION_TYPE_80211A;

    case 2:                    // 80211B
      return MODULATION_TYPE_80211B;

    case 3:                    // 80211BG
      return MODULATION_TYPE_80211BG;

    default:                   // 80211B
      return MODULATION_TYPE_80211B;
    }
}


/**
*
* @brief get the unicast datarate index
*
* @retval datarate index
*
*/
ACE_UINT16 
IEEE80211ABG::MACConfig::getUnicastDataRateIndex () const
{
  return configItems_.u16UnicastDataRateIndex_;
}


/**
*
* @brief get the multicast datarate index
*
* @retval datarate index
*
*/
ACE_UINT16 
IEEE80211ABG::MACConfig::getMulticastDataRateIndex () const
{
   return configItems_.u16MulticastDataRateIndex_;
}


/**
*
* @brief get the unicast datarate
*
* @retval datarate in Kbps
*
*/
ACE_UINT32 
IEEE80211ABG::MACConfig::getUnicastDataRateKbps () const
{
  return UnicastDataRateIndexTable[configItems_.u16UnicastDataRateIndex_];
}


/**
*
* @brief get the multicast datarate
*
* @retval datarate in Kbps
*
*/
ACE_UINT32 
IEEE80211ABG::MACConfig::getMulticastDataRateKbps () const
{
  return MulticastDataRateIndexTable[configItems_.u16MulticastDataRateIndex_];
}



/**
*
* @brief get the max ptp distance
*
* @retval max ptp distance
*
*/
ACE_UINT32 
IEEE80211ABG::MACConfig::getMaxP2pDistance () const
{
  return configItems_.u32MaxP2pDistance_;
}


/**
*
* @brief get the number of access categories (queues)
*
* @retval number of access categories (queues)
*
*/
ACE_UINT8 
IEEE80211ABG::MACConfig::getNumAccessCategories () const
{
  return configItems_.bWmmEnable_ == true ? configItems_.u8NumAccessCategories_ : IEEE80211ABG::NUM_ACCESS_CATEGORIES_MIN;
}


/**
*
* @brief get the queue size for a given queue index
*
* @param idx queue index
*
* @retval queue size
*
*/
ACE_UINT16 
IEEE80211ABG::MACConfig::getQueueSize (ACE_UINT8 idx) const
{
  return configItems_.queueConfigVec_.at (idx).u16QueueSize_;
}


/**
*
* @brief get the min contention window size for a given queue index
*
* @param idx queue index
*
* @retval min contention window size
*
*/
ACE_UINT16
IEEE80211ABG::MACConfig::getCwMin (ACE_UINT8 idx) const
{
  return configItems_.queueConfigVec_.at (idx).u16CwMin_;
}


/**
*
* @brief get the max contention window size for a given queue index
*
* @param idx queue index
*
* @retval max contention window size
*
*/
ACE_UINT16
IEEE80211ABG::MACConfig::getCwMax (ACE_UINT8 idx) const
{
  return configItems_.queueConfigVec_.at (idx).u16CwMax_;
}


/**
*
* @brief get the aifs for a given queue index
*
* @param idx queue index
*
* @retval aifs
*
*/
ACE_UINT32
IEEE80211ABG::MACConfig::getAifsUsec (ACE_UINT8 idx) const
{
  return configItems_.queueConfigVec_.at (idx).u32AifsUsec_;
}


/**
*
* @brief get the txop for a given queue index
*
* @param idx queue index
*
* @retval txop
*
*/
ACE_Time_Value
IEEE80211ABG::MACConfig::getTxOpTime (ACE_UINT8 idx) const
{
  return configItems_.queueConfigVec_.at (idx).tvTxOpTime_;
}


/**
*
* @brief get the retry limit for a given queue index
*
* @param idx queue index
*
* @retval retry limit
*
*/
ACE_UINT8
IEEE80211ABG::MACConfig::getRetryLimit (ACE_UINT8 idx) const
{
  return configItems_.queueConfigVec_.at (idx).u8RetryLimit_;
}




/**
*
* @brief get the flow control enable status
*
* @retval flow control enable
*
*/
bool
IEEE80211ABG::MACConfig::getFlowControlEnable () const
{
  return configItems_.bFlowControlEnable_;
}



/**
*
* @brief get the number of flow control tokens
*
* @retval flow control tokens
*
*/
ACE_UINT16 
IEEE80211ABG::MACConfig::getFlowControlTokens () const
{
  return configItems_.u16FlowControlTokens_;
}


/**
*
* @brief get the pcr uri
*
* @retval pcr uri
*
*/
std::string 
IEEE80211ABG::MACConfig::getPcrUri () const
{
  return configItems_.sPcrUri_;
}



ACE_Time_Value 
IEEE80211ABG::MACConfig::getNeighborTimeout () const
{
  return configItems_.tvNeighborTimeout_;
}



ACE_Time_Value 
IEEE80211ABG::MACConfig::getChannelActivityTimerInterval () const
{
  return configItems_.tvChannelActivityTimerInterval_;
}


ACE_UINT16 
IEEE80211ABG::MACConfig::getRtsThreshold () const
{
  return  configItems_.u16RtsThreshold_;
}

/**
*
* @brief set config item by name, value and optional index when applicable
*
* @param sName item name
* @param sValue item value
*
* @retval true on success, false on failure
*
*/
bool 
IEEE80211ABG::MACConfig::set (const std::string sName, const std::string sValue)
{
  if (sName == "mode")
    {
      configItems_.u8ModeIndex_ =
        EMANEUtils::ParameterConvert (sValue.c_str ()).toUINT32 
          (IEEE80211ABG::MODULATION_TYPE_INDEX_MIN, IEEE80211ABG::MODULATION_TYPE_INDEX_MAX);

      return true;
    }
  else if (sName == "enablepromiscuousmode")
    {
      configItems_.bPromiscousEnable_ = EMANEUtils::ParameterConvert (sValue.c_str ()).toBool ();

      return true;
    }
  else if (sName == "distance")
    {
      configItems_.u32MaxP2pDistance_ =
        EMANEUtils::ParameterConvert (sValue.c_str ()).toUINT32 (MAX_DISTANCE_MIN, MAX_DISTANCE_MAX);

      return true;
    }
  else if (sName == "unicastrate")
    {
      configItems_.u16UnicastDataRateIndex_ =
        EMANEUtils::ParameterConvert (sValue.c_str ()).
          toUINT16 (IEEE80211ABG::UNICAST_DATARATE_INDEX_MIN, IEEE80211ABG::UNICAST_DATARATE_INDEX_MAX);

      return true;
    }
  else if (sName == "multicastrate")
    {
      configItems_.u16MulticastDataRateIndex_ =
        EMANEUtils::ParameterConvert (sValue.c_str ()).
          toUINT16 (IEEE80211ABG::MULTICAST_DATARATE_INDEX_MIN, IEEE80211ABG::MULTICAST_DATARATE_INDEX_MAX);

      return true;
    }
  else if(sName == "rtsthreshold")
    {
      configItems_.u16RtsThreshold_ = EMANEUtils::ParameterConvert(sValue.c_str()).
        toUINT16(IEEE80211ABG::RTS_THRESHOLD_MIN, IEEE80211ABG::RTS_THRESHOLD_MAX);

      return true;
    }
  else if (sName == "wmmenable")
    {
      configItems_.bWmmEnable_ = EMANEUtils::ParameterConvert (sValue.c_str ()).toBool ();

      return true;
    }
  else if (sName == "numaccesscategories")
    {
      configItems_.u8NumAccessCategories_ =
        EMANEUtils::ParameterConvert (sValue.c_str ()).
          toUINT8 (IEEE80211ABG::NUM_ACCESS_CATEGORIES_MIN, IEEE80211ABG::NUM_ACCESS_CATEGORIES_MAX);

      return true;
    }
  else if (sName == "queuesize")
    {
      IndexValueVector vec;

      if (getValues (sValue, vec, QUEUE_SIZE_MIN, QUEUE_SIZE_MAX) == true)
        {
          for (size_t i = 0; i < vec.size (); ++i)
            {
              configItems_.queueConfigVec_.at (vec.at (i).index_).u16QueueSize_ = vec[i].value_;
            }
          return true;
        }
      else
        {
          return false;
        }
    }
  else if (sName == "cwmin")
    {
      IndexValueVector vec;

      if (getValues (sValue, vec, CWMIN_MIN, CWMIN_MAX) == true)
        {
          for (size_t i = 0; i < vec.size (); ++i)
            {
              configItems_.queueConfigVec_.at (vec.at (i).index_).u16CwMin_ = vec[i].value_;
            }
          return true;
        }
      else
        {
          return false;
        }
    }
  else if (sName == "cwmax")
    {
      IndexValueVector vec;

      if (getValues (sValue, vec, CWMAX_MIN, CWMAX_MAX) == true)
        {
          for (size_t i = 0; i < vec.size (); ++i)
            {
              configItems_.queueConfigVec_.at (vec.at (i).index_).u16CwMax_ = vec[i].value_;
            }
          return true;
        }
      else
        {
          return false;
        }
    }
  else if (sName == "aifs")
    {
      IndexValueVector vec;

      if (getValues (sValue, vec, AIFS_USEC_MIN, AIFS_USEC_MAX) == true)
        {
          for (size_t i = 0; i < vec.size (); ++i)
            {
              configItems_.queueConfigVec_.at (vec.at (i).index_).u32AifsUsec_ = vec[i].value_;
            }
          return true;
        }
      else
        {
          return false;
        }
    }
  else if (sName == "txop")
    {
      IndexValueVector vec;

      if (getValues (sValue, vec, TXOP_TIME_USEC_MIN, TXOP_TIME_USEC_MAX) == true)
        {
          for (size_t i = 0; i < vec.size (); ++i)
            {
              configItems_.queueConfigVec_.at (vec.at (i).index_).tvTxOpTime_.set (vec[i].value_);
            }
          return true;
        }
      else
        {
          return false;
        }
    }
  else if (sName == "retrylimit")
    {
      IndexValueVector vec;

      if (getValues (sValue, vec, RETRY_LIMIT_MIN, RETRY_LIMIT_MAX) == true)
        {
          for (size_t i = 0; i < vec.size (); ++i)
            {
              configItems_.queueConfigVec_.at (vec.at (i).index_).u8RetryLimit_ = vec[i].value_;
            }
          return true;
        }
      else
        {
          return false;
        }
    }
  else if (sName == "flowcontrolenable")
    {
      configItems_.bFlowControlEnable_ = EMANEUtils::ParameterConvert (sValue.c_str ()).toBool ();

      return true;
    }
  else if (sName == "flowcontroltokens")
    {
      configItems_.u16FlowControlTokens_ = EMANEUtils::ParameterConvert (sValue.c_str ()).
                              toUINT16 (FLOW_CONTROL_TOKENS_MIN, FLOW_CONTROL_TOKENS_MAX);

      return true;
    }
  else if (sName == "pcrcurveuri")
    {
      configItems_.sPcrUri_ = sValue;

      return true;
    }
  else if (sName == "neighbortimeout")
    {
      const ACE_INT32 val = EMANEUtils::ParameterConvert (sValue.c_str ()).
                               toINT32 (NEIGHBOR_TIMEOUT_SEC_MIN, NEIGHBOR_TIMEOUT_SEC_MAX);

      configItems_.tvNeighborTimeout_.msec(val * 1000);

      return true;
    }
  else if (sName == "channelactivityestimationtimer")
    {
      const ACE_INT32 val = EMANEUtils::ParameterConvert (sValue.c_str ()).
                               toINT32 (CHANNEL_ACTIVITY_TIMER_MSEC_MIN, CHANNEL_ACTIVITY_TIMER_MSEC_MAX);

      configItems_.tvChannelActivityTimerInterval_.msec(val);

      return true;
    }

  return false;
}


/**
*
* @brief verify that configuration items are set correctly
*
* @param err error string
*
* @retval true on success, false on failure
*
*/
bool 
IEEE80211ABG::MACConfig::verify (std::string & err) const
{
  err = "";

  // verify mode parameters
  switch (getModulationType ())
    {
      // 80211A
    case MODULATION_TYPE_80211A:
      // unicast data rate index is only valid for 5 to 12 inclusive
      if (getUnicastDataRateIndex () < 5 || getUnicastDataRateIndex () > 12)
        {
          err += ":unicast data rate index must be [5-12] for modulation type 80211A";
        }
      // multicast data rate index is only valid for 5 to 12 inclusive
      if (getMulticastDataRateIndex () < 5 || getMulticastDataRateIndex () > 12)
        {
          err += ":multiicast data rate index must be [5-12] for modulation type 80211A";
        }
      break;


      // 80211B
    case MODULATION_TYPE_80211B:
      // unicast data rate index is only valid for 0 to 4 inclusive
      if (getUnicastDataRateIndex () > 4)
        {
          err += ":unicast data rate index must be [0-4] for modulation type 80211B";
        }
      // multicast data rate index is only valid for 0 to 4 inclusive
      if (getMulticastDataRateIndex () > 4)
        {
          err += ":multiicast data rate index must be [0-4] for modulation type 80211B";
        }
      break;


      // 80211BG
    case MODULATION_TYPE_80211BG:
      // unicast data rate index is only valid for 0 to 12 inclusive
      if (getUnicastDataRateIndex () > 12)
        {
          err += ":unicast data rate index must be [0-12] for modulation type 80211BG";
        }
      // multicast data rate index is only valid for 0 to 12 inclusive
      if (getMulticastDataRateIndex () > 12)
        {
          err += ":multiicast data rate index must be [0-12] for modulation type 80211BG";
        }
      break;


    default:
      err += ":unsupported modulation type";
    }


  return err == "";
}


/**
*
* @brief parse the index and value
*
* @param str string to be parsed
* @param vec vector of index, value pairs to be returned 
* @param min min allowable value
* @param max max allowable value
*
* @retval true on success, false on failure
*
*/
bool 
IEEE80211ABG::MACConfig::getValues (const std::string & str, IndexValueVector & vec, float min, float max) const
{
  std::string sValues = str;

  int iTokenCount;
  char * pzToken;
  struct IndexValuePair valuePair;

  iTokenCount = 0;

  while ((pzToken = ACE_OS::strtok (iTokenCount == 0 ? &sValues[0] : NULL, " \t\n")) != NULL)
    {
      ++iTokenCount;

      if (sscanf (pzToken, "%hhu:%f", &valuePair.index_, &valuePair.value_) == 2)
        {
          if (valuePair.value_ >= min && valuePair.value_ <= max)
            {
              vec.push_back (valuePair);
            }
          else
            {
              return false;
            }
        }
      else
        {
          return false;
        }
    }

  return true;
}
