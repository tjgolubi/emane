/*
 * Copyright (c) 2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef EMANE_COMMONPHYFREQINFO_HEADER_
#define EMANE_COMMONPHYFREQINFO_HEADER_

#include "emane/emanetypes.h"
#include "emane/emanenet.h"
#include "emaneutils/netutils.h"

#include <ace/Time_Value.h>
#include <ace/Basic_Types.h>

#include <string>
#include <vector>

namespace EMANE
{

    /**
     * @class  PHYTxFrequencyInfo
     *
     * @brief  Defines center frequency, tx offset and duration
     *
     */
    class PHYTxFrequencyInfo 
     {
       public:
       /**
        *
        * @brief default initializer 
        *
        */
        PHYTxFrequencyInfo ()
         { }

       /**
        *
        * @brief paramter constructor 
        *
        * @param u64CenterFrequencyHz center frequency in Hz
        * @param tvOffset             transmit offset
        * @param tvDuration           duration
        *
        */
        PHYTxFrequencyInfo (const ACE_UINT64 u64CenterFrequencyHz,
                            const ACE_Time_Value & tvOffset,
                            const ACE_Time_Value & tvDuration) :
          data_(u64CenterFrequencyHz, tvOffset, tvDuration)
        { }


        /**
         *
         * @return returns the pkt duration
         *
         */
        ACE_Time_Value getDuration() const
         {
           return  ACE_Time_Value(data_.u32DurationSec_, data_.u32DurationUsec_);
         }

        /**
         *
         * @param tv sets the pkt duration
         *
         */
         void setDuration(const ACE_Time_Value &tv)
         {
           data_.u32DurationSec_  = tv.sec();
           data_.u32DurationUsec_ = tv.usec();
         }
 
        /**
         *
         * @return retruns the tx offset time
         *
         */
         ACE_Time_Value getTxOffset() const
          {
            return  ACE_Time_Value(data_.u32TxOffsetSec_, data_.u32TxOffsetUsec_);
          }

         /**
          *
          * @param tv sets the tx offset time
          *
          */
         void setTxOffset(const ACE_Time_Value &tv)
          {
            data_.u32TxOffsetSec_  = tv.sec();
            data_.u32TxOffsetUsec_ = tv.usec();
          }

         /**
          *
          * @param  frequency  the center frequency in Hz
          *
          */
         void setCenterFrequencyHz(const ACE_UINT64 frequency)
          {
            data_.u64CenterFrequencyHz_ = frequency;
          }

         /**
          *
          * @return retruns the center frequency in Hz
          *
          */
         ACE_UINT64 getCenterFrequencyHz() const
          {
            return  data_.u64CenterFrequencyHz_;
          }

         /**
          *
          * @brief host to network byte order conversion
          *
          */
         void toHostByteOrder()
          {
            // long
            data_.u32TxOffsetSec_   =  ACE_NTOHL(data_.u32TxOffsetSec_);
            data_.u32TxOffsetUsec_  =  ACE_NTOHL(data_.u32TxOffsetUsec_);
            data_.u32DurationSec_   =  ACE_NTOHL(data_.u32DurationSec_);
            data_.u32DurationUsec_  =  ACE_NTOHL(data_.u32DurationUsec_);
 
            // long long
            data_.u64CenterFrequencyHz_ =  EMANE::NTOHLL(data_.u64CenterFrequencyHz_);
          }
 
          /**
           *
           * @brief network to host byte order conversion
           *
           */
          void toNetworkByteOrder()
           {
             // long
             data_.u32TxOffsetSec_   =  ACE_HTONL(data_.u32TxOffsetSec_);
             data_.u32TxOffsetUsec_  =  ACE_HTONL(data_.u32TxOffsetUsec_);
             data_.u32DurationSec_   =  ACE_HTONL(data_.u32DurationSec_);
             data_.u32DurationUsec_  =  ACE_HTONL(data_.u32DurationUsec_);

             // long long
             data_.u64CenterFrequencyHz_ =  EMANE::HTONLL(data_.u64CenterFrequencyHz_);
           }
 
          /**
           *
           * @brief paramter format helper method
           *
           */
          std::string format() const
          {
            char fmtBuff[256];
  
            snprintf(fmtBuff, sizeof(fmtBuff), "txfreqinfo: freq %s, offset %ld:%06ld, duration %ld:%06ld",
                     EMANEUtils::formatFrequency(getCenterFrequencyHz()).c_str(),
                     getTxOffset().sec(),
                     getTxOffset().usec(),
                     getDuration().sec(),
                     getDuration().usec());
   
            return fmtBuff;
          }


       private:
        /**
         * @struct  PHYTxFrequencyData
         *
         * @brief  Defines center frequency, tx offset and duration
         *
         */
        struct PHYTxFrequencyData 
        {
          /**
           *
           * @brief default constructor 
           *
           */
           PHYTxFrequencyData () :
             u64CenterFrequencyHz_(0),
             u32TxOffsetSec_(0),
             u32TxOffsetUsec_(0),
             u32DurationSec_(0),
             u32DurationUsec_(0)
           { }

          /**
           *
           * @brief paramter constructor 
           *
           * @param u64CenterFrequencyHz center frequency in Hz
           * @param tvOffset             transmit offset
           * @param tvDuration           duration
           *
           */
           PHYTxFrequencyData (const ACE_UINT64 u64CenterFrequencyHz,
                             const ACE_Time_Value & tvOffset,
                             const ACE_Time_Value &tvDuration) :
              u64CenterFrequencyHz_(u64CenterFrequencyHz),
              u32TxOffsetSec_(tvOffset.sec()),
              u32TxOffsetUsec_(tvOffset.usec()),
              u32DurationSec_(tvDuration.sec()),
              u32DurationUsec_(tvDuration.usec())
           { }

 
           ACE_UINT64            u64CenterFrequencyHz_;    // center frequency in Hz
           ACE_UINT32            u32TxOffsetSec_;          // pkt tx time offset in sec
           ACE_UINT32            u32TxOffsetUsec_;         // pkt tx time offset in usec
           ACE_UINT32            u32DurationSec_;          // pkt duration in sec
           ACE_UINT32            u32DurationUsec_;         // pkt duration in usec
        } __attribute__((packed));
 
        struct PHYTxFrequencyData data_;
     } __attribute__((packed));

    /**
     *
     * @brief  PHYTxFrequencyInfo container and iterators
     *
     */
    typedef std::vector<PHYTxFrequencyInfo>         PHYTxFrequencyInfoItems;
    typedef PHYTxFrequencyInfoItems::iterator       PHYTxFrequencyInfoItemsIter;
    typedef PHYTxFrequencyInfoItems::const_iterator PHYTxFrequencyInfoItemsConstIter;




    /**
     * @class  PHYRxFrequencyInfo
     *
     * @brief  Defines center frequency, tx offset, duration and noise floor
     *
     */
    class PHYRxFrequencyInfo
     {
        public:
         /**
          *
          * @brief default initializer 
          *
          */
         PHYRxFrequencyInfo ()
         { }

         /**
          *
          * @brief paramter constructor 
          *
          * @param u64CenterFrequencyHz center frequency in Hz
          * @param tvOffset             transmit offset
          * @param tvDuration           duration
          * @param fNoiseFloor          noise floor in dBm
          *
          */
         PHYRxFrequencyInfo (const ACE_UINT64 u64CenterFrequencyHz,
                             const ACE_Time_Value &tvOffset,
                             const ACE_Time_Value &tvDuration,
                             const float fNoiseFloor) :
          data_(u64CenterFrequencyHz, tvOffset, tvDuration, fNoiseFloor * DB_SCALE_FACTOR)
         { }

        /**
         *
         * @return returns the pkt duration
         *
         */
        ACE_Time_Value getDuration() const
         {
           return  ACE_Time_Value(data_.u32DurationSec_, data_.u32DurationUsec_);
         }

        /**
         *
         * @param tv sets the pkt duration
         *
         */
         void setDuration(const ACE_Time_Value &tv)
         {
           data_.u32DurationSec_  = tv.sec();
           data_.u32DurationUsec_ = tv.usec();
         }
 
        /**
         *
         * @return retruns the tx offset time
         *
         */
         ACE_Time_Value getTxOffset() const
          {
            return  ACE_Time_Value(data_.u32TxOffsetSec_, data_.u32TxOffsetUsec_);
          }

         /**
          *
          * @param tv sets the tx offset time
          *
          */
         void setTxOffset(const ACE_Time_Value &tv)
          {
            data_.u32TxOffsetSec_  = tv.sec();
            data_.u32TxOffsetUsec_ = tv.usec();
          }

         /**
          *
          * @param  frequency  the center frequency in Hz
          *
          */
         void setCenterFrequencyHz(const ACE_UINT64 frequency)
          {
            data_.u64CenterFrequencyHz_ = frequency;
          }

         /**
          *
          * @return retruns the center frequency in Hz
          *
          */
         ACE_UINT64 getCenterFrequencyHz() const
          {
            return  data_.u64CenterFrequencyHz_;
          }

         /**
          *
          * @return returns the noise floor in dBm
          *
          */
         float getNoiseFloordBm () const
          {
            return ((float) data_.i32NoiseFloorScaleddBm_ / DB_SCALE_FACTOR);
          }

         /**
          *
          * @param  fNoiseFloor noise floor in dBm
          *
          */
         void setNoiseFloordBm (const float fNoiseFloor)
          {
            data_.i32NoiseFloorScaleddBm_ = fNoiseFloor * DB_SCALE_FACTOR;
          }

         /**
          *
          * @brief host to network byte order conversion
          *
          */
         void toHostByteOrder()
          {
            // long
            data_.i32NoiseFloorScaleddBm_  =  ACE_NTOHL(data_.i32NoiseFloorScaleddBm_);
            data_.u32TxOffsetSec_          =  ACE_NTOHL(data_.u32TxOffsetSec_);
            data_.u32TxOffsetUsec_         =  ACE_NTOHL(data_.u32TxOffsetUsec_);
            data_.u32DurationSec_          =  ACE_NTOHL(data_.u32DurationSec_);
            data_.u32DurationUsec_         =  ACE_NTOHL(data_.u32DurationUsec_);
 
            // long long
            data_.u64CenterFrequencyHz_    =  EMANE::NTOHLL(data_.u64CenterFrequencyHz_);
          }
 
          /**
           *
           * @brief network to host byte order conversion
           *
           */
          void toNetworkByteOrder()
           {
             // long
             data_.i32NoiseFloorScaleddBm_  =  ACE_HTONL(data_.i32NoiseFloorScaleddBm_);
             data_.u32TxOffsetSec_          =  ACE_HTONL(data_.u32TxOffsetSec_);
             data_.u32TxOffsetUsec_         =  ACE_HTONL(data_.u32TxOffsetUsec_);
             data_.u32DurationSec_          =  ACE_HTONL(data_.u32DurationSec_);
             data_.u32DurationUsec_         =  ACE_HTONL(data_.u32DurationUsec_);

             // long long
             data_.u64CenterFrequencyHz_    =  EMANE::HTONLL(data_.u64CenterFrequencyHz_);
           }
 
        
          /**
           *
           * @brief paramter format helper method
           *
           */
          std::string format() const
            {
              char fmtBuff[256];
  
              snprintf(fmtBuff, sizeof(fmtBuff), "rxfreqinfo: freq %s, offset %ld:%06ld, duration %ld:%06ld, noise floor %5.3f dBm",
                       EMANEUtils::formatFrequency(getCenterFrequencyHz()).c_str(),
                       getTxOffset().sec(),
                       getTxOffset().usec(),
                       getDuration().sec(),
                       getDuration().usec(),
                       getNoiseFloordBm());
  
              return fmtBuff;
            }

       private:
        // scale factor for noise floor
        const static float DB_SCALE_FACTOR = 0xffff;

        /**
         * @struct  PHYRxFrequencyData
         *
         * @brief  Defines center frequency, tx offset, duration and nosie floor
         *
         */
        struct PHYRxFrequencyData 
        {
          /**
           *
           * @brief default constructor 
           *
           */
           PHYRxFrequencyData () :
             u64CenterFrequencyHz_(0),
             u32TxOffsetSec_(0),
             u32TxOffsetUsec_(0),
             u32DurationSec_(0),
             u32DurationUsec_(0),
             i32NoiseFloorScaleddBm_(-0x80000000)
           { }

          /**
           *
           * @brief paramter constructor 
           *
           * @param u64CenterFrequencyHz center frequency in Hz
           * @param tvOffset             transmit offset
           * @param tvDuration           duration
           * @param fNoiseFloordBm       noise floor
           *
           */
           PHYRxFrequencyData (const ACE_UINT64 u64CenterFrequencyHz,
                               const ACE_Time_Value &tvOffset,
                               const ACE_Time_Value &tvDuration,
                               const float fNoiseFloorScaleddBm) :
              u64CenterFrequencyHz_(u64CenterFrequencyHz),
              u32TxOffsetSec_(tvOffset.sec()),
              u32TxOffsetUsec_(tvOffset.usec()),
              u32DurationSec_(tvDuration.sec()),
              u32DurationUsec_(tvDuration.usec()),
              i32NoiseFloorScaleddBm_(fNoiseFloorScaleddBm)
           { }

 
           ACE_UINT64            u64CenterFrequencyHz_;    // center frequency in Hz
           ACE_UINT32            u32TxOffsetSec_;          // pkt tx time offset in sec
           ACE_UINT32            u32TxOffsetUsec_;         // pkt tx time offset in usec
           ACE_UINT32            u32DurationSec_;          // pkt duration in sec
           ACE_UINT32            u32DurationUsec_;         // pkt duration in usec
           ACE_INT32             i32NoiseFloorScaleddBm_;  // noise floor in dBm
        } __attribute__((packed));
 
        struct PHYRxFrequencyData data_;
     } __attribute__((packed));

    /**
     *
     * @brief  PHYRxFrequencyInfo container and iterators
     *
     */
    typedef std::vector<PHYRxFrequencyInfo>         PHYRxFrequencyInfoItems;
    typedef PHYRxFrequencyInfoItems::iterator       PHYRxFrequencyInfoItemsIter;
    typedef PHYRxFrequencyInfoItems::const_iterator PHYRxFrequencyInfoItemsConstIter;






  /**
   * @class  PHYTxFrequencyInfoManager
   *
   * @brief manages PHYTxFrequencyInfoItems and provdides maethods to copy items to/from
   *        flat buffer space
   *
   */
    class PHYTxFrequencyInfoManager 
    {
     public:
    /**
     *
     * @param items reference to a container of PHYTxFrequencyInfoItems to be copied
     *
     */
      void setFrequencyInfo (const PHYTxFrequencyInfoItems & items);

    /**
     *
     * @return returns reference to container of PHYTxFrequencyInfoItems 
     *
     */
      const PHYTxFrequencyInfoItems & getFrequencyInfo () const;

    /**
     *
     * @return returns current stored number of entries
     *
     */
      size_t numEntries() const;

    /**
     *
     * @return returns the size of a PHYTxFrequencyInfoItem
     *
     */
      size_t entrySize() const;

    /**
     *
     * @return returns the the total size of all stored items
     *
     */
      size_t totalSize() const;

    /**
     *
     * @brief clears all stored items
     *
     */
      void clear();

    /**
     *
     * @param buff buffer location to copy items to
     * @param num max  number of items to copy
     *
     * @return returns the number of bytes copied
     */
      size_t copyToBuffer (void * buff, const size_t num) const;

    /**
     *
     * @param buff buffer location to copy items from
     * @param num max number of items to copy
     *
     * @return returns the number of bytes copied
     */
      size_t copyFromBuffer (const void * buff, const size_t num);

    /**
     *
     * @return returns a formated string for logging
     */
      std::string format() const;

     private:
    /**
     *
     * @brief item container
     *
     */
      PHYTxFrequencyInfoItems vec_;
    };





  /**
   * @class  PHYRxFrequencyInfoManager
   *
   * @brief manages PHYRxFrequencyInfoItems and provdides maethods to copy items to/from
   *        flat buffer space
   *
   */
    class PHYRxFrequencyInfoManager 
    {
     public:
    /**
     *
     * @param items a constant reference to a container of PHYRxFrequencyInfoItems
     *
     */
     void setFrequencyInfo (const PHYRxFrequencyInfoItems & items);

    /**
     *
     * @return returns a const reference to a container of PHYRxFrequencyInfoItems
     *
     */
     const PHYRxFrequencyInfoItems & getFrequencyInfo () const;

    /**
     *
     * @return returns current stored number of entries
     *
     */
      size_t numEntries() const;

    /**
     *
     * @return returns the size of a PHYTxFrequencyInfoItem
     *
     */
      size_t entrySize() const;

    /**
     *
     * @return returns the the total size of all stored items
     *
     */
      size_t totalSize() const;

    /**
     *
     * @brief clears all stored items
     *
     */
      void clear();

    /**
     *
     * @param buff buffer location to copy items to
     * @param num max  number of items to copy
     *
     * @return returns the number of bytes copied
     */
      size_t copyToBuffer (void * buff, const size_t num) const;

    /**
     *
     * @param buff buffer location to copy items from
     * @param num max number of items to copy
     *
     * @return returns the number of bytes copied
     */
      size_t copyFromBuffer (const void * buff, const size_t num);

    /**
     *
     * @return returns a formated string for logging
     */
      std::string format() const;

     private:
    /**
     *
     * @brief item container
     *
     */
       PHYRxFrequencyInfoItems vec_;
    };
}

#include "emane/emanecommonphyfreqinfo.inl"

#endif //EMANE_COMMONPHYFREQINFO_HEADER_
