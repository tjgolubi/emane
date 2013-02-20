/*
 * Copyright (c) 2008-2012 - DRS CenGen, LLC, Columbia, Maryland
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

#ifndef IEEE80211ABGMACLAYER_HEADER_
#define IEEE80211ABGMACLAYER_HEADER_

#include "emane/emanemaclayerimpl.h"
#include "emane/emanemactypes.h"
#include "emanecontrolmessages/universalphycontrolsendmessage.h"
#include "emanecontrolmessages/universalphycontrolrecvmessage.h"
#include "ieee80211abgmacheader.h"
#include "downstreamqueue.h"
#include "transmissiontxstate.h"
#include "macstatistics.h"
#include "emaneutils/flowcontrolmanager.h"
#include "macconfig.h"
#include "modetimingparameters.h"
#include "pcrmanager.h"
#include "neighbormanager.h"
#include "collisiontable.h"

#include <ace/Condition_T.h>
#include <ace/Thread_Mutex.h>
#include <ace/Timer_Queue_Adapters.h>
#include <ace/Timer_Heap.h>

#include <map>
#include <vector>


namespace IEEE80211ABG
{
  /**
   *
   * @brief structure used to define parameters to detect duplicate frames
   *
   */
  struct SequenceEntry
   {
     ACE_UINT16     seq_;        // sequence number
     ACE_Time_Value tv_;         // validity time

     /**
     *
     * default constructor
     *
     */

     SequenceEntry (ACE_UINT16 seq, const ACE_Time_Value & tv) : 
      seq_ (seq), 
      tv_ (tv) 
     { }
   };

  typedef std::vector < SequenceEntry > SequenceVector;

  typedef std::map < EMANE::NEMId, SequenceVector > DuplicateMap;
  typedef DuplicateMap::iterator DuplicateMapIter;

  class TransmissionTxState;

  /**
   * @class IEEE80211abgMACLayer 
   *  
   * @brief IEEE 80211 ABG MAC implementation
   *
   */
  class IEEE80211abgMACLayer : public EMANE::MACLayerImplementor
  {
   public:
    IEEE80211abgMACLayer (EMANE::NEMId id, EMANE::PlatformServiceProvider *pPlatformService);

    ~IEEE80211abgMACLayer ();

    /*
     * Component Interface
     */
    void initialize () throw (EMANE::InitializeException);

    void configure (const EMANE::ConfigurationItems & items)
      throw (EMANE::ConfigureException);

    void start () throw (EMANE::StartException);

    void postStart ();

    void stop () throw (EMANE::StopException);

    void destroy () throw ();

    /*
     * Upstream Interface
     */
    void processUpstreamControl (const EMANE::ControlMessage &);

    void processUpstreamPacket (const EMANE::CommonMACHeader &, EMANE::UpstreamPacket &, const EMANE::ControlMessage &);


    /*
     * Downstream Interface
     */
    void processDownstreamControl (const EMANE::ControlMessage &);

    void processDownstreamPacket (EMANE::DownstreamPacket &, const EMANE::ControlMessage &);

    void processEvent (const EMANE::EventId &, const EMANE::EventObjectState &);

    void processTimedEvent(ACE_UINT32 taskType, long eventId, const ACE_Time_Value &tv, const void *arg);


    /**
     *
     * collision type none, clobber or noise
     *
     */
    enum COLLISION_TYPE { COLLISION_TYPE_NONE                    = 0, 
                          COLLISION_TYPE_CLOBBER_RX_DURING_TX    = 1,
                          COLLISION_TYPE_NOISE_COMMON_RX         = 2, 
                          COLLISION_TYPE_CLOBBER_RX_HIDDEN_BUSY  = 4, 
                          COLLISION_TYPE_NOISE_HIDDEN_RX         = 8 };


    /**
     *
     * get the NEM id
     *
     * @retval NEM id
     *
     */
    EMANE::NEMId getId ()const;


    /**
     *
     * get next sequence number
     *
     * @retval seqNumber next sequence number
     *
     */
    ACE_UINT16 getNextSequenceNumber ();



    /**
     *
     * send a downstream multicast data packet
     * 
     * @param entry downstream queue entry
     * 
     */
    void sendDownstreamMulticastData (DownstreamQueueEntry &);


    /**
     *
     * send a downstream unicast data packet
     * 
     * @param entry downstream queue entry
     * 
     */
    void sendDownstreamUnicastData (DownstreamQueueEntry &entry);


    /**
     *
     * send a downstream unicast cts ctrl packet
     * 
     * @param entry downstream queue entry
     * @param origin the origin of the unicast rts/cts exchange
     * 
     */
    void sendDownstreamUnicastCts (DownstreamQueueEntry & entry, EMANE::NEMId origin);

    /**
     *
     * send a downstream message
     * 
     * @param entry downstream queue entry
     * @param macHeader specific mac header for the message
     * 
     */
    void sendDownstreamMessage (const DownstreamQueueEntry & entry, IEEE80211ABG::MacHeader & macHeader);

    /**
     *
     * check if a duplicate packet has been received
     *
     * @param src the source nem
     * @param seq sequence number of packet
     * 
     * @retval true if src and seq match the last received values, otherwise false.
     *
     */
    bool isDuplicate (EMANE::NEMId src, ACE_UINT16 seq);


    /**
     *
     * add token to flow control
     *
     * @retval true on success false on failure
     *
     */
    bool addToken ();


    /**
     *
     * remove token from flow control
     *
     * @retval true on success false on failure
     *
     */
    bool removeToken ();


    /**
     *
     * set the pre and post delays for a downstream queue entry
     *
     * @param entry the downstream entry
     *
     */
    void setDelayTime(IEEE80211ABG::DownstreamQueueEntry &entry);


    /**
     *
     * get the type of collision during rx
     *
     * @param src the src of the pkt 
     *
     * @param qidx the queue service type
     *
     * @param retries the number of retries
     *
     * @retval returns the COLLISION_TYPE
     *
     */
    COLLISION_TYPE checkForRxCollision(EMANE::NEMId src, ACE_UINT8 qidx, ACE_UINT8 retries);


    /**
     * provides access to the MAC layer statistics
     *
     * @return reference to the statistics object
     */
    MACStatistics & getStatistics ();

    /**
     * provides access to the MAC layer config
     *
     * @return reference to the config object
     */
    MACConfig & getConfig ();

    /**
     * provides access to the MAC layer mode timing 
     *
     * @return reference to the configuration object
     */
    ModeTimingParameters & getModeTiming ();


  private:
    static const EMANE::RegistrationId registrationId_ = EMANE::REGISTERED_EMANE_MAC_IEEE_802_11_ABG;

    EMANE::NEMId id_;

    MACStatistics macStatistics_;

    DownstreamQueue downstreamQueue_;

    ACE_thread_t downstreamThread_;

    ACE_THR_FUNC_RETURN processDownstreamQueue ();

    bool running_;

    ACE_Thread_Mutex mutex_;

    ACE_Condition < ACE_Thread_Mutex > cond_;

    TransmissionTxState *pTxState_;

    PCRManager pcrManager_;

    NeighborManager neighborManager_;

    EMANEUtils::FlowControlManager flowControlManager_;

    CollisionTable collisionTable_;

    ACE_UINT16 seqNumber_;

    DuplicateMap duplicateMap_;

    MACConfig  macConfig_;

    ModeTimingParameters modeTiming_;

    ACE_Time_Value tvLastRxEndTime_;

    ACE_Time_Value tvLastUpdateTime_;

    void handleUpstreamPacket (EMANE::UpstreamPacket & pkt, const EMANE::UniversalPhyControlRecvMessage &ctrl);

    bool checkUpstremReception (EMANE::UpstreamPacket &pkt, 
                                const float fRxPowerdBm, 
                                const float fNoiseFloordBm,
                                const IEEE80211ABG::MacHeader & macHeader,
                                int tryNum);

    void changeDownstreamState (TransmissionTxState *);

    bool checkPOR (float fSINR, size_t packetSize, ACE_UINT16 u16DataRateIndex);

    ACE_UINT8 dscpToQindex(ACE_UINT8 dscp) const;

    friend class TransmissionTxState;
  };
}

#endif //IEEE80211ABGMACLAYER_HEADER_
