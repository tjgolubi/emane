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


#include "downstreamqueue.h"
#include "macconfig.h"
#include "macstatistics.h"

#include <ace/Assert.h>



/**
 *
 * @brief queue constructor
 * 
 */
IEEE80211ABG::DownstreamQueue::DownstreamQueue (EMANE::NEMId id, EMANE::StatisticServiceProvider *pOwner):
 id_ (id),
 numCategories_ (NUM_ACCESS_CATEGORIES_DEFAULT), 
 cond_ (mutex_), 
 canceled_ (false),
 pOwner_ (pOwner)
{
  // initialize sizes
  for (ACE_UINT8 idx = 0; idx < IEEE80211ABG::NUM_ACCESS_CATEGORIES_MAX; ++idx)
    {
      categories_[idx].registerStatistics(idx,pOwner_);
      categories_[idx].category_ = idx;
      categories_[idx].maxQueueSize_ = IEEE80211ABG::QUEUE_SIZE_DEFAULT;
      categories_[idx].numEntrySizeMax_ = IEEE80211ABG::MAX_PKT_SIZE_DEFAULT;
      categories_[idx].numHighWaterMark_ = 0;

    }
}


/**
 *
 * @brief queue destructor
 * 
 */
IEEE80211ABG::DownstreamQueue::~DownstreamQueue ()
{
  for (ACE_UINT8 idx = 0; idx < IEEE80211ABG::NUM_ACCESS_CATEGORIES_MAX; ++idx)
    {
      //pOwner_->unregisterContainer (&categories_[idx]);
    }
}



/**
 *
 * @brief get total number of entries (all queues)
 *
 * @retval number of entries
 * 
 */
size_t 
IEEE80211ABG::DownstreamQueue::total ()
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  size_t total = 0;

  // total queue size
  for (ACE_UINT8 idx = 0; idx < numCategories_; ++idx)
    {
      // sum total for each queue (category)
      total += categories_[idx].queue_.size ();
    }

  return total;
}



/**
 *
 * @brief set the max number of entries for a given queue index
 *
 * @param max max number of entrie size
 * @param idx queue index
 * 
 */
void
IEEE80211ABG::DownstreamQueue::maxsize (size_t max, ACE_UINT8 idx)
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  // clear entries that extend past the max limit
  while (max < categories_[idx].queue_.size ())
    {
      categories_[idx].queue_.pop ();
    }

  // set new size
  categories_[idx].maxQueueSize_ = max;
}


/**
 *
 * @brief get the number of entries for a given queue index
 *
 * @param idx queue index
 *
 * @retval number of entries
 * 
 */
size_t 
IEEE80211ABG::DownstreamQueue::size (ACE_UINT8 idx)
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  return categories_[idx].queue_.size ();
}


/**
 *
 * @brief set the number of categories (queues)
 *
 * @param categories
 *
 */
void
IEEE80211ABG::DownstreamQueue::categories (ACE_UINT8 categories)
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  for (ACE_UINT8 idx = categories; idx < numCategories_; ++idx)
    {
      // clear queues that will no longer be used
      while (categories_[numCategories_ - idx].queue_.empty () == false)
        {
          categories_[numCategories_ - idx].queue_.pop ();
        }
    }

  // set new size
  numCategories_ = categories;
}


/**
 *
 * @brief blocking dequeue, returns highest priority item first
 *
 * @retval queue entry
 *
 */
IEEE80211ABG::DownstreamQueueEntry 
IEEE80211ABG::DownstreamQueue::dequeue ()
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  // queue index (category)
  ACE_UINT8 idx = 0;

  // wait while queue is empty, and not canceled, wait for idx to be set
  while (empty (idx) && !canceled_)
    {
      cond_.wait ();
    }

  // woke up canceled
  if (canceled_)
    {
      // create empty entry
      return DownstreamQueueEntry ();
    }
  // get entry
  DownstreamQueueEntry entry = categories_[idx].queue_.front ();

  // pop entry 
  categories_[idx].queue_.pop ();

  // bump packets dequeued
  if (entry.pkt_.getPacketInfo ().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS)
    {
      ++(categories_[idx].numMcastPacketsDequeued_);
      categories_[idx].numMcastBytesDequeued_ += entry.pkt_.length ();
    }
  else
    {
      ++(categories_[idx].numUcastPacketsDequeued_);
      categories_[idx].numUcastBytesDequeued_ += entry.pkt_.length ();
    }

  return entry;
}


/**
 *
 * @brief enqueue, inserts items by priority, signals on success.
 *
 */
bool 
IEEE80211ABG::DownstreamQueue::enqueue (DownstreamQueueEntry & entry)
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  // queue disabled if max size for this category is 0
  if (categories_[entry.qidx_].maxQueueSize_ == 0)
    {
      // bump overflow count
      if (entry.pkt_.getPacketInfo ().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS)
        {
          ++(categories_[entry.qidx_].numMcastPacketsOverFlow_);
          categories_[entry.qidx_].numMcastBytesOverFlow_ += entry.pkt_.length ();
        }
      else
        {
          ++(categories_[entry.qidx_].numUcastPacketsOverFlow_);
          categories_[entry.qidx_].numUcastBytesOverFlow_ += entry.pkt_.length ();
        }

      // drop 
      return false;
    }
  else
    {
      // check for queue overflow
      while (categories_[entry.qidx_].queue_.size () >= categories_[entry.qidx_].maxQueueSize_)
        {
          // bump overflow count
          if (entry.pkt_.getPacketInfo ().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS)
            {
              ++(categories_[entry.qidx_].numMcastPacketsOverFlow_);
              categories_[entry.qidx_].numMcastBytesOverFlow_ += entry.pkt_.length ();
            }
          else
            {
              ++(categories_[entry.qidx_].numUcastPacketsOverFlow_);
              categories_[entry.qidx_].numUcastBytesOverFlow_ += entry.pkt_.length ();
            }

          // pop entry
          categories_[entry.qidx_].queue_.pop ();
        }
    }

  // if max msdu is enabled and entry too large
  if ((categories_[entry.qidx_].numEntrySizeMax_.getValue () != 0) &&
      (entry.pkt_.length () > categories_[entry.qidx_].numEntrySizeMax_.getValue ()))
    {
      // bump entry exceeded msdu
      if (entry.pkt_.getPacketInfo ().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS)
        {
          ++(categories_[entry.qidx_].numMcastPacketsTooLarge_);
          categories_[entry.qidx_].numMcastBytesTooLarge_ += entry.pkt_.length ();
        }
      else
        {
          ++(categories_[entry.qidx_].numUcastPacketsTooLarge_);
          categories_[entry.qidx_].numUcastBytesTooLarge_ += entry.pkt_.length ();
        }

      // drop
      return false;
    }
  // push entry to back of queue
  categories_[entry.qidx_].queue_.push (entry);

  // bump enque pkt/byte count
  if (entry.pkt_.getPacketInfo ().destination_ == EMANE::NEM_BROADCAST_MAC_ADDRESS)
    {
      ++(categories_[entry.qidx_].numMcastPacketsEnqueued_);
      categories_[entry.qidx_].numMcastBytesEnqueued_ += entry.pkt_.length ();
    }
  else
    {
      ++(categories_[entry.qidx_].numUcastPacketsEnqueued_);
      categories_[entry.qidx_].numUcastBytesEnqueued_ += entry.pkt_.length ();
    }

  // bump high water mark
  if (categories_[entry.qidx_].queue_.size () > categories_[entry.qidx_].numHighWaterMark_.getValue ())
    {
      categories_[entry.qidx_].numHighWaterMark_ = categories_[entry.qidx_].queue_.size ();
      categories_[entry.qidx_].numHighWaterMax_ = categories_[entry.qidx_].maxQueueSize_;
    }
  // signal entry added
  cond_.signal ();

  // success
  return true;
}


/**
 *
 * @brief empty status of all queues.
 *
 * @param ref index of highest priority non-empty queue.
 * 
 * @retval returns true if all active queues are empty, otherwise returns false and sets queue index.
 *
 */
bool 
IEEE80211ABG::DownstreamQueue::empty (ACE_UINT8 & ref)
{
  // called internally from guarded section

  // try higher priority first, work down to lower priority
  for (int idx = numCategories_ - 1; idx >= 0; --idx)
    {
      // queue not empty
      if (categories_[idx].queue_.empty () == false)
        {
          // set queue index that is ready
          ref = idx;

          // return queue not empty
          return false;
        }
    }

  // return queue empty
  return true;
}

/**
 *
 * @brief set the cancel flag to true and signal wakeup
 *
 *
 */
void
IEEE80211ABG::DownstreamQueue::cancel ()
{
  ACE_Guard < ACE_Thread_Mutex > m (mutex_);

  // set canceled flag
  canceled_ = true;

  // signal wake up
  cond_.signal ();
}
