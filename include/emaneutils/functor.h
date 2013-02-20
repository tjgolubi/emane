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

#ifndef UTILSFUNCTOR_HEADER_
#define UTILSFUNCTOR_HEADER_

namespace EMANEUtils
{
  /**
   * @class Functor
   *
   * @brief Generic function object 
   */
  template <class TRet>
  class Functor
  {
  public:
    virtual TRet operator()() = 0;
    virtual ~Functor(){}
    
  protected:
    Functor(){}
  };

  /**
   * @class Functor1Param
   *
   * @brief Generic single parameter function object 
   */
  template <class T, class TRet, class TParam1>
  class Functor1Param : public Functor<TRet>
  {
  public:
    Functor1Param(T & ref,TRet (T::*f)(TParam1),TParam1 param1):
      ref_(ref),func_(f),param1_(param1){};
    
    TRet operator()()
    {
      return (ref_.*func_)(param1_); 
    }
    
  private:
    T & ref_;
    TRet (T::*func_)(TParam1);
    TParam1 param1_;
  };

  /**
   * @class Functor2Param
   *
   * @brief Generic two parameter function object 
   */
  template <class T, class TRet, class TParam1, class TParam2>
  class Functor2Param : public Functor<TRet>
  {
  public:
    Functor2Param(T & ref,TRet (T::*f)(TParam1,TParam2),TParam1 param1, TParam2 param2):
      ref_(ref),func_(f),param1_(param1),param2_(param2){};
    
    TRet operator()()
    {
      return (ref_.*func_)(param1_,param2_);
    }
    
  private:
    T & ref_;
    TRet (T::*func_)(TParam1,TParam2);
    TParam1 param1_;
    TParam2 param2_;
  };

  /**
   * @class Functor3Param
   *
   * @brief Generic three parameter function object 
   */
  template <class T, class TRet, class TParam1, class TParam2, class TParam3>
  class Functor3Param : public Functor<TRet>
  {
  public:
    Functor3Param(T & ref,TRet (T::*f)(TParam1,TParam2,TParam3),TParam1 param1, TParam2 param2, TParam3 param3):
      ref_(ref),func_(f),param1_(param1),param2_(param2),param3_(param3){};
    
    TRet operator()()
    {
      return (ref_.*func_)(param1_,param2_,param3_);
    }
    
  private:
    T & ref_;
    TRet (T::*func_)(TParam1,TParam2,TParam3);
    TParam1 param1_;
    TParam2 param2_;
    TParam3 param3_;
  };

  /**
   * @class Functor4Param
   *
   * @brief Generic four parameter function object 
   */
  template <class T, class TRet, class TParam1, class TParam2, class TParam3, class TParam4>
  class Functor4Param : public Functor<TRet>
  {
  public:
    Functor4Param(T & ref,TRet (T::*f)(TParam1,TParam2,TParam3,TParam4),TParam1 param1, TParam2 param2, TParam3 param3,TParam4 param4):
      ref_(ref),func_(f),param1_(param1),param2_(param2),param3_(param3),param4_(param4){};
    
    TRet operator()()
    {
      return (ref_.*func_)(param1_,param2_,param3_,param4_);
    }
    
  private:
    T & ref_;
    TRet (T::*func_)(TParam1,TParam2,TParam3,TParam4);
    TParam1 param1_;
    TParam2 param2_;
    TParam3 param3_;
    TParam4 param4_;
  };


  /**
   * Helper function to create a one parameter function object
   */
  template<class T, class TRet, class TParam1>
  Functor<TRet> * makeFunctor(T & ref,TRet (T::*f)(TParam1),TParam1 param1)
  {
    return new Functor1Param<T,TRet,TParam1>(ref,f,param1);
  }

  /**
   * Helper function to create a two parameter function object
   */
  template<class T, class TRet, class TParam1, class TParam2>
  Functor<TRet> * makeFunctor(T & ref,TRet (T::*f)(TParam1,TParam2),TParam1 param1,TParam2 param2)
  {
    return new Functor2Param<T,TRet,TParam1,TParam2>(ref,f,param1,param2);
  }

  /**
   * Helper function to create a three parameter function object
   */
  template<class T, class TRet, class TParam1, class TParam2,class TParam3>
  Functor<TRet> * makeFunctor(T & ref,TRet (T::*f)(TParam1,TParam2,TParam3),TParam1 param1,TParam2 param2,TParam3 param3)
  {
    return new Functor3Param<T,TRet,TParam1,TParam2,TParam3>(ref,f,param1,param2,param3);
  }

  /**
   * Helper function to create a four parameter function object
   */
  template<class T, class TRet, class TParam1, class TParam2, class TParam3, class TParam4>
  Functor<TRet> * makeFunctor(T & ref,TRet (T::*f)(TParam1,TParam2,TParam3,TParam4),TParam1 param1,TParam2 param2,TParam3 param3, TParam4 param4)
  {
    return new Functor4Param<T,TRet,TParam1,TParam2,TParam3,TParam4>(ref,f,param1,param2,param3,param4);
  }


}

#endif // UTILSFUNCTOR_HEADER_
