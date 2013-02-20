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

#include "gpsdlocationagent.h"
#include "emane/emaneconstants.h"

#include "emaneutils/parameterconvert.h"
#include <sstream>

#include <ace/LSOCK_Connector.h>
#include <ace/SOCK_Connector.h>
#include <ace/DEV_Connector.h>
#include <ace/FILE_Connector.h>
#include <ace/TTY_IO.h>
#include <ace/OS_NS_time.h>
#include <ace/OS_NS_sys_time.h>

#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h> 
#endif

namespace
{
  // configuration defaults
  const char * pzDefualtGPSDControlSocket     = "/tmp/gpsd.control";
  const char * pzDefaultPseudoTerminalFile    = "/tmp/gpsdlocation.pty";
  const char * pzDefaultGPSDConnectionEnabled = "off";
  
  EMANE::ConfigurationDefinition defs[] =
    {
      {false,true,1,"gpsdcontrolsocket",    pzDefualtGPSDControlSocket,    0,"GPSD Control Socket"},
      {false,true,1,"pseudoterminalfile",   pzDefaultPseudoTerminalFile,   0,"File holding pseudo term name"},
      {true, true,1,"gpsdconnectionenabled",pzDefaultGPSDConnectionEnabled,0,"Actively connect to gpsd"},
      {0,0,0,0,0,0,0},
    };
}


GPSDLocationAgent::GPSDLocationAgent(EMANE::NEMId nemId, EMANE::PlatformServiceProvider *pPlatformService):
  EventAgent(nemId, pPlatformService),
  cacheIndex_(0),
  nemId_(nemId),
  masterPTY_(0),
  slavePTY_(0),
  currentPos(NULL)
{
  configRequirements_ = EMANE::loadConfigurationRequirements(defs);
}

GPSDLocationAgent::~GPSDLocationAgent(){}

EMANE::EventRegistrationList GPSDLocationAgent::getEventRegistrationList()
{
  EMANE::EventId id = LocationEvent::EVENT_ID;
  EMANE::EventRegistrationList regList;
  regList.push_back(id);
  return regList;
}
    
void GPSDLocationAgent::processEvent(const EMANE::EventId & eventId, const EMANE::EventObjectState & state)
{
  
  pPlatformService_->log(EMANE::DEBUG_LEVEL,"processEvent Called");  

  if(eventId ==  LocationEvent::EVENT_ID)
    {
      bool bFound = false;
      
      LocationEvent event(state);
      
      size_t numberEntries =  event.getNumberOfEntries();
      
      const LocationEvent::LocationEntry *e = event.getEntries();
      
      if(cacheIndex_ < numberEntries)
        {
          if(e[cacheIndex_].u16Node_ == nemId_)
            {
              bFound = true;
            }
        }
      
      if(!bFound)
        {
          for(size_t i = 0; i < numberEntries; ++i)
            {
              if(e[i].u16Node_ == nemId_)
                {
                  cacheIndex_ = i;
                  bFound = true;
                  break;
                }
            }
        }
      
      if(bFound)
        {
          if(currentPos)
          {       
            delete [] currentPos;
          }
          currentPos = new LocationEvent::LocationEntry[numberEntries];
          memcpy(currentPos,e,numberEntries * sizeof(LocationEvent::LocationEntry));
        }
    }

}

void GPSDLocationAgent::processTimedEvent(ACE_UINT32, long, const ACE_Time_Value &, const void *)
{
  if(currentPos != NULL)
  {
    float fLatitude = currentPos[cacheIndex_].i32LatitudeMARCS_   / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;
    float fLongitude = currentPos[cacheIndex_].i32LongitudeMARCS_ / EMANE::MILLI_ARC_SECONDS_PER_DEGREE;

    sendSpoofedNMEA(fLatitude,fLongitude,currentPos[cacheIndex_].i32AltitudeMeters_);

    pPlatformService_->log(EMANE::DEBUG_LEVEL,"gpsdlocationaganet NEM: %hu lat: %f lon: %f alt:%d",
          nemId_,
          fLatitude,
          fLongitude,
          currentPos[cacheIndex_].i32AltitudeMeters_);
  }
}

void GPSDLocationAgent::initialize()
  throw(EMANE::InitializeException){}

void GPSDLocationAgent::configure(const EMANE::ConfigurationItems & items)
  throw(EMANE::ConfigureException)
{
  Component::configure(items);
}

void GPSDLocationAgent::start()
  throw(EMANE::StartException)
{
  std::string sGPSDControlSocket;
  EMANE::ConfigurationRequirements::iterator iter = configRequirements_.begin();
  char pzName[MAXPATHLEN - 1];

  memset(pzName,0,sizeof(pzName));

  try
    {
      for(;iter != configRequirements_.end();++iter)
        {
          if(iter->second.bPresent_)
            {
              if(iter->first == "gpsdcontrolsocket")
                {
                  sGPSDControlSocket = iter->second.item_.getValue();
                }
              else if(iter->first == "gpsdconnectionenabled")
                {
                  bGPSDConnectionEnabled_ = 
                    EMANEUtils::ParameterConvert(iter->second.item_.getValue()).toBool();
                }
              else if(iter->first == "pseudoterminalfile")
                {
                  sPseudoTerminalFile_ = iter->second.item_.getValue();
                }
            }
          else if(iter->second.bRequired_)
            {
              std::stringstream ssDescription;
              ssDescription<<"GPSDLocationAgent: Missing configuration item "<<iter->first<<std::ends;
              throw EMANE::StartException(ssDescription.str());
            }
        }
    }
  catch(EMANEUtils::ParameterConvert::ConversionException & exp)
    {
      std::stringstream sstream;
      sstream<<"GPSDLocationAgent: Parameter "<<iter->first<<": "<<exp.what()<<std::ends;
      throw EMANE::StartException(sstream.str());
    }
  
  ACE_LSOCK_Stream gpsdControlStream;
  gpsdControlAddr_.set(sGPSDControlSocket.c_str());
  ACE_LSOCK_Connector connect;

  
  if(bGPSDConnectionEnabled_)
    {
      if(connect.connect(gpsdControlStream,gpsdControlAddr_) == -1)
        {
          std::stringstream ssDescription;
          ssDescription<<"unable to open GPSD control socket"<<std::ends;
          throw EMANE::StartException(ssDescription.str());
        }
    }

  if(openpty(&masterPTY_, &slavePTY_, &pzName[1],NULL, NULL) == -1)
    {
      throw EMANE::StartException("Unable to open pseudo terminal for gpsd"); 
    }

  ACE_TTY_IO slaveTTY;
  ACE_DEV_Connector devConnector;
  ACE_TTY_IO::Serial_Params serial_params;

  if(devConnector.connect(slaveTTY,ACE_DEV_Addr(&pzName[1])) == -1)
    {
      throw EMANE::StartException("Unable to open slave pseudo terminal for gpsd"); 
    }

  slaveTTY.control(ACE_TTY_IO::GETPARAMS,&serial_params);

  serial_params.baudrate = 4800;
  serial_params.xonlim = 0;
  serial_params.xofflim = 0;
  serial_params.readmincharacters = 1;
  serial_params.readtimeoutmsec = -1;
  serial_params.paritymode = "none";
  serial_params.ctsenb = false;
  serial_params.rtsenb = 0;
  serial_params.xinenb = false;
  serial_params.xoutenb = false;
  serial_params.modem = false;
  serial_params.rcvenb = true;
  serial_params.dsrenb = false;
  serial_params.dtrdisable = false;
  serial_params.databits = 8;
  serial_params.stopbits = 1;

  slaveTTY.control(ACE_TTY_IO::SETPARAMS,&serial_params);

  slaveTTY.close();
  
  // open up the file containing the name of the pseudo ternimal
  ACE_FILE_Connector connector;
  if(connector.connect(pseudoTerminalNameFile_,
                       ACE_FILE_Addr(sPseudoTerminalFile_.c_str()),
                       0,
                       ACE_Addr::sap_any,
                       0,
                       O_WRONLY | O_CREAT | O_TRUNC) == -1)
    {
      std::stringstream ssDescription;
      ssDescription<<"unable to open pseudo terminal name file: "<<sPseudoTerminalFile_<<std::ends;
      throw EMANE::StartException(ssDescription.str());
    }
 

  sClientPTTYName_ = &pzName[1];

  pzName[0] = '+';
  pzName[strlen(pzName)] = '\n';
 
  // store the pseudo terminal name
  pseudoTerminalNameFile_.send(&pzName[1],strlen(pzName)-1);
  pseudoTerminalNameFile_.close();
  
  if(bGPSDConnectionEnabled_)
    {
      gpsdControlStream.send(pzName,strlen(pzName));
      
      gpsdControlStream.close();

      ACE_INET_Addr addr("localhost:2947");
      ACE_SOCK_Connector connect2;

      if(connect2.connect(gpsdClientStream_,addr) == -1)
        {
          std::stringstream ssDescription;
          ssDescription<<"unable to open GPSD stream socket"<<std::ends;
          throw EMANE::StartException(ssDescription.str());
        }

      gpsdClientStream_.send("w+r+\n", 5);
    }

  
    ACE_Time_Value tvInterval(1);
    ACE_Time_Value tvTimeOut = ACE_OS::gettimeofday();
    eventId = pPlatformService_->scheduleTimedEvent(0,NULL,tvTimeOut+tvInterval,tvInterval);

    if(eventId >= 0)
      {
       pPlatformService_->log(EMANE::DEBUG_LEVEL,"Timer Scheduled");             
      }
    else
      {
       pPlatformService_->log(EMANE::DEBUG_LEVEL,"Timer Not Scheduled");  
      }
}

void GPSDLocationAgent::stop()
  throw(EMANE::StopException)
{
  if(!sClientPTTYName_.empty())
    {
      if(bGPSDConnectionEnabled_)
        {
          ACE_LSOCK_Stream gpsdControlStream;
          ACE_LSOCK_Connector connect;
          
          if(connect.connect(gpsdControlStream,gpsdControlAddr_) != -1)
            {
              std::string sTmp = sClientPTTYName_;
              sTmp.append(1,'-');
              sTmp.push_back('\n');
              gpsdControlStream.send(sTmp.c_str(),sTmp.length());
            }
        }

      // remove the pseudo terminal filename file
      pseudoTerminalNameFile_.unlink();
    }
  pPlatformService_->cancelTimedEvent(eventId);
  }

void GPSDLocationAgent::destroy()
  throw()
{}

void GPSDLocationAgent::sendSpoofedNMEA(float fLatitude, float fLongitude, ACE_INT32 i32Altitude)
{
  time_t t;
  tm tmval;
  char buf[1024];

  const int NUM_GSV_STRINGS   = 2;
  const int NUM_SV_PER_STRING = 4;

  int elv[NUM_GSV_STRINGS * NUM_SV_PER_STRING] = {41,   9,  70,  35,  10,  53,  2, 48 };
  int azm[NUM_GSV_STRINGS * NUM_SV_PER_STRING] = {104, 84,  30, 185, 297, 311, 29, 64 };
  int snr[NUM_GSV_STRINGS * NUM_SV_PER_STRING] = {41,  51,  39,  25,  25,  21, 29, 32 };

  char cLatitudeHemisphere  =  (fLatitude  > 0) ? 'N' : 'S';
  char cLongitudeHemisphere =  (fLongitude > 0) ? 'E' : 'W';
  
  if(fLatitude < 0)
    {
      fLatitude *= -1;
    }
  
  if(fLongitude < 0)
    {
      fLongitude *= -1;
    }
  
  int iLongitudeDegrees = static_cast<int>(fLongitude);
  int iLatitudeDegrees  = static_cast<int>(fLatitude);
  
  float fLongitudeMinutes = (fLongitude - iLongitudeDegrees) * 60.0;
  float fLatitudeMinutes  = (fLatitude - iLatitudeDegrees) * 60.0;
  
  int iLongitudeMinutes = static_cast<int>(fLongitudeMinutes);
  int iLatitudeMinutes  = static_cast<int>(fLatitudeMinutes);

  int iGPSquality = 2;
  int iNumSatellitesUsed = NUM_GSV_STRINGS * NUM_SV_PER_STRING;

  float fDOP = 1.8;
  float fHorizontalDOP = 1.1;
  float fVerticalDOP   = 1.3;

  float fGeoidalHeight = -34.0;
  
  fLongitudeMinutes -= iLongitudeMinutes;
  fLatitudeMinutes  -= iLatitudeMinutes;
  
  fLongitudeMinutes *= 10000;
  fLatitudeMinutes  *= 10000;
  
  ACE_OS::time(&t);
  ACE_OS::gmtime_r(&t,&tmval);

  /* NMEA GGA */
  snprintf(buf,sizeof(buf),"$GPGGA,%02d%02d%02d,%02d%02d.%04d,%c,%03d%02d.%04d,%c,%d,%02d,%.1f,%.1f,M,%.1f,M,,",
           tmval.tm_hour,
           tmval.tm_min,
           tmval.tm_sec,
           iLatitudeDegrees,
           iLatitudeMinutes,
           static_cast<int>(fLatitudeMinutes),
           cLatitudeHemisphere,
           iLongitudeDegrees,
           iLongitudeMinutes,
           static_cast<int>(fLongitudeMinutes),
           cLongitudeHemisphere,
           iGPSquality,
           iNumSatellitesUsed,
           fHorizontalDOP,
           static_cast<float>(i32Altitude),
           fGeoidalHeight
           );
  
  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));


  /* NMEA RMC */
  snprintf(buf,sizeof(buf),"$GPRMC,%02d%02d%02d,A,%02d%02d.%04d,%c,%03d%02d.%04d,%c,000.0,000.0,%02d%02d%02d,000.0,%c",
           tmval.tm_hour,
           tmval.tm_min,
           tmval.tm_sec,
           iLatitudeDegrees,
           iLatitudeMinutes,
           static_cast<int>(fLatitudeMinutes),
           cLatitudeHemisphere,
           iLongitudeDegrees,
           iLongitudeMinutes,
           static_cast<int>(fLongitudeMinutes),
           cLongitudeHemisphere,
           tmval.tm_mday,
           tmval.tm_mon + 1,
           (tmval.tm_year + 1900) % 100,
           cLongitudeHemisphere
           );
  
  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));


  /* NMEA GSA */
  snprintf(buf,sizeof(buf), "$GPGSA,A,3,01,02,03,04,05,06,07,08,,,,,%2.1f,%2.1f,%2.1f",
           fDOP, fHorizontalDOP, fVerticalDOP);

  doCheckSumNMEA(buf,sizeof(buf));
  write(masterPTY_,buf,strlen(buf));
    
 
  for(int i = 0; i < NUM_GSV_STRINGS; ++i) 
   { 
    const int a = (NUM_SV_PER_STRING * i) + 0;
    const int b = (NUM_SV_PER_STRING * i) + 1;
    const int c = (NUM_SV_PER_STRING * i) + 2;
    const int d = (NUM_SV_PER_STRING * i) + 3;

    /* NMEA GSV */
    snprintf(buf,sizeof(buf), "$GPGSV,%d,%d,%02d,%02d,%02d,%03d,%02d,%02d,%02d,%03d,%02d,%02d,%02d,%03d,%02d,%02d,%02d,%03d,%02d",
             NUM_GSV_STRINGS,
             i + 1,
             iNumSatellitesUsed,
             a + 1,  elv[a], azm[a], snr[a],
             b + 2,  elv[b], azm[b], snr[b],
             c + 3,  elv[c], azm[c], snr[c],
             d + 4,  elv[d], azm[d], snr[d]);
 
    doCheckSumNMEA(buf,sizeof(buf));
    write(masterPTY_,buf,strlen(buf));
  }
}


void GPSDLocationAgent::doCheckSumNMEA(char *buf, size_t len)
{
  char foo[6] = {0};

  char chksum = buf[1];
  
  for(unsigned int i = 2; i < strlen(buf); ++i)
    {
      chksum ^= buf[i];
    }
  
  snprintf(foo, sizeof(foo), "*%02X\r\n", static_cast<unsigned int>(chksum));
  
  strncat(buf, foo, len - strlen(buf) - 1);
}

DECLARE_EVENT_AGENT(GPSDLocationAgent);
