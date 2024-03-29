//
//  ACEdome.cpp
//  ACE rotation drive unit
//
//  Created by Rodolphe Pineau on 6/11/2018.


#include "acedome.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

CACEDome::CACEDome()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dHomeAz = 180;
    m_nHomingTries = 0;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_bCalibrating = false;

    m_bShutterOpened = false;
    m_bDropoutDisabled = true;

    m_bParked = true;
    m_bCloseOnPark = false;
    m_bHomed = false;
    m_sFirmwareVersion.clear();

    m_nNbRainSensors = 0;
    m_nCurrentShutterAction = UNKNOWN;
    
    m_nShutterStateD1 = UNKNOWN;
    m_nShutterStateD2 = UNKNOWN;
    
#ifdef ACE_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\ACELog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/ACELog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/ACELog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CACEDome::CACEDome()  Version 2020_12_16_2315.\n", timestamp);
    fprintf(Logfile, "[%s] CACEDome::CACEDome() Called\n", timestamp);
    fflush(Logfile);
#endif

}

CACEDome::~CACEDome()
{

}

int CACEDome::Connect(const char *pszPort)
{
    int nErr;

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::Connect] Connect called.\n", timestamp);
    fprintf(Logfile, "[%s] [CACEDome::Connect] Connecting to port %s.\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 19200 8N1
    nErr = m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }
    m_pSerx->purgeTxRx();
    m_bIsConnected = true;

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::Connect] Connected.\n", timestamp);
    fprintf(Logfile, "[%s] [CACEDome::Connect] Getting Firmware.\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_sFirmwareVersion);
    if(nErr) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::Connect] Error Getting Firmware : err = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return ERR_CMDFAILED;
    }

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::Connect] Got Firmware : %s\n", timestamp, m_sFirmwareVersion.c_str());
    fflush(Logfile);
#endif

    nErr = getShutterState();
    setDecimalFormat(2); // 2 decimals for degrees.
    getDomeAzCoast(m_dCoastAz);
    getDomeHomeAz(m_dHomeAz);
    getDomeStepPerRev(m_nNbStepPerRev);
    getDomeHomeAz(m_dCurrentAzPosition);
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::Connect]m_dCoastAz            : %3.2f\n", timestamp, m_dCoastAz);
    fprintf(Logfile, "[%s] [CACEDome::Connect]m_dHomeAz             : %3.2f\n", timestamp, m_dHomeAz);
    fprintf(Logfile, "[%s] [CACEDome::Connect]m_nNbStepPerRev       : %d\n", timestamp, m_nNbStepPerRev);
    fprintf(Logfile, "[%s] [CACEDome::Connect]m_dCurrentAzPosition  : %3.2f\n", timestamp, m_dCurrentAzPosition);
    fflush(Logfile);
#endif


    return SB_OK;
}


void CACEDome::Disconnect()
{
    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}


int CACEDome::readResponse(char *pszRespBuffer, int nBufferLen, unsigned long &nbRead)
{
    int nErr = ACE_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined ACE_DEBUG && ACE_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::readResponse] nBytesWaiting = %d\n", timestamp, nBytesWaiting);
        fflush(Logfile);
#endif
        
        if(!nBytesWaiting) {
            if(nbTimeouts++ >= NB_RX_WAIT) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CRTIDome::CACEDome] bytesWaitingRx timeout, no data for %d loops\n", timestamp, NB_RX_WAIT);
                fflush(Logfile);
#endif
                nErr = PROMPT_TIMEOUT;
                break;
            }
            m_pSleeper->sleep(MAX_READ_WAIT_TIMEOUT);
            continue;
        }

        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= nBufferLen)
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, MAX_TIMEOUT);
        else {
            nErr = PROMPT_TIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined ACE_DEBUG && ACE_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::readResponse] readFile error.\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) {// timeout
#if defined ACE_DEBUG && ACE_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::readResponse] readFile error ulBytesRead != nBytesWaiting , nErr = %d\n", timestamp, nErr);
            fflush(Logfile);
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    } while ( strstr(pszRespBuffer,">\r") == NULL && strstr(pszRespBuffer,">\n") == NULL && ulTotalBytesRead < nBufferLen );

#if defined ACE_DEBUG && ACE_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::readResponse] on Exit nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    if(!ulTotalBytesRead)
        nErr = ERR_RXTIMEOUT;
    
    nbRead = ulTotalBytesRead;

    return nErr;
}


int CACEDome::domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = ACE_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
    unsigned long nbRead;
    
    m_pSerx->purgeTxRx();

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::domeCommand] Sending %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    if(pszResult) {
        // read response
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, nbRead);
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::domeCommand] response  code is %d with data :\n%s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif

        if(nErr && nErr != PROMPT_TIMEOUT)
            return nErr;

        strncpy(pszResult, szResp, nResultMaxLen);
        nErr = ACE_OK;
    }


    return nErr;

}

int CACEDome::getDomeAz(double &dDomeAz)
{
    int nErr = ACE_OK;
    std::vector<std::string> svPosition;
    std::string sHeading;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    dDomeAz = m_dCurrentAzPosition;

    if(m_bCalibrating)
        return nErr;

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::getDomeAz] m_dCurrentAzPosition : %3.2f\n", timestamp, m_dCurrentAzPosition);
    fflush(Logfile);
#endif

    nErr = getShortStatus(); // this timeout from time to time.
    if(nErr)
        return ACE_OK; // let's ignore the error and not change the data, they'll get pick up on the next request.

    // look for Home or Posn
    sHeading = findField(m_svShortStatus, "Home");
    if(!sHeading.size()) {
        sHeading = findField(m_svShortStatus, "Posn");
        if(!sHeading.size()) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::getDomeAz] Home or  Posn not found in fields !!! \n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }
    }

    // convert Az string to double
    if(sHeading.size()) {
        nErr = parseFields(sHeading.c_str(), svPosition, ' ');
        if(nErr) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::getDomeAz] parseFields for '%s' failed !!! \n", timestamp, sHeading.c_str());
            fflush(Logfile);
#endif
            return nErr;
        }
        if(svPosition.size()>1) {
            dDomeAz = fabs(fmod(atof(svPosition[1].c_str()), 360.0f));    // the modulo is just to be safe
            m_dCurrentAzPosition = dDomeAz;
        }
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        else
        {
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::getDomeAz] svPosition.size is toos small : %d\n", timestamp, int(svPosition.size()));
            fflush(Logfile);
            return nErr;
        }
#endif
    }
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::getDomeAz] m_dCurrentAzPosition : %3.2f\n", timestamp, m_dCurrentAzPosition);
    fflush(Logfile);
#endif

    return nErr;
}

int CACEDome::getDomeEl(double &dDomeEl)
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    getShutterState();

    if(!m_bShutterOpened)
    {
        dDomeEl = 0.0;
    }
    else {
        dDomeEl = 90.0;
    }

    m_dCurrentElPosition = dDomeEl;
    
    return nErr;
}


int CACEDome::getDomeHomeAz(double &dAz)
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getExtendedStatus();
    if(nErr)
        return nErr;
    // convert Az string to double
    sLine = findField(m_svExtStatus, "Home Azimuth");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ':');
        if(svFields.size()>1) {
            dAz = atof(svFields[1].c_str());
            m_dHomeAz = dAz;
        }

    }
    return nErr;
}

int CACEDome::getDomeParkAz(double &dAz)
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return nErr;

    getDomeHomeAz(dAz);
    return nErr;
}


int CACEDome::getShutterState()
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    nErr = getShortStatus(); // this timesout from time to time.
    if(nErr)
        return ACE_OK; // let's ignore the error and not change the data, they'll get pick up on the next request.
    // both shutter need to be open to report OPEN
    sLine = findField(m_svShortStatus, "D1 ");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ' ');
        if(svFields.size()>1) {
            if(svFields[1]=="SHUT") {
                m_nShutterStateD1 = CLOSED;
            }
            else if (svFields[1]=="OPEN") {
                m_nShutterStateD1 = OPEN;
            }
            else
                m_nShutterStateD1 = OPENING_D1;
        }
        if(m_nShutterStateD1 == OPEN)
            m_bShutterOpened = true;
    }

    if(!m_bDropoutDisabled) {
        sLine = findField(m_svShortStatus, "D2 ");
        if(!sLine.empty()) {
            parseFields(sLine.c_str(), svFields, ' ');
            if(svFields.size()>1) {
                if(svFields[1]=="SHUT") {
                    m_nShutterStateD2 = CLOSED;
                }
                else if (svFields[1]=="OPEN") {
                    m_nShutterStateD2 = OPEN;
                }
                else
                    m_nShutterStateD2 = OPENING_D2;
            }
        }
        if(m_nShutterStateD1 == OPEN && m_nShutterStateD2 == OPEN)
            m_bShutterOpened = true;
        else
            m_bShutterOpened = false;
    }
    return nErr;
}


int CACEDome::getDomeStepPerRev(int &nStepPerRev)
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::getDomeStepPerRev] [IN] m_nNbStepPerRev =  %d\n", timestamp, m_nNbStepPerRev);
    fflush(Logfile);
#endif

    nErr = getExtendedStatus();
    if(nErr)
        return nErr;

    // need to parse to find the Encoder Counts per 360 field.
    sLine = findField(m_svExtStatus, "Encoder Counts per 360");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ':');
        if(svFields.size()>1) {
            nStepPerRev=atoi(trim(svFields[1]," ").c_str());
        }
        else
            nStepPerRev = 0;
        m_nNbStepPerRev = nStepPerRev;
    }
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::getDomeStepPerRev] [OUT] m_nNbStepPerRev =  %d\n", timestamp, m_nNbStepPerRev);
    fflush(Logfile);
#endif

    return nErr;
}

int CACEDome::setDomeStepPerRev(int nStepPerRev)
{
    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%d LM\r\n", nStepPerRev);
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
    if(!nErr)
        m_nNbStepPerRev = nStepPerRev;

    return nErr;
}


int CACEDome::isDomeMoving(bool &bIsMoving)
{
    int nErr = ACE_OK;
    std::string sStatusLine;
    std::string sMouvement;
    std::vector<std::string> svStatusLineFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bIsMoving = true;
    nErr = getShortStatus(); // this timesout from time to time.
    if(nErr)
        return ACE_OK; // let's ignore the error and not change the data, they'll get pick up on the next request.

    if(m_svShortStatus.size()<3)
        return ACE_OK; // we got a weird response

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isDomeMoving] m_svShortStatus.size() =  %lu\n", timestamp, m_svShortStatus.size());
    fflush(Logfile);
#endif

    bIsMoving = false;
    // look for RL or RR
    sMouvement = findField(m_svShortStatus, "RL");
    if(!sMouvement.size()) {
        sMouvement = findField(m_svShortStatus, "RR");
        if(!sMouvement.size())
            return nErr;
    }
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isDomeMoving] sMouvement =  %s\n", timestamp, sMouvement.c_str());
    fflush(Logfile);
#endif
    nErr = parseFields(sMouvement.c_str(), svStatusLineFields, ' ');
    if(nErr)
        return nErr;
    if(svStatusLineFields.size()>=2) {
        if(svStatusLineFields[1] == "00")
            bIsMoving = false;
        else
            bIsMoving = true;
    }

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isDomeMoving] bIsMoving =  %s\n", timestamp, bIsMoving?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;
}

int CACEDome::isDomeAtHome(bool &bAtHome)
{
    int nErr = ACE_OK;
    std::string sStatusLine;
    std::vector<std::string> svStatusLineFields;
    std::vector<std::string> svPosition;
    std::string sHeading;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bAtHome = false;
    nErr = getShortStatus(); // this timesout from time to time.
    if(nErr)
        return ACE_OK; // let's ignore the error and not change the data, they'll get pick up on the next request.


    if(m_svShortStatus.size()<1)
        return ACE_OK; // we got a weird response

    // look for Home
    sStatusLine = findField(m_svShortStatus, "Home");
    if(sStatusLine.size()) {
         bAtHome = true;
        m_bHomePassed = true;
    }

    // look for Home or Posn
    sHeading = findField(m_svShortStatus, "Home");
    if(!sHeading.size()) {
        sHeading = findField(m_svShortStatus, "Posn");
        if(!sHeading.size())
            return nErr;
    }

    // convert Az string to double
    if(sHeading.size()) {
        nErr = parseFields(sHeading.c_str(), svPosition, ' ');
        if(nErr)
            return nErr;
        if(svPosition.size()>1) {
            m_dCurrentAzPosition = fmod(atof(svPosition[1].c_str()), 360.0f);
        }
    }

    return nErr;
  
}


int CACEDome::syncDome(double dAz, double dEl)
{
    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%3.2f RE\r\n", dAz);
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_dCurrentAzPosition = fmod(dAz , 360.0f);
    return nErr;
}

int CACEDome::parkDome()
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCloseOnPark) {
        m_bClosedDone = false;
        nErr = closeShutter();
    }
    // there is no park position so we set park = home
    nErr = goHome();

    return nErr;
}

int CACEDome::unparkDome()
{
    m_bParked = false;
    // there is no park position so we set park = home and on unpark ask for the home position.
    getDomeAz(m_dCurrentAzPosition);
    if(m_bOpenOnUnpark)
        openShutter();
    return 0;
}

int CACEDome::gotoAzimuth(double dNewAz)
{

    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    dNewAz = fabs(dNewAz); // should solve the -0.00
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%3.2f MV\r\n", dNewAz);
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_dGotoAz = dNewAz;
    m_nGotoTries = 0;
    
    return nErr;
}

int CACEDome::openShutter()
{
    int nErr = ACE_OK;
    bool isRaining;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;
    // is it raining ?
    nErr = getRainState(isRaining);
    if(isRaining) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::openShutter] It's raining... not opening \n", timestamp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    nErr = domeCommand("OP\r\n", NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    m_nCurrentShutterAction = OPENING_D1;
    m_nShutterStateD1 = OPENING_D1;
    return nErr;
}

int CACEDome::closeShutter()
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    if(!m_bDropoutDisabled) {
        nErr = domeCommand("UP\r\n", NULL, SERIAL_BUFFER_SIZE);
        m_nCurrentShutterAction = CLOSING_D2;
        m_nShutterStateD2 = CLOSING_D2;
    }
    else {
        nErr = domeCommand("CL\r\n", NULL, SERIAL_BUFFER_SIZE);
        m_nCurrentShutterAction = CLOSING_D1;
        m_nShutterStateD1 = CLOSING_D1;
    }

    return nErr;
}

int CACEDome::getFirmwareVersion(std::string& sVersion)
{
    int nErr = ACE_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string>    svHelpOutput;
    std::string sFirmware;
    int nbTimeout = 0;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;
    
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::getFirmwareVersion] sVersion.size() : %d \n", timestamp, (int)sVersion.size());
    fflush(Logfile);
#endif

    if(m_sFirmwareVersion.size()) {
        sVersion = m_sFirmwareVersion;
        return nErr;
    }

    do {
        nErr = domeCommand("HELP\r\n", szResp, SERIAL_BUFFER_SIZE);
        if(nErr != ERR_RXTIMEOUT)
            break;
        nbTimeout++;
        m_pSleeper->sleep(250);
    } while (nbTimeout < 5);
    
    if(nErr && nErr != PROMPT_TIMEOUT ) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::getFirmwareVersion] nErr : %d \n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    nErr = parseFields(szResp, svHelpOutput, '\n');
    if(nErr) {
        return nErr;
    }
    
    sFirmware = findField(svHelpOutput, "Revision");

    if(sFirmware.size()) {
        sVersion = sFirmware;
    }
    return nErr;
}


int CACEDome::goHome()
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;
    m_bHomePassed = false;
    nErr = domeCommand("HM\r\n", NULL, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CACEDome::calibrate()
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating)
        return SB_OK;

    nErr = domeCommand("LR\r\n", NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_bCalibrating = true;
    
    return nErr;
}

int CACEDome::isGoToComplete(bool &bComplete)
{
    int nErr = 0;
    double dDomeAz = 0;
    bool bIsMoving;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bIsMoving = true;
    nErr = isDomeMoving(bIsMoving);
    if(nErr) {
        return nErr;
        }

    getDomeAz(dDomeAz);

    if(bIsMoving) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::isGoToComplete] Dome is moving, domeAz = %f, mGotoAz = %f\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
        fflush(Logfile);
#endif
        bComplete = false;
        return nErr;
    }

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isGoToComplete] Dome is NOT moving, domeAz = %f, mGotoAz = %f\n", timestamp, ceil(dDomeAz), ceil(m_dGotoAz));
    fflush(Logfile);
#endif

    if ((floor(m_dGotoAz) <= floor(dDomeAz)+m_dCoastAz) && (floor(m_dGotoAz) >= floor(dDomeAz)-m_dCoastAz)) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::isGoToComplete] GOTO completed.\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::isGoToComplete] GOTO ERROR, not moving but not at target\n", timestamp);
        fflush(Logfile);
#endif
        if(m_nGotoTries == 0) {
            bComplete = false;
            m_nGotoTries = 1;
            gotoAzimuth(m_dGotoAz);
        }
        else {
            m_nGotoTries = 0;
            bComplete = false;
            nErr = ERR_CMDFAILED;
        }
    }

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isGoToComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;
}

int CACEDome::isOpenComplete(bool &bComplete)
{
    int nErr = 0;
    bool isRaining;

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState();
    if(nErr == ERR_RXTIMEOUT | nErr == PROMPT_TIMEOUT) {
        // if we got a timeout we need to ignore it on this call.
        return ACE_OK;
    }
    else if (nErr)
        return ERR_CMDFAILED;

    bComplete = false;
    // is it raining ?
    nErr = getRainState(isRaining);

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] isRaining = %s\n", timestamp, isRaining?"Yes":"No");
    fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] m_nCurrentShutterAction = %d\n", timestamp, m_nCurrentShutterAction);
    fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] m_nShutterStateD1 = %d\n", timestamp, m_nShutterStateD1);
    fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] m_nShutterStateD2 = %d\n", timestamp, m_nShutterStateD2);
    fflush(Logfile);
#endif

    if(isRaining)
        return ERR_CMDFAILED;

    if(!m_bDropoutDisabled) {
        // which shutter are we opening ?
        if(m_nCurrentShutterAction == OPENING_D1 && m_nShutterStateD1 == OPEN) {
    #if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] D1 is open\n", timestamp);
            fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] m_bDropoutDisabled = %s\n", timestamp, m_bDropoutDisabled?"TRUE":"FALSE");
            fflush(Logfile);
    #endif

            //now open bottom shutter
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] Opening D2\n", timestamp);
            fflush(Logfile);
#endif
            nErr = domeCommand("DN\r\n", NULL, SERIAL_BUFFER_SIZE);
            if(nErr) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] ERROR Opening D2\n", timestamp);
                fflush(Logfile);
#endif
                return nErr;
            }
            m_nCurrentShutterAction = OPENING_D2;
        }
        else if(m_nCurrentShutterAction == OPENING_D2 && m_nShutterStateD2 == OPEN) {
            m_nCurrentShutterAction = OPEN;
            m_bShutterOpened = true;
            bComplete = true;
            m_dCurrentElPosition = 90.0;
        }
        else if (m_nShutterStateD1 == OPEN && m_nShutterStateD2 == OPEN) {
            m_nCurrentShutterAction = OPEN;
            m_bShutterOpened = true;
            bComplete = true;
            m_dCurrentElPosition = 90.0;
        }
        else {
            m_bShutterOpened = false;
            bComplete = false;
            m_dCurrentElPosition = 0.0;
        }
    }
    else {
        if(m_nShutterStateD1 == OPEN) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] D1 is OPEN\n", timestamp);
            fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] All shutter are opened\n", timestamp);
            fflush(Logfile);
#endif
            m_nCurrentShutterAction = OPEN;
            m_bShutterOpened = true;
            bComplete = true;
            m_dCurrentElPosition = 90.0;
        }
        else {
            m_bShutterOpened = false;
            bComplete = false;
            m_dCurrentElPosition = 0.0;
        }
    }

    
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isOpenComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;
}

int CACEDome::isCloseComplete(bool &bComplete)
{
    int nErr=0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState();
    if(nErr == ERR_RXTIMEOUT || nErr == PROMPT_TIMEOUT) {
        // if we got a timeout we need to ignore it on this call.
        return ACE_OK;
    }
    else if (nErr)
        return ERR_CMDFAILED;

    bComplete = false;

    if(!m_bDropoutDisabled) {
    #if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] m_nCurrentShutterAction = %d\n", timestamp, m_nCurrentShutterAction);
        fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] m_nShutterStateD2 = %d\n", timestamp, m_nShutterStateD1);
        fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] m_nShutterStateD2 = %d\n", timestamp, m_nShutterStateD2);
        fflush(Logfile);
    #endif

        // which shutter are we closing ?
        if(m_nCurrentShutterAction == CLOSING_D2 && m_nShutterStateD2 == CLOSED) {
    #if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] D2 is CLOSED\n", timestamp);
            fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] CLosing D1\n", timestamp);
            fflush(Logfile);
    #endif
            //now close main shutter
            nErr = domeCommand("CL\r\n", NULL, SERIAL_BUFFER_SIZE);
            if(nErr) {
    #if defined ACE_DEBUG && ACE_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] ERROR CLosing D1 : %d\n", timestamp, nErr);
                fflush(Logfile);
    #endif
                return nErr;
            }
            m_nCurrentShutterAction = CLOSING_D1;
        }
        else if(m_nCurrentShutterAction == CLOSING_D1 && m_nShutterStateD1 == CLOSED) {
    #if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] D1 is CLOSED\n", timestamp);
            fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] All shutter are closed\n", timestamp);
            fflush(Logfile);
    #endif
            m_nCurrentShutterAction = CLOSED;

            m_bShutterOpened = false;
            bComplete = true;
            m_dCurrentElPosition = 0.0;
        }
        else if (m_nShutterStateD1 == CLOSED && m_nShutterStateD2 == CLOSED) {
            m_bShutterOpened = false;
            bComplete = true;
            m_dCurrentElPosition = 0.0;
            m_nCurrentShutterAction = CLOSED;
        }
        else {
            m_bShutterOpened = true;
            bComplete = false;
            m_dCurrentElPosition = 90.0;
        }
    }
    else {
        if(m_nShutterStateD1 == CLOSED) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] D1 is CLOSED\n", timestamp);
            fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] All shutter are closed\n", timestamp);
            fflush(Logfile);
#endif
            m_nCurrentShutterAction = CLOSED;
            
            m_bShutterOpened = false;
            bComplete = true;
            m_dCurrentElPosition = 0.0;
        }
        else {
            m_bShutterOpened = true;
            bComplete = false;
            m_dCurrentElPosition = 90.0;
        }
    }
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isCloseComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;
}


int CACEDome::isParkComplete(bool &bComplete)
{
    int nErr = ACE_OK;
    bool bCloseComplete;
    bool bHomed;

    bHomed = false;
    bCloseComplete = true;
    if(m_bCloseOnPark) {
        if(!m_bClosedDone) {
            nErr |= isCloseComplete(bCloseComplete);
            if(bCloseComplete) {
                m_bClosedDone = true;
                bCloseComplete = true;
            }
            else
                bCloseComplete = false;
        }
        else
            bCloseComplete = true;
    }
    nErr |= isFindHomeComplete(bHomed);

    bComplete = bHomed & bCloseComplete;

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isParkComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;;
}

int CACEDome::isUnparkComplete(bool &bComplete)
{
    int nErr = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    if(m_bOpenOnUnpark) {
       nErr = isOpenComplete(bComplete);
    }
    else if(!m_bParked)
        bComplete = true;

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isUnparkComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;
}

int CACEDome::isFindHomeComplete(bool &bComplete)
{
    int nErr = 0;
    bool bIsMoving;
    bool bIsAtHome = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bIsMoving = true;
    nErr = isDomeMoving(bIsMoving);
    if(nErr)
        return nErr;

    if(bIsMoving) {
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    // it's no longer monving so we're probably at home
    bIsAtHome = true;
    nErr = isDomeAtHome(bIsAtHome);
    if(nErr)
        return nErr;

    if(bIsAtHome){
        m_bHomed = true;
        bComplete = true;
    }
    else {
        // did we just pass home
        if ((ceil(m_dCurrentAzPosition) <= ceil(m_dHomeAz)+m_dCoastAz) && (ceil(m_dCurrentAzPosition) >= ceil(m_dHomeAz)-m_dCoastAz)) {
            m_nHomingTries = 0;
            gotoAzimuth(m_dHomeAz); // back out a bit
            bComplete = true;
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isFindHomeComplete] Close to home, backing out to %3.2f !!!\n", timestamp, m_dHomeAz);
            fflush(Logfile);
#endif
        }
        else {
            // we're not moving and we're not at the home position !!!
#if defined ACE_DEBUG && ACE_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CACEDome::isFindHomeComplete] Not moving and not at home !!!\n", timestamp);
            fflush(Logfile);
#endif
            if(m_nHomingTries == 0) {
                bComplete = false;
                m_nHomingTries = 1;
                gotoAzimuth(m_dHomeAz);
            }
            else {
                bComplete = false;
                m_bHomed = false;
                m_bParked = false;
                nErr = ERR_CMDFAILED;
            }
        }
    }

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isFindHomeComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fflush(Logfile);
#endif

    return nErr;
}


int CACEDome::isCalibratingComplete(bool &bComplete)
{
    int nErr = 0;
    bool bIsMoving;
    bComplete = false;
    int timeout = 0;
    bool bIsAtHome = true;

    int nTmp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bIsMoving = true;
    nErr = isDomeMoving(bIsMoving);
    if(nErr) {
        return nErr;
    }

    if(bIsMoving)
        bComplete = false;
    else    // no longer moving so we're done calibrating
        bComplete = true;

    if(bComplete) {
        m_bCalibrating = false;
        nErr = getDomeStepPerRev(nTmp);
        goHome(); // this should be a very small move as calibration end at home
        while(true) {
            isDomeAtHome(bIsAtHome);
            if(bIsAtHome)
                break;
            else {
                m_pSleeper->sleep(1000);
                timeout++;
                if(timeout>10) // 10 seconds should be enough
                    break;
            }

        }
    }
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::isCalibratingComplete] bComplete = %s\n", timestamp, bComplete?"TRUE":"FALSE");
    fprintf(Logfile, "[%s] [CACEDome::isCalibratingComplete] m_nNbStepPerRev = %d\n", timestamp, m_nNbStepPerRev);
    fflush(Logfile);
#endif

    return nErr;
}


int CACEDome::abortCurrentCommand()
{
    int nErr = ACE_OK;
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bCalibrating = false;
    if(m_nCurrentShutterAction != OPEN || m_nCurrentShutterAction != CLOSED)
        m_nCurrentShutterAction = SHUTTER_ERROR;

    nErr = domeCommand("ST\r\n", NULL, SERIAL_BUFFER_SIZE);

    return (nErr);
}

#pragma mark - Getter / Setter

int CACEDome::getWatchdogResetTimer()
{
    int nErr = ACE_OK;
    std::string sWatchdogTimer;
    std::vector<std::string> svWatchdogTimer;
    int nWatchdogTimer;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getExtendedStatus();
    if(nErr)
        return nErr;

    // need to parse to find the Encoder Counts per 360 field.
    sWatchdogTimer = findField(m_svExtStatus, "Watchdog Reset Time");
    if(!sWatchdogTimer.empty()) {
        parseFields(sWatchdogTimer.c_str(), svWatchdogTimer, ':');
        if(svWatchdogTimer.size()>1) {
            nWatchdogTimer=atoi(trim(svWatchdogTimer[1]," ").c_str());
        }
        else
            nWatchdogTimer = 0;
        m_nWatchdogTimer = nWatchdogTimer;
    }
    return nErr;

}

int CACEDome::setWatchdogResetTimer(int nSeconds)
{
    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%d WT\r\n", nSeconds);
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);

    return nErr;
}


int CACEDome::setHomeAz(double dAz)
{
    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%d HZ\r\n", int(dAz));
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_dHomeAz = dAz;
    return nErr;
}


int CACEDome::getAutoShutdown(bool &bEnabled)
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;
    int nEnabled;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bEnabled = false;
    nErr = getExtendedStatus();
    if(nErr)
        return nErr;
    // convert Az string to double
    sLine = findField(m_svExtStatus, "Rain-Snow enabled");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ':');
        if(svFields.size()>1) {
            nEnabled = atoi(svFields[1].c_str());
            bEnabled = nEnabled==1?true:false;
        }

    }
    return nErr;

}

int CACEDome::setAutoShutdown(bool bEnabled)
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(bEnabled) {
        nErr = domeCommand("ON\r\n", NULL, SERIAL_BUFFER_SIZE);
    }
    else{
        nErr = domeCommand("OF\r\n", NULL, SERIAL_BUFFER_SIZE);
    }
    if(nErr)
        return nErr;

    return nErr;

}


int CACEDome::getRainShutdown(bool &bEnabled)
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;
    int nEnabled;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bEnabled = false;
    nErr = getExtendedStatus();
    if(nErr)
        return nErr;
    // convert Az string to double
    sLine = findField(m_svExtStatus, "Rain-Snow enabled");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ':');
        if(svFields.size()>1) {
            nEnabled = atoi(svFields[1].c_str());
            bEnabled = nEnabled==1?true:false;
        }

    }
    return nErr;

}

int CACEDome::setRainShutdown(bool bEnabled)
{
    int nErr = ACE_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(bEnabled) {
        nErr = domeCommand("RN\r\n", NULL, SERIAL_BUFFER_SIZE);
    }
    else {
        nErr = domeCommand("RF\r\n", NULL, SERIAL_BUFFER_SIZE);
    }
    if(nErr)
        return nErr;

    return nErr;
}

int CACEDome::setNbRainSensors(int nNbSensors)

{
    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%d RS\r\n", nNbSensors);
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
    if(!nErr)
        m_nNbRainSensors = nNbSensors;

    return nErr;

}

int CACEDome::getNbRainSensors(int &nNbSensors)
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getExtendedStatus();
    if(nErr)
        return nErr;

    // need to parse to find the Encoder Counts per 360 field.
    sLine = findField(m_svExtStatus, "Rain sensors");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ':');
        if(svFields.size()>1) {
            nNbSensors=atoi(trim(svFields[1]," ").c_str());
        }
        else
            nNbSensors = 0;
        m_nNbRainSensors = nNbSensors;
    }
    return nErr;

}

int CACEDome::getRainState(bool &isRaining)
{
    int nErr;
    std::string sStatus;
    std::vector<std::string> svStatusLineFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    isRaining = false;
    nErr = getShortStatus(); // this timesout from time to time.
    if(nErr)
        return ACE_OK; // let's ignore the error and not change the data, they'll get pick up on the next request.

    if(m_svShortStatus.size()<3)
        return ACE_OK; // we got a weird response

#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::getRainState] m_svShortStatus.size() =  %lu\n", timestamp, m_svShortStatus.size());
    fflush(Logfile);
#endif

    // look for [ON] RAIN as we don't want to report that it's raining if rain shutdown is not ON.
    sStatus = findField(m_svShortStatus, "RAIN");
    if(sStatus.size()) {
#if defined ACE_DEBUG && ACE_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CACEDome::getRainState] sStatus =  %s\n", timestamp, sStatus.c_str());
        fflush(Logfile);
#endif

        if(sStatus.find("ON")!= -1) {
            isRaining = true;
        }
    }
    return nErr;
}


void CACEDome::getDropoutDisabled(bool &bDisabled)
{
    bDisabled = m_bDropoutDisabled;

}

void CACEDome::setDropoutDisabled(bool bDisabled)
{
#if defined ACE_DEBUG && ACE_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CACEDome::setDropoutDisabled] bDisabled = %s\n", timestamp, bDisabled?"TRUE":"FALSE");
    fflush(Logfile);
#endif
    m_bDropoutDisabled = bDisabled;
}

void CACEDome::setDecimalFormat(int nNbDecimals)
{
    char szBuf[SERIAL_BUFFER_SIZE];

    if(nNbDecimals>2)
        nNbDecimals = 2;

    if(!m_bIsConnected)
        return;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%d DP\r\n", nNbDecimals);
    domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
}

int CACEDome::getDomeAzCoast(double &dAz)
{
    int nErr = ACE_OK;
    std::string sLine;
    std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getExtendedStatus();
    if(nErr)
        return nErr;
    // convert Az string to double
    sLine = findField(m_svExtStatus, "Coast");
    if(!sLine.empty()) {
        parseFields(sLine.c_str(), svFields, ':');
        if(svFields.size()>1) {
            dAz = atof(svFields[1].c_str());
        }
    }
    return nErr;
}

int CACEDome::setDomeAzCoast(double dAz)
{
    int nErr = ACE_OK;
    char szBuf[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "%d CS\r\n", int(dAz));
    nErr = domeCommand(szBuf, NULL, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_dCoastAz = dAz;
    return nErr;

}

void CACEDome::setCloseOnPark(bool bEnabled)
{
    m_bCloseOnPark  = bEnabled;
}

void CACEDome::getCloseOnPark(bool &bEnabled)
{
    bEnabled = m_bCloseOnPark;
}

void CACEDome::setOpenOnUnpark(bool bEnabled)
{
    m_bOpenOnUnpark = bEnabled;
}

void CACEDome::getOpenOnUnpark(bool &bEnabled)
{
    bEnabled = m_bOpenOnUnpark;
}


int CACEDome::getShortStatus()
{
    int nErr = ACE_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nbTimeout = 0;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    do {
        nErr = domeCommand("?\r\n", szResp, SERIAL_BUFFER_SIZE);
        if(nErr != ERR_RXTIMEOUT)
            break;
        nbTimeout++;
        m_pSleeper->sleep(250);
    } while (nbTimeout < 5);
        
    if(nErr && nErr != PROMPT_TIMEOUT )
        return nErr;

    if(nErr)
        return nErr;
    // szResp contains the state fields.
    nErr = parseFields(szResp, m_svShortStatus, '\r');
    return nErr;
}

int CACEDome::getExtendedStatus()
{
    int nErr = ACE_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nbTimeout = 0;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    do {
        nErr = domeCommand("+\r\n", szResp, SERIAL_BUFFER_SIZE);
        if(nErr != ERR_RXTIMEOUT)
            break;
        nbTimeout++;
        m_pSleeper->sleep(250);
    } while (nbTimeout < 5);

    if(nErr && nErr != PROMPT_TIMEOUT )
        return nErr;
    // szResp contains the state fields.
    nErr = parseFields(szResp, m_svExtStatus, '\r');
    return nErr;
}


int CACEDome::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = ACE_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator)) {
        svFields.push_back(trim(sSegment," \r\n"));
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}

std::string& CACEDome::trim(std::string &str, const std::string& filter )
{
    return ltrim(rtrim(str, filter), filter);
}

std::string& CACEDome::ltrim(std::string& str, const std::string& filter)
{
    str.erase(0, str.find_first_not_of(filter));
    return str;
}

std::string& CACEDome::rtrim(std::string& str, const std::string& filter)
{
    str.erase(str.find_last_not_of(filter) + 1);
    return str;
}

std::string CACEDome::findField(std::vector<std::string> &svFields, const std::string& token)
{
    for(int i=0; i<svFields.size(); i++){
        if(svFields[i].find(token)!= -1) {
            return svFields[i];
        }
    }
    return std::string();
}


