//
//  ACEdome.h
//
//  Created by Rodolphe Pineau on 6/11/2016.
//  ACE rotation drive unit for Pulsar Dome X2 plugin

#ifndef __ACE_DOME__
#define __ACE_DOME__
#include <math.h>
#include <string.h>
#include <time.h>

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"

// #define ACE_DEBUG 2

#define DRIVER_VERSION      1.5

#define SERIAL_BUFFER_SIZE 2048
#define MAX_TIMEOUT 500
#define MAX_READ_WAIT_TIMEOUT 25
#define NB_RX_WAIT 2

// error codes
// Error code
enum ACEDomeErrors {ACE_OK=0, NOT_CONNECTED, CANT_CONNECT, BAD_CMD_RESPONSE, COMMAND_FAILED};
enum ACEDomeShutterState {OPEN=0, CLOSED, OPENING_D1, OPENING_D2, CLOSING_D1, CLOSING_D2, SHUTTER_ERROR, UNKNOWN};

class CACEDome
{
public:
    CACEDome();
    ~CACEDome();

    int     Connect(const char *szPort);
    void    Disconnect(void);
    bool    IsConnected(void) { return m_bIsConnected; }

    void    SetSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void    setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; }
    void    setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper; }

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double newAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(std::string& version);
    int goHome();
    int calibrate();

    // command complete functions
    int isGoToComplete(bool &complete);
    int isOpenComplete(bool &complete);
    int isCloseComplete(bool &complete);
    int isParkComplete(bool &complete);
    int isUnparkComplete(bool &complete);
    int isFindHomeComplete(bool &complete);
    int isCalibratingComplete(bool &complete);

    int abortCurrentCommand();

    // getter/setter
    int getDomeStepPerRev(int &nStepPerRev);
    int setDomeStepPerRev(int nStepPerRev);

    int getWatchdogResetTimer();
    int setWatchdogResetTimer(int nSeconds);
    
    int getDomeAz(double &dDomeAz);
    int getDomeEl(double &dDomeEl);

    int setHomeAz(double dAz);
    int getDomeHomeAz(double &dAz);

    int getAutoShutdown(bool &bEnabled);
    int setAutoShutdown(bool bEnabled);

    int getRainShutdown(bool &bEnabled);
    int setRainShutdown(bool bEnabled);

    int getNbRainSensors(int &nNbSensors);
    int setNbRainSensors(int nNbSensors);

    int getRainState(bool &isRaining);

    void getDropoutDisabled(bool &bDisabled);
    void setDropoutDisabled(bool bDisabled);

    void setDecimalFormat(int nNbDecimals);

    int getDomeAzCoast(double &dAz);
    int setDomeAzCoast(double dAz);

    void setCloseOnPark(bool bEnabled);
    void getCloseOnPark(bool &bEnable);

    void setOpenOnUnpark(bool bEnabled);
    void getOpenOnUnpark(bool &bEnabled);

    void setDebugLog(bool enable);

protected:
    
    int             domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             readResponse(char *pszRespBuffer, int bufferLen, unsigned long &nbRead);

    int             getShutterState();
    int             getDomeParkAz(double &dAz);
    int             isDomeMoving(bool &bIsMoving);
    int             isDomeAtHome(bool &bAtHome);

    int             getExtendedStatus();
    int             getShortStatus();

    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);
    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);
    std::string     findField(std::vector<std::string> &svFields, const std::string& token);

    LoggerInterface *m_pLogger;
    SleeperInterface    *m_pSleeper;

    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bHomePassed;
    bool            m_bParked;
    bool            m_bClosedDone;
    bool            m_bCalibrating;
    bool            m_bDropoutDisabled;

    bool            m_bOpenOnUnpark;
    bool            m_bCloseOnPark;

    int             m_nNbStepPerRev;
    int             m_nWatchdogTimer;
    double          m_dHomeAz;
    double          m_dCoastAz;

    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;
    int             m_nGotoTries;
    int             m_nHomingTries;
    SerXInterface   *m_pSerx;
    
    std::string     m_sFirmwareVersion;
    int             m_nCurrentShutterAction;
    int             m_nShutterStateD1, m_nShutterStateD2;
    bool            m_bShutterOpened;

    int             m_nNbRainSensors;

    std::vector<std::string>    m_svShortStatus;
    std::vector<std::string>    m_svExtStatus;

#ifdef ACE_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;      // LogFile
#endif

};

#endif
