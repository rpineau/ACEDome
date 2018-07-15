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

#define ACE_DEBUG 2

#define SERIAL_BUFFER_SIZE 2048
#define MAX_TIMEOUT 250
#define LOG_BUFFER_SIZE 256

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
    void    setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };

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
    int getNbTicksPerRev();
    int setNbTicksPerRev(int nTicks);
    int getWatchdogResetTimer();
    int setWatchdogResetTimer(int nSeconds);
    
    double getHomeAz();
    int setHomeAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getAutoShutdown(bool &bEnabled);
    int setAutoShutdown(bool bEnabled);

    int getRainShutdown(bool &bEnabled);
    int setRainShutdown(bool bEnabled);

    void getDropoutDisabled(bool &bDisabled);
    void setDropoutDisabled(bool bDisabled);

    void setDecimalFormat(int nNbDecimals);


    void setDebugLog(bool enable);

protected:
    
    int             readResponse(char *pszRespBuffer, int bufferLen);
    int             getDomeAz(double &dDomeAz);
    int             getDomeEl(double &dDomeEl);
    int             getDomeHomeAz(double &dAz);
    int             getDomeParkAz(double &dAz);
    int             getShutterState();
    int             getDomeStepPerRev(int &nStepPerRev);
    int             setDomeStepPerRev(int nStepPerRev);

    int             isDomeMoving(bool &bIsMoving);
    int             isDomeAtHome(bool &bAtHome);

    int             domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             getExtendedStatus();
    int             getShortStatus();
    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);
    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);
    std::string     findField(std::vector<std::string> &svFields, const std::string& token);

    LoggerInterface *m_pLogger;
    bool            m_bDebugLog;
    
    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bParked;
    bool            m_bCalibrating;
    bool            m_bDropoutDisabled;

    int             m_nNbStepPerRev;
    int             m_nWatchdogTimer;
    double          m_dHomeAz;
    

    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;

    double          m_dGotoAz;
    
    SerXInterface   *m_pSerx;
    
    std::string     m_sFirmwareVersion;
    int             m_nCurrentShutterAction;
    int             m_nShutterStateD1, m_nShutterStateD2;
    bool            m_bShutterOpened;

    int             m_nMotorState;

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
