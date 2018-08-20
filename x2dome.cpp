#include <stdio.h>
#include <string.h>
#include "x2dome.h"
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serialportparams2interface.h"


X2Dome::X2Dome(const char* pszSelection, 
							 const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyXForMounts,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyXForMounts;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_bCalibratingDome = false;
    
    m_ACEDome.SetSerxPointer(pSerX);
    m_ACEDome.setLogger(pLogger);

    if (m_pIniUtil)
    {   
        m_ACEDome.setDropoutDisabled( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_DROPOUT, false) );
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}


int X2Dome::establishLink(void)					
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_ACEDome.Connect(szPort);
    if(nErr)
        m_bLinked = false;
    else
        m_bLinked = true;

    if (m_pIniUtil)
    {
        m_ACEDome.setAutoShutdown( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_AUTOSHUTDOWN, true) );
        m_ACEDome.setRainShutdown( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_RAINSHUTDOWN, true) );
        m_ACEDome.setDropoutDisabled( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_DROPOUT, false) );
        m_ACEDome.setCloseOnPark( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_CLOSE_ON_PARK, false) );
        m_ACEDome.setOpenOnUnpark( m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_OPEN_ON_UNPARK, false) );
    }
	return nErr;
}

int X2Dome::terminateLink(void)					
{
    X2MutexLocker ml(GetMutex());
    m_ACEDome.Disconnect();
	m_bLinked = false;
	return SB_OK;
}

 bool X2Dome::isLinked(void) const				
{
	return m_bLinked;
}


int X2Dome::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    
    return SB_OK;
}

#pragma mark - UI binding

int X2Dome::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    double dHomeAz;
    double dAzimuthCoast;
    int nTicks;
    int nNbRainSensors;
    int nWatchdogTimer;
    bool bEnabled;
    bool bEnabledAutoShut;
    bool bEnabledRainShut;
    bool bDisableDropout;
    bool bDisabled;
    bool bOpenOnUnpark;
    bool bCloseOnPark;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("ACEDome.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());
    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);

    // set controls state depending on the connection state
    if(m_bLinked) {
        // home position
        dx->setPropertyDouble("homePosition","value", m_ACEDome.getHomeAz());
        // ticks per rev
        dx->setPropertyInt("ticksPerRev","value", m_ACEDome.getNbTicksPerRev());
        dx->setEnabled("pushButton",true);
        //Az Coast
        m_ACEDome.getDomeAzCoast(dAzimuthCoast);
        dx->setPropertyDouble("azimuthCoast","value", dAzimuthCoast);
        // auto shutdown
        m_ACEDome.getAutoShutdown(bEnabled);
        dx->setChecked("autoShutdown",bEnabled);
        // auto rain
        m_ACEDome.getRainShutdown(bEnabled);
        dx->setChecked("rainShutdown",bEnabled);

        m_ACEDome.getNbRainSensors(nNbRainSensors);
        dx->setPropertyInt("nbRainSensors","value",nNbRainSensors);

        // watchdog
        dx->setPropertyInt("watchdogInterval","value", m_ACEDome.getWatchdogResetTimer());
        // dropout enable/disable
        m_ACEDome.getDropoutDisabled(bDisabled);
        dx->setChecked("disableDropout",bDisabled);

        // open/close on park/unpark
        m_ACEDome.getCloseOnPark(bCloseOnPark);
        dx->setChecked("closeOnPark",bCloseOnPark);
        m_ACEDome.getOpenOnUnpark(bOpenOnUnpark);
        dx->setChecked("openOnUnpark",bOpenOnUnpark);

    }
    else {
        snprintf(szTmpBuf,16,"NA");
        dx->setPropertyDouble("homePosition","value", 0);
        dx->setPropertyInt("ticksPerRev","value", 0);
        dx->setPropertyDouble("azimuthCoast","value", 0);
        dx->setPropertyInt("watchdogInterval","value", 0);
        dx->setEnabled("pushButton",false);
        dx->setEnabled("homePosition",false);
        dx->setEnabled("ticksPerRev",false);
        dx->setEnabled("azimuthCoast",false);
        dx->setEnabled("autoShutdown",false);
        dx->setEnabled("rainShutdown",false);
        dx->setEnabled("watchdogInterval",false);
        dx->setEnabled("disableDropout",false);
    }

    m_bCalibratingDome = false;

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK)
    {
        dx->propertyDouble("homePosition", "value", dHomeAz);
        dx->propertyInt("ticksPerRev", "value", nTicks);
        dx->propertyDouble("azimuthCoast", "value", dAzimuthCoast);
        dx->propertyInt("watchdogInterval", "value", nWatchdogTimer);
        bEnabledAutoShut = dx->isChecked("autoShutdown");
        bEnabledRainShut = dx->isChecked("rainShutdown");
        dx->propertyInt("nbRainSensors","value",nNbRainSensors);
        bDisableDropout = dx->isChecked("disableDropout");
        bCloseOnPark = dx->isChecked("closeOnPark");
        bOpenOnUnpark = dx->isChecked("openOnUnpark");

        if(m_bLinked)
        {
            m_ACEDome.setHomeAz(dHomeAz);
            // set ticks
            m_ACEDome.setNbTicksPerRev(nTicks);
            // set Azimuth Coasr
            m_ACEDome.setDomeAzCoast(dAzimuthCoast);
            // set watchdog
            m_ACEDome.setWatchdogResetTimer(nWatchdogTimer);
            // set autoshutdown
            m_ACEDome.setAutoShutdown(bEnabledAutoShut);
            // set rain shutdown
            m_ACEDome.setRainShutdown(bEnabledAutoShut);
            // rain sensors
            m_ACEDome.setNbRainSensors(nNbRainSensors);
            // enable/disabled dropout
            m_ACEDome.setDropoutDisabled(bDisableDropout);
            // open/close on park/unpark
            m_ACEDome.setOpenOnUnpark(bOpenOnUnpark);
            m_ACEDome.setCloseOnPark(bCloseOnPark);
        }
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_AUTOSHUTDOWN, bEnabledAutoShut);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_RAINSHUTDOWN, bEnabledRainShut);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_DROPOUT, bDisableDropout);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_CLOSE_ON_PARK, bCloseOnPark);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_OPEN_ON_UNPARK, bOpenOnUnpark);
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    bool bComplete = false;
    int nErr;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    char szErrorMessage[SERIAL_BUFFER_SIZE];

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked"))
        m_ACEDome.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        if(m_bLinked) {
            if(m_bCalibratingDome) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_ACEDome.isCalibratingComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled("pushButton",true);
                    uiex->setEnabled("pushButtonOK",true);
                    snprintf(szErrorMessage, SERIAL_BUFFER_SIZE, "Error calibrating dome : Error %d", nErr);
                    uiex->messageBox("ACEDome Calibrate", szErrorMessage);
                    m_bCalibratingDome = false;
                    return;;
                }
                
                if(!bComplete) {
                    return;
                }
                
                // enable "ok" and "calibrate"
                uiex->setEnabled("pushButton",true);
                uiex->setEnabled("pushButtonOK",true);
                // read step per rev from dome
                snprintf(szTmpBuf,16,"%d",m_ACEDome.getNbTicksPerRev());
                uiex->setPropertyString("ticksPerRev","text", szTmpBuf);
                m_bCalibratingDome = false;
                
            }
            
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_clicked"))
    {
        if(m_bLinked) {
            // disable "ok" and "calibrate"
            uiex->setEnabled("pushButton",false);
            uiex->setEnabled("pushButtonOK",false);
            m_ACEDome.calibrate();
            m_bCalibratingDome = true;
        }
    }

}

//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const					
{
	str = "ACE SmartDome Controller";
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const					
{
    str = "ACE SmartDome Controller";
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
    str = "ACE SmartDome Controller";
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)					
{
    X2MutexLocker ml(GetMutex());

    if(m_bLinked) {
        std::string cFirmware;
        m_ACEDome.getFirmwareVersion(cFirmware);
        str = cFirmware.c_str();
    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    str = "ACE SmartDome Controller";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const	
{
    str = "ACE SmartDome Controller X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//
//DomeDriverInterface
//
#pragma mark - DomeDriverInterface

int X2Dome::dapiGetAzEl(double* pdAz, double* pdEl)
{
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    *pdAz = m_ACEDome.getCurrentAz();
    *pdEl = m_ACEDome.getCurrentEl();
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
    int nErr;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    else
        return SB_OK;
}

int X2Dome::dapiAbort(void)
{

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    m_ACEDome.abortCurrentCommand();

    return SB_OK;
}

int X2Dome::dapiOpen(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.openShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.closeShutter();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    // nErr = m_ACEDome.closeShutter();
    // if(nErr)
    //    return ERR_CMDFAILED;

    nErr = m_ACEDome.parkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiUnpark(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.unparkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiFindHome(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.goHome();
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.isGoToComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;
    return SB_OK;
}

int X2Dome::dapiIsOpenComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;
    
    nErr = m_ACEDome.isOpenComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int	X2Dome::dapiIsCloseComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.isCloseComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.isUnparkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    int nErr;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_ACEDome.syncDome(dAz, dEl);
    if(nErr)
        return ERR_CMDFAILED;
	return SB_OK;
}

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Dome::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Dome::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort);
    
}


void X2Dome::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}




