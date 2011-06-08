// drvIP231.cc
// Asyn driver to control and monitor vmic4132 VME board.
// Adapted from testAsynPortDriver.cpp written by Mark Rivers.
// Started on 1/30/2011, zms.
//------------------------------------------------------------------------------
#include <string.h>

#include <cantProceed.h>
#include <errlog.h>
#include <dbDefs.h>
#include <drvSup.h>
#include <devLib.h>
#include <epicsExport.h>
#include <iocsh.h>

#include "drvIpac.h"
#include "drvIP231.h"

static const char *dname="drvIP231";
static const char* __mode[]={"Null","transparent","simultaneous"};

asynStatus drvIP231::_configure( int carr,int slot,char* mode){
//------------------------------------------------------------------------------
// One time configuration.  Where:
//  carr is carrier number
//  slot is the daughter car position or slot {0..3},
//  mode is either "transparent" or "simultaneous".
//------------------------------------------------------------------------------
  ipac_idProm_t* id; char* base; int i,manuf,model;
  byte offh,offl,gainh,gainl;  word offst,gain; short offval,gaival;
  
  _pmem=0;
  if(ipmCheck( carr,slot)){
    errlogPrintf( "%s::_configure: bad carrier or slot\n",dname);
    return(asynError);
  }
  id=(ipac_idProm_t*)ipmBaseAddr( carr,slot,ipac_addrID);
  base=(char*)ipmBaseAddr( carr,slot,ipac_addrIO);
  manuf=id->manufacturerId&0xff;
  _model=model=id->modelId&0xff;
  if(manuf!=ACROID){
    errlogPrintf( "%s::_configure: bad manufacture, is %x should be %x\n",
		dname,manuf,ACROID);
    return(asynError);
  }
  switch(model){
    case IP231_8:	_nchan=8; break;
    case IP231_16:	_nchan=16; break;
    default:	errlogPrintf( "%s::_configure: bad model, is %x not {%x,%x}\n",
				dname,model,IP231_8,IP231_16);
		return(asynError);
  }
  _carrier=carr;
  _slot=slot;
  _pmem=(ip231_t*)base;
  for( i=0; i<_nchan; i++){
    offh=_pmem->calData[i*4+0];
    offl=_pmem->calData[i*4+1];
    gainh=_pmem->calData[i*4+2];
    gainl=_pmem->calData[i*4+3];
    offst=(offh<<8)+offl;
    gain=(gainh<<8)+gainl;
    offval=(short)offst;
    gaival=(short)gain;
    _slope[i]=1.0+(double)gaival/262144.0;
    _offst[i]=(double)offval/4.0-(double)gaival/8.0;
  }
  _reset();
  if(!strcmp(mode,__mode[dacModeTransp])) _mode=dacModeTransp;
  else if(!strcmp(mode,__mode[dacModeSimult])) _mode=dacModeSimult;
  else{
    errlogPrintf( "%s::_configure: mode %s illegal\n",dname,mode);
    return(asynError);
  }
  _setMode(_mode);
  printf( "%s::_configure: successfully configured\n",dname);
  return(asynSuccess);
}
void drvIP231::_writeChan( int addr,int v){
//------------------------------------------------------------------------------
  int tmp; word dac; uint mask;
  if(!_pmem) return;
  tmp=(int)(v*_slope[addr]+_offst[addr]);
  dac=MAX(0,MIN(tmp,_maxval));
  mask=(1<<addr);
  while(!(_pmem->wstat&mask));
  _pmem->data[addr]=dac;
}
void drvIP231::_reset(){
//------------------------------------------------------------------------------
// soft reset.  All analog output channels set to 0 V.
//------------------------------------------------------------------------------
  int i,v=0x8000;
  if(!_pmem) return;
  _pmem->cntrl=CRRESET;
  for( i=0; i<_nchan; i++){
    _writeChan( i,v);
    setIntegerParam( i,_aoData,v);
    callParamCallbacks(i);
  }
  _mode=dacModeNull;
  setIntegerParam( _mbbiMode,_mode);
  callParamCallbacks();
}
void drvIP231::_setMode( _mode_t m){
//------------------------------------------------------------------------------
  if(!_pmem) return;
  _mode=m;
  switch(_mode){
    case dacModeTransp:	_pmem->transp=0xffff; break;
    case dacModeSimult:	_pmem->simult=0xffff; break;
    default:		break;
  }
  setIntegerParam( _mbbiMode,_mode);
  callParamCallbacks();
}
asynStatus drvIP231::_readChan( int addr,int* v){
//------------------------------------------------------------------------------
  word dac; uint mask;
  if(!_pmem) return(asynError);
  mask=(1<<addr);
  while(!(_pmem->wstat&mask));
  dac=_pmem->data[addr];
  *v=(int)((dac-_offst[addr])/_slope[addr]);
  return(asynSuccess);
}
void drvIP231::_readChanls(){
//------------------------------------------------------------------------------
  int i,v;
  for( i=0; i<_nchan; i++){
    _readChan( i,&v);
    setIntegerParam( i,_aiData,v);
    callParamCallbacks(i);
  }
}
void drvIP231::_trigger(){
//------------------------------------------------------------------------------
// If in "simultaneous" trigger mode, trigger module.
//------------------------------------------------------------------------------
  if(!_pmem) return;
  if(_mode==dacModeSimult) _pmem->trigger=0xffff;
}
asynStatus drvIP231::getBounds( asynUser* pau,epicsInt32* lo,epicsInt32* hi){
//------------------------------------------------------------------------------
// Reimplementation of asynPortDriver virtual function which the bounds of
// raw values needed for slope calculation for linear conversion.
//------------------------------------------------------------------------------
  *lo=0; *hi=_maxval;
  return(asynSuccess);
}
asynStatus drvIP231::writeInt32( asynUser* pau,epicsInt32 v){
//------------------------------------------------------------------------------
// This method overrides the virtual method in asynPortDriver.  Here we service
// all write requests comming from EPICS records.
// Parameters:
//  pau	(in) structure containing addr and reason.
//  v   (in) this is the command index, which together with
//              pau->reason define the command to be sent.
//------------------------------------------------------------------------------
  asynStatus stat=asynSuccess; int ix,addr;

  stat=getAddress(pau,&addr); if(stat!=asynSuccess) return(stat);
  ix=pau->reason;
  if(addr<0||addr>=_nchan) return(asynError);
  switch( ix){
    case ixBoRead:	_readChanls(); break;
    case ixBoTrig:	_trigger(); break;
    case ixBoRset:	_reset(); break;
    case ixMbboMode:	_setMode( (_mode_t)v); break;
    case ixAoData:	_writeChan(addr,v); break;
    default:		break;
  }
  return(stat);
}
drvIP231::drvIP231(const char* port,int carr,int slot,char* mode):
  asynPortDriver( port,MAX_ADDR,N_PARAMS,
	asynInt32Mask|asynDrvUserMask,asynInt32Mask,ASYN_MULTIDEVICE,1,0,0){
//------------------------------------------------------------------------------
// Constructor for the drvIP231 class. Calls constructor for the asynPortDriver
// base class. Where
//   port  The name of the asyn port driver to be created.
//   carr	Acromag carrier number
//   slot	Acromag carrier slot occupied by this daughter card
//   mode	either "transparent" or "simultaneous"
// Parameters passed to the asynPortDriver constructor:
//  port name
//  max address
//  parameter table size
//  interface mask
//  interrupt mask,
//  asyn flags,
//  auto connect
//  priority
//  stack size
//------------------------------------------------------------------------------
  int nbts=strlen(port)+1;
  _port=(char*)callocMustSucceed( nbts,sizeof(char),dname);
  strcpy((char*)_port,port);

  createParam( aoDataStr,	asynParamInt32,	&_aoData);
  createParam( mbbiModeStr,	asynParamInt32,	&_mbbiMode);
  createParam( boReadStr,	asynParamInt32,	&_boRead);
  createParam( boTrigStr,	asynParamInt32,	&_boTrig);
  createParam( boRsetStr,	asynParamInt32,	&_boRset);
  createParam( mbboModeStr,	asynParamInt32,	&_mbboMode);
  createParam( aiDataStr,	asynParamInt32,	&_aiData);

  _configure( carr,slot,mode);
  _maxval=0xffff;
}
void drvIP231::report( FILE* fp,int level){
//------------------------------------------------------------------------------
// level description
//  0	 presence
//  1	 registers
//  2	 DAC channels
//  3	 asynPortDriver stuff
//  >3	 more asynPortDriver stuff
//------------------------------------------------------------------------------
  int j,dac;

  if(!_pmem){
    fprintf(fp,"%s::report: driver configuration failed\n",dname);
    return;
  }
  fprintf(fp," %s::report: carrier %d slot %d is present.\n",
		dname,_carrier,_slot);
  if(level>0){
    fprintf(fp,"    model=0x%x, mode=%s, numChannls=%d\n",
	_model,__mode[_mode],_nchan);
    if(level>1){
      for( j=0; j<MAX_ADDR; j++){
	_readChan( j,&dac);
	fprintf(fp,"    dac[%.2d]=0x%.4x\n",j,_pmem->data[j]);
      }
    }
  }
  if(level>2) asynPortDriver::report( fp,level-3);
}

// Configuration routines.  Called directly, or from the iocsh function below
extern "C" {

int drvIP231Configure( const char* port,int carr,int slot,char* mode){
/*------------------------------------------------------------------------------
 * EPICS iocsh callable function to call constructor for the drvIP231 class.
 *  portName The name of the asyn port driver to be created.
 *  maxPoints The maximum  number of points in the volt and time arrays
 *----------------------------------------------------------------------------*/
  new drvIP231(port,carr,slot,mode);
  return(asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0 = { "port",iocshArgString};
static const iocshArg initArg1 = { "carr",iocshArgInt};
static const iocshArg initArg2 = { "slot",iocshArgInt};
static const iocshArg initArg3 = { "mode",iocshArgString};
static const iocshArg * const initArgs[]=
	{&initArg0,&initArg1,&initArg2,&initArg3};
static const iocshFuncDef initFuncDef = {"drvIP231Configure",4,initArgs};
static void initCallFunc(const iocshArgBuf *args){
  drvIP231Configure(args[0].sval,args[1].ival,args[2].ival,args[3].sval);
}
void drvIP231Register(void){
  iocshRegister(&initFuncDef,initCallFunc);
}
epicsExportRegistrar(drvIP231Register);
}
