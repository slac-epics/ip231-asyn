// drvIP231.h
// 
// Asyn driver that inherits from the asynPortDriver class.  This is an
// adaptation of testAsynPortDriver.cpp written by Mark Rivers.
// Started on 2/24/2011, zms
//-----------------------------------------------------------------------------
#ifndef _drvIP231_h
#define _drvIP231_h

#include "asynPortDriver.h"

#ifndef SIZE
#define SIZE(x)         (sizeof(x)/sizeof(x[0]))
#endif
#ifndef MIN
#define MIN(a,b)        (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b)        (((a)>(b))?(a):(b))
#endif
#define MAX_ADDR	16
#define BUF_SZ		2048
#define NAME_LEN	23
#define ACROID		0xa3
#define IP231_16	0x34
#define IP231_8		0x33
#define CRRESET		0x80

typedef unsigned char	byte;
typedef unsigned short	word;
typedef unsigned int	uint;
typedef struct{
  volatile word	data[MAX_ADDR];
  volatile word	transp,		// w/o transparent mode
		simult,		// w/o simultaneous mode
		trigger,	// w/o simultaneous mode output trigger
		wstat,		// r/o write status register
		cntrl;		// r/w control register, bit 7 (1) resets
  volatile byte	eepromS,	// r/o EEPROM status
		eepromC;	// r/w EEPROM control, bit0 ->1 to enable write
  volatile word	fill[10];
  volatile byte	calData[64];	// r/o gain and offset, D8 access.
} ip231_t;

#define aoDataStr	"AO_DATA"	// asynInt32,		w/o
#define mbbiModeStr	"MBBI_MODE"	// asynInt32,		w/o
#define boReadStr	"BO_READ"	// asynInt32,		w/o
#define boTrigStr	"BO_TRIG"	// asynInt32,		w/o
#define boRsetStr	"BO_RSET"	// asynInt32,		w/o
#define mbboModeStr	"MBBO_MODE"	// asynInt32,		w/o
#define aiDataStr	"AI_DATA"	// asynInt32,		r/o

typedef enum{dacModeNull,dacModeTransp,dacModeSimult} _dacmode_t;

class drvIP231 : public asynPortDriver {
public:
  drvIP231( const char* port,int carr,int slot,char* mode);
  virtual void report(FILE* fp,int level);
  virtual asynStatus getBounds( asynUser* pau,epicsInt32* lo,epicsInt32* hi);
  virtual asynStatus writeInt32( asynUser* pau,epicsInt32 v);
protected:

  int	_aoData, _mbbiMode,  _boRead, _boTrig, _boRset, _mbboMode, _aiData;

  #define FRST_COMMAND _aoData
  #define LAST_COMMAND _aiData

  enum{ ixAoData,ixMbbiMode,ixBoRead,ixBoTrig,ixBoRset,ixMbboMode,ixAiData};
 
private:
  asynStatus	_configure( int carr,int slot,char* mode);
  void		_writeChan( int addr,int v);
  asynStatus	_readChan( int addr,int* v);
  void		_readChanls();
  void		_trigger();
  void		_reset();
  void		_setMode( _dacmode_t m);
  const char*	_port;
  const char*	_name;
  word		_carrier;
  word		_slot;
  word		_model;
  word		_nchan;
  _dacmode_t	_mode;
  ip231_t*	_pmem;
  float		_offst[MAX_ADDR],
		_slope[MAX_ADDR];
  epicsInt32	_maxval;
};
#define N_PARAMS (&LAST_COMMAND - &FRST_COMMAND + 1)

#endif // _drvIP231_h
