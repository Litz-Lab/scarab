#ifndef _TAGE64K_H_
#define _TAGE64K_H_

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <math.h>

#include "cbp_to_scarab.h"
//#include "bt9.h"
//#include "bt9_reader.h"



#define BORNTICK  1024
//To get the predictor storage budget on stderr  uncomment the next line
#define PRINTSIZE
#include <vector>



#define SC			// 8.2 % if TAGE alone
#define IMLI			// 0.2 %
#define LOCALH

#ifdef LOCALH			// 2.7 %
#define LOOPPREDICTOR		//loop predictor enable
#define LOCALS			//enable the 2nd local history
#define LOCALT			//enables the 3rd local history

#endif



#define CONFWIDTH 7		//for the counters in the choser
#define HISTBUFFERLENGTH 4096	// we use a 4K entries history buffer to store the branch history (this allows us to explore using history length up to 4K)





// utility class for index computation
// this is the cyclic shift register for folding 
// a long global history into a smaller number of bits; see P. Michaud's PPM-like predictor at CBP-1
class cbp64_folded_history
{
public:


  unsigned comp;
  int CLENGTH;
  int OLENGTH;
  int OUTPOINT;

    cbp64_folded_history ()
  {
  }


  void init (int original_length, int compressed_length)
  {
    comp = 0;
    OLENGTH = original_length;
    CLENGTH = compressed_length;
    OUTPOINT = OLENGTH % CLENGTH;

  }

  void update (uint8_t * h, int PT)
  {
    comp = (comp << 1) ^ h[PT & (HISTBUFFERLENGTH - 1)];
    comp ^= h[(PT + OLENGTH) & (HISTBUFFERLENGTH - 1)] << OUTPOINT;
    comp ^= (comp >> CLENGTH);
    comp = (comp) & ((1 << CLENGTH) - 1);
  }

};




class cbp64_bentry			// TAGE bimodal table entry  
{
public:
  int8_t hyst;
  int8_t pred;


    cbp64_bentry ()
  {
    pred = 0;

    hyst = 1;
  }

};
class cbp64_gentry			// TAGE global table entry
{
public:
  int8_t ctr;
  uint tag;
  int8_t u;

    cbp64_gentry ()
  {
    ctr = 0;
    u = 0;
    tag = 0;


  }
};





//#ifdef LOOPPREDICTOR
//parameters of the loop predictor
#define LOGL 5
#define WIDTHNBITERLOOP 10	// we predict only loops with less than 1K iterations
#define LOOPTAG 10		//tag width in the loop predictor


//#endif

/*
int
predictorsize ()
{
  int STORAGESIZE = 0;
  int inter = 0;


  STORAGESIZE +=
    NBANKHIGH * (1 << (logg[BORN])) * (CWIDTH + UWIDTH + TB[BORN]);
  STORAGESIZE += NBANKLOW * (1 << (logg[1])) * (CWIDTH + UWIDTH + TB[1]);

  STORAGESIZE += (SIZEUSEALT) * ALTWIDTH;
  STORAGESIZE += (1 << LOGB) + (1 << (LOGB - HYSTSHIFT));
  STORAGESIZE += m[NHIST];
  STORAGESIZE += PHISTWIDTH;
  STORAGESIZE += 10;		//the TICK counter

  fprintf (stderr, " (TAGE %d) ", STORAGESIZE);
#ifdef SC
#ifdef LOOPPREDICTOR

  inter = (1 << LOGL) * (2 * WIDTHNBITERLOOP + LOOPTAG + 4 + 4 + 1);
  fprintf (stderr, " (LOOP %d) ", inter);
  STORAGESIZE += inter;

#endif

  inter += WIDTHRES;
  inter = WIDTHRESP * ((1 << LOGSIZEUP));	//the update threshold counters
  inter += 3 * EWIDTH * (1 << LOGSIZEUPS);	// the extra weight of the partial sums
  inter += (PERCWIDTH) * 3 * (1 << (LOGBIAS));

  inter +=
    (GNB - 2) * (1 << (LOGGNB)) * (PERCWIDTH) +
    (1 << (LOGGNB - 1)) * (2 * PERCWIDTH);
  inter += Gm[0];		//global histories for SC
  inter += (PNB - 2) * (1 << (LOGPNB)) * (PERCWIDTH) +
    (1 << (LOGPNB - 1)) * (2 * PERCWIDTH);
//we use phist already counted for these tables

#ifdef LOCALH
  inter +=
    (LNB - 2) * (1 << (LOGLNB)) * (PERCWIDTH) +
    (1 << (LOGLNB - 1)) * (2 * PERCWIDTH);
  inter += NLOCAL * Lm[0];
  inter += EWIDTH * (1 << LOGSIZEUPS);
#ifdef LOCALS
  inter +=
    (SNB - 2) * (1 << (LOGSNB)) * (PERCWIDTH) +
    (1 << (LOGSNB - 1)) * (2 * PERCWIDTH);
  inter += NSECLOCAL * (Sm[0]);
  inter += EWIDTH * (1 << LOGSIZEUPS);

#endif
#ifdef LOCALT
  inter +=
    (TNB - 2) * (1 << (LOGTNB)) * (PERCWIDTH) +
    (1 << (LOGTNB - 1)) * (2 * PERCWIDTH);
  inter += NTLOCAL * Tm[0];
  inter += EWIDTH * (1 << LOGSIZEUPS);
#endif









#endif



#ifdef IMLI

  inter += (1 << (LOGINB - 1)) * PERCWIDTH;
  inter += Im[0];

  inter += IMNB * (1 << (LOGIMNB - 1)) * PERCWIDTH;
  inter += 2 * EWIDTH * (1 << LOGSIZEUPS);	// the extra weight of the partial sums
  inter += 256 * IMm[0];
#endif
  inter += 2 * CONFWIDTH;	//the 2 counters in the choser
  STORAGESIZE += inter;


  fprintf (stderr, " (SC %d) ", inter);
#endif
#ifdef PRINTSIZE
  fprintf (stderr, " (TOTAL %d bits %d Kbits) ", STORAGESIZE,
	   STORAGESIZE / 1024);
  fprintf (stdout, " (TOTAL %d bits %d Kbits) ", STORAGESIZE,
	   STORAGESIZE / 1024);
#endif


  return (STORAGESIZE);


}
*/



class cbp64_lentry			//loop predictor entry
{
public:
  uint16_t NbIter;		//10 bits
  uint8_t confid;		// 4bits
  uint16_t CurrentIter;		// 10 bits

  uint16_t TAG;			// 10 bits
  uint8_t age;			// 4 bits
  bool dir;			// 1 bit

  //39 bits per entry    
  cbp64_lentry ()
  {
    confid = 0;
    CurrentIter = 0;
    NbIter = 0;
    TAG = 0;
    age = 0;
    dir = false;



  }

};


class TAGE64K
{
private:
  

  //The statistical corrector components

#define PERCWIDTH 6		//Statistical corrector  counter width 5 -> 6 : 0.6 %
  //The three BIAS tables in the SC component
  //We play with the TAGE  confidence here, with the number of the hitting bank
#define LOGBIAS 8
  int8_t Bias[(1 << LOGBIAS)];
#define INDBIAS (((((PC ^(PC >>2))<<1)  ^  (LowConf &(LongestMatchPred!=alttaken))) <<1) +  pred_inter) & ((1<<LOGBIAS) -1)
  int8_t BiasSK[(1 << LOGBIAS)];
#define INDBIASSK (((((PC^(PC>>(LOGBIAS-2)))<<1) ^ (HighConf))<<1) +  pred_inter) & ((1<<LOGBIAS) -1)

  int8_t BiasBank[(1 << LOGBIAS)];

#define INDBIASBANK (pred_inter + (((HitBank+1)/4)<<4) + (HighConf<<1) + (LowConf <<2) +((AltBank!=0)<<3)+ ((PC^(PC>>2))<<7)) & ((1<<LOGBIAS) -1)



  //In all th GEHL components, the two tables with the shortest history lengths have only half of the entries.

  // IMLI-SIC -> Micro 2015  paper: a big disappointment on  CBP2016 traces
#ifdef IMLI
#define LOGINB 8		// 128-entry
#define INB 1
  int Im[INB] = { 8 };
  int8_t IGEHLA[INB][(1 << LOGINB)] = { {0} };

  int8_t *IGEHL[INB];

#define LOGIMNB 9		// 2* 256 -entry
#define IMNB 2

  int IMm[IMNB] = { 10, 4 };
  int8_t IMGEHLA[IMNB][(1 << LOGIMNB)] = { {0} };

  int8_t *IMGEHL[IMNB];
  long long IMHIST[256];

#endif

  //global branch GEHL
#define LOGGNB 10		// 1 1K + 2 * 512-entry tables
#define GNB 3
  int Gm[GNB] = { 40, 24, 10 };
  int8_t GGEHLA[GNB][(1 << LOGGNB)] = { {0} };

  int8_t *GGEHL[GNB];

  //variation on global branch history
#define PNB 3
#define LOGPNB 9		// 1 1K + 2 * 512-entry tables
  int Pm[PNB] = { 25, 16, 9 };
  int8_t PGEHLA[PNB][(1 << LOGPNB)] = { {0} };

  int8_t *PGEHL[PNB];

  //first local history
#define LOGLNB  10		// 1 1K + 2 * 512-entry tables
#define LNB 3
  int Lm[LNB] = { 11, 6, 3 };
  int8_t LGEHLA[LNB][(1 << LOGLNB)] = { {0} };

  int8_t *LGEHL[LNB];
#define  LOGLOCAL 8
#define NLOCAL (1<<LOGLOCAL)
#define INDLOCAL ((PC ^ (PC >>2)) & (NLOCAL-1))
  long long L_shist[NLOCAL];	//local histories

  // second local history
#define LOGSNB 9		// 1 1K + 2 * 512-entry tables
#define SNB 3
  int Sm[SNB] = { 16, 11, 6 };
  int8_t SGEHLA[SNB][(1 << LOGSNB)] = { {0} };

  int8_t *SGEHL[SNB];
#define LOGSECLOCAL 4
#define NSECLOCAL (1<<LOGSECLOCAL)	//Number of second local histories
#define INDSLOCAL  (((PC ^ (PC >>5))) & (NSECLOCAL-1))
  long long S_slhist[NSECLOCAL];

  //third local history
#define LOGTNB 10		// 2 * 512-entry tables
#define TNB 2
  int Tm[TNB] = { 9, 4 };
  int8_t TGEHLA[TNB][(1 << LOGTNB)] = { {0} };

  int8_t *TGEHL[TNB];
#define NTLOCAL 16
#define INDTLOCAL  (((PC ^ (PC >>(LOGTNB)))) & (NTLOCAL-1))	// different hash for the history
  long long T_slhist[NTLOCAL];





  // playing with putting more weights (x2)  on some of the SC components
  // playing on using different update thresholds on SC
  //update threshold for the statistical corrector
#define VARTHRES
#define WIDTHRES 12
#define WIDTHRESP 8
#ifdef VARTHRES
#define LOGSIZEUP 6		//not worth increasing
#else
#define LOGSIZEUP 0
#endif
#define LOGSIZEUPS  (LOGSIZEUP/2)
  int updatethreshold;
  int Pupdatethreshold[(1 << LOGSIZEUP)];	//size is fixed by LOGSIZEUP
#define INDUPD (PC ^ (PC >>2)) & ((1 << LOGSIZEUP) - 1)
#define INDUPDS ((PC ^ (PC >>2)) & ((1 << (LOGSIZEUPS)) - 1))
  int8_t WG[(1 << LOGSIZEUPS)];
  int8_t WL[(1 << LOGSIZEUPS)];
  int8_t WS[(1 << LOGSIZEUPS)];
  int8_t WT[(1 << LOGSIZEUPS)];
  int8_t WP[(1 << LOGSIZEUPS)];
  int8_t WI[(1 << LOGSIZEUPS)];
  int8_t WIM[(1 << LOGSIZEUPS)];
  int8_t WB[(1 << LOGSIZEUPS)];
#define EWIDTH 6
  int LSUM;

  // The two counters used to choose between TAGE and SC on Low Conf SC
  int8_t FirstH, SecondH;
  bool MedConf;			// is the TAGE prediction medium confidence
#define  POWER
//use geometric history length

#define NHIST 36		// twice the number of different histories

#define NBANKLOW 10		// number of banks in the shared bank-interleaved for the low history lengths
#define NBANKHIGH 20		// number of banks in the shared bank-interleaved for the  history lengths

int SizeTable[NHIST + 1];


#define BORN 13			// below BORN in the table for low history lengths, >= BORN in the table for high history lengths,

// we use 2-way associativity for the medium history lengths
#define BORNINFASSOC 9		//2 -way assoc for those banks 0.4 %
#define BORNSUPASSOC 23

/*in practice 2 bits or 3 bits par branch: around 1200 cond. branchs*/

#define MINHIST 6		//not optimized so far
#define MAXHIST 3000


#define LOGG 10			/* logsize of the  banks in the  tagged TAGE tables */
#define TBITS 8			//minimum width of the tags  (low history lengths), +4 for high history lengths


bool NOSKIP[NHIST + 1];		// to manage the associativity for different history lengths
bool LowConf;
bool HighConf;



#define NNN 1			// number of extra entries allocated on a TAGE misprediction (1+NNN)
#define HYSTSHIFT 2		// bimodal hysteresis shared by 4 entries
#define LOGB 13			// log of number of entries in bimodal predictor


#define PHISTWIDTH 27		// width of the path history used in TAGE
#define UWIDTH 1		// u counter width on TAGE (2 bits not worth the effort for a 512 Kbits predictor 0.2 %)
#define CWIDTH 3		// predictor counter width on the TAGE tagged tables


//the counter(s) to chose between longest match and alternate prediction on TAGE when weak counters
#define LOGSIZEUSEALT 4
bool AltConf;			// Confidence on the alternate prediction
#define ALTWIDTH 5
#define SIZEUSEALT  (1<<(LOGSIZEUSEALT))
#define INDUSEALT (((((HitBank-1)/8)<<1)+AltConf) % (SIZEUSEALT-1))

  
public:
  int THRES;

  TAGE64K (void);
  uns8 IsFull(void);
  void reinit ();
  int bindex (UINT64 PC);
  int F (long long A, int size, int bank);
  int gindex (unsigned int PC, int bank, long long hist, cbp64_folded_history * ch_i);
  uint16_t gtag (unsigned int PC, int bank, cbp64_folded_history * ch0, cbp64_folded_history * ch1);
  void ctrupdate (int8_t & ctr, bool taken, int nbits);
  bool getbim ();
  void baseupdate (bool Taken);
  int MYRANDOM ();
  void Tagepred (UINT64 PC);
  bool GetPrediction (UINT64 PC, int* bp_confidence);
  void HistoryUpdate (UINT64 PC, OpType opType, bool taken, UINT64 target, long long &X, int &Y,
                      cbp64_folded_history * H, cbp64_folded_history * G,
                      cbp64_folded_history * J);

  void UpdatePredictor (UINT64 PC, OpType opType, bool resolveDir, bool predDir, UINT64 branchTarget);
  int Gpredict (UINT64 PC, long long BHIST, int *length, int8_t ** tab, int NBR, int logs, int8_t * W);
  void Gupdate (UINT64 PC, bool taken, long long BHIST, int *length, int8_t ** tab, int NBR, int logs, int8_t * W);
  void TrackOtherInst (UINT64 PC, OpType opType, bool taken, UINT64 branchTarget);
  int lindex (UINT64 PC);
  bool getloop (UINT64 PC);
  void loopupdate (UINT64 PC, bool Taken, bool ALLOC);

  //LP START
  cbp64_lentry *ltable;			//loop predictor table
  //variables for the loop predictor
  bool predloop;			// loop predictor prediction
  int LIB;
  int LI;
  int LHIT;			//hitting way in the loop predictor
  int LTAG;			//tag on the loop predictor
  bool LVALID;			// validity of the loop predictor prediction
  int8_t WITHLOOP;		// counter to monitor whether or not loop prediction is beneficial
  //LP end
  
  int8_t use_alt_on_na[SIZEUSEALT];
//very marginal benefit
long long GHIST;
int8_t BIM;

int TICK;			// for the reset of the u counter
uint8_t ghist[HISTBUFFERLENGTH];
int ptghist;
long long phist;		//path history
cbp64_folded_history ch_i[NHIST + 1];	//utility for computing TAGE indices
cbp64_folded_history ch_t[2][NHIST + 1];	//utility for computing TAGE tags

//For the TAGE predictor
cbp64_bentry *btable;			//bimodal TAGE table
cbp64_gentry *gtable[NHIST + 1];	// tagged TAGE tables
int m[NHIST + 1];
int TB[NHIST + 1];
int logg[NHIST + 1];

int GI[NHIST + 1];		// indexes to the different tables are computed only once  
uint GTAG[NHIST + 1];		// tags for the different tables are computed only once  
int BI;				// index of the bimodal table
bool pred_taken;		// prediction
bool alttaken;			// alternate  TAGEprediction
bool tage_pred;			// TAGE prediction
bool LongestMatchPred;
int HitBank;			// longest matching bank
int AltBank;			// alternate matching bank
int Seed;			// for the pseudo-random number generator
bool pred_inter;
long long IMLIcount;		// use to monitor the iteration number
  
};



#endif
