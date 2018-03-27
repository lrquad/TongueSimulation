#pragma once
#include "HTK/HShell.h"
#include "HTK/HMem.h"
#include "HTK/HMath.h"
#include "HTK/HSigP.h"
#include "HTK/HWave.h"
#include "HTK/HVQ.h"
#include "HTK/HAudio.h"
#include "HTK/HParm.h"
#include "HTK/HLabel.h"
#include "HTK/HModel.h"

#include <vector>
#define STACKSIZE 100000        /* assume ~100K wave files */

class WavMFCC
{
public:
	WavMFCC();
	~WavMFCC();

	virtual void streamWavFrameToMFCC(double* wav,double* output);

	virtual void resetBuffer();

	int getTotalframescount() const { return totalframescount; }
	void setTotalframescount(int val) { totalframescount = val; }
	int getFrSize() const { return frSize; }
	void setFrSize(int val) { frSize = val; }
protected:

	void ZeroMeanFrame(Vector v);

	//input 3*numCepCoef 
	void adddeltas();


	char* hcopy_version;
	char* hcopy_vc_id;

	ParmBuf b;
	MemHeap iStack;          /* input stack */
	FBankInfo fbankinfo;


	int numChans;
	int frSize;
	int srcSampRate;
	int loFBankFreq;
	int hiFBankFreq;
	int numCepCoef;
	int cepLifter;

	BooleanC usePower;
	BooleanC takeLogs;
	BooleanC doubleFFT;

	double warpFreq ;
	double warpLowerCutOff ;
	double warpUpperCutOff;
	double preEmph;

	Vector s;
	Vector c;
	Vector fbank;

	std::vector<double> mfcc_; //13X1

	std::vector<double> filterWindow;

	std::vector<double> streamBuffer; // store the previous 2 frames data
	int bufferRow;
	int bufferCol;
	int firstrow;
	bool bufferfilled;
	int totalframescount;

};

