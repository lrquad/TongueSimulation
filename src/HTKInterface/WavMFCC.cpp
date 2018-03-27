#include "WavMFCC.h"
#include <iostream>
#include <fstream>




WavMFCC::WavMFCC()
{
	//config
	numChans = 26;
	frSize = 400;
	srcSampRate = 625;
	loFBankFreq = -1;
	hiFBankFreq = -1;
	numCepCoef = 12;
	cepLifter = 22;

	usePower = FALSEC;
	takeLogs = TRUEC;
	doubleFFT = FALSEC;
	warpFreq = 1;
	warpLowerCutOff = 0;
	warpUpperCutOff = 0;
	preEmph = 0.97;

	InitSigP();

	CreateHeap(&iStack, "InBuf", MSTAK, 1, 0.0, STACKSIZE, LONG_MAX);

	fbankinfo = InitFBank(&iStack,frSize,srcSampRate,numChans,loFBankFreq,hiFBankFreq,
		usePower,takeLogs,doubleFFT,
		warpFreq,
		warpLowerCutOff,warpUpperCutOff);
	fbank = CreateVector(&iStack,numChans);

	s = CreateVector(&iStack, frSize);
	c = CreateVector(&iStack, numCepCoef);

	filterWindow.resize(5);
	filterWindow[0] = 0.2;
	filterWindow[1] = 0.1;
	filterWindow[2] = 0.0;
	filterWindow[3] = -0.1;
	filterWindow[4] = -0.2;

	streamBuffer.resize((numCepCoef + 1) * 3 *7);
	//7X39 matrix
	bufferRow = 7;
	bufferCol = (numCepCoef + 1) * 3;

	std::fill(streamBuffer.begin(), streamBuffer.end(), 0);
	bufferfilled = false;
	firstrow = 0;
	totalframescount = 0;

	mfcc_.resize(13);
}


WavMFCC::~WavMFCC()
{
	ResetHeap(&iStack);
}

void WavMFCC::streamWavFrameToMFCC(double* wav, double* output)
{
	for (int i = 0; i < frSize; i++)
	{
		s[i + 1] = wav[i];
	}

	ZeroMeanFrame(s);
	PreEmphasise(s, preEmph);
	Ham(s);

	Wave2FBank(s, fbank, NULL, fbankinfo);

	FBank2MFCC(fbank, c, numCepCoef);

	WeightCepstrum(c, 1, numCepCoef, cepLifter);

	for (int i = 0; i < numCepCoef; i++)
	{
		mfcc_[i] = c[i + 1];
	}
	mfcc_[numCepCoef] = FBank2C0(fbank);

	if (!bufferfilled)
	{
		for (int i = 0; i < numCepCoef+1; i++)
		{
			streamBuffer[0 + i*bufferRow] = mfcc_[i];
			streamBuffer[1 + i*bufferRow] = mfcc_[i];
			streamBuffer[2 + i*bufferRow] = mfcc_[i];
			streamBuffer[3 + i*bufferRow] = mfcc_[i];
		}
		bufferfilled = true;
		firstrow = 0;
	}

	for (int i = 0; i < numCepCoef+1; i++)
	{
		int row = 0;
		row = (firstrow + 4) % bufferRow;
		streamBuffer[row + i*bufferRow] = mfcc_[i];
		row = (firstrow + 5) % bufferRow;
		streamBuffer[row + i*bufferRow] = mfcc_[i];
		row = (firstrow + 6) % bufferRow;
		streamBuffer[row + i*bufferRow] = mfcc_[i];
	}
	

	adddeltas();

	int row = firstrow + 2 + 2;
	row %= bufferRow;
	for (int i = 0; i < bufferCol; i++)
	{
		output[i] = streamBuffer[i*bufferRow + row];
	}
	
	//update stream
	firstrow++;
	firstrow %= bufferRow;
	totalframescount++;
}

void WavMFCC::resetBuffer()
{
	std::fill(streamBuffer.begin(), streamBuffer.end(), 0);
	bufferfilled = false;
	firstrow = 0;
	totalframescount = 0;
}

void WavMFCC::ZeroMeanFrame(Vector v)
{
	int size, i;
	float sum = 0.0, off;

	size = VectorSize(v);
	for (i = 1; i <= size; i++) sum += v[i];
	off = sum / size;
	for (i = 1; i <= size; i++) v[i] -= off;
}

void WavMFCC::adddeltas()
{
	//compute delta
	int offset = numCepCoef + 1;
	for (int i = 0; i < numCepCoef + 1; i++)
	{
		int col = offset + i;
		int oricol = i;
		
		for (int j = 0; j < 3; j++)
		{
			int row = firstrow + 2 + j;
			row %= bufferRow;

			double value = 0;
			for (int k = -2; k <= 2; k++)
			{
				int trow = row + k;
				trow = (trow + bufferRow) % bufferRow;
				value += streamBuffer[oricol*bufferRow + trow] * filterWindow[-k + 2];
			}
			streamBuffer[col*bufferRow + row] = value;
		}
	}

	//compute deltadelta;
	int doffset = offset + numCepCoef + 1;
	for (int i = 0; i < numCepCoef + 1; i++)
	{
		int col = doffset + i;
		int oricol = i + offset;

		int row = firstrow + 2 + 2;
		row %= bufferRow;
		double value = 0;
		for (int k = -2; k <= 2; k++)
		{
			int trow = row + k;
			if (k > 0)
			{
				trow = row;
			}
			trow = (trow + bufferRow) % bufferRow;
			value += streamBuffer[oricol*bufferRow + trow] * filterWindow[-k + 2];
		}
		streamBuffer[col*bufferRow + row] = value;
	}
}
