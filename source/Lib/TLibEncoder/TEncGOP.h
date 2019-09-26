/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TEncGOP.h
 \brief    GOP encoder class (header)
 */

#ifndef __TENCGOP__
#define __TENCGOP__

#include <list>

#include <stdlib.h>

#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "SEIwrite.h"

#include "TEncAnalyze.h"
#include "TEncRateCtrl.h"
#include <vector>

#include <thread>



// Open CV header files
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

//! \ingroup TLibEncoder
//! \{

class TEncTop;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TEncGOP
{
private:
    //  Data
    Bool                    m_bLongtermTestPictureHasBeenCoded;
    Bool                    m_bLongtermTestPictureHasBeenCoded2;
    UInt                    m_numLongTermRefPicSPS;
    UInt                    m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
    Bool                    m_ltRefPicUsedByCurrPicFlag[MAX_NUM_LONG_TERM_REF_PICS];
    Int                     m_iLastIDR;
    Int                     m_iGopSize;
    Int                     m_iNumPicCoded;
    Bool                    m_bFirst;
    
    
    // Hossam: threshold for mu method-P
    const Double          THRESHOLD_MU_22 = 90;
    const Double          THRESHOLD_MU_27 = 90;
    
    // Epsilon-P
    const Double          THRESHOLD_EPS_22 = 90;
    const Double          THRESHOLD_EPS_27 = 80;
    
    // Epsilon-B
    const Double          THRESHOLD_EPS_B_22 = 80;
    const Double          THRESHOLD_EPS_B_27 = 55;
    
    // mu-B
    const Double          THRESHOLD_MU_B_22 = 75;
    const Double          THRESHOLD_MU_B_27 = 55;
    

    
   // Hossam: Look Ahead variable
    Int                     m_iNumPicCodedActual;

    
    
#if ALLOW_RECOVERY_POINT_AS_RAP
    Int                     m_iLastRecoveryPicPOC;
#endif
    
    //  Access channel
    TEncTop*                m_pcEncTop;
    //  TEncCfg*                m_pcCfg;
    TEncSlice*              m_pcSliceEncoder;
    TComList<TComPic*>*     m_pcListPic;
    
    TEncEntropy*            m_pcEntropyCoder;
    TEncCavlc*              m_pcCavlcCoder;
    TEncSbac*               m_pcSbacCoder;
    TEncBinCABAC*           m_pcBinCABAC;
    TComLoopFilter*         m_pcLoopFilter;
    
    SEIWriter               m_seiWriter;
    
    //--Adaptive Loop filter
    TEncSampleAdaptiveOffset*  m_pcSAO;
    TComBitCounter*         m_pcBitCounter;
    TEncRateCtrl*           m_pcRateCtrl;
    // indicate sequence first
    Bool                    m_bSeqFirst;
    
    // Hossam: look ahead flag
    Bool m_bSeqFirstLookAhead;
    
    // clean decoding refresh
    Bool                    m_bRefreshPending;
    Int                     m_pocCRA;
    std::vector<Int>        m_storedStartCUAddrForEncodingSlice;
    std::vector<Int>        m_storedStartCUAddrForEncodingSliceSegment;
#if FIX1172
    NalUnitType             m_associatedIRAPType;
    Int                     m_associatedIRAPPOC;
#endif
    
    std::vector<Int> m_vRVM_RP;
    UInt                    m_lastBPSEI;
    UInt                    m_totalCoded;
    UInt                    m_cpbRemovalDelay;
    UInt                    m_tl0Idx;
    UInt                    m_rapIdx;
    Bool                    m_activeParameterSetSEIPresentInAU;
    Bool                    m_bufferingPeriodSEIPresentInAU;
    Bool                    m_pictureTimingSEIPresentInAU;
    Bool                    m_nestedBufferingPeriodSEIPresentInAU;
    Bool                    m_nestedPictureTimingSEIPresentInAU;
    
    // Hossam: look ahead
    UInt general_GOP_id;
    
    // Hossam: look ahead
    UInt pocCurrFactor;
    
    // Hossam: look ahead
    UInt actual_GOP_id;
    
    // Hossam: look ahead
    UInt pocCurrFactorActual;
    
    // Hossam: turn on and off mu method in the middle
    Bool isMuMethodON;

    // Hossam: turn on and off epsilon method in the middle
    Bool isEpsilonMethodON;

    
    // mu(i, j) --> Coding dependency between frame i and frame j
    std::vector<Double> mu_ij;

    
    // mu(i, j) --> Coding dependency between frame i and frame j (Coming from Ref1)
    std::vector<Double> mu_ij_ref1;

    
    // epsilon is the propogation factor of frame i
    std::vector<Double> epsilon_array;
    
    // distortion of frame 1, 2, 3, ...upto p
    std::vector<Double> distortion_in_sliding_window;
    
    // rate of frame 1, 2, 3, ... upto p
    std::vector<UInt> rate_in_sliding_window;
    
    // sigma squared of the original distortion (could be for all references and could be for reference 0)
    std::vector<Double> sigma_squared_in_sliding_window;
    
    
    // sigma squared for reference 1
    std::vector<Double> sigma_squared_in_sliding_window_Ref1;

    
    // progress in a sliding window
    UInt progress_in_sliding_window;
    
    // progress in actual encoding
    UInt encoding_actual_progress;
    
    // Length of the current sliding window (p or less)
    UInt sliding_window_length;
    
    // Start window
    Int start_prop_window;
    
    // end window
    Int end_prop_window;

    // Start window
    Int start_prop_window_mu;
    
    // end window
    Int end_prop_window_mu;

    
public:
    
    TEncCfg*                m_pcCfg;
    
    // Hossam: GOP_STR_TYPE
    //  UInt                    m_gop_str_type;
    //  string                  m_inputFileName;
    
    TEncGOP();
    virtual ~TEncGOP();
    
    Void  create      ();
    Void  destroy     ();
    
    Void  init        ( TEncTop* pcTEncTop );
    Void  compressGOP ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                       std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // Hossam: Scene Change
//    Void  compressGOPNew ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
//                       std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );

    Void  compressGOPNew ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                       std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // Hossam: P frames epsilon without look ahead
    Void  compressGOPNewInter ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                          std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // compressGOP conservative
    Void  compressGOPNewInterConservative ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                               std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // Hossam: P frames epsilon without look ahead -- SC included
    Void  compressGOPNewInterSC ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                               std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // For benchMarks
    Void  compressGOPNewInterSCBench ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                                 std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // Hossam: P frames epsilon using mu prediction
    Void  compressGOPNewInterMu ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                               std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // Hossam: P frames epsilon using mu prediction and SC
    Void  compressGOPNewInterMuSC ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                                 std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // Hossam: P frames epsilon using mu prediction multiple frames
    Void  compressGOPNewInterMuMult ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                                 std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // Hossam: P frames look ahead
    Void  compressGOPNewP04 ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                          std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    Void  compressGOPNewP (Bool isMockEncoding, Bool isAddPSNR, TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                           std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    Void  compressGOPNewPMock (Bool isMockEncoding, Bool isAddPSNR, TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                           std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // used for benchmarks for scene change
    Void  compressGOPNewBench ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                          std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // used for benchmarks for scene change epsilon
    Void  compressGOPNewBenchInterSC ( TComList<TComPic*>& rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
                               std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // loops over the mu and dumps it for you
    Void dumpMuValues(const std::vector<Double> &mu_ij, const UInt &start_for_i, const UInt &last_frame_encoded, Bool isGroupBased) const;
    
    // loops over the mu and dumps it for you multiple
    Void dumpMuValuesMult(const std::vector<Double> &mu_ij, const std::vector<Double> &mu_ij_ref1, const UInt &start_for_i, const UInt &last_frame_encoded, Bool isGroupBased) const;
    
    
    // gets you the offset for Ref2 in Low-Delay case
    Int getLDOffsetRef2 (Int i) const
    {
        Int offset = -2;
        static int off_ref [] = {-5, -2, -3, -4};
        Int fetch_index = i % 4 == 0? 3: i % 4 - 1;
        
        if(i > 10)
        {
            offset = (i==11)? -3: off_ref[fetch_index];
        }
        
        return offset;
    }
    
    
    // Hossam: reset the window parameters for epsilon method
    Void resetWindowParamtersEps()
    {
        start_prop_window = 0;
        end_prop_window = 8;
    }
    
    // Hossam: reset the window parameters for mu method
    Void resetWindowParamtersMu()
    {
        start_prop_window_mu = 3;
        end_prop_window_mu = 7;
    }

    // Hossam: set the propogation length
    Void setAdaptivePropagationLength(Int input)
    {
        g_pPropLength = input;
    }
    
    
    // Hossam: debug look ahead
    Void showMeRcListsGOP(TComList<TComPic*>& rcListPic);
    

    Void incrGOPId()
    {
        general_GOP_id  = (general_GOP_id+1)%m_iGopSize;
    }
    
    Void incrGOPIdActual()
    {
        actual_GOP_id  = (actual_GOP_id+1)%m_iGopSize;
    }
    
    Void setNumPicCodedGOP(Int val)
    {
        m_iNumPicCoded = val;
    }
    
    Void displayDAndSigmaSlidingWindowList(UInt loop_to)
    {
        cout << "\nDisplay the distortion contents: " << endl;
        for(UInt i = 0; i <= loop_to; i++ )
        {
            if(i >= distortion_in_sliding_window.size()) continue;
            cout << "< " << i  << ", " <<  distortion_in_sliding_window.at(i) << ">, ";
        }
        
        cout << "\n" << endl;
        cout << "\nDisplay the sigma contents: " << endl;
        for(UInt i = 0; i <= loop_to; i++ )
        {
            if(i >= sigma_squared_in_sliding_window.size()) continue;
            cout << "< " << i  << ", " <<  sigma_squared_in_sliding_window.at(i) << ">, ";
        }
        
        cout << "\n " << endl;
        
        
        cout << "\n" << endl;
        cout << "\nDisplay the sigma1 contents: " << endl;
        for(UInt i = 0; i <= loop_to; i++ )
        {
            if(i >= sigma_squared_in_sliding_window_Ref1.size()) continue;
            cout << "< " << i  << ", " <<  sigma_squared_in_sliding_window_Ref1.at(i) << ">, ";
        }
        
        cout << "\n " << endl;
    }
    
    Void displayCodingDepList()
    {
        cout << "\nDisplay the mu_ij contents" << endl;
        for(UInt i = 0; i < mu_ij.size(); i++ )
        {
            cout << "< " << i  << ", " <<  mu_ij.at(i) << ">, ";
        }
        
        cout << "\n " << endl;
        
        cout << "\n" << endl;
        cout << "\nDisplay the mu_ij_ref1 contents: " << endl;
        for(UInt i = 0; i < mu_ij_ref1.size(); i++ )
        {
            cout << "< " << i  << ", " <<  mu_ij.at(i) << ">, ";
        }
    }
    
    // used for mult
    Void displayCodingDepList(std::vector<Double> mu1, std::vector<Double> mu2)
    {
        cout << "\nDisplay the mu_ij contents: " << mu1.size() << endl;
        for(UInt i = 0; i < mu1.size(); i++ )
        {
            cout << "< " << (i+1)  << ", " <<  mu1.at(i) << ">, ";
        }
        
        cout << "\n " << endl;
        
        cout << "\n" << endl;
        cout << "\nDisplay the mu_ij_ref1 contents: " << mu2.size()  << endl;
        for(UInt i = 0; i < mu2.size(); i++ )
        {
            cout << "< " << (i+1)  << ", " <<  mu2.at(i) << ">, ";
        }
    }
    
    Void displayReferenceUtilizations(UInt loopto)
    {
        for(UInt i = 0; i <= loopto; i++)
        {
            m_pcSliceEncoder->printFrameReferenceUtitilizationGOP(i);
        }
    }
    
    // Hossam: Look Ahead
//    Void copyFromLookAheadBufferSAO(TComPic)
//    {
//        cout << "\n &&&&&&&&&ACTUAL After Mock&&&&&&&&& \n" << endl;
//        m_pcSAO->printSAOBuffer(pcPic, pcPic->getSlice(0)->getDepth());
//        m_pcSAO->printLookAheadBuffer(pcPic, pcPic->getSlice(0)->getDepth());
//        
//        
//        // Hossam: look ahead
//        cout << "CopyFromLookAhead to restore the original status of the buffer " << endl;
//        m_pcSAO->copyFromLookAhead();
//        
//        cout << "\n &&&&&&&&&ACTUAL After Copy&&&&&&&&& \n" << endl;
//        m_pcSAO->printSAOBuffer(pcPic, pcPic->getSlice(0)->getDepth());
//        m_pcSAO->printLookAheadBuffer(pcPic, pcPic->getSlice(0)->getDepth());
//
//    }
    
    
    Void  xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream*& rpcBitstreamRedirect);
    
    
    Int   getGOPSize()          { return  m_iGopSize;  }
    
    TComList<TComPic*>*   getListPic()      { return m_pcListPic; }
    
    Void  printOutSummary      ( UInt uiNumAllPicCoded, Bool isField, const Bool printMSEBasedSNR, const Bool printSequenceMSE );
    Void  preLoopFilterPicAll  ( TComPic* pcPic, UInt64& ruiDist, UInt64& ruiBits );
    
    // Print reference utilization
    Void dumpReferenceUtilization();
    
    
    // Print reference utilization accumulated
    Void dumpReferenceUtilizationAccumulated();
    Void dumpReferenceUtilizationAccumulatedHelper();

    
    Void dumpReferenceUtilizationHelper(const Double * reference_counts, Int rPOC);

    
    TEncSlice*  getSliceEncoder()   { return m_pcSliceEncoder; }
    NalUnitType getNalUnitType( Int pocCurr, Int lastIdr, Bool isField );
    Void arrangeLongtermPicturesInRPS(TComSlice *, TComList<TComPic*>& );
    
protected:
    TEncRateCtrl* getRateCtrl()       { return m_pcRateCtrl;  }
    
protected:
    
    Void  xInitGOP          ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Bool isField );
    
    // Hossam: Scene change
      Void  xGetBufferNew        ( TComList<TComPic*>& rcListPic, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, Int pocCurr, Bool isField);
//    Void  xGetBufferNew        ( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, Bool isField, TComList<TComPic*>& rcListPicAttempt );
    
    
    Void  xGetBuffer        ( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, Bool isField );
    
    // Hossam: Look Ahead
    Void  xGetBufferActual        ( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, Bool isField );

    Void  xGetBufferMock        ( TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRecOut, Int iNumPicRcvd, Int iTimeOffset, TComPic*& rpcPic, TComPicYuv*& rpcPicYuvRecOut, Int pocCurr, Bool isField );

    
    Void  xCalculateAddPSNR          ( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit&, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // Hossam: inter without look ahead
    Void  xCalculateAddPSNRInter          ( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit&, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    // Hossam: look Ahead
    Void  xCalculateAddPSNRActual          ( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit&, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    
    // Calculates MSSIM between two open cv matrices
    Scalar getMSSIM( const Mat& i1, const Mat& i2);
    double getPSNR(const Mat& I1, const Mat& I2, int iSize);


    
    // Calculate the propogation factor of the current frame and coding dependency between frames i, j
    Void calculateCurrentEpsilon(TComPic* pcPic);
    
    //
    Void calculate4Epsilons(TComPic* pcPic);
    
    // mu prediction method
    Void calculateCurrentEpsilonMu(TComPic* pcPic);
    
    
    // mu prediction method for multiple reference frames
    Void calculateCurrentEpsilonMuMult(TComPic* pcPic);
    Double calculateCurrentEpsilonMuMultHelper(TComPic* pcPic, Int current_i, Int prev_i = 0, Int level = 1);
    // used for debugging
    Double mock_calculateCurrentEpsilonMuMultHelper(TComPic* pcPic, Int current_i, Int prev_i = 0, Int level = 1, Double eps = 0);
    Double mock2_calculateCurrentEpsilonMuMultHelper(TComPic* pcPic, Int current_i, Int prev_i = 0, Int level = 1);
    Double mock3_calculateCurrentEpsilonMuMultHelper(TComPic* pcPic, Int current_i, Int prev_i = 0, Int level = 1);


    
    // get me the right/closest i, j (Pred, Ref for Mult)
    Void getPredRefIndexMult(Int cur_poc, Int& i, Int& j)
    {
      while(i >= cur_poc)
      {
          i = i - 4;
          j = j - 4;
      }
    }
    
    
    Void  xCalculateAddPSNRMock          ( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit&, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );

    
    Void  xCalculateInterlacedAddPSNR( TComPic* pcPicOrgFirstField, TComPic* pcPicOrgSecondField,
                                      TComPicYuv* pcPicRecFirstField, TComPicYuv* pcPicRecSecondField,
                                      const AccessUnit& accessUnit, Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE );
    
    UInt64 xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1);
    
    Double xCalculateRVM();
    
    SEIActiveParameterSets*           xCreateSEIActiveParameterSets (TComSPS *sps);
    SEIFramePacking*                  xCreateSEIFramePacking();
    SEISegmentedRectFramePacking*     xCreateSEISegmentedRectFramePacking();
    SEIDisplayOrientation*            xCreateSEIDisplayOrientation();
    SEIToneMappingInfo*               xCreateSEIToneMappingInfo();
    SEITempMotionConstrainedTileSets* xCreateSEITempMotionConstrainedTileSets ();
    SEIKneeFunctionInfo*              xCreateSEIKneeFunctionInfo();
    SEIChromaSamplingFilterHint*      xCreateSEIChromaSamplingFilterHint(Bool bChromaLocInfoPresent, Int iHorFilterIndex, Int iVerFilterIdc);
    
    Void xCreateLeadingSEIMessages (/*SEIMessages seiMessages,*/ AccessUnit &accessUnit, TComSPS *sps);
    Int xGetFirstSeiLocation (AccessUnit &accessUnit);
    Void xResetNonNestedSEIPresentFlags()
    {
        m_activeParameterSetSEIPresentInAU = false;
        m_bufferingPeriodSEIPresentInAU    = false;
        m_pictureTimingSEIPresentInAU      = false;
    }
    Void xResetNestedSEIPresentFlags()
    {
        m_nestedBufferingPeriodSEIPresentInAU    = false;
        m_nestedPictureTimingSEIPresentInAU      = false;
    }
    Void dblMetric( TComPic* pcPic, UInt uiNumSlices );
};// END CLASS DEFINITION TEncGOP

// ====================================================================================================================
// Enumeration
// ====================================================================================================================
enum PROCESSING_STATE
{
    EXECUTE_INLOOPFILTER,
    ENCODE_SLICE
};

enum SCALING_LIST_PARAMETER
{
    SCALING_LIST_OFF,
    SCALING_LIST_DEFAULT,
    SCALING_LIST_FILE_READ
};

//! \}

#endif // __TENCGOP__

