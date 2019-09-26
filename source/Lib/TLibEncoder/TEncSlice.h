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

/** \file     TEncSlice.h
 \brief    slice encoder class (header)
 */

#ifndef __TENCSLICE__
#define __TENCSLICE__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComPicYuv.h"
#include "TEncCu.h"
#include "WeightPredAnalysis.h"
#include "TEncRateCtrl.h"


// Include the offset calculator
 #include "TEncOffsetCalc.h"

//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncGOP;

//class TEncOffsetCalc;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// slice encoder class
class TEncSlice
: public WeightPredAnalysis
{
private:
    // encoder configuration
    TEncCfg*                m_pcCfg;                              ///< encoder configuration class
    
    // Hossam: I think this thing here is not used
    // pictures
    TComList<TComPic*>*     m_pcListPic;                          ///< list of pictures
    TComPicYuv*             m_apcPicYuvPred;                      ///< prediction picture buffer
    TComPicYuv*             m_apcPicYuvResi;                      ///< residual picture buffer
    
    // processing units
    TEncGOP*                m_pcGOPEncoder;                       ///< GOP encoder
    TEncCu*                 m_pcCuEncoder;                        ///< CU encoder
    
    // encoder search
    TEncSearch*             m_pcPredSearch;                       ///< encoder search class
    
    // coding tools
    TEncEntropy*            m_pcEntropyCoder;                     ///< entropy encoder
    TEncCavlc*              m_pcCavlcCoder;                       ///< CAVLC encoder
    TEncSbac*               m_pcSbacCoder;                        ///< SBAC encoder
    TEncBinCABAC*           m_pcBinCABAC;                         ///< Bin encoder CABAC
    TComTrQuant*            m_pcTrQuant;                          ///< transform & quantization
    
    // Offset Calculator
    TEncOffsetCalc            m_pcOffsetCalc;                     ///< offset calculator

//    TEncOffsetCalc*            m_pcOffsetCalc;                     ///< offset calculator
    
    // Hossam: Multiple References
    UInt64                  m_uiPicTotalBitsRef1;                     ///< total bits for the picture from Ref1
    UInt64                  m_uiPicTotalBitsRef2;                     ///< total bits for the picture from Ref2
    UInt64                  m_uiPicTotalBits_hevc;                     ///< total bits for the picture (hevc ml analysis)

    
    // Hossam: Scene change QP Configurations
    // m_piRdPicQp, m_pdRdPicQp
    // RD optimization
    TComBitCounter*         m_pcBitCounter;                       ///< bit counter
    TComRdCost*             m_pcRdCost;                           ///< RD cost computation
    TEncSbac***             m_pppcRDSbacCoder;                    ///< storage for SBAC-based RD optimization
    TEncSbac*               m_pcRDGoOnSbacCoder;                  ///< go-on SBAC encoder
    UInt64                  m_uiPicTotalBits;                     ///< total bits for the picture
    UInt64                  m_uiPicDist;                          ///< total distortion for the picture
    Double                  m_dPicRdCost;                         ///< picture-level RD cost
    Double*                 m_pdRdPicLambda;                      ///< array of lambda candidates
    Double*                 m_pdRdPicQp;                          ///< array of picture QP candidates (double-type for lambda)
    Int*                    m_piRdPicQp;                          ///< array of picture QP candidates (Int-type)
    TEncBinCABAC*           m_pcBufferBinCoderCABACs;             ///< line of bin coder CABAC
    TEncSbac*               m_pcBufferSbacCoders;                 ///< line to store temporary contexts
    TEncBinCABAC*           m_pcBufferLowLatBinCoderCABACs;       ///< dependent tiles: line of bin coder CABAC
    TEncSbac*               m_pcBufferLowLatSbacCoders;           ///< dependent tiles: line to store temporary contexts
    TEncRateCtrl*           m_pcRateCtrl;                         ///< Rate control manager
    UInt                    m_uiSliceIdx;
    std::vector<TEncSbac*> CTXMem;
    
    Void     setUpLambda(TComSlice* slice, const Double dLambda, Int iQP);
    Void     calculateBoundingCUAddrForSlice(UInt &uiStartCUAddrSlice, UInt &uiBoundingCUAddrSlice, Bool &bReachedTileBoundary, TComPic*& rpcPic, Bool bEncodeSlice, Int sliceMode, Int sliceArgument, UInt uiSliceCurEndCUAddr);

// Hossam: Scene change
public:
    UInt intra_modes;
    
    // For each <rPOC, iRef>
    Double reference_utitilization_rates[4][4];
    
    // Vector of reference utitlization for each frame and ref1 <POC, ref1>
    std::vector<Double> reference1_utilization_rates_perFrame;
    
    // Vector of reference utitlization for each frame and ref2 <POC, ref2>
    std::vector<Double> reference2_utilization_rates_perFrame;

    // Vector of reference utitlization for each frame and ref3 <POC, ref3>
    std::vector<Double> reference3_utilization_rates_perFrame;
    
    // Vector of reference utitlization for each frame and ref2 <POC, ref4>
    std::vector<Double> reference4_utilization_rates_perFrame;

    
    // Vector of bits for each frame and ref1 <POC, ref1>
    std::vector<Double> reference1_bits_perFrame;
    
    // Vector of bits for each frame and ref2 <POC, ref2>
    std::vector<Double> reference2_bits_perFrame;

    
    
    // For each iRef and Slice
    Double reference_utilization_counts[4];
    
    // Hossam: look ahead
    Int current_qp_offset;
    Double current_epsilon;
    
    // current qp_factor for mu method
    Double current_qp_factor_mu;
    Double current_qp_mu;
    Double current_lambda_mu;
    
    // epsilon is the propogation factor of frame i
    std::vector<Double> four_epsilons;
    std::vector<Int> four_offsets;
    
    
    // Yao variables:
    Double yao_intra_prev_mean;
    Double yao_intra_current_mean;
    Double yao_intra_prev_std;
    Double yao_intra_current_std;
    Double yao_content_variation;
    Double yao_threshold;
    Double yao_tUP;
    Double yao_tDown;
    Double yao_thresholdFinal;
    Double yao_average_QP;
    
    
    // Sastre variables
    Double sastre_avg_tillK;
    Double sastre_intra_count_tillK;
    Double sastre_span; // n
    Double sastre_Tf;
    Double sastre_Tlim;
    Double sastre_Ta;
    Double sastre_S;
    Double sastre_alpha;
    
    
public:
    TEncSlice();
    virtual ~TEncSlice();
    
    Int getTotalPicBits(){ return m_uiPicTotalBits; }
    
    Void    create              ( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth );
    Void    destroy             ();
    Void    init                ( TEncTop* pcEncTop );
    
    /// preparation of slice encoding (reference marking, QP and lambda)
    Void    initEncSlice        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd,
                                 Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField );
    
    // Hossam: Scene change:
    Void    initEncSliceNew        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                 Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc);
    
    // Removed this for now, we can use the offset calculator later
//        Void    initEncSliceNew        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
//                                     Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth);
    // Hossam: Scene change ORG
    Void    initEncSliceNewAttempt        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                           Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth);
    
    // Update your bits stats
    Void updateRef1Ref2Bits()
    {
        reference1_bits_perFrame.back() = m_uiPicTotalBitsRef1;
        reference2_bits_perFrame.back() = m_uiPicTotalBitsRef2;
        
        cout << "updateREf12: Ref1: " << reference1_bits_perFrame.back() << ", Ref2: " << reference2_bits_perFrame.back() << endl;
    }
    
    // Hossam: Scene Change ORG
    Void    modifyQP        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                             Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth);
    
    
    Void    modifyQPMock        ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                             Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth);
    
    
    Void    modifyQPActual       ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                 Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth);
    
    
    Void    modifyQPInter       ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                  Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    
    Void    modifyQPInterConservative       ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                 Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window, Bool isEpsilonMethodON = true);
    
    Void    modifyQPInterMu       ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                 Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    // adjust the QPfactors and offsets for interdep + benchmark SCs (to avoid reencoding)
    Void    setMockQPForSCOperation       ( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                 Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    
    // gets QPFact Inter according to experiments
    // epsilon methods LDP
    Double    getQPFactInter       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    Double    getQPFactInter1step       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    
    // mu methods LDP
    Double    getQPFactInterMu       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    Double    getQPFactInterMu1step       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);

    // epsilon methods LDB
    Double    getQPFactInterLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    Double    getQPFactInter1stepLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);

    
    // mu methods LDB
    Double    getQPFactInterMuLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);
    
    Double    getQPFactInterMu1stepLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);

    
    // Gets the default LD QP factor
    Double    getQPFactLDDefault       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window);


    
    // Calculate the current QP offset for the current frame
    Void calculateCurrentQPOffset( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                  Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window, Double epsilon_i);
    
    
    Void calculate4QPOffsets(TComPic* pcPic, std::vector<Double> epsilon_array, std::vector<Double> mu_ij, std::vector<Double> sigma_array);

    
    // resets the 4 epsilons to zero
    Void reset4Epsilons() {four_epsilons.clear();}
    
    
    float    getLambda        ( Int index);

    
    Void    resetQP             ( TComPic* pic, Int sliceQP, Double lambda );
    // compress and encode slice
    Void    precompressSlice    ( TComPic*& rpcPic                                );      ///< precompress slice for multi-loop opt.
    Void    compressSlice       ( TComPic*& rpcPic                                );      ///< analysis stage of slice
    
    
    // Hossam: Scene change
//    Int    getIntraModesCount       ( TComPic*& rpcPic                                );      ///< analysis stage of slice'
    
    // Hossam: Scene change
    Void    xExtractSliceInfo       ( TComPic*& rpcPic                                );      ///< analysis stage of slice


    // Hossam: print reference utilization
    Void printReferenceUtitilization(UInt cur_poc)
    {
        if(cur_poc == 0)
            return;
        
        UInt rPOC = cur_poc % 4;
        
        cout << "\n" << endl;
        for(Int i = 0; i < 4; i++)
        {
            Int currnet_print = rPOC==0? 4:rPOC;
            cout << "Reference Utilization [" << currnet_print << "]" << "[" << i << "]: " << reference_utitilization_rates[rPOC][i] << endl;
        }
        
        for(Int i = 0; i < 4; i++)
        {
            cout << cur_poc << ") Reference Utilization [" << i << "]" << "[" << rPOC << "]: " << reference_utilization_counts[i] << endl;
        }
    }
    
    
    // Hossam: print reference utilization
    Void printFrameReferenceUtitilization(UInt cur_poc)
    {
        if(cur_poc == 0)
        {
            return;
        }
        
        static int off2Array[] = {-5, -2, -3, -4};
        Int fetch_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
        Int test_ref1_poc = cur_poc - 1;
        Int off2 = -2;
        
        if(cur_poc > 10)
        {
            off2 = (cur_poc == 11)? -3 : off2Array[fetch_index];
        }
            
        Int test_ref2_poc = cur_poc + off2;

        cout << "\n" << endl;
        
        if(cur_poc == 1)
        {
            cout << "Reference1 Utilization [" << cur_poc << ", " << test_ref1_poc << "]: " << reference1_utilization_rates_perFrame.at(cur_poc) << endl;
            return;
        }
        
        cout << "Reference1 Utilization [" << cur_poc << ", " << test_ref1_poc << "]: " << reference1_utilization_rates_perFrame.at(cur_poc) << endl;
        cout << "Reference2 Utilization [" << cur_poc << ", " << test_ref2_poc << "]: " << reference2_utilization_rates_perFrame.at(cur_poc) << endl;
    }
    
    // Hossam: print reference utilization
    Void printFrameReferenceUtitilizationGOP(UInt cur_poc)
    {
        if(cur_poc == 0)
        {
            return;
        }
        
        static int off2Array[] = {-5, -2, -3, -4};
        Int fetch_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
        Int test_ref1_poc = cur_poc - 1;
        Int off2 = -2;
        
        if(cur_poc > 10)
        {
            off2 = (cur_poc == 11)? -3 : off2Array[fetch_index];
        }
        
        Int test_ref2_poc = cur_poc + off2;
        
        if(cur_poc == 1)
        {
            cout << "\nReference1 Utilization [" << cur_poc << ", " << test_ref1_poc << "]: " << reference1_utilization_rates_perFrame.at(cur_poc)/100.0 << endl;
            return;
        }
        
        cout << "[" << cur_poc << ", " << test_ref1_poc << "]: " << reference1_utilization_rates_perFrame.at(cur_poc)/100.0 << " [" << cur_poc << ", " << test_ref2_poc << "]: " << reference2_utilization_rates_perFrame.at(cur_poc)/100.0 << endl;
    }
    
    
    // Hossam: Scene Change
    // xCreate the YUV residuals buffer
    Void xCreateSCResidualsBuffer(TComPic*  pcPic, TComSlice* pcSlice);

    // Hossam: Scene change
    Void xDumpResiduals(TComSlice* pcSlice);
    
    // Hossam: Scene change
    Void    precompressSliceNew    ( TComPic*& rpcPic                                );      ///< precompress slice for multi-loop opt.

    // Hossam: Scene Change
    Void    compressSliceNew       ( TComPic*& rpcPic                                );      ///< analysis stage of slice
    
    // Hossam: Scene change
    Void    precompressSliceNewOrg    ( TComPic*& rpcPic                                );      ///< precompress slice for multi-loop opt.
    
    // Hossam: Scene Change
    Void    compressSliceNewOrg       ( TComPic*& rpcPic  , Int iRef  = -1);      ///< analysis stage of slice
    
    
    // compress and encode slice
    Bool    compressSliceBench       ( TComPic*& rpcPic                                );      ///< analysis stage of slice
    

    // Yao SC
    Bool    isSceneChangeYao       (TComSlice* pcSlice    ,  TComPic* pcPic );      ///< analysis stage of slice
    
    // Sastre SC
    Bool    isSceneChangeSastre       (TComSlice* pcSlice    ,  TComPic* pcPic );      ///< analysis stage of slice

    
    Void    calCostSliceI       ( TComPic*& rpcPic );
    Void    encodeSlice         ( TComPic*& rpcPic, TComOutputBitstream* pcSubstreams  );
    
    // misc. functions
    Void    setSearchRange      ( TComSlice* pcSlice  );                                  ///< set ME range adaptively
    UInt64  getTotalBits        ()  { return m_uiPicTotalBits; }
    
    TEncCu*        getCUEncoder() { return m_pcCuEncoder; }                        ///< CU encoder
    Void    xDetermineStartAndBoundingCUAddr  ( UInt& uiStartCUAddr, UInt& uiBoundingCUAddr, TComPic*& rpcPic, Bool bEncodeSlice );
    UInt    getSliceIdx()         { return m_uiSliceIdx;                    }
    Void    setSliceIdx(UInt i)   { m_uiSliceIdx = i;                       }
    Void      initCtxMem( UInt i );
    Void      setCtxMem( TEncSbac* sb, Int b )   { CTXMem[b] = sb; }
    
private:
    Double  xGetQPValueAccordingToLambda ( Double lambda );
};

//! \}

#endif // __TENCSLICE__
