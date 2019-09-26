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

/** \file     TEncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __TENCCU__
#define __TENCCU__

// Include files
#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComDataCU.h"

#include "TEncEntropy.h"
#include "TEncSearch.h"
#include "TEncRateCtrl.h"
//! \ingroup TLibEncoder
//! \{

class TEncTop;
class TEncSbac;
class TEncCavlc;
class TEncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
class TEncCu
{

    
// Hossam: Scene Change
public:
//    TComList<TComYuv*>      m_scResidualFrame;   ///< dynamic list of CUs for residual frame
//    TComList<TComPic*>      m_scResidualFrame;   ///< dynamic list of CUs for residual frame

//    TComList<TComPicYuv*>      m_scResidualFrame;   ///< dynamic list of CUs for residual frame
    
    // Hossam: Scene change
    TComPicYuv*      m_scResidualFrame;   ///< Residual frame for SC
    
//    Pel*             allData_Y;
//    Pel*             allData_U;
//    Pel*             allData_V;
    
    Bool isCompressCUFirstTime;
    
    // Hossam: Scene change
    TComYuv**   scTempResidual; // temp residual at every depth
    TComYuv**   scBestResidual; // best residual at every depth
    
    
    // Hossam: bits accumulatr
    Int bitsAccumulator;
    Int bitsAccumulatorRes;
    Int bitsAccumulatorPred;
    Int bitsAccumulatorSplitFlag;
    Int bitsAccumulatorPredMode;
    Int bitsAccumulatorPartSize;
    Int bitsAccumulatorIPCMFlag;
    Int bitsAccumulatorPredInfo;
    Int bitsAccumulatorCoeff;
    
    Int bitsAccumulatorTerminateInfo;
    Int bitsAccumulatorCUTransquantBypassFlag;
    Int bitsAccumulatorSkipFlag;
    Int bitsAccumulatorMergIndex;
    
    Int bitsSnapEntropyBeforePerCu;
    Int bitsSnapEntropyAfterPerCu;
    Int bitsAccumulateEntropyBits;
    
    
    
private:

    // Contains a picture and slice header pointers
  TComDataCU**            m_ppcBestCU;      ///< Best CUs in each depth
  TComDataCU**            m_ppcTempCU;      ///< Temporary CUs in each depth
  UChar                   m_uhTotalDepth;

    // YUV values and depth
  TComYuv**               m_ppcPredYuvBest; ///< Best Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvBest; ///< Best Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvBest; ///< Best Reconstruction Yuv for each depth
  TComYuv**               m_ppcPredYuvTemp; ///< Temporary Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvTemp; ///< Temporary Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvTemp; ///< Temporary Reconstruction Yuv for each depth
  TComYuv**               m_ppcOrigYuv;     ///< Original Yuv for each depth

  //  Data : encoder control
  Bool                    m_bEncodeDQP;
  Bool                    m_CodeChromaQpAdjFlag;
  Int                     m_ChromaQpAdjIdc;

  //  Access channel
  TEncCfg*                m_pcEncCfg;
  TEncSearch*             m_pcPredSearch;
  TComTrQuant*            m_pcTrQuant;
  TComBitCounter*         m_pcBitCounter;
  TComRdCost*             m_pcRdCost;

  TEncEntropy*            m_pcEntropyCoder;
  TEncCavlc*              m_pcCavlcCoder;
  TEncSbac*               m_pcSbacCoder;
  TEncBinCABAC*           m_pcBinCABAC;

  // SBAC RD
  TEncSbac***             m_pppcRDSbacCoder;
  TEncSbac*               m_pcRDGoOnSbacCoder;
  TEncRateCtrl*           m_pcRateCtrl;

public:
  /// copy parameters from encoder class
  Void  init                ( TEncTop* pcEncTop );

  /// create internal buffers
  Void  create              ( UChar uhTotalDepth, UInt iMaxWidth, UInt iMaxHeight, ChromaFormat chromaFormat );

  /// destroy internal buffers
  Void  destroy             ();

// Hossam: Scene change
Void xExtractCUInfo( TComDataCU* pcCU, UInt uiAbsPartIdx,   UInt uiDepth, UInt uiPartUnitIdx);

    
  /// CU analysis function
  Void  compressCU          ( TComDataCU*&  rpcCU );

    /// Hossam: Scene change CU analysis function
    Void  compressCUNew          ( TComDataCU*&  rpcCU );
    
    
    /// Hossam: Scene change CU analysis function from Org
    Void  compressCUNewOrg          ( TComDataCU*&  rpcCU, Int which_reference = -1 );
 
    
    /// Hossam: Scene change CU analysis function
    Void  saveResidualSignal          (TComDataCU* rpcBestCU, Char uhDepth);
    
    // Hossam: Scene Change
    Void  setCompressCUFirstTime          (Bool flag);
    
  /// CU encoding function
  Void  encodeCU            ( TComDataCU*    pcCU );

  Void setBitCounter        ( TComBitCounter* pcBitCounter ) { m_pcBitCounter = pcBitCounter; }

  Int   updateLCUDataISlice ( TComDataCU* pcCU, Int LCUIdx, Int width, Int height );

protected:
  Void  finishCU            ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );
#if AMP_ENC_SPEEDUP
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug), PartSize eParentPartSize = NUMBER_OF_PART_SIZES );
#else
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
  Void  xEncodeCU           ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );

#if AMP_ENC_SPEEDUP
    Void  xCompressCUNew         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug), PartSize eParentPartSize = NUMBER_OF_PART_SIZES );
#else
    Void  xCompressCUNew         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
    
    
#if AMP_ENC_SPEEDUP
    Void  xCompressCURunAtDepth         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug), PartSize eParentPartSize = NUMBER_OF_PART_SIZES );
#else
    Void  xCompressCURunAtDepth         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
    
// Hossam: Scene Change: Orginal ME
#if AMP_ENC_SPEEDUP
    Void  xCompressCURunAtDepthOrg         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug), PartSize eParentPartSize = NUMBER_OF_PART_SIZES, Int which_reference = -1 );
#else
    Void  xCompressCURunAtDepthOrg         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth        );
#endif
    
  Int   xComputeQP          ( TComDataCU* pcCU, UInt uiDepth );
  Void  xCheckBestMode      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo=true));

  Void  xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU DEBUG_STRING_FN_DECLARE(sDebug), Bool *earlyDetectionSkipMode );

#if AMP_MRG
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG = false  );
#else
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif

  Void  xCheckRDCostIntra   ( TComDataCU *&rpcBestCU,
                              TComDataCU *&rpcTempCU,
                              Double      &cost,
                              PartSize     ePartSize
                              DEBUG_STRING_FN_DECLARE(sDebug)
                            );

    
// Hossam: Scene change
 Void  xSaveResiduals      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth);

    
// Hossam: Scene change
Void  xUpdateResiduals      (Bool isUpdate);
    
// Hossam: Scene change
Void  xUpdateBestResiduals      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth);
    
// Hossam: Scene Change
Void  xPropogateBestResiduals      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth);


// Hossam: Scene Change
Void  xPropogateBestResidualsAtDepth      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth);

    
// Hossam: Scene change
  Void  xCheckBestModeNew      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo=true));

// Hossam: Scene change
Void  xCopyTempToBestSC      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo=true));
    

// Hossam: Scene change
Void  xCheckBestModeLastCall      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo=true));
    
    
 Void  xCheckRDCostMerge2Nx2NNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU DEBUG_STRING_FN_DECLARE(sDebug), Bool *earlyDetectionSkipMode );
    
#if AMP_MRG
    Void  xCheckRDCostInterNew   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG = false  );
#else
    Void  xCheckRDCostInterNew   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif
    
#if AMP_MRG
    Void  xCheckRDCostInterAtDepth   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG = false  );
#else
    Void  xCheckRDCostInterAtDepth   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif


// Hossam: Scene change
#if AMP_MRG
    Void  xCheckRDCostInterAtDepthOrg   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG = false, Int which_reference = -1  );
#else
    Void  xCheckRDCostInterAtDepthOrg   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif
    
    
    Void  xCheckRDCostIntraNew   ( TComDataCU *&rpcBestCU,
                               TComDataCU *&rpcTempCU,
                               Double      &cost,
                               PartSize     ePartSize
                               DEBUG_STRING_FN_DECLARE(sDebug)
                               );
    


    

  Void  xCheckDQP           ( TComDataCU*  pcCU );

  Void  xCheckIntraPCM      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU                      );
  Void  xCopyAMVPInfo       ( AMVPInfo* pSrc, AMVPInfo* pDst );
  Void  xCopyYuv2Pic        (TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY );
  Void  xCopyYuv2Tmp        ( UInt uhPartUnitIdx, UInt uiDepth );

  Bool getdQPFlag           ()                        { return m_bEncodeDQP;        }
  Void setdQPFlag           ( Bool b )                { m_bEncodeDQP = b;           }

  Bool getCodeChromaQpAdjFlag() { return m_CodeChromaQpAdjFlag; }
  Void setCodeChromaQpAdjFlag( Bool b ) { m_CodeChromaQpAdjFlag = b; }

#if ADAPTIVE_QP_SELECTION
  // Adaptive reconstruction level (ARL) statistics collection functions
  Void xLcuCollectARLStats(TComDataCU* rpcCU);
  Int  xTuCollectARLStats(TCoeff* rpcCoeff, TCoeff* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples );
#endif

#if AMP_ENC_SPEEDUP
#if AMP_MRG
  Void deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver);
#else
  Void deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver);
#endif
#endif

  Void  xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv );
};

//! \}

#endif // __TENCMB__
