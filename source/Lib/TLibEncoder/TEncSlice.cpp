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

/** \file     TEncSlice.cpp
 \brief    slice encoder class
 */

#include "TEncTop.h"
#include "TEncSlice.h"
#include <math.h>

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncSlice::TEncSlice()
{
    // Hossam: look ahead
    current_qp_offset = 0;
    current_epsilon = 0;
    
    // Hossam: mu method
    current_qp_factor_mu = 0;
    current_qp_mu = 0;
    current_lambda_mu = 0;
    
    m_apcPicYuvPred = NULL;
    m_apcPicYuvResi = NULL;
    
    m_pdRdPicLambda = NULL;
    m_pdRdPicQp     = NULL;
    m_piRdPicQp     = NULL;
    m_pcBufferSbacCoders    = NULL;
    m_pcBufferBinCoderCABACs  = NULL;
    m_pcBufferLowLatSbacCoders    = NULL;
    m_pcBufferLowLatBinCoderCABACs  = NULL;
}

TEncSlice::~TEncSlice()
{
    for (std::vector<TEncSbac*>::iterator i = CTXMem.begin(); i != CTXMem.end(); i++)
    {
        delete (*i);
    }
}

Void TEncSlice::initCtxMem(  UInt i )
{
    for (std::vector<TEncSbac*>::iterator j = CTXMem.begin(); j != CTXMem.end(); j++)
    {
        delete (*j);
    }
    CTXMem.clear();
    CTXMem.resize(i);
}

Void TEncSlice::create( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )
{
    // Hossam: Scene change
    intra_modes = 0;
    
    // Hossam: init utilization rates
    for(int i = 0 ; i < 4 ; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            reference_utitilization_rates[i][j] = 0;
        }
        reference_utilization_counts[i] = 0;
    }
    
    // create prediction picture
    if ( m_apcPicYuvPred == NULL )
    {
        m_apcPicYuvPred  = new TComPicYuv;
        m_apcPicYuvPred->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
    }
    
    // create residual picture
    if( m_apcPicYuvResi == NULL )
    {
        m_apcPicYuvResi  = new TComPicYuv;
        m_apcPicYuvResi->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
    }
    
    // Create the offset calculator
    m_pcOffsetCalc.create();
}

Void TEncSlice::destroy()
{
    // destroy prediction picture
    if ( m_apcPicYuvPred )
    {
        m_apcPicYuvPred->destroy();
        delete m_apcPicYuvPred;
        m_apcPicYuvPred  = NULL;
    }
    
    // destroy residual picture
    if ( m_apcPicYuvResi )
    {
        m_apcPicYuvResi->destroy();
        delete m_apcPicYuvResi;
        m_apcPicYuvResi  = NULL;
    }
    
    // free lambda and QP arrays
    if ( m_pdRdPicLambda ) { xFree( m_pdRdPicLambda ); m_pdRdPicLambda = NULL; }
    if ( m_pdRdPicQp     ) { xFree( m_pdRdPicQp     ); m_pdRdPicQp     = NULL; }
    if ( m_piRdPicQp     ) { xFree( m_piRdPicQp     ); m_piRdPicQp     = NULL; }
    
    if ( m_pcBufferSbacCoders )
    {
        delete[] m_pcBufferSbacCoders;
    }
    if ( m_pcBufferBinCoderCABACs )
    {
        delete[] m_pcBufferBinCoderCABACs;
    }
    if ( m_pcBufferLowLatSbacCoders )
        delete[] m_pcBufferLowLatSbacCoders;
    if ( m_pcBufferLowLatBinCoderCABACs )
        delete[] m_pcBufferLowLatBinCoderCABACs;
    
    // Destroy the offset calculator
    m_pcOffsetCalc.destroy();
}

Void TEncSlice::init( TEncTop* pcEncTop )
{
    m_pcCfg             = pcEncTop;
    m_pcListPic         = pcEncTop->getListPic();
    
    m_pcGOPEncoder      = pcEncTop->getGOPEncoder();
    m_pcCuEncoder       = pcEncTop->getCuEncoder();
    m_pcPredSearch      = pcEncTop->getPredSearch();
    
    m_pcEntropyCoder    = pcEncTop->getEntropyCoder();
    m_pcCavlcCoder      = pcEncTop->getCavlcCoder();
    m_pcSbacCoder       = pcEncTop->getSbacCoder();
    m_pcBinCABAC        = pcEncTop->getBinCABAC();
    m_pcTrQuant         = pcEncTop->getTrQuant();
    
    m_pcBitCounter      = pcEncTop->getBitCounter();
    m_pcRdCost          = pcEncTop->getRdCost();
    m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
    m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();
    
    // create lambda and QP arrays
    m_pdRdPicLambda     = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
    m_pdRdPicQp         = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
    m_piRdPicQp         = (Int*   )xMalloc( Int,    m_pcCfg->getDeltaQpRD() * 2 + 1 );
    m_pcRateCtrl        = pcEncTop->getRateCtrl();
    
    
    // Init the offset calculator
    m_pcOffsetCalc.init();
    
#if IS_YAO_SCD
    // Yao variables
    yao_intra_current_mean = yao_intra_prev_mean = yao_intra_prev_std = yao_intra_current_std
    = yao_content_variation = yao_threshold = yao_tUP = yao_tDown = yao_thresholdFinal = yao_average_QP = 0;
#endif
    
#if IS_SASTRE_SCD
    sastre_avg_tillK = sastre_intra_count_tillK = sastre_span = sastre_Tf = sastre_Tlim = sastre_Ta = sastre_S = sastre_alpha = 0;
#endif
    
}



Void
TEncSlice::setUpLambda(TComSlice* slice, const Double dLambda, Int iQP)
{
    // store lambda
    m_pcRdCost ->setLambda( dLambda );
    
    // for RDO
    // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
    Double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
    for(UInt compIdx=1; compIdx<MAX_NUM_COMPONENT; compIdx++)
    {
        const ComponentID compID=ComponentID(compIdx);
        Int chromaQPOffset = slice->getPPS()->getQpOffset(compID) + slice->getSliceChromaQpDelta(compID);
        Int qpc=(iQP + chromaQPOffset < 0) ? iQP : getScaledChromaQP(iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc());
        Double tmpWeight = pow( 2.0, (iQP-qpc)/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
        m_pcRdCost->setDistortionWeight(compID, tmpWeight);
        dLambdas[compIdx]=dLambda/tmpWeight;
    }
    
#if RDOQ_CHROMA_LAMBDA
    // for RDOQ
    m_pcTrQuant->setLambdas( dLambdas );
#else
    m_pcTrQuant->setLambda( dLambda );
#endif
    
    // For SAO
    slice   ->setLambdas( dLambdas );
}



/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast      POC of last picture
 \param pocCurr     current POC
 \param iNumPicRcvd   number of received pictures
 \param iTimeOffset   POC offset for hierarchical structure
 \param iDepth        temporal layer depth
 \param rpcSlice      slice header class
 \param pSPS          SPS associated with the slice
 \param pPPS          PPS associated with the slice
 */

// Cathy
static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};
static int Qpoff[] ={3, 2, 3, 1};
//static int depths[] ={3, 2, 3, 1, 0};
static int depths[] ={2, 1, 2, 0};
static int track = 0;
static bool isSc = false;
static float lambdaArr[]  = {20.1964, 14.7968, 20.1964, 7.34014, 4.88345};
static float lambdaArr2[] = {88.7808, 66.5506, 88.7808, 23.3035, 15.504};
static float lambdaArr3[] = {360.156, 273.428, 360.156, 73.984, 49.2221};
static float lambdaArr4[] = {1193.14, 946.995, 1193.14, 234.885, 156.271};
static float lambdaArr5[] = {3787.98, 3006.52, 3787.98, 745.712, 496.128};

static int trackLambda = 0;
static Int comingFromSmooth = 0;// 0, 1, 2
static Int caseNumber = -1;

// May hatem --- WORKING VERSION NEED DEBUGGING FOR OFFSET CALC
//Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth )
//{
//    Double dQP;
//    Double dLambda;
//    
//    // Hossam: Scene change
////    Double lambdaHelper = 0;
//    
//    rpcSlice = pcPic->getSlice(0);
//    rpcSlice->setSPS( pSPS );
//    rpcSlice->setPPS( pPPS );
//    rpcSlice->setSliceBits(0);
//    rpcSlice->setPic( pcPic );
//    rpcSlice->initSlice();
//    rpcSlice->setPicOutputFlag( true );
//    rpcSlice->setPOC( pocCurr );
//    
//    // depth computation based on GOP size
//    Int depth;
//    {
//#if FIX_FIELD_DEPTH
//        Int poc = rpcSlice->getPOC();
//        if(isField)
//        {
//            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
//        }
//        else
//        {
//            poc = poc % m_pcCfg->getGOPSize();
//        }
//#else
//        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
//#endif
//        
//        if ( poc == 0 )
//        {
//            depth = 0;
//        }
//        else
//        {
//            Int step = m_pcCfg->getGOPSize();
//            depth    = 0;
//            for( Int i=step>>1; i>=1; i>>=1 )
//            {
//                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
//                {
//                    if ( j == poc )
//                    {
//                        i=0;
//                        break;
//                    }
//                }
//                step >>= 1;
//                depth++;
//            }
//        }
//        
//#if FIX_FIELD_DEPTH
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        if(poc != 0)
//        {
//#endif
//            if (isField && ((rpcSlice->getPOC() % 2) == 1))
//            {
//                depth ++;
//            }
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        }
//#endif
//#endif
//    }
//    
//
//    // slice type
//    SliceType eSliceType;
//    
//    eSliceType=B_SLICE;
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif
//    
//    rpcSlice->setSliceType    ( eSliceType );
//    
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // Non-referenced frame marking
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    if(pocLast == 0)
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(false);
//    }
//    else
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
//    }
//    rpcSlice->setReferenced(true);
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // QP setting
//    // ------------------------------------------------------------------------------------------------------------------
//    // ------------------------------------------------------------------------------------------------------------------
//    // Depth ADJUSMENT
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    
//    //    cout << "Indie QP modifyyyyy  "  << boolalpha << isSceneChange<< endl;
//    if (isSceneChange) {
//        
//        Int offset = m_pcOffsetCalc.getOffset(rpcSlice, isSmooth, isSceneChange, scState, lastSc, m_pcCfg->getQP());
//        Int qpFactIdx = m_pcOffsetCalc.getQpFactIdx();
//        Int depthIdx = m_pcOffsetCalc.getDepthIdx();
//        
//        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(offset);
//        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[qpFactIdx]);
//        depth = depths[depthIdx];
//    }
//    else // Concept I and other frames
//    {
//        
//        if( (rpcSlice->getPOC() != 0) && lastSc != -1)
//        {
//            // Concept I
//            if(rpcSlice->getPOC() == lastSc + 1)
//            {
//                Int offset = m_pcOffsetCalc.getOffset(rpcSlice, isSmooth, isSceneChange, scState, lastSc, m_pcCfg->getQP());
//                Int qpFactIdx = m_pcOffsetCalc.getQpFactIdx();
//                Int depthIdx = m_pcOffsetCalc.getDepthIdx();
//
//                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(offset);
//                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[qpFactIdx]);
//                depth = depths[depthIdx];
//                
//                if (isSmooth) {
//                    // caseNumber 4 = try the coming from Smooth =  2
//                    comingFromSmooth = 1;
//                }
//                else
//                {
//                    comingFromSmooth = 2;
//                }
//           
//                // Reset the track
//                track = 0;
//                
//                
//            }// end if(scState==4)
//            else
//            {
//                
//                //                cout << "COMING FROM SMOOTH: " << comingFromSmooth << endl;
//                if (comingFromSmooth == 2) {
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[1]);
//                    
//                    //                    depth = depths[1]; // 2
//                    depth = depths[0]; // 2
//                    
//                    //static int depths[] ={3, 2, 3, 1, 0};
//                    //                    static int depths[] ={2, 1, 2, 0};
//                    
//                    
//                }
//                else{
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
//                    depth = depths[track];
//                    
//                    
//                    //static int depths[] ={3, 2, 3, 1, 0};
//                    //                    static int depths[] ={2, 1, 2, 0};
//                    
//                }
//                
//                track = (track+1)%m_pcCfg->getGOPSize();
//                
//            }// end normal case
//            
//        }// end entering the normal case
//        
//    }// end adjusting the QP and lambda
//    
//    
//    
//    dQP = m_pcCfg->getQP();
//    
//    
//    
//    if(eSliceType!=I_SLICE)
//    {
//        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
//        {
//            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//        }
//    }
//    
//    // modify QP
//    // Hossam: Scene Change
//    Int* pdQPs = m_pcCfg->getdQPs();
//    if ( pdQPs)
//    {
//        dQP += pdQPs[ rpcSlice->getPOC() ];
//    }
//    
//    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
//    {
//        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
//        m_pcCfg->setDeltaQpRD(0);
//    }
//    
//    
//    // Hossam: Scene Change
//    if(isSceneChange || rpcSlice->getPOC() == lastSc + 1)
//    {
//        dQP = m_pcCfg->getQP() + m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//    }
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // Lambda computation
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    //     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    
//    Int iQP;
//    Double dOrigQP = dQP;
//    
//    // pre-compute lambda and QP values for all possible QP candidates
//    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
//    {
//        // compute QP value
//        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
//        
//        // compute lambda value
//        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
//        Int    SHIFT_QP = 12;
//        
//        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
//        
//        //         cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
//        
//#if FULL_NBIT
//        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//        Int    bitdepth_luma_qp_scale = 0;
//#endif
//        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
//#if FULL_NBIT
//        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
//#endif
//        
//        //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//        
//        // Hossam: Computing eSliceType I or P or B
//        // Case #1: I or P-slices (key-frame)
//        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
//        if ( eSliceType==I_SLICE )
//            //        if ( eSliceType==I_SLICE || (pocCurr == lastSc + 1 && isSmooth))
//        {
//            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQpFactor: " << dQPFactor << "\n" << endl;
//            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
//            
//            dQPFactor=0.57*dLambda_scale;
//        }
//        
////        cout << "\nPARAMS OF THE EQUATN: " << endl;
////        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
////        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp2222 " << qp_temp << endl;
////        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH depthhh2222 " << depth << endl;
//        
//        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
//        
//        //        cout << "\nCathy: TEncSlice:  FRAMEEEEEEeEEEEEEEE dQp: " << dQP << ", QpFact: " << dQPFactor << ", dLambda: " << dLambda << "\n" << endl;
////        cout << "\nCathy: TEncSlice: AFTER PARAMS LAMDA WITH dLambda " << dLambda << "\n" << endl;
//        //        cout << "m_pcCfg->getDeltaQpRD(): " << m_pcCfg->getDeltaQpRD() << "\n" << endl;// 0
//        
//        //      getchar();
//        
//        
//        if ( depth>0 )
//        {
//            
//            
//#if FULL_NBIT
//            
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#else
//            //            cout << depth << " INSIDE IF STATEMENT: "  << ", qp_temp: " << qp_temp << endl;
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#endif
//        }
//        
//        //        cout << "Before  !m_pcCfg->getUseHADME() : " << dLambda << endl;
//        // if hadamard is used in ME process
//        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
//        {
//            dLambda *= 0.95;
//        }
//        //        cout << "After  !m_pcCfg->getUseHADME() : " << dLambda << endl;
//        
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//        
//        m_pdRdPicLambda[iDQpIdx] = dLambda;
//        m_pdRdPicQp    [iDQpIdx] = dQP;
//        m_piRdPicQp    [iDQpIdx] = iQP;
//    }
//    
//    // obtain dQP = 0 case
//    dLambda = m_pdRdPicLambda[0];
//    dQP     = m_pdRdPicQp    [0];
//    iQP     = m_piRdPicQp    [0];
//    
//    //    cout << "Before  rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
//    if( rpcSlice->getSliceType( ) != I_SLICE )
//    {
//        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );// equals 0
//    }
//    
////    cout << "\nCathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda " << dLambda  << endl;
//    //     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH lambdaHelper " << lambdaHelper << endl;
//    //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    
//    setUpLambda(rpcSlice, dLambda, iQP);
//    
//    
//    //    cout << pocCurr << "Cathy: TEncSlice: ATTEMPT 2 MODIFYYY LAMDA WITH dLambda " << dLambda << "\n" << endl;
//    
//#if HB_LAMBDA_FOR_LDC
//    // restore original slice type
//    
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif // EFFICIENT_FIELD_IRAP
//    
//    
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
//    rpcSlice->setSliceType        ( eSliceType );
//#endif
//    
//    //*************************************************************************************
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
//    
//    // Hossam: Code Tweaking -- Second trial start
//    
//
//    if(isSceneChange)
//        //    if(isSceneChange && caseNumber != 4)
//    {
//        Int sc = rpcSlice->getPOC();
//        cout << "Yang: TEncSlice: initEncSlice: Forcing an CONCEPTUAL I frame XXXX " << sc << "\n" << endl;
//        rpcSlice->setSliceType(I_SLICE);
//        
//    }
//    // Hossam Code Tweaking -- Second trial end
//    //*************************************************************************************
//    
//    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
//    {
//        dQP = xGetQPValueAccordingToLambda( dLambda );
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//    }
//    
//    rpcSlice->setSliceQp           ( iQP );
//#if ADAPTIVE_QP_SELECTION
//    rpcSlice->setSliceQpBase       ( iQP );
//#endif
//    rpcSlice->setSliceQpDelta      ( 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
//    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    
//    if ( m_pcCfg->getDeblockingFilterMetric() )
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag(true);
//        rpcSlice->setDeblockingFilterDisable(false);
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
//    {
//        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
//        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
//        if ( !rpcSlice->getDeblockingFilterDisable())
//        {
//            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//            }
//            else
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//            }
//        }
//    }
//    else
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag( false );
//        rpcSlice->setDeblockingFilterDisable( false );
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    
//    rpcSlice->setDepth            ( depth );
//    
//    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    if(eSliceType==I_SLICE)
//    {
//        pcPic->setTLayer(0);
//    }
//    rpcSlice->setTLayer( pcPic->getTLayer() );
//    
//    assert( m_apcPicYuvPred );
//    assert( m_apcPicYuvResi );
//    
//    pcPic->setPicYuvPred( m_apcPicYuvPred );
//    pcPic->setPicYuvResi( m_apcPicYuvResi );
//    
//    // Hossam: Settting Slice Mode
//    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
//    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
//    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
//    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
//    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
//    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
//}


static int trackModifyQP1 = 0;
static int trackModifyQP2 = 0;

static int mock_trackModifyQP1 = 0;
static int mock_trackModifyQP2 = 0;
Void TEncSlice::modifyQP( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth)
{
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
    Int depth;
    Double dQPFactor;
    
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
    depth = rpcSlice->getDepth();
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        dQP += Qpoff[trackModifyQP1];
        trackModifyQP1 = (trackModifyQP1 + 1)% m_pcCfg->getGOPSize();
        
        dQPFactor = QpFact[trackModifyQP2];
        trackModifyQP2 = (trackModifyQP2 + 1)% m_pcCfg->getGOPSize();
        
    }
    
    // Mock: Set the mock for next time
    mock_trackModifyQP1 = trackModifyQP1;
    mock_trackModifyQP2 = trackModifyQP2;

    
//    if(rpcSlice -> getPOC() == 6 || rpcSlice -> getPOC() == 7)
//    {
//        switch (g_qpInit) {
//            case 22:
//                //                dQP = g_qpInit + 2;
//                //                dQPFactor = g_qpFact;
//                break;
//            case 27:
//                //                dQP = g_qpInit + 7;
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.12;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 37:
//                if (rpcSlice ->getPOC() == 6 ) {
//                    dQP = g_qpInit - 1;
//                    dQPFactor = g_qpFact;
//
//                }
//                else{
//                    dQP = g_qpInit + 3;
//                    dQPFactor = 0.11;
//
//                }
//                
////                dQPFactor = 0.11;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            default:
//                break;
//        }// end switch
//        
//        // Set the QP factor
////                dQPFactor = g_qpFact;
//
//    } // end if
//    else
//    if(rpcSlice -> getPOC() == 5)
//    {
//        switch (g_qpInit) {
//            case 22:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.60;
//                break;
//            case 27:
//                //                dQP = g_qpInit + 7;
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.40;
//                //                dQPFactor = 0.20;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                //                dQPFactor = 0.12;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 37:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                //                dQPFactor = 0.11;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            default:
//                break;
//        }// end switch
//        
//        // Set the QP factor
//        //        dQPFactor = g_qpFact;
//    } // end if
//    else
//    if (rpcSlice->getPOC () == 0) {
//        switch (g_qpInit) {
//            case 22:
//                dQP = g_qpInit;
//                break;
//            case 27:
////                dQP = g_qpInit - 2;
//                dQP = g_qpInit - 1;
//                break;
//            case 32:
//                
//                dQP = g_qpInit - 3;
//                break;
//            case 37:
//                
//                dQP = g_qpInit - 3;
//                break;
//            default:
//                break;
//        }
//    }
//    else if(rpcSlice -> getPOC() % 4 == 0)
//    {
//        switch (g_qpInit) {
//            case 22:
//                
//                dQP = g_qpInit + 5;
//                dQPFactor = 0.67;
//                break;
//            case 27:
//                
//                dQP = g_qpInit + 5;
//                dQPFactor = 0.60;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 1;
//                dQPFactor = 0.79;
//                break;
//            case 37:
//                
//                dQP = g_qpInit + 1;
//                dQPFactor = 0.78;
//                break;
//            default:
//                break;
//          // 0.578
////                dQPFactor = 0.578;
//                //                dQPFactor = 0.579;
//                //                dQPFactor = 0.580; // -0.007 dB, 0.11%
//                //                dQPFactor = 0.581; //
//          
//                
//                
//        }// end switch
//        
//
//
////         0.578
////        dQPFactor = g_qpFact;
//        
//    }
//    else if(rpcSlice -> getPOC() % 4 == 1)
//    {
//        switch (g_qpInit) {
//            case 22:
////                dQP = g_qpInit + 1;
//                break;
//            case 27:
////                dQP = g_qpInit + 7;
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.20;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.12;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 37:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.11;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            default:
//                break;
//        }// end switch
//        
//        // Set the QP factor
////        dQPFactor = g_qpFact;
//    } // end if
//    else if(rpcSlice -> getPOC() % 4 == 2)
//    {
//        switch (g_qpInit) {
//            case 22:
////                dQP = g_qpInit + 2;
////                dQPFactor = g_qpFact;
//                break;
//            case 27:
//                //                dQP = g_qpInit + 7;
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.12;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 37:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.11;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            default:
//                break;
//        }// end switch
//        
//        // Set the QP factor
////        dQPFactor = g_qpFact;
//    } // end if
//    else if(rpcSlice -> getPOC() % 4 == 3)
//    {
//        switch (g_qpInit) {
//            case 22:
////                dQP = g_qpInit + 3;
////                dQPFactor = g_qpFact;
//
//                break;
//            case 27:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.20;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.12;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            case 37:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.11;
//                cout << "Qp factor: " << dQPFactor << endl;
//                break;
//            default:
//                break;
//        }// end switch
//        
//        // Set the QP factor
////        dQPFactor = g_qpFact;
//    } // end if

// ***COMMENT: Change the predicted and reference frame
    // Hossam: test the predicted and reference frame
//    cout << "Changing the code to hardcoded predicted and reference frame effect [P-P case] initSliceNew " << endl;
//    UInt pred_poc = 2;
//    UInt start_poc = 14;
//    UInt end_poc = 16;
//    if(rpcSlice->getPOC() == pred_poc)
//    {
//        dQP = 32;
//    }
////    else if(rpcSlice->getPOC() == (pred_poc+start_poc)/2)
////    else if(rpcSlice->getPOC() == (start_poc+end_poc)/2)
//    else if(rpcSlice->getPOC() == 1)
//    {
//        dQP = g_qpRef;
//    }
//    
    // Change Delta QP
    // Empty
    
    ///
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        //        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Read it only for the I_Slice
        if (eSliceType == I_SLICE) {
            dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        }
        
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
    //        cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
    //    cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    //    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
    ////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
    
    
    // Empty
    //////////////////////////////////////////////////////////////////////////////////////////
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
    
#if SC_ENABLE_PRINT_TWO
    cout << "Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
//    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}


Void TEncSlice::modifyQPMock( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth)
{
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
    Int depth;
    Double dQPFactor;
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
    depth = rpcSlice->getDepth();
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        dQP += Qpoff[mock_trackModifyQP1];
        mock_trackModifyQP1 = (mock_trackModifyQP1 + 1)% m_pcCfg->getGOPSize();
        
        dQPFactor = QpFact[mock_trackModifyQP2];
        mock_trackModifyQP2 = (mock_trackModifyQP2 + 1)% m_pcCfg->getGOPSize();
    }
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        //        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Read it only for the I_Slice
        if (eSliceType == I_SLICE) {
            dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        }
        
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
    //        cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
    //    cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    //    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
    ////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
    
    
    // Empty
    //////////////////////////////////////////////////////////////////////////////////////////
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
#if SC_ENABLE_PRINT_TWO
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
    //    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}

// Calculate qp offsets
Void TEncSlice:: calculate4QPOffsets(TComPic* pcPic, std::vector<Double> epsilon_array, std::vector<Double> mu_ij, std::vector<Double> sigma_array)
{
//    cout << "Calculate the current 4 QP offsets for the current POC  " << pcPic->getPOC() << endl;
    
    
    // 1 2 3 4
//    static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};

    Double dQP_factor_for_zero = 0.4845;
    Double epsilon_zero = epsilon_array.front();
    
    cout << "Epsilon contains all eps from 0 to 4 " << endl;
    
    //    cout << "curr idx: " << curr_idx << " sigma sz: " << sigma_array.size() << ", eps size: " << epsilon_array.size() << endl;
//    Int gop_sz = 4;
    
    // Hossam: useless code
    Int curr_poc = pcPic->getPOC();
    
#if IS_ADAPTIVE_PROPAGATION_LENGTH
    for(Int i = 1; i < epsilon_array.size(); i++)
    {
        Double epsilon_i = epsilon_array.at(i);
        if(four_epsilons.size() >= 4)
        {
            
            cout << "xXXXXXXXX EEEEEEEE if(four_epsilons >= 4) Modify the start " << endl;
            four_epsilons.at(i-1) = epsilon_i;
        }
        else{
            four_epsilons.push_back(epsilon_i);
        }
        
#if IS_DUMP_EPSILON_I
        string fileName2 = "";
        std::ostringstream oss2;
        oss2 << "Gen//Seq-TXT//" << g_input_FileName << "_epsilon" << g_qpInit  << ".txt";
        fileName2 = oss2.str();
        Char* pYUVFileName2 = fileName2.empty()? NULL: strdup(fileName2.c_str());
        FILE* mse_pFile2 = fopen (pYUVFileName2, "at");
        fprintf(mse_pFile2, "%f\t\t\t\t \n", epsilon_i);
        fclose(mse_pFile2);
#endif
        
    }

#else
    
    for(Int i = 1; i < epsilon_array.size(); i++)
    {
        Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
        Double dQP_factor_for_i = QpFact[qp_factor_index];
        
        Double epsilon_i = epsilon_array.at(i);
        
        if(four_epsilons.size() >= 4)
        {
            
            cout << "xXXXXXXXX EEEEEEEE if(four_epsilons >= 4) Modify the start " << endl;
            four_epsilons.at(i-1) = epsilon_i;
        }
        else{
            four_epsilons.push_back(epsilon_i);
        }
        
        // Hossam: Useless code
        // >>>>>
        // method is turned off
        if(epsilon_i < 1)
        {
            Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
            current_qp_offset = Qpoff[qp_factor_index];
        }
        else{
            
            Double up = epsilon_zero * dQP_factor_for_zero;
            Double down = epsilon_i * dQP_factor_for_i;
            
            Double deltaQP_i = 3*log2(up/down);
            
            current_qp_offset = round(deltaQP_i);
            
            if(deltaQP_i < 0)
            {
                Double pos_delta = deltaQP_i * -1;
                current_qp_offset = -1*round(pos_delta);
            }
        }
        
        four_offsets.push_back(current_qp_offset);
        // <<<<<<
        
//        cout << "Zero_qp_fact " << dQP_factor_for_zero << " eps_zero: " << epsilon_zero << ", " << " i_qp_fact: " << dQP_factor_for_i << " eps_i: " <<  epsilon_i << endl;
//        cout << "Zero_Up parameter: " << up << " down parameter: " << down << endl;
//        cout << "\n(epsilon_zero/epsilon_i): " << (epsilon_zero/epsilon_i) << endl;
//        cout << "\n ******QP offset for frame-" << (pcPic->getPOC() + i - 1) << ", " << deltaQP_i << ", " << current_qp_offset  << "\n" << endl;
        
        // calculate
//        Double cur_lambda = lambdaArr3[qp_factor_index]/epsilon_i;
//        cout << (pcPic->getPOC() + i - 1) << ") cur_lambda " << cur_lambda << " old lambda " << lambdaArr3[qp_factor_index] << endl;
//        Double desired_qp = 3 * log2(cur_lambda/dQP_factor_for_i) + 12;
//        cout << (pcPic->getPOC() + i - 1)  << ") desired_qp: " << desired_qp  << ", fourEps " << four_epsilons.front() << endl;
        
        
        // increment the current poc
        curr_poc++;
    }
#endif
    
} // end calculate4QPOffsets




Void TEncSlice::modifyQPActual( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth)
{
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
    Int depth;
    Double dQPFactor;
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
    depth = rpcSlice->getDepth();
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        // Hossam: look ahead take from the current_qp_offset
        dQP += Qpoff[trackModifyQP1];
//        dQP += current_qp_offset;
        trackModifyQP1 = (trackModifyQP1 + 1)% m_pcCfg->getGOPSize();
        
        dQPFactor = QpFact[trackModifyQP2];
        trackModifyQP2 = (trackModifyQP2 + 1)% m_pcCfg->getGOPSize();
        
    }
    
    
    
    // Mock: Set the mock for next time
    mock_trackModifyQP1 = trackModifyQP1;
    mock_trackModifyQP2 = trackModifyQP2;
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        //        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Read it only for the I_Slice
        if (eSliceType == I_SLICE) {
            dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        }
        
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
    //        cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
    //    cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    //    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
    ////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
//    if( rpcSlice->getSliceType( ) != I_SLICE )
//    {
//        dLambda = dLambda / current_epsilon;
//    }
    // Empty
    //////////////////////////////////////////////////////////////////////////////////////////
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
#if SC_ENABLE_PRINT_TWO
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
    //    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}

// Hossam: gets the qp factor inter
Double  TEncSlice::getQPFactInterLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.5;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.4;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    return qpFactor;
}

Double  TEncSlice::getQPFactInter1stepLDB  ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.4;
                break;
            case 32:
                qpFactor = 0.4;
                break;
            case 37:
                qpFactor = 0.4;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
        
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.8;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.4;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.8;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
        
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
        
    }
    
    return qpFactor;
}


// Hossam: gets the qp factor inter
Double  TEncSlice::getQPFactInterMu       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.5;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.9;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    
    return qpFactor;
}

Double  TEncSlice::getQPFactInterMu1step  ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.4;
                break;
            case 32:
                qpFactor = 0.4;
                break;
            case 37:
                qpFactor = 0.4;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
        
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.8;
                break;
            case 37:
                qpFactor = 0.9;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    
    return qpFactor;
}


// Hossam: gets the qp factor inter
Double  TEncSlice::getQPFactInterMuLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.5;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.8;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    
    return qpFactor;

}

Double  TEncSlice::getQPFactInterMu1stepLDB  ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.4;
                break;
            case 32:
                qpFactor = 0.4;
                break;
            case 37:
                qpFactor = 0.4;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.9;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    
    return qpFactor;
}




// Hossam: gets the qp factor inter
Double  TEncSlice::getQPFactInter       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    
    UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
    Int cur_poc = relativePOC;
//    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
               qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.5;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }

        
    }
    
    return qpFactor;
}

Double  TEncSlice::getQPFactInter1step  ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
    Int cur_poc = relativePOC;

//    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.4;
                break;
            case 37:
                qpFactor = 0.5;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.7;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.9;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.8;
                break;
            case 32:
                qpFactor = 0.6;
                break;
            case 37:
                qpFactor = 0.8;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
        
    }
    
    return qpFactor;
}


Double    getQPFactInterMuLDB       ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    Int cur_poc = rpcSlice->getPOC();
    Int gop_sz = 4;
    Double qpFactor;
    
    if(cur_poc % gop_sz == 0)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.5;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.5;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 1)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else if(cur_poc % gop_sz == 2)
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.4;
                break;
            case 27:
                qpFactor = 0.5;
                break;
            case 32:
                qpFactor = 0.5;
                break;
            case 37:
                qpFactor = 0.6;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    else
    {
        switch (g_qpInit) {
            case 22:
                qpFactor = 0.6;
                break;
            case 27:
                qpFactor = 0.6;
                break;
            case 32:
                qpFactor = 0.7;
                break;
            case 37:
                qpFactor = 0.7;
                break;
                
            default:
                Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
                qpFactor = QpFact[qp_factor_index];
                break;
        }
        
    }
    
    return qpFactor;

}


// March 12, gets QP factor
Double  TEncSlice::getQPFactLDDefault  ( TComPic*  pcPic, Int iGOPid,   TComSlice*& rpcSlice, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
    Int cur_poc = relativePOC;
    Int qp_factor_index = cur_poc % 4 == 0? 3: cur_poc % 4 - 1;
    Double qpFactor = QpFact[qp_factor_index];
    
    // rpoc 1
//    if(cur_poc % 4 == 1)
//    {
//        qpFactor = g_qpFact;
//        cout << "New QP Factor " << qpFactor << " for " << rpcSlice->getPOC() << endl;
//    }
    
    return qpFactor;
}


//static int QpoffInter0[] ={1, 3, 5, 7};
///////// P frames ////

#if IS_LDP

#if IS_EPSILON_PRED_METHOD

#if IS_ENABLE_ACTUAL_LD
//static int QpoffInter0[] = {3, 5, 5, 7};
static int QpoffInter0[] = {6, 6, 6, 6};
#else
static int QpoffInter0[] = {5, 5, 5, 5};
#endif

#else // mu method

#if IS_ENABLE_ACTUAL_LD
//static int QpoffInter0[] = {7, 7, 7, 7};
//static int QpoffInter0[] = {8, 8, 8, 8};

// Yang offset
// static int QpoffInter0[] = {0, 5, 5, 5};

// Yang offset conference:
//static int QpoffInter0[] = {3, 5, 5, 5};
static int QpoffInter0[] = {0, 0, 0, 0};

// bqTerrace offset
//static int QpoffInter0[] = {1, 7, 7, 7};



#else
static int QpoffInter0[] = {3, 5, 5, 5};
#endif

#endif

///////// P frames ////


#else

///////// B frames ////
#if IS_EPSILON_PRED_METHOD

#if IS_ENABLE_ACTUAL_LD
//static int QpoffInter0[] = {5, 5, 7, 7};
//static int QpoffInter0[] = {6, 6, 6, 6};

// Yang offset
static int QpoffInter0[] = {0, 5, 5, 5};

#else
static int QpoffInter0[] = {5, 5, 5, 5};
#endif

#else // mu method

// not done yet (i think that i copied what happened in epsilon)
#if IS_ENABLE_ACTUAL_LD
//static int QpoffInter0[] = {5, 5, 5, 7};
//static int QpoffInter0[] = {5, 5, 7, 7};


// Yang offset
static int QpoffInter0[] = {0, 5, 5, 5};

#else
static int QpoffInter0[] = {5, 5, 5, 5};
#endif

#endif

//////////////////////

#endif // endif LDP

Void TEncSlice::modifyQPInter( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
//    Int depth;
    Double dQPFactor;
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
    

    
    // Hossam: depth computation based on GOP size and SC existence -- this is inserted here for SCD computation
    Int depth;
    {
#if FIX_FIELD_DEPTH
        
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        Int poc = relativePOC;
//        Int poc = rpcSlice->getPOC();
        if(isField)
        {
            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
        }
        else
        {
            poc = poc % m_pcCfg->getGOPSize();
        }
#else
        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
        
        if ( poc == 0 )
        {
            depth = 0;
        }
        else
        {
            Int step = m_pcCfg->getGOPSize();
            depth    = 0;
            for( Int i=step>>1; i>=1; i>>=1 )
            {
                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
                {
                    if ( j == poc )
                    {
                        i=0;
                        break;
                    }
                }
                step >>= 1;
                depth++;
            }
        }
     
// Hossam: do this cause apparently that I-SLICE should not have the same lambda, as the first one
// For testing
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD || !IS_INTER_DEP_WITH_SC
        depth = rpcSlice->getDepth();
#endif
        
#if FIX_FIELD_DEPTH
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        if(poc != 0)
        {
#endif
            if (isField && ((rpcSlice->getPOC() % 2) == 1))
            {
                depth ++;
            }
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        }
#endif
#endif
    }
    ///--- End depth computation
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        
        
        // Hossam: I think that it is better to use the qp_factor_index to locate the correct offset
        // static counters screw things up and is a bad habit
        // Hossam: look ahead take from the current_qp_offset
        // Hossam: do this cause apparently that I-SLICE should not have the same lambda, as the first one
#if IS_STUDENT_Energy_SCD || IS_STUDENT_SCD
        dQP += Qpoff[trackModifyQP1];
#else
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        Int curr_poc = relativePOC;
        Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;

        dQP += Qpoff[qp_factor_index];
        
        // Change 1
//        dQP = Qpoff[0];
#endif
        
//        dQP += Qpoff[trackModifyQP1];
        //        dQP += current_qp_offset;
        trackModifyQP1 = (trackModifyQP1 + 1)% m_pcCfg->getGOPSize();

        
#if IS_LDP
        
#if IS_ENABLE_ACTUAL_LD

#if IS_EPSILON_PRED_METHOD
        // actual LD:
        dQPFactor = getQPFactInter(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method

        // actual LD:
        dQPFactor = getQPFactInterMu(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
        
#else // 1step case
        
#if IS_EPSILON_PRED_METHOD
        // 1step LD:
        dQPFactor = getQPFactInter1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
        
#else // mu method
        // 1step LD:
        dQPFactor = getQPFactInterMu1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
        
        
#endif // end if actual ld or one step

#else // LDB
        
#if IS_ENABLE_ACTUAL_LD
        
#if IS_EPSILON_PRED_METHOD
        // actual LD:
        dQPFactor = getQPFactInterLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
        
        // actual LD: // not done yet
        dQPFactor = getQPFactInterMuLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
        
#else // 1step case
        
#if IS_EPSILON_PRED_METHOD
        // 1step LDB:
        dQPFactor = getQPFactInter1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
        
#else // mu method
        // 1step LDB: // not done yet
        dQPFactor = getQPFactInterMu1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
        
        
#endif // end if actual ld or one step
        
        
#endif

        
//        if(pcPic->getPOC() % 4 == 1)
//        {
//            dQPFactor = g_qpFact;
//        }
//        else {
//                dQPFactor = QpFact[trackModifyQP2];
//        }
        
        
#if USE_DEFAULT_QP_FACTORS
        // Change 3
        dQPFactor = getQPFactLDDefault(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif


        
        trackModifyQP2 = (trackModifyQP2 + 1)% m_pcCfg->getGOPSize();
    }
   
    if (eSliceType == I_SLICE) {
        switch (g_qpInit) {
            case 22:
                dQP = g_qpInit - QpoffInter0[0];
                break;
            case 27:
                dQP = g_qpInit - QpoffInter0[1];
                break;
            case 32:
                dQP = g_qpInit - QpoffInter0[2];
                break;
            case 37:
                dQP = g_qpInit - QpoffInter0[3];
                break;
            default:
                break;
        }
        
        
        dQP = g_qpInit - g_qpRef;
    }
    
//    else if(rpcSlice -> getPOC() == 5)
//    {
//        switch (g_qpInit) {
//            case 22:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.60;
//                break;
//            case 27:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.40;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                break;
//            case 37:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                break;
//            default:
//                break;
//        }// end switch
//        
//    } // end if
//    else if(rpcSlice -> getPOC() % 4 == 0)
//    {
//        switch (g_qpInit) {
//            case 22:
//                dQP = g_qpInit + 5;
//                dQPFactor = 0.67;
//                break;
//            case 27:
//                
//                dQP = g_qpInit + 5;
//                dQPFactor = 0.60;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 1;
//                dQPFactor = 0.79;
//                break;
//            case 37:
//                
//                dQP = g_qpInit + 1;
//                dQPFactor = 0.78;
//                break;
//            default:
//                break;
//        }// end switch
//    }
//    else if(rpcSlice -> getPOC() % 4 == 1)
//    {
//        switch (g_qpInit) {
//            case 22:
//                break;
//            case 27:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.20;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.12;
//                break;
//            case 37:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.11;
//                break;
//            default:
//                break;
//        }// end switch
//        
//        // Set the QP factor
//        //        dQPFactor = g_qpFact;
//    } // end if
//    else if(rpcSlice -> getPOC() % 4 == 2)
//    {
//        switch (g_qpInit) {
//            case 22:
//                break;
//            case 27:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.20;
//                break;
//            case 32:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.12;
//                break;
//            case 37:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.11;
//                break;
//            default:
//                break;
//        }// end switch
//        
//    } // end if
//    else if(rpcSlice -> getPOC() % 4 == 3)
//    {
//        switch (g_qpInit) {
//            case 22:
//                break;
//            case 27:
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.20;
//                break;
//            case 32:
//                
//                dQP = g_qpInit + 3;
//                dQPFactor = 0.12;
//                break;
//            case 37:
//                dQP = g_qpInit + 2;
//                dQPFactor = 0.11;
//                break;
//            default:
//                break;
//        }// end switch
//    } // end if
//
    
    
    // Mock: Set the mock for next time
    mock_trackModifyQP1 = trackModifyQP1;
    mock_trackModifyQP2 = trackModifyQP2;
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        //        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Read it only for the I_Slice
        if (eSliceType == I_SLICE) {
            dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        }
        
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQP " << dQP << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
//                cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
//        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY ADJUSTTTTTT WITH dLambda " << dLambda << endl;
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
        
        
//        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH dLambda " << dLambda << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH m_pdRdPicLambda " << m_pdRdPicLambda[iDQpIdx] << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH iDQpIdx " << iDQpIdx << endl;
    }
    
    
//    cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH m_pdRdPicLambda " << m_pdRdPicLambda[0] << endl;
//    cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH iDQpIdx " << dLambda << endl;
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
//            cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
//        cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    //    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
    ////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
        if( rpcSlice->getSliceType( ) != I_SLICE )
        {
            UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
//            if(rpcSlice->getPOC() >= end_prop_window + 1)
            if(relativePOC >= end_prop_window + 1)
            {
//                Int curr_poc = rpcSlice->getPOC();
                Int curr_poc = relativePOC;
                Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
                Double dQP_factor_for_i;
                
#if IS_LDP
                
#if IS_ENABLE_ACTUAL_LD
                
#if IS_EPSILON_PRED_METHOD
                // actual LD:
                dQP_factor_for_i = getQPFactInter(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
                
                // actual LD:
                dQP_factor_for_i = getQPFactInterMu(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
                
#else // 1step case
                
#if IS_EPSILON_PRED_METHOD
                // 1step LD:
                dQP_factor_for_i = getQPFactInter1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
                
#else // mu method
                // 1step LD:
                dQP_factor_for_i = getQPFactInterMu1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
                
                
#endif // end if actual ld or one step
                
#else // LDB
                
#if IS_ENABLE_ACTUAL_LD
                
#if IS_EPSILON_PRED_METHOD
                // actual LD:
                dQP_factor_for_i = getQPFactInterLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
                
                // actual LD: // not done yet
                dQP_factor_for_i = getQPFactInterMuLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
                
#else // 1step case
                
#if IS_EPSILON_PRED_METHOD
                // 1step LDB:
                dQP_factor_for_i = getQPFactInter1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
                
#else // mu method
                // 1step LDB: // not done yet
                dQP_factor_for_i = getQPFactInterMu1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
                
                
#endif // end if actual ld or one step
                
                
#endif
                
#if USE_DEFAULT_QP_FACTORS
                // Change 3
                dQPFactor = getQPFactLDDefault(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif

                
                // calculate
                Int eps_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
                
                cout << "(ModifyQPInter): EPS index is curr mod 4 - 1 cause I don't do 8 XXXXXX " << eps_index << ", " << four_epsilons.size() << endl;
                Double epsilon_i = four_epsilons.at(eps_index);
                
                Double cur_lambda;
                Double old_lambda;
                switch (g_qpInit) {
                    case 22:
                        old_lambda = lambdaArr[qp_factor_index];
                        break;
                    case 27:
                        old_lambda = lambdaArr2[qp_factor_index];
                        break;
                    case 32:
                        old_lambda = lambdaArr3[qp_factor_index];
                        break;
                    case 37:
                        old_lambda = lambdaArr4[qp_factor_index];
                        break;
                    default:
                        old_lambda = lambdaArr3[qp_factor_index];
                        break;
                }
                 //// qp factor
                
                // Method is truned on or off
                if(epsilon_i < 1)
                {
//                    int x = curr_poc % 4 == 0? 0:+5;
//                    current_qp_offset = Qpoff[qp_factor_index] + x;
                    current_qp_offset = Qpoff[qp_factor_index];
                    iQP = g_qpInit + current_qp_offset;
                    dLambda = old_lambda; // has 5 elements not 4
                    
//                    int x = curr_poc % 4 == 0? 1:+2;
//                    dLambda = old_lambda*x;
//                    iQP = round(3 * log2(dLambda/QpFact[qp_factor_index]) + 12);
                    //                 cout << "\n\n Will enter special case for ever \n\n " << endl;
                }
                else{
                    
                    cur_lambda = old_lambda/epsilon_i;
//                    cout << (pcPic->getPOC()) << ") cur_lambda " << cur_lambda << " old lambda " << old_lambda << " old qp " << iQP << "  qpFactInter: " << dQP_factor_for_i << ", epsilon_i: " << epsilon_i  << endl;
                    Double desired_qp = 3 * log2(cur_lambda/dQP_factor_for_i) + 12;
                    //                Double desired_qp = 4.20005 * log2(cur_lambda/dQP_factor_for_i) + 13.7122;
                    
                    //                cout << (pcPic->getPOC())  << ") desired_qp: " << desired_qp << endl;
                    iQP = round(desired_qp);
                    dLambda = cur_lambda;
                }

#if GEN_QPOffset_Files
                string fileName_qp = "";
                std::ostringstream oss_qp;
                Char* pYUVFileName_qp;
                FILE* my_pFile_qp;
                Int offset = iQP - g_qpInit;
                oss_qp << "Gen//Seq-TXT//" << g_input_FileName << "_offset" << ".txt";
                fileName_qp = oss_qp.str();
                pYUVFileName_qp = fileName_qp.empty()? NULL: strdup(fileName_qp.c_str());
                my_pFile_qp = fopen (pYUVFileName_qp, "at");
                fprintf(my_pFile_qp, "%d\t\t\t\t %d\t\t\t\t %f\t\t\t\t %f\t\t\t\t %f\n", curr_poc, offset, epsilon_i, old_lambda, cur_lambda);
                fclose(my_pFile_qp);
#endif 
            } // end if: modify after you reach the point you can modify

        }
    
//    cout << "RELATIVE POCCCC " << (lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc) << endl;
//    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda << ", qp: " << iQP  <<  ", qpFactor: " << dQPFactor << endl;
    // Empty
    //////////////////////////////////////////////////////////////////////////////////////////
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
#if SC_ENABLE_PRINT_TWO
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
    //    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}


// March 12: Conservative QP inter
Void TEncSlice::modifyQPInterConservative( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window, Bool isEpsilonMethodON)
{
    
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
    //    Int depth;
    Double dQPFactor;
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
    
    
    
    // Hossam: depth computation based on GOP size and SC existence -- this is inserted here for SCD computation
    Int depth;
    {
#if FIX_FIELD_DEPTH
        
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        Int poc = relativePOC;
        //        Int poc = rpcSlice->getPOC();
        if(isField)
        {
            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
        }
        else
        {
            poc = poc % m_pcCfg->getGOPSize();
        }
#else
        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
        
        if ( poc == 0 )
        {
            depth = 0;
        }
        else
        {
            Int step = m_pcCfg->getGOPSize();
            depth    = 0;
            for( Int i=step>>1; i>=1; i>>=1 )
            {
                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
                {
                    if ( j == poc )
                    {
                        i=0;
                        break;
                    }
                }
                step >>= 1;
                depth++;
            }
        }
        
        // Hossam: do this cause apparently that I-SLICE should not have the same lambda, as the first one
        // For testing
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD || !IS_INTER_DEP_WITH_SC
        depth = rpcSlice->getDepth();
#endif
        
#if FIX_FIELD_DEPTH
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        if(poc != 0)
        {
#endif
            if (isField && ((rpcSlice->getPOC() % 2) == 1))
            {
                depth ++;
            }
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        }
#endif
#endif
    }
    ///--- End depth computation
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        
        
// Hossam: I think that it is better to use the qp_factor_index to locate the correct offset
// static counters screw things up and is a bad habit
// Hossam: look ahead take from the current_qp_offset
// Hossam: do this cause apparently that I-SLICE should not have the same lambda, as the first one
#if IS_STUDENT_Energy_SCD || IS_STUDENT_SCD
        dQP += Qpoff[trackModifyQP1];
#else
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        Int curr_poc = relativePOC;
        Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
        
        dQP += Qpoff[qp_factor_index];
#endif
        
        //        dQP += Qpoff[trackModifyQP1];
        //        dQP += current_qp_offset;
        trackModifyQP1 = (trackModifyQP1 + 1)% m_pcCfg->getGOPSize();
        
#if IS_LDP
        
#if IS_ENABLE_ACTUAL_LD
        
#if IS_EPSILON_PRED_METHOD
        // actual LD:
        dQPFactor = getQPFactInter(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
        
        // actual LD:
        dQPFactor = getQPFactInterMu(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
        
#else // 1step case
        
#if IS_EPSILON_PRED_METHOD
        // 1step LD:
        dQPFactor = getQPFactInter1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
        
#else // mu method
        // 1step LD:
        dQPFactor = getQPFactInterMu1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
        
        
#endif // end if actual ld or one step
        
#else // LDB
        
#if IS_ENABLE_ACTUAL_LD
        
#if IS_EPSILON_PRED_METHOD
        // actual LD:
        dQPFactor = getQPFactInterLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
        
        // actual LD: // not done yet
        dQPFactor = getQPFactInterMuLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
        
#else // 1step case
        
#if IS_EPSILON_PRED_METHOD
        // 1step LDB:
        dQPFactor = getQPFactInter1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
        
#else // mu method
        // 1step LDB: // not done yet
        dQPFactor = getQPFactInterMu1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
        
        
#endif // end if actual ld or one step
        
        
#endif

        // Conservative Change: For frames not yet in the window, take the default QP factor
        // Hossam: ignore the above code -- get the QP factor default for P frames -- be conservative
        // XXXX This thing you need to make sure of -- QP 22 and 27 or all.
//        if(g_qpInit == 22 || g_qpInit == 27)
//        {
            dQPFactor = getQPFactLDDefault(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);

//        }
        
        trackModifyQP2 = (trackModifyQP2 + 1)% m_pcCfg->getGOPSize();
    }
    
    if (eSliceType == I_SLICE) {
        switch (g_qpInit) {
            case 22:
                dQP = g_qpInit - QpoffInter0[0];
                break;
            case 27:
                dQP = g_qpInit - QpoffInter0[1];
                break;
            case 32:
                dQP = g_qpInit - QpoffInter0[2];
                break;
            case 37:
                dQP = g_qpInit - QpoffInter0[3];
                break;
            default:
                break;
        }
        
        // Conservative Change: For frames not yet in the window, take the default QP factor
        // Hossam: ignore the above code -- get the QP factor default for P frames -- be conservative
        // XXXX This thing you need to make sure of -- QP 22 and 27 or all.
        dQP = g_qpInit;
    }
    
    
    
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        //        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Read it only for the I_Slice
        if (eSliceType == I_SLICE) {
            dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        }
        
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
        //                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
        //                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQP " << dQP << endl;
        //                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
        //                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //                cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        //        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY ADJUSTTTTTT WITH dLambda " << dLambda << endl;
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
        
        
        //        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH dLambda " << dLambda << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH m_pdRdPicLambda " << m_pdRdPicLambda[iDQpIdx] << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH iDQpIdx " << iDQpIdx << endl;
    }
    
    
    //    cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH m_pdRdPicLambda " << m_pdRdPicLambda[0] << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT SHERRRY222 ADJUSTTTTTT WITH iDQpIdx " << dLambda << endl;
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
    //            cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
    //        cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    //    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
    ////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        //            if(rpcSlice->getPOC() >= end_prop_window + 1)
        if(relativePOC >= end_prop_window + 1)
        {
            
            //                Int curr_poc = rpcSlice->getPOC();
            Int curr_poc = relativePOC;
            Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
            Double dQP_factor_for_i;
            
#if IS_LDP
            
#if IS_ENABLE_ACTUAL_LD
            
#if IS_EPSILON_PRED_METHOD
            // actual LD:
            dQP_factor_for_i = getQPFactInter(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
            
            // actual LD:
            dQP_factor_for_i = getQPFactInterMu(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
            
#else // 1step case
            
#if IS_EPSILON_PRED_METHOD
            // 1step LD:
            dQP_factor_for_i = getQPFactInter1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
            
#else // mu method
            // 1step LD:
            dQP_factor_for_i = getQPFactInterMu1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
            
            
#endif // end if actual ld or one step
            
#else // LDB
            
#if IS_ENABLE_ACTUAL_LD
            
#if IS_EPSILON_PRED_METHOD
            // actual LD:
            dQP_factor_for_i = getQPFactInterLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#else // mu method
            
            // actual LD: // not done yet
            dQP_factor_for_i = getQPFactInterMuLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // prediction method if
            
#else // 1step case
            
#if IS_EPSILON_PRED_METHOD
            // 1step LDB:
            dQP_factor_for_i = getQPFactInter1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
            
#else // mu method
            // 1step LDB: // not done yet
            dQP_factor_for_i = getQPFactInterMu1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
#endif // end if prediction method
            
            
#endif // end if actual ld or one step
            
            
#endif
            // Conservative Change: For frames not yet in the window, take the default QP factor
            // Hossam: ignore the above code -- get the QP factor default for P frames -- be conservative
            // XXXX This thing you need to make sure of -- QP 22 and 27 or all.
            dQP_factor_for_i = getQPFactLDDefault(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window, end_prop_window);
            
            // calculate
            Int eps_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
            
            cout << "(ModifyQPInter): EPS index is curr mod 4 - 1 cause I don't do 8 XXXXXX " << eps_index << ", " << four_epsilons.size() << endl;
            Double epsilon_i = four_epsilons.at(eps_index);
            
            Double cur_lambda;
            Double old_lambda;
            switch (g_qpInit) {
                case 22:
                    old_lambda = lambdaArr[qp_factor_index];
                    break;
                case 27:
                    old_lambda = lambdaArr2[qp_factor_index];
                    break;
                case 32:
                    old_lambda = lambdaArr3[qp_factor_index];
                    break;
                case 37:
                    old_lambda = lambdaArr4[qp_factor_index];
                    break;
                default:
                    old_lambda = lambdaArr3[qp_factor_index];
                    break;
            }
            //// qp factor in the start of the window
//            UInt last_frame_encoded = rpcSlice->getPOC();
//            UInt start_for_i = lastSc==-1? 1:lastSc+1;
//            // if you did not go beyond the checking point, still do not change anything
//            if((g_qpInit == 22 || g_qpInit == 27) && last_frame_encoded < start_for_i + 24)
//            {
//                current_qp_offset = Qpoff[qp_factor_index];
//                iQP = g_qpInit + current_qp_offset;
//                dLambda = old_lambda;
//                
//                cout << "HELLO CASE PR 1" << endl;
//            }
//            // Method is turned on or off
//            else
            if(epsilon_i < 1)
            {
                //                    int x = curr_poc % 4 == 0? 0:+5;
                //                    current_qp_offset = Qpoff[qp_factor_index] + x;
                current_qp_offset = Qpoff[qp_factor_index];
                iQP = g_qpInit + current_qp_offset;
                dLambda = old_lambda; // has 5 elements not 4
                
                //                    int x = curr_poc % 4 == 0? 1:+2;
                //                    dLambda = old_lambda*x;
                //                    iQP = round(3 * log2(dLambda/QpFact[qp_factor_index]) + 12);
                //                 cout << "\n\n Will enter special case for ever \n\n " << endl;
                
                cout << "HELLO CASE PR 2" << endl;
            }
            else{
                
                cout << "HELLO CASE PR 3" << endl;
                cur_lambda = old_lambda/epsilon_i;
                cout << (pcPic->getPOC()) << ") cur_lambda " << cur_lambda << " old lambda " << old_lambda << " old qp " << iQP << "  qpFactInter: " << dQP_factor_for_i << ", epsilon_i: " << epsilon_i  << endl;
                
                
                Double desired_qp = 3 * log2(cur_lambda/dQP_factor_for_i) + 12;
                
                
                //    Double desired_qp = 4.20005 * log2(cur_lambda/dQP_factor_for_i) + 13.7122;
                
                cout << (pcPic->getPOC())  << ") desired_qp: " << desired_qp << endl;
                iQP = round(desired_qp);
                dLambda = cur_lambda;
            }
            
#if GEN_QPOffset_Files
            string fileName_qp = "";
            std::ostringstream oss_qp;
            Char* pYUVFileName_qp;
            FILE* my_pFile_qp;
            Int offset = iQP - g_qpInit;
            oss_qp << "Gen//Seq-TXT//" << g_input_FileName << "_offset" << ".txt";
            fileName_qp = oss_qp.str();
            pYUVFileName_qp = fileName_qp.empty()? NULL: strdup(fileName_qp.c_str());
            my_pFile_qp = fopen (pYUVFileName_qp, "at");
            fprintf(my_pFile_qp, "%d\t\t\t\t %d\t\t\t\t %f\t\t\t\t %f\t\t\t\t %f\n", curr_poc, offset, epsilon_i, old_lambda, cur_lambda);
            fclose(my_pFile_qp);
#endif
        } // end if: modify after you reach the point you can modify
        
    }
    
    //    cout << "RELATIVE POCCCC " << (lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc) << endl;
    //    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda << ", qp: " << iQP  <<  ", qpFactor: " << dQPFactor << endl;
    // Empty
    //////////////////////////////////////////////////////////////////////////////////////////
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
#if SC_ENABLE_PRINT_TWO
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
    //    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}



Void TEncSlice::setMockQPForSCOperation( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
    Int depth;
    Double dQPFactor;
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
    depth = rpcSlice->getDepth();
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        
        // Hossam: look ahead take from the current_qp_offset
        // Hossam: Scene change: let the SCD Engine operate at QP = 27; // 22 for now
        dQP = SC_QP_ORG;
        dQPFactor = QpFact[3];
    }
    
    // Mock: Set the mock for next time
    mock_trackModifyQP1 = trackModifyQP1;
    mock_trackModifyQP2 = trackModifyQP2;
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif

        
        //        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
    // Hossam: Scene Change ORG -- PICK THE ONE FOR QP @ 22
    dLambda = lambdaArr[4];
    setUpLambda(rpcSlice, dLambda, iQP);
    
    
   // cout << rpcSlice->getPOC() << ") MOCK lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#if SC_ENABLE_PRINT_TWO
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
    //    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}


Void TEncSlice::modifyQPInterMu( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window, Int end_prop_window)
{
    
    
    Double dQP;
    Double dLambda;
    SliceType eSliceType;
//    Int depth;
    Double dQPFactor;
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    eSliceType = rpcSlice->getSliceType();
//    depth = rpcSlice->getDepth(); // XXX might recompute the depth here
    
    
    // Hossam: depth computation based on GOP size and SC existence -- this is inserted here for SCD computation
    Int depth;
    {
#if FIX_FIELD_DEPTH
        
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        Int poc = relativePOC;
        //        Int poc = rpcSlice->getPOC();
        if(isField)
        {
            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
        }
        else
        {
            poc = poc % m_pcCfg->getGOPSize();
        }
#else
        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
        
        if ( poc == 0 )
        {
            depth = 0;
        }
        else
        {
            Int step = m_pcCfg->getGOPSize();
            depth    = 0;
            for( Int i=step>>1; i>=1; i>>=1 )
            {
                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
                {
                    if ( j == poc )
                    {
                        i=0;
                        break;
                    }
                }
                step >>= 1;
                depth++;
            }
        }
    } // end depth computation
    
// Hossam: do this cause apparently that I-SLICE should not have the same lambda, as the first one
// For testing in my SCDs and if I don't run SCD at all
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD || !IS_INTER_DEP_WITH_SC
        depth = rpcSlice->getDepth();
#endif
    
    
    dQP = m_pcCfg->getQP();
    
    if(eSliceType != I_SLICE)
    {
        // Hossam: look ahead take from the current_qp_offset
        dQP += current_qp_offset;
        dQPFactor = current_qp_factor_mu;
        
    }
    // I_slice or scene change
    else
    {
#if RUN_DEFAULT_QP_PROP_EXPERIMENT
        dQP += 0;
#else
        dQP += current_qp_offset;
#endif
        
        dQP = g_qpInit + current_qp_offset;
    }
    
    
    // ***COMMENT: Change the predicted and reference frame
    // Hossam: test the predicted and reference frame
//    cout << "Changing the code to hardcoded predicted and reference frame effect [P-P case] initSliceNew " << endl;
//    static int off_ref [] = {-5, -2, -3, -4};
//    Int test_pred_poc = 17;
//    Int fetch_index = test_pred_poc % 4 == 0? 3: test_pred_poc % 4 - 1;
//    Int test_ref1_poc = test_pred_poc - 1; Int test_ref2_poc = test_pred_poc + off_ref[fetch_index];
//    
////    cout << "Pred: " << test_pred_poc << ", Ref1: " << test_ref1_poc << ", Ref2: " << test_ref2_poc << endl;
//    if(rpcSlice->getPOC() == test_pred_poc)
//    {
//        dQP = 32;
//    }
//    else if (rpcSlice->getPOC() == test_ref1_poc)
//    {
//        dQP = g_qpRef;
//    }
//    else if (rpcSlice->getPOC() == test_ref2_poc)
//    {
//        dQP = g_qpRef2;
//    }
    

    // Mock: Set the mock for next time
    mock_trackModifyQP1 = trackModifyQP1;
    mock_trackModifyQP2 = trackModifyQP2;
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        Double dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        //        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Read it only for the I_Slice
        if (eSliceType == I_SLICE) {
            dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        }
        
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//                cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
//                cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
    //        cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
    //    cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    //    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    //    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
  
        

//    cout << "[TURN OFFFFF FALSEXXXX] Turning off modifyQpInter && Changing the code to hardcoded predicted and reference frame effect [P-P case] initSliceNew " << endl;
    ////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
//    if( rpcSlice->getSliceType( ) != I_SLICE && false)
    if( rpcSlice->getSliceType( ) != I_SLICE)
    {
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
//        if(rpcSlice->getPOC() >= end_prop_window + 1)
        if(relativePOC >= end_prop_window + 1)
        {
            iQP = round(current_qp_mu);
            dLambda = current_lambda_mu;
            
#if GEN_QPOffset_Files
            string fileName_qp = "";
            std::ostringstream oss_qp;
            Char* pYUVFileName_qp;
            FILE* my_pFile_qp;
            Int offset = iQP - g_qpInit;
            oss_qp << "Gen//Seq-TXT//" << g_input_FileName << "_offset" << ".txt";
            fileName_qp = oss_qp.str();
            pYUVFileName_qp = fileName_qp.empty()? NULL: strdup(fileName_qp.c_str());
            my_pFile_qp = fopen (pYUVFileName_qp, "at");
            fprintf(my_pFile_qp, "%d\t\t\t\t %d\t\t\t\t %f\t\t\t\t %f\t\t\t\t %f\n", curr_poc, offset, epsilon_i, old_lambda, cur_lambda);
            fclose(my_pFile_qp);
#endif 
        } // end if: modify after you reach the point you can modify
        
    }
    
//       cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice Muuuu: SCENE CHANGE BACK TRACKING LAMBDA: " << current_qp_factor_mu << ", lambda "  << dLambda << ", qp: " << iQP  << ", current_qp_mu " << current_qp_mu <<  ", crr qp off " << current_qp_offset << endl;
    
    cout << "\n-----------" << endl;
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice Muuuu: Lambda = " << dLambda << ", qp = " << iQP << ", qpFactor: " << dQPFactor << endl;
    cout << "-----------\n" << endl;
    // Empty
    //////////////////////////////////////////////////////////////////////////////////////////
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
#if SC_ENABLE_PRINT_TWO
    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: SCENE CHANGE BACK TRACKING LAMBDA: " << dLambda  << endl;
#endif
    
    //    cout << "Cathy: TEncSlice: MODIFY QPPPeeeee 1111 " << dLambda  << endl;
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    // Setting QPs
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    
}


Void TEncSlice:: calculateCurrentQPOffset( TComPic*  pcPic, Int pocLast, Int pocCurr, Int     iNumPicRcvd,
                                          Int iGOPid,   TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField,  Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth, Int start_prop_window_mu, Int end_prop_window_mu, Double epsilon_i)
{
//    cout << "Calculate the current QP offset for the current POC  " << pcPic->getPOC() << endl;
    
    if(rpcSlice->getSliceType() == I_SLICE)
    {
        switch (g_qpInit) {
            case 22:
                current_qp_offset = -1*QpoffInter0[0];
                break;
            case 27:
                current_qp_offset = -1*QpoffInter0[1];
                break;
            case 32:
                current_qp_offset = -1*QpoffInter0[2];
                break;
            case 37:
                current_qp_offset = -1*QpoffInter0[3];
                break;
            default:
                break;
                
        } // end switch
        
//        current_qp_offset = -1*g_qpRef;
//        cout << "HARD HARD CODE HARD CODE QP OFFSET FOR I " << current_qp_offset << endl;

    }
    else
    {
//        cout << "calculateCurrentQPOffset: poc : << " << rpcSlice->getPOC() << ", end_prop_window_mu: " << end_prop_window_mu << endl;
        
/////// Current_QP_factor_mu
#if IS_LDP
        
#if IS_ENABLE_ACTUAL_LD
        
#if IS_EPSILON_PRED_METHOD
        // actual LD:
        current_qp_factor_mu = getQPFactInter(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#else // mu method
        
        
#if RUN_DEFAULT_QP_PROP_EXPERIMENT
        // actual LD:
        current_qp_factor_mu = getQPFactLDDefault(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#else
        // actual LD:
        current_qp_factor_mu = getQPFactInterMu(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#endif
#endif // prediction method if
        
#else // 1step case
        
#if IS_EPSILON_PRED_METHOD
        // 1step LD:
        current_qp_factor_mu = getQPFactInter1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
        
#else // mu method
        // 1step LD:
        current_qp_factor_mu = getQPFactInterMu1step(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#endif // end if prediction method
        
        
#endif // end if actual ld or one step
        
#else // LDB
        
#if IS_ENABLE_ACTUAL_LD
        
#if IS_EPSILON_PRED_METHOD
        // actual LD:
        current_qp_factor_mu = getQPFactInterLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#else // mu method
        
        // actual LD: // not done yet
        current_qp_factor_mu = getQPFactInterMuLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#endif // prediction method if
        
#else // 1step case
        
#if IS_EPSILON_PRED_METHOD
        // 1step LDB:
        current_qp_factor_mu = getQPFactInter1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
        
#else // mu method
        // 1step LDB: // not done yet
        current_qp_factor_mu = getQPFactInterMu1stepLDB(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#endif // end if prediction method
        
        
#endif // end if actual ld or one step
        
        
#endif

/////// Default QP factor experiment:
        
#if RUN_DEFAULT_QP_PROP_EXPERIMENT
        // actual LD:
        current_qp_factor_mu = getQPFactLDDefault(pcPic, iGOPid, rpcSlice, isSceneChange, lastSc, scState, isSmooth, start_prop_window_mu, end_prop_window_mu);
#endif
        
        UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
        
//        cout << "HARD CODE TURNING OFF THE MU MODIFY QP INTER " << rpcSlice->getPOC() << ", end_prop_window_mu: " << end_prop_window_mu << endl;
//        if(relativePOC >= end_prop_window_mu + 1 && false)
        if(relativePOC >= end_prop_window_mu + 1)
        {
//             cout << "NOT DEFAULT calculateCurrentQPOffset: poc : << " << rpcSlice->getPOC() << ", end_prop_window_mu: " << end_prop_window_mu << endl;
        
            // calculate
            UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
            Int curr_poc = relativePOC;

//            Int curr_poc = rpcSlice->getPOC();
            Double cur_lambda;
            Double old_lambda;
            Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;

            switch (g_qpInit) {
                case 22:
                    old_lambda = lambdaArr[qp_factor_index];
                    break;
                case 27:
                    old_lambda = lambdaArr2[qp_factor_index];
                    break;
                case 32:
                    old_lambda = lambdaArr3[qp_factor_index];
                    break;
                case 37:
                    old_lambda = lambdaArr4[qp_factor_index];
                    break;
                default:
                    old_lambda = lambdaArr3[qp_factor_index];
                    break;
            }
            
            // If I am BQTerrace or sequence with high motion
//            cout << "\n \n Aux Check relativePOC " << relativePOC << ", epsilon_i " << epsilon_i << endl;
//            if(relativePOC > 8 && epsilon_i < 1)
            // if any chance epsilon is negative, use the default case
            if(epsilon_i < 1)
            {
                current_qp_factor_mu = QpFact[qp_factor_index];
                current_qp_offset = Qpoff[qp_factor_index];
                current_qp_mu = g_qpInit + current_qp_offset;
                current_lambda_mu = old_lambda; // has 5 elements not 4
//                cout << "\n\n Will enter special case for ever \n\n " << endl;
            }
            else
            {
                cur_lambda = old_lambda/epsilon_i;
                //                cout << (pcPic->getPOC()) << ") cur_lambda " << cur_lambda << " old lambda " << old_lambda << " old qp " << iQP  << endl;
                
                // set the mu parameters
                current_qp_mu = 3 * log2(cur_lambda/current_qp_factor_mu) + 12;
                current_lambda_mu = cur_lambda;
            }
            
//            cout << "QP: " << round(current_qp_mu) << ", qpFact: " << current_qp_factor_mu << endl;
        }  // end if relativePOC >= end_prop_window
        
        else
        {
            
//            cout << "default calculateCurrentQPOffset: poc : << " << rpcSlice->getPOC() << ", end_prop_window_mu: " << end_prop_window_mu << endl;           
            UInt relativePOC = lastSc==-1? rpcSlice->getPOC(): rpcSlice->getPOC()-lastSc;
            Int curr_poc = relativePOC;
            Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
            current_qp_offset = Qpoff[qp_factor_index];
            
//            current_qp_offset = Qpoff[trackModifyQP1];
//            trackModifyQP1 = (trackModifyQP1 + 1)% m_pcCfg->getGOPSize();
            
//             cout << "default calculateCurrentQPOffset: poc : << " << rpcSlice->getPOC() << ", end_prop_window_mu: " << end_prop_window_mu  << ", off" << current_qp_offset << endl;

        }
    }
}



// Hossam: Scene change: This is the one currently used 20/05/2016
///// Samah
///------- OLD VERSION OF INIT WORKING WITHOUT OFFSET ---///////
Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc )
{
    
//    cout << " MOUNIRRRRRRRRRR InitSlice " << endl;
    Double dQP;
    Double dLambda;
    
    rpcSlice = pcPic->getSlice(0);
    rpcSlice->setSPS( pSPS );
    rpcSlice->setPPS( pPPS );
    rpcSlice->setSliceBits(0);
    rpcSlice->setPic( pcPic );
    rpcSlice->initSlice();
    rpcSlice->setPicOutputFlag( true );
    rpcSlice->setPOC( pocCurr );
    
    // depth computation based on GOP size
    Int depth;
    {
#if FIX_FIELD_DEPTH
        Int poc = rpcSlice->getPOC();
        if(isField)
        {
            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
        }
        else
        {
            poc = poc % m_pcCfg->getGOPSize();
        }
#else
        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
        
        if ( poc == 0 )
        {
            depth = 0;
        }
        else
        {
            Int step = m_pcCfg->getGOPSize();
            depth    = 0;
            for( Int i=step>>1; i>=1; i>>=1 )
            {
                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
                {
                    if ( j == poc )
                    {
                        i=0;
                        break;
                    }
                }
                step >>= 1;
                depth++;
            }
        }
        
#if FIX_FIELD_DEPTH
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        if(poc != 0)
        {
#endif
            if (isField && ((rpcSlice->getPOC() % 2) == 1))
            {
                depth ++;
            }
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        }
#endif
#endif
    }
    
    
    // slice type
    SliceType eSliceType;
    
    eSliceType=B_SLICE;
#if EFFICIENT_FIELD_IRAP
    if(!(isField && pocLast == 1))
    {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
        }
        else
        {
#endif
            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
        }
#endif
#if EFFICIENT_FIELD_IRAP
    }
#endif
    
    rpcSlice->setSliceType    ( eSliceType );
    
    
    // ------------------------------------------------------------------------------------------------------------------
    // Non-referenced frame marking
    // ------------------------------------------------------------------------------------------------------------------
    
    if(pocLast == 0)
    {
        rpcSlice->setTemporalLayerNonReferenceFlag(false);
    }
    else
    {
        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
    }
    rpcSlice->setReferenced(true);
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
    //    cout << "Indie QP modifyyyyy  "  << boolalpha << isSceneChange<< endl;
    if (isSceneChange) {
        
        // Fix QP by track --> Next step
        //        track = 3;
        //        -12 bad
        
        //        cout << "Indie QP modifyyyyy  " << endl;
        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(SC_QP_OFFSET);
        
        //                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-6);
        //                        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-12);
        //      Fix: //  m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
        
        track = 0;
        isSc = true;
        eSliceType = I_SLICE;
        rpcSlice->setSliceType(I_SLICE);
    }
    else
    {
        ////        cout << "\n Cathy: TEncSlice: LAST SCCCCCCC! " << lastSc;
        if( (rpcSlice->getPOC() != 0) && lastSc != -1)
        {
            // Fix QP by track --> Next step
            //             track = 1;
            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
            track = (track+1)%m_pcCfg->getGOPSize();
            
        }
        
    }
    
    
    
    dQP = m_pcCfg->getQP();
    
    
    
    if(eSliceType!=I_SLICE)
    {
        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
        {
            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
        }
    }
    
    // Hossam: Scene change
    if(isSceneChange)
    {
        dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
    
    // modify QP
    Int* pdQPs = m_pcCfg->getdQPs();
    if ( pdQPs )
    {
        dQP += pdQPs[ rpcSlice->getPOC() ];
    }
    
    
    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
    {
        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
        m_pcCfg->setDeltaQpRD(0);
    }
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        if ( eSliceType==I_SLICE )
        {
            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQpFactor: " << dQPFactor << "\n" << endl;
            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
            
            dQPFactor=0.57*dLambda_scale;
        }
        

        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //        cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
        //        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
#if HB_LAMBDA_FOR_LDC
    // restore original slice type
    
#if EFFICIENT_FIELD_IRAP
    if(!(isField && pocLast == 1))
    {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
        }
        else
        {
#endif
            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
        }
#endif
#if EFFICIENT_FIELD_IRAP
    }
#endif // EFFICIENT_FIELD_IRAP
    
    
    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
    rpcSlice->setSliceType        ( eSliceType );
#endif
    
    //*************************************************************************************
    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
    
    // Hossam: Code Tweaking -- Second trial start
    
    Int sc = rpcSlice->getPOC();
    Int n = 2; /// 3rd frame
    n = 5;
    
    //    if(n == sc)
    if(isSceneChange)
    {
        cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
        rpcSlice->setSliceType(I_SLICE);
        
    }
    
    
    
    // rpcSlice -> getSliceQp()
    // rpcSlice -> setSliceQp(<#Int i#>)
    //    cout << "\nYang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
    //    getchar();
    //    if(sc == n)
    // Sequence 1
    /*
     if(
     sc == 13 ||
     sc == 54 ||
     sc == 110 ||
     sc == 130 ||
     sc == 165 ||
     sc == 180 ||
     sc == 213 ||
     sc == 277
     )
     /*
     // Sequence 2
     if(
     sc == 30 ||
     sc == 84 ||
     sc == 165 ||
     sc == 240 ||
     sc == 300
     )
     
     {
     cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
     rpcSlice->setSliceType(I_SLICE);
     //        getchar();
     }
     */
    // Hossam Code Tweaking -- Second trial end
    //*************************************************************************************
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
    
    if ( m_pcCfg->getDeblockingFilterMetric() )
    {
        rpcSlice->setDeblockingFilterOverrideFlag(true);
        rpcSlice->setDeblockingFilterDisable(false);
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }
    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
    {
        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
        if ( !rpcSlice->getDeblockingFilterDisable())
        {
            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
            {
                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
            }
            else
            {
                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
            }
        }
    }
    else
    {
        rpcSlice->setDeblockingFilterOverrideFlag( false );
        rpcSlice->setDeblockingFilterDisable( false );
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }
    
    rpcSlice->setDepth            ( depth );
    
    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    if(eSliceType==I_SLICE)
    {
        pcPic->setTLayer(0);
    }
    rpcSlice->setTLayer( pcPic->getTLayer() );
    
    assert( m_apcPicYuvPred );
    assert( m_apcPicYuvResi );
    
    pcPic->setPicYuvPred( m_apcPicYuvPred );
    pcPic->setPicYuvResi( m_apcPicYuvResi );
    
    // Hossam: Settting Slice Mode
    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
    
    
//    cout << "Slice Type: " << rpcSlice->getSliceType() << endl;
//    cout << "Slice Directions: " << (rpcSlice->isInterP()? 1:2) << endl;
    
}


/////


//Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc )
//
// Working version without offset calculator

// hashas

//Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth )
//{
//    Double dQP;
//    Double dLambda;
//    
//    // Hossam: Scene change
//    Double lambdaHelper = 0;
//    
//    rpcSlice = pcPic->getSlice(0);
//    rpcSlice->setSPS( pSPS );
//    rpcSlice->setPPS( pPPS );
//    rpcSlice->setSliceBits(0);
//    rpcSlice->setPic( pcPic );
//    rpcSlice->initSlice();
//    rpcSlice->setPicOutputFlag( true );
//    rpcSlice->setPOC( pocCurr );
//    
//    // depth computation based on GOP size
//    Int depth;
//    {
//#if FIX_FIELD_DEPTH
//        Int poc = rpcSlice->getPOC();
//        if(isField)
//        {
//            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
//        }
//        else
//        {
//            poc = poc % m_pcCfg->getGOPSize();
//        }
//#else
//        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
//#endif
//        
//        if ( poc == 0 )
//        {
//            depth = 0;
//        }
//        else
//        {
//            Int step = m_pcCfg->getGOPSize();
//            depth    = 0;
//            for( Int i=step>>1; i>=1; i>>=1 )
//            {
//                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
//                {
//                    if ( j == poc )
//                    {
//                        i=0;
//                        break;
//                    }
//                }
//                step >>= 1;
//                depth++;
//            }
//        }
//        
//#if FIX_FIELD_DEPTH
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        if(poc != 0)
//        {
//#endif
//            if (isField && ((rpcSlice->getPOC() % 2) == 1))
//            {
//                depth ++;
//            }
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        }
//#endif
//#endif
//    }
//    
////    // Hossam: Scene Change Depth modification
////    if(isSceneChange)
////    {
////        depth = 0;
////    }
////    // Hossam: Scene change to make it higher lambda for non smooth frames (20.xx)
////    if (pocCurr == lastSc + 1) {
////        if (!isSmooth) {
////            depth = 1;
////        }
////    }
//
//    
//    // slice type
//    SliceType eSliceType;
//    
//    eSliceType=B_SLICE;
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif
//    
//    rpcSlice->setSliceType    ( eSliceType );
//    
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // Non-referenced frame marking
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    if(pocLast == 0)
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(false);
//    }
//    else
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
//    }
//    rpcSlice->setReferenced(true);
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // QP setting
//    // ------------------------------------------------------------------------------------------------------------------
//    // ------------------------------------------------------------------------------------------------------------------
//    // Depth ADJUSMENT
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    
//    //    cout << "Indie QP modifyyyyy  "  << boolalpha << isSceneChange<< endl;
//    if (isSceneChange) {
//        
////        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
////                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
////            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+2);
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+0);
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+2);
////        lambdaHelper = getLambda(1);
//        
//                Int offset = m_pcOffsetCalc.getOffset(rpcSlice, isSmooth, isSceneChange, scState, lastSc, m_pcCfg->getQP());
//                Int qpFactIdx = m_pcOffsetCalc.getQpFactIdx();
//                Int depthIdx = m_pcOffsetCalc.getDepthIdx();
//        if (pocCurr % 4 == 0) {
//            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
////            depth = depths[3]; // 1
//
//
//            depth = depths[3]; // 0
//            
////            eSliceType = I_SLICE;
////            rpcSlice->setSliceType(I_SLICE);
//            caseNumber  = 4;
//            
////            // Reset the track = 0;
////             track = 0;
////
////            // Coming from Smooth
////            comingFromSmooth = 1;
////            
//
//            //static int depths[] ={3, 2, 3, 1, 0};
//            //                    static int depths[] ={2, 1, 2, 0};
//
//        }
//        else // if(pocCurr % 4 == 1)
//        {
//            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
////            depth = depths[3]; // 1
//            
//             depth = depths[1];
//            
//            
//             caseNumber  = 1;
//            //static int depths[] ={3, 2, 3, 1, 0};
//            //                    static int depths[] ={2, 1, 2, 0};
//
//            
//        }
////         lambdaHelper = getLambda(3);
//        
//        cout << "Verify SC offset: " <<  m_pcCfg->getGOPEntry(iGOPid).m_QPOffset << ", "  << offset << endl;
//        cout << "Verify SC factor: " <<  m_pcCfg->getGOPEntry(iGOPid).m_QPFactor << ", "  << QpFact[qpFactIdx] << endl;
//        cout << "Verify SC depth: " << depth << ", "  << depths[depthIdx] << endl;
//        
//        
//        
//    }
//    else // Concept I and other frames
//    {
//        
//        if( (rpcSlice->getPOC() != 0) && lastSc != -1)
//        {
//            
//            Int offset = m_pcOffsetCalc.getOffset(rpcSlice, isSmooth, isSceneChange, scState, lastSc, m_pcCfg->getQP());
//
//            //            if(scState == 4)
//            if(rpcSlice->getPOC() == lastSc + 1)
//            {
//                Int offset = m_pcOffsetCalc.getOffset(rpcSlice, isSmooth, isSceneChange, scState, lastSc, m_pcCfg->getQP());
//                Int qpFactIdx = m_pcOffsetCalc.getQpFactIdx();
//                Int depthIdx = m_pcOffsetCalc.getDepthIdx();
//                
//                if(isSmooth)
//                {
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    
//                    if (caseNumber == 4) { // Not working for now -- needs fixing - I closed it from SCD Engine
////                        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
////                        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//                        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[0]);
//                        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[0]);
//                        depth = depths[0];
//                        comingFromSmooth = 2;
//                    }
//                    else
//                    {
//                    
//                        switch (m_pcCfg->getQP()) {
//                            case 22:
//                                //                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(SC_QP_OFFSET);
//                                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-2);
//                                break;
//                            case 27:
//                                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-2);
//                                break;
//                            case 32:
//                            case 37:
//                                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-4);
//                                break;
//                            case 42:
//                                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-6);
//                                break;
//                            default:
//                                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(0);
//                                break;
//                        }
//                        
//                    }// else caseNumber = 4
//                    
//                    lambdaHelper = getLambda(4);
//
////                    depth = depths[3]; // 1
//                     depth = depths[1]; // 1
//                    
//                    
//                    // Set the depth to 0 in case 4
////                    depth = depths[3];
//                    
////                    //static int depths[] ={3, 2, 3, 1, 0};
////                    static int depths[] ={2, 1, 2, 0};
//
////                    // Reset the track
////                    track = 0;
//                    
//                    // Smooth
//                    comingFromSmooth = 1;
//                    
//                    cout << "Verify Concept I offset: " <<  m_pcCfg->getGOPEntry(iGOPid).m_QPOffset << ", "  << offset << endl;
//                    cout << "Verify Concept I factor: " <<  m_pcCfg->getGOPEntry(iGOPid).m_QPFactor << ", "  << QpFact[qpFactIdx] << endl;
//                    cout << "Verify Concept I depth: " << depth << ", "  << depths[depthIdx] << endl;
//
//
//                }
//                else
//                {
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[2]);
//
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
////                    lambdaHelper = getLambda(3);
//                    
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+3);
////                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
////                    depth = depths[0]; // 3
//                    
//                    depth = depths[0];// 2
//                    //static int depths[] ={3, 2, 3, 1, 0};
////                    static int depths[] ={2, 1, 2, 0};
//
////                    // Reset the track
////                    track = 1;
//
//                    // Not Smooth
//                    comingFromSmooth = 2;
//                }
//                
//                // Reset the track lambda
//                trackLambda = 0;
//                
////                // Reset the track
//                track = 0;
//                
//                // Force the I frame
//                //                eSliceType = I_SLICE;
//                //                rpcSlice->setSliceType(I_SLICE);
//                
//            }// end if(scState==4)
//            else
//            {
//                
////                cout << "COMING FROM SMOOTH: " << comingFromSmooth << endl;
//                if (comingFromSmooth == 2) {
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[1]);
//                   
////                    depth = depths[1]; // 2
//                      depth = depths[0]; // 2
//                    
//                    //static int depths[] ={3, 2, 3, 1, 0};
////                    static int depths[] ={2, 1, 2, 0};
//
//
//                }
//                else{
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
//                    depth = depths[track];
//
//                    
//                    //static int depths[] ={3, 2, 3, 1, 0};
//                    //                    static int depths[] ={2, 1, 2, 0};
//                    
//                }
//                
//                lambdaHelper = getLambda(trackLambda);
//                
//                track = (track+1)%m_pcCfg->getGOPSize(); trackLambda = (trackLambda+1) % m_pcCfg->getGOPSize();
//                
//            }// end normal case
//            
//        }// end entering the normal case
//        
//    }// end adjusting the QP and lambda
//    
//    
//    
//    dQP = m_pcCfg->getQP();
//    
//    
//    
//    if(eSliceType!=I_SLICE)
//    {
//        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
//        {
//            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//        }
//    }
//    
//    // modify QP
//    // Hossam: Scene Change
//    Int* pdQPs = m_pcCfg->getdQPs();
//    if ( pdQPs)
//    {
//        dQP += pdQPs[ rpcSlice->getPOC() ];
//    }
//    
//    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
//    {
//        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
//        m_pcCfg->setDeltaQpRD(0);
//    }
//    
//    
//    // Hossam: Scene Change
//    if(isSceneChange || rpcSlice->getPOC() == lastSc + 1)
//    {
//        dQP = m_pcCfg->getQP() + m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//    }
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // Lambda computation
//    // ------------------------------------------------------------------------------------------------------------------
//    
////     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    
//    Int iQP;
//    Double dOrigQP = dQP;
//    
//    // pre-compute lambda and QP values for all possible QP candidates
//    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
//    {
//        // compute QP value
//        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
//        
//        // compute lambda value
//        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
//        Int    SHIFT_QP = 12;
//        
//        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
//        
////         cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
//        
//#if FULL_NBIT
//        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//        Int    bitdepth_luma_qp_scale = 0;
//#endif
//        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
//#if FULL_NBIT
//        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
//#endif
//        
////        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//        
//        // Hossam: Computing eSliceType I or P or B
//        // Case #1: I or P-slices (key-frame)
//        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
//        if ( eSliceType==I_SLICE )
////        if ( eSliceType==I_SLICE || (pocCurr == lastSc + 1 && isSmooth))
//        {
//            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQpFactor: " << dQPFactor << "\n" << endl;
//            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
//            
//            dQPFactor=0.57*dLambda_scale;
//        }
//        
//        cout << "\nPARAMS OF THE EQUATN: " << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp2222 " << qp_temp << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH depthhh2222 " << depth << endl;
//        
//        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
//        
////        cout << "\nCathy: TEncSlice:  FRAMEEEEEEeEEEEEEEE dQp: " << dQP << ", QpFact: " << dQPFactor << ", dLambda: " << dLambda << "\n" << endl;
//        cout << "\nCathy: TEncSlice: AFTER PARAMS LAMDA WITH dLambda " << dLambda << "\n" << endl;
////        cout << "m_pcCfg->getDeltaQpRD(): " << m_pcCfg->getDeltaQpRD() << "\n" << endl;// 0
//        
//        //      getchar();
//        
//        
//        if ( depth>0 )
//        {
//            
//           
//#if FULL_NBIT
//            
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#else
////            cout << depth << " INSIDE IF STATEMENT: "  << ", qp_temp: " << qp_temp << endl;
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#endif
//        }
//        
////        cout << "Before  !m_pcCfg->getUseHADME() : " << dLambda << endl;
//        // if hadamard is used in ME process
//        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
//        {
//            dLambda *= 0.95;
//        }
////        cout << "After  !m_pcCfg->getUseHADME() : " << dLambda << endl;
//        
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//        
//        m_pdRdPicLambda[iDQpIdx] = dLambda;
//        m_pdRdPicQp    [iDQpIdx] = dQP;
//        m_piRdPicQp    [iDQpIdx] = iQP;
//    }
//    
//    // obtain dQP = 0 case
//    dLambda = m_pdRdPicLambda[0];
//    dQP     = m_pdRdPicQp    [0];
//    iQP     = m_piRdPicQp    [0];
//    
////    cout << "Before  rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
//    if( rpcSlice->getSliceType( ) != I_SLICE )
//    {
//        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );// equals 0
//    }
////    cout << "After  rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
//    
//    //    // Hossam: Scene Change
//    //    if(rpcSlice->getPOC() == lastSc + 1)
//    //    {
//    //        if(isSmooth)
//    //        {
//    ////            cout << "QP ORGGGGGGGGGGG1111: " << m_pcCfg->getQP() << endl;
//    ////            iQP = m_pcCfg->getQP() + SC_QP_OFFSET;
//    ////            dQP = +1;
//    //            iQP = m_pcCfg->getQP() + SC_QP_OFFSET;
//    //            dQP = SC_QP_OFFSET;
//    //
//    //        }
//    //        else
//    //        {
//    ////            cout << "QP ORGGGGGGGGGGG2222: " << m_pcCfg->getQP() << endl;
//    //            iQP = m_pcCfg->getQP() + 1;
//    //            dQP = +1;
//    //        }
//    //
//    //    }
//    
//    cout << "\nCathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda " << dLambda  << endl;
////     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH lambdaHelper " << lambdaHelper << endl;
////        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    
//    // Hossam: Scene Change: Ignore the dlamda calculation and take it from the table for now
////     dLambda = lambdaHelper; // New change: leave it calculated programitically!
//
////    if(isSceneChange)
////        dLambda = min(dLambda, lambdaHelper); // New change: leave it calculated programitically!
////    else if (pocCurr == lastSc + 1) {
////        dLambda = isSmooth? min(dLambda, lambdaHelper): max(dLambda, lambdaHelper);
////    }
////    else
////        dLambda = lambdaHelper;
//    
//    setUpLambda(rpcSlice, dLambda, iQP);
//    
//    
////    cout << pocCurr << "Cathy: TEncSlice: ATTEMPT 2 MODIFYYY LAMDA WITH dLambda " << dLambda << "\n" << endl;
//    
//#if HB_LAMBDA_FOR_LDC
//    // restore original slice type
//    
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif // EFFICIENT_FIELD_IRAP
//    
//    
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
//    rpcSlice->setSliceType        ( eSliceType );
//#endif
//    
//    //*************************************************************************************
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
//    
//    // Hossam: Code Tweaking -- Second trial start
//    
//    Int sc = rpcSlice->getPOC();
//    Int n = 2; /// 3rd frame
//    n = 5;
//    
//    //    if(n == sc)
//    //    if(isSceneChange)
//    //    {
//    //        cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//    //        rpcSlice->setSliceType(I_SLICE);
//    //
//    //    }
//    
//    //    if(scState == 4)
//    
//    // Modify
//    //    if(rpcSlice->getPOC() == lastSc + 1)
//    //    {
//    //        cout << "Yang: TEncSlice: initEncSlice: Forcing an CONCEPTUAL I frame XXXX " << sc << "\n" << endl;
//    //        rpcSlice->setSliceType(I_SLICE);
//    //
//    //    }
//    
//    if(isSceneChange)
////    if(isSceneChange && caseNumber != 4)
//    {
//        cout << "Yang: TEncSlice: initEncSlice: Forcing an CONCEPTUAL I frame XXXX " << sc << "\n" << endl;
//        rpcSlice->setSliceType(I_SLICE);
//        
//    }
//    
//    
//    // rpcSlice -> getSliceQp()
//    // rpcSlice -> setSliceQp(<#Int i#>)
//    //    cout << "\nYang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//    //    getchar();
//    //    if(sc == n)
//    // Sequence 1
//    /*
//     if(
//     sc == 13 ||
//     sc == 54 ||
//     sc == 110 ||
//     sc == 130 ||
//     sc == 165 ||
//     sc == 180 ||
//     sc == 213 ||
//     sc == 277
//     )
//     /*
//     // Sequence 2
//     if(
//     sc == 30 ||
//     sc == 84 ||
//     sc == 165 ||
//     sc == 240 ||
//     sc == 300
//     )
//     
//     {
//     cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//     rpcSlice->setSliceType(I_SLICE);
//     //        getchar();
//     }
//     */
//    // Hossam Code Tweaking -- Second trial end
//    //*************************************************************************************
//    
//    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
//    {
//        dQP = xGetQPValueAccordingToLambda( dLambda );
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//    }
//    
//    rpcSlice->setSliceQp           ( iQP );
//#if ADAPTIVE_QP_SELECTION
//    rpcSlice->setSliceQpBase       ( iQP );
//#endif
//    rpcSlice->setSliceQpDelta      ( 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
//    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    
//    if ( m_pcCfg->getDeblockingFilterMetric() )
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag(true);
//        rpcSlice->setDeblockingFilterDisable(false);
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
//    {
//        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
//        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
//        if ( !rpcSlice->getDeblockingFilterDisable())
//        {
//            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//            }
//            else
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//            }
//        }
//    }
//    else
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag( false );
//        rpcSlice->setDeblockingFilterDisable( false );
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    
//    rpcSlice->setDepth            ( depth );
//    
//    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    if(eSliceType==I_SLICE)
//    {
//        pcPic->setTLayer(0);
//    }
//    rpcSlice->setTLayer( pcPic->getTLayer() );
//    
//    assert( m_apcPicYuvPred );
//    assert( m_apcPicYuvResi );
//    
//    pcPic->setPicYuvPred( m_apcPicYuvPred );
//    pcPic->setPicYuvResi( m_apcPicYuvResi );
//    
//    // Hossam: Settting Slice Mode
//    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
//    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
//    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
//    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
//    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
//    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
//}


///////
// Hossam: Scene Change org
// coming from Smooth
Void TEncSlice::initEncSliceNewAttempt( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth )
{
    Double dQP;
    Double dLambda;
    
    
    rpcSlice = pcPic->getSlice(0);
    rpcSlice->setSPS( pSPS );
    rpcSlice->setPPS( pPPS );
    rpcSlice->setSliceBits(0);
    rpcSlice->setPic( pcPic );
    rpcSlice->initSlice();
    rpcSlice->setPicOutputFlag( true );
    rpcSlice->setPOC( pocCurr );
    
    
    // depth computation based on GOP size
    Int depth;
    {
#if FIX_FIELD_DEPTH
        Int poc = rpcSlice->getPOC();
        if(isField)
        {
            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
        }
        else
        {
            poc = poc % m_pcCfg->getGOPSize();
        }
#else
        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
        
        if ( poc == 0 )
        {
            depth = 0;
        }
        else
        {
            Int step = m_pcCfg->getGOPSize();
            depth    = 0;
            for( Int i=step>>1; i>=1; i>>=1 )
            {
                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
                {
                    if ( j == poc )
                    {
                        i=0;
                        break;
                    }
                }
                step >>= 1;
                depth++;
            }
        }
        
#if FIX_FIELD_DEPTH
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        if(poc != 0)
        {
#endif
            if (isField && ((rpcSlice->getPOC() % 2) == 1))
            {
                depth ++;
            }
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        }
#endif
#endif
    }
    
    
    // slice type
    SliceType eSliceType;
    eSliceType=B_SLICE;
#if EFFICIENT_FIELD_IRAP
    if(!(isField && pocLast == 1))
    {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
        }
        else
        {
#endif
            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
        }
#endif
#if EFFICIENT_FIELD_IRAP
    }
#endif
    
    rpcSlice->setSliceType    ( eSliceType );
    
    
    // ------------------------------------------------------------------------------------------------------------------
    // Non-referenced frame marking
    // ------------------------------------------------------------------------------------------------------------------
    
    if(pocLast == 0)
    {
        rpcSlice->setTemporalLayerNonReferenceFlag(false);
    }
    else
    {
        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
    }
    rpcSlice->setReferenced(true);
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------------------------
    // Depth ADJUSMENT
    // ------------------------------------------------------------------------------------------------------------------
    
    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(0);
    //    depth = depths[4];
    // Hossam: Scene change: let the SCD Engine operate at QP = 27; // 22 for now
    dQP = SC_QP_ORG;
    //    dQP = m_pcCfg->getQP();
    
    
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    //     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
        //         cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        
        // Hossam: Scene change ORG QP factor of the I frame
//        Double dQPFactor = m_pcCfg->getGOPEntry(0).m_QPFactor;
        //        if ( eSliceType==I_SLICE )
        //            //        if ( eSliceType==I_SLICE || (pocCurr == lastSc + 1 && isSmooth))
        //        {
        //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQpFactor: " << dQPFactor << "\n" << endl;
        //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
        
        dQPFactor=0.57*dLambda_scale;
        //        }
        
        //        cout << "\nPARAMS OF THE EQUATN: " << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp2222 " << qp_temp << endl;
        //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH depthhh2222 " << depth << endl;
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //        cout << "\nCathy: TEncSlice:  FRAMEEEEEEeEEEEEEEE dQp: " << dQP << ", QpFact: " << dQPFactor << ", dLambda: " << dLambda << "\n" << endl;
        //        cout << "\nCathy: TEncSlice: AFTER PARAMS LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //        cout << "m_pcCfg->getDeltaQpRD(): " << m_pcCfg->getDeltaQpRD() << "\n" << endl;// 0
        
        //      getchar();
        
        
        if ( depth>0 )
        {
            
            
#if FULL_NBIT
            
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            //            cout << depth << " INSIDE IF STATEMENT: "  << ", qp_temp: " << qp_temp << endl;
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        //        cout << "Before  !m_pcCfg->getUseHADME() : " << dLambda << endl;
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        //        cout << "After  !m_pcCfg->getUseHADME() : " << dLambda << endl;
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        
        // Hossam: Scene Change ORG -- PICK THE ONE FOR QP @ 22
        dLambda = lambdaArr[4];
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    //    cout << "Before  rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );// equals 0
    }
    
    //    cout << "\nCathy: TEncSlice: ATTEMPT 000000 ADJUSTTTTTT WITH dLambda " << dLambda  << endl;
    
    
    // Hossam: Scene Change ORG -- PICK THE ONE FOR QP @ 22
    dLambda = lambdaArr[4];
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
    
#if SC_ENABLE_PRINT_TWO
    cout << pocCurr << "Cathy: TEncSlice: WILL SCENE CHANGE WITH LAMDBDA VALUE  " << dLambda << "\n" << endl;
#endif
    
//        cout << pocCurr << "Cathy: TEncSlice: ATTEMPT 2 MODIFYYY LAMDA WITH dLambda " << dLambda << "\n" << endl;
    
#if HB_LAMBDA_FOR_LDC
    // restore original slice type
    
#if EFFICIENT_FIELD_IRAP
    if(!(isField && pocLast == 1))
    {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
        }
        else
        {
#endif
            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
        }
#endif
#if EFFICIENT_FIELD_IRAP
    }
#endif // EFFICIENT_FIELD_IRAP
    
    
    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
    rpcSlice->setSliceType        ( eSliceType );
#endif
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
//    cout << "InitEncSliceNew: SLICEEEEE QPPPPPP: " << iQP << endl;
//    cout << "\n" << endl;
    
    
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
    
    if ( m_pcCfg->getDeblockingFilterMetric() )
    {
        rpcSlice->setDeblockingFilterOverrideFlag(true);
        rpcSlice->setDeblockingFilterDisable(false);
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }
    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
    {
        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
        if ( !rpcSlice->getDeblockingFilterDisable())
        {
            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
            {
                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
            }
            else
            {
                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
            }
        }
    }
    else
    {
        rpcSlice->setDeblockingFilterOverrideFlag( false );
        rpcSlice->setDeblockingFilterDisable( false );
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }
    
    
    rpcSlice->setDepth            ( depth );
    
    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    if(eSliceType==I_SLICE)
    {
        pcPic->setTLayer(0);
    }
    rpcSlice->setTLayer( pcPic->getTLayer() );
    
    assert( m_apcPicYuvPred );
    assert( m_apcPicYuvResi );
    
    pcPic->setPicYuvPred( m_apcPicYuvPred );
    pcPic->setPicYuvResi( m_apcPicYuvResi );
    
    // Hossam: Settting Slice Mode
    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
    
    
//    cout << "Slice Type: " << rpcSlice->getSliceType() << endl; // 0
//    cout << "Slice Directions: " << (rpcSlice->isInterP()? 1:2) << endl; // 2
}




float TEncSlice::getLambda(Int index)
{
    float lambdaHelper;
    Int QP = m_pcCfg->getQP();
    
    if(QP == 22)
    {
        lambdaHelper = lambdaArr[index];
    }
    else if(QP == 27)
    {
        lambdaHelper = lambdaArr2[index];
    }
    else if(QP ==32)
    {
        lambdaHelper = lambdaArr3[index];
    }
    else if(QP==37)
    {
        lambdaHelper = lambdaArr4[index];
    }
    else{
        lambdaHelper = lambdaArr5[index];
    }
    
    return lambdaHelper;
    
}

// Hossam: this method is used as long as there is no scene change
Void TEncSlice::initEncSlice( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField )
{
    Double dQP;
    Double dLambda;
    
    rpcSlice = pcPic->getSlice(0);
    rpcSlice->setSPS( pSPS );
    rpcSlice->setPPS( pPPS );
    rpcSlice->setSliceBits(0);
    rpcSlice->setPic( pcPic );
    rpcSlice->initSlice();
    rpcSlice->setPicOutputFlag( true );
    rpcSlice->setPOC( pocCurr );
    
    // depth computation based on GOP size
    Int depth;
    {
#if FIX_FIELD_DEPTH
        Int poc = rpcSlice->getPOC();
        if(isField)
        {
            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
        }
        else
        {
            poc = poc % m_pcCfg->getGOPSize();
        }
#else
        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
#endif
        
        if ( poc == 0 )
        {
            depth = 0;
        }
        else
        {
            Int step = m_pcCfg->getGOPSize();
            depth    = 0;
            for( Int i=step>>1; i>=1; i>>=1 )
            {
                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
                {
                    if ( j == poc )
                    {
                        i=0;
                        break;
                    }
                }
                step >>= 1;
                depth++;
            }
        }
        
#if FIX_FIELD_DEPTH
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        if(poc != 0)
        {
#endif
            if (isField && ((rpcSlice->getPOC() % 2) == 1))
            {
                depth ++;
            }
#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
        }
#endif
#endif
    }
    
    
    // slice type
    SliceType eSliceType;
    
    eSliceType=B_SLICE;
#if EFFICIENT_FIELD_IRAP
    if(!(isField && pocLast == 1))
    {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
        }
        else
        {
#endif
            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
        }
#endif
#if EFFICIENT_FIELD_IRAP
    }
#endif
    
    rpcSlice->setSliceType    ( eSliceType );
    
    
    // ------------------------------------------------------------------------------------------------------------------
    // Non-referenced frame marking
    // ------------------------------------------------------------------------------------------------------------------
    
    if(pocLast == 0)
    {
        rpcSlice->setTemporalLayerNonReferenceFlag(false);
    }
    else
    {
        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
    }
    rpcSlice->setReferenced(true);
    
    // ------------------------------------------------------------------------------------------------------------------
    // QP setting
    // ------------------------------------------------------------------------------------------------------------------
    
//    if (pocCurr % 2 != 0) {
//        dQP = 22;
//    }
//    else
        dQP = m_pcCfg->getQP();
    
    
    
    if(eSliceType!=I_SLICE)
    {
        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
        {
            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
        }
    }
    
    // modify QP
    Int* pdQPs = m_pcCfg->getdQPs();
    if ( pdQPs )
    {
        dQP += pdQPs[ rpcSlice->getPOC() ];
    }
    
    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
    {
        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
        m_pcCfg->setDeltaQpRD(0);
    }
    
    // ------------------------------------------------------------------------------------------------------------------
    // Lambda computation
    // ------------------------------------------------------------------------------------------------------------------
    
    Int iQP;
    Double dOrigQP = dQP;
    
    // pre-compute lambda and QP values for all possible QP candidates
    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
    {
        // compute QP value
        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
        
        // compute lambda value
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Int    SHIFT_QP = 12;
        
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
        
#if FULL_NBIT
        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
        Int    bitdepth_luma_qp_scale = 0;
#endif
        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
        
        // Hossam: Computing eSliceType I or P or B
        // Case #1: I or P-slices (key-frame)
        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
        if ( eSliceType==I_SLICE )
        {
            dQPFactor=0.57*dLambda_scale;
        }
        
//        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 111 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
        
        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
        
        //        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
        //      getchar();
        
        if ( depth>0 )
        {
#if FULL_NBIT
            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
        }
        
        // if hadamard is used in ME process
        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
        {
            dLambda *= 0.95;
        }
        
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
        
        m_pdRdPicLambda[iDQpIdx] = dLambda;
        m_pdRdPicQp    [iDQpIdx] = dQP;
        m_piRdPicQp    [iDQpIdx] = iQP;
    }
    
    // obtain dQP = 0 case
    dLambda = m_pdRdPicLambda[0];
    dQP     = m_pdRdPicQp    [0];
    iQP     = m_piRdPicQp    [0];
    
    
//        cout << "Before  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
    if( rpcSlice->getSliceType( ) != I_SLICE )
    {
        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    }

//    cout << "After  ATTEMPT 1 rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
//    cout << "\nCathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    cout << "Cathy: TEncSlice: ATTEMPT 11111 ADJUSTTTTTT WITH dlamda " << dLambda  << endl;
    
    
    setUpLambda(rpcSlice, dLambda, iQP);
    
    //    cout << "\n";
    //    cout << pocCurr << "Cathy: TEncSlice: ATTEMPT 1 LAMDA WITH dLambda " << dLambda << "\n" << endl;
    
#if HB_LAMBDA_FOR_LDC
    // restore original slice type
    
#if EFFICIENT_FIELD_IRAP
    if(!(isField && pocLast == 1))
    {
#endif // EFFICIENT_FIELD_IRAP
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(m_pcCfg->getDecodingRefreshType() == 3)
        {
            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
        }
        else
        {
#endif
            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
#if ALLOW_RECOVERY_POINT_AS_RAP
        }
#endif
#if EFFICIENT_FIELD_IRAP
    }
#endif // EFFICIENT_FIELD_IRAP
    
    
    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
    rpcSlice->setSliceType        ( eSliceType );
#endif
    
    //*************************************************************************************
    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
    
    // Hossam: Code Tweaking -- Second trial start
    
    Int sc = rpcSlice->getPOC();
    Int n = 2; /// 3rd frame
    
    // rpcSlice -> getSliceQp()
    // rpcSlice -> setSliceQp(<#Int i#>)
    //    cout << "\nYang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
    //    getchar();
    //    if(sc == n)
    // Sequence 1
    /*
     if(
     sc == 13 ||
     sc == 54 ||
     sc == 110 ||
     sc == 130 ||
     sc == 165 ||
     sc == 180 ||
     sc == 213 ||
     sc == 277
     )
     /*
     // Sequence 2
     if(
     sc == 30 ||
     sc == 84 ||
     sc == 165 ||
     sc == 240 ||
     sc == 300
     )
     
     {
     cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
     rpcSlice->setSliceType(I_SLICE);
     //        getchar();
     }
     */
    // Hossam Code Tweaking -- Second trial end
    //*************************************************************************************
    
    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
    {
        dQP = xGetQPValueAccordingToLambda( dLambda );
        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
    }
    
    rpcSlice->setSliceQp           ( iQP );
#if ADAPTIVE_QP_SELECTION
    rpcSlice->setSliceQpBase       ( iQP );
#endif
    rpcSlice->setSliceQpDelta      ( 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
    
    if ( m_pcCfg->getDeblockingFilterMetric() )
    {
        rpcSlice->setDeblockingFilterOverrideFlag(true);
        rpcSlice->setDeblockingFilterDisable(false);
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }
    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
    {
        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
        if ( !rpcSlice->getDeblockingFilterDisable())
        {
            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
            {
                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
            }
            else
            {
                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
            }
        }
    }
    else
    {
        rpcSlice->setDeblockingFilterOverrideFlag( false );
        rpcSlice->setDeblockingFilterDisable( false );
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }
    
    rpcSlice->setDepth            ( depth );
    
    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
    if(eSliceType==I_SLICE)
    {
        pcPic->setTLayer(0);
    }
    rpcSlice->setTLayer( pcPic->getTLayer() );
    
    assert( m_apcPicYuvPred );
    assert( m_apcPicYuvResi );
    
    pcPic->setPicYuvPred( m_apcPicYuvPred );
    pcPic->setPicYuvResi( m_apcPicYuvResi );
    
    // Hossam: Settting Slice Mode
    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
    
//#if IS_YAO_SCD
//    UInt t    =  rpcSlice->getPOC();
//    g_average_QP = (1.0*(t-1)/t))g_average_QP + (1.0/t)*
//    
//#endif
    
}


Void TEncSlice::resetQP( TComPic* pic, Int sliceQP, Double lambda )
{
    TComSlice* slice = pic->getSlice(0);
    
    // store lambda
    slice->setSliceQp( sliceQP );
#if ADAPTIVE_QP_SELECTION
    slice->setSliceQpBase ( sliceQP );
#endif
    setUpLambda(slice, lambda, sliceQP);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSlice::setSearchRange( TComSlice* pcSlice )
{
    Int iCurrPOC = pcSlice->getPOC();
    Int iRefPOC;
    Int iGOPSize = m_pcCfg->getGOPSize();
    Int iOffset = (iGOPSize >> 1);
    Int iMaxSR = m_pcCfg->getSearchRange();
    Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;
    
    for (Int iDir = 0; iDir <= iNumPredDir; iDir++)
    {
        //RefPicList e = (RefPicList)iDir;
        RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
        for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
        {
            iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
            Int iNewSR = Clip3(8, iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
            m_pcPredSearch->setAdaptiveSearchRange(iDir, iRefIdx, iNewSR);
        }
    }
}


// Hossam: Scene change precompRess for Original
/**
 - multi-loop slice encoding for different slice QP
 .
 \param rpcPic    picture class
 */
Void TEncSlice::precompressSliceNewOrg( TComPic*& rpcPic )
{
    // Hossam: this step can be overlooked now: DeltaQpRd is 0 now
    // if deltaQP RD is not used, simply return
    if ( m_pcCfg->getDeltaQpRD() == 0 )
    {
        return;
    }
    
    if ( m_pcCfg->getUseRateCtrl() )
    {
        printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
        assert(0);
    }
    
    TComSlice* pcSlice        = rpcPic->getSlice(getSliceIdx());
    Double     dPicRdCostBest = MAX_DOUBLE;
    UInt       uiQpIdxBest = 0;
    
    Double dFrameLambda;
#if FULL_NBIT
    Int    SHIFT_QP = 12 + 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
    Int    SHIFT_QP = 12;
#endif
    
    // set frame lambda
    if (m_pcCfg->getGOPSize() > 1)
    {
        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
    }
    else
    {
        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
    }

    // Hossam: Scene Change It does not come here for now!
//    cout << "Monna:: TEncSlice:: preCompressSliceNewOrg:: dFrameLambda: " << dFrameLambda << ", m_piRdPicQp[0]: " << m_piRdPicQp << endl;
    
    m_pcRdCost      ->setFrameLambda(dFrameLambda);
    
    // for each QP candidate
    for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
    {
        pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
#if ADAPTIVE_QP_SELECTION
        pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
#endif
        setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);
        
        // try compress
        // Hossam: Change to compressSlice new org
        compressSliceNewOrg(rpcPic);
        
//        compressSliceNew   ( rpcPic );
        //        compressSlice   ( rpcPic );
        
        Double dPicRdCost;
        UInt64 uiPicDist        = m_uiPicDist;
        UInt64 uiALFBits        = 0;
        
        m_pcGOPEncoder->preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
        
        // compute RD cost and choose the best
        dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);
        
        if ( dPicRdCost < dPicRdCostBest )
        {
            uiQpIdxBest    = uiQpIdx;
            dPicRdCostBest = dPicRdCost;
        }
    }
    
    // set best values
    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
#if ADAPTIVE_QP_SELECTION
    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
#endif
    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);
}



/**
 - multi-loop slice encoding for different slice QP
 .
 \param rpcPic    picture class
 */
Void TEncSlice::precompressSliceNew( TComPic*& rpcPic )
{
    // Hossam: this step can be overlooked now: DeltaQpRd is 0 now
    // if deltaQP RD is not used, simply return
    if ( m_pcCfg->getDeltaQpRD() == 0 )
    {
        return;
    }
    
    if ( m_pcCfg->getUseRateCtrl() )
    {
        printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
        assert(0);
    }
    
    TComSlice* pcSlice        = rpcPic->getSlice(getSliceIdx());
    Double     dPicRdCostBest = MAX_DOUBLE;
    UInt       uiQpIdxBest = 0;
    
    Double dFrameLambda;
#if FULL_NBIT
    Int    SHIFT_QP = 12 + 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
    Int    SHIFT_QP = 12;
#endif
    
    // set frame lambda
    if (m_pcCfg->getGOPSize() > 1)
    {
        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
    }
    else
    {
        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
    }
    m_pcRdCost      ->setFrameLambda(dFrameLambda);
    
    // for each QP candidate
    for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
    {
        pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
#if ADAPTIVE_QP_SELECTION
        pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
#endif
        setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);
        
        // try compress
        // Hossam: Change to compressSlice new
        
        
        compressSliceNew   ( rpcPic );
        //        compressSlice   ( rpcPic );
        
        Double dPicRdCost;
        UInt64 uiPicDist        = m_uiPicDist;
        UInt64 uiALFBits        = 0;
        
        m_pcGOPEncoder->preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
        
        // compute RD cost and choose the best
        dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);
        
        if ( dPicRdCost < dPicRdCostBest )
        {
            uiQpIdxBest    = uiQpIdx;
            dPicRdCostBest = dPicRdCost;
        }
    }
    
    // set best values
    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
#if ADAPTIVE_QP_SELECTION
    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
#endif
    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);
}


/**
 - multi-loop slice encoding for different slice QP
 .
 \param rpcPic    picture class
 */
Void TEncSlice::precompressSlice( TComPic*& rpcPic )
{
    // if deltaQP RD is not used, simply return
    if ( m_pcCfg->getDeltaQpRD() == 0 )
    {
        return;
    }
    
    if ( m_pcCfg->getUseRateCtrl() )
    {
        printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
        assert(0);
    }
    
    TComSlice* pcSlice        = rpcPic->getSlice(getSliceIdx());
    Double     dPicRdCostBest = MAX_DOUBLE;
    UInt       uiQpIdxBest = 0;
    
    Double dFrameLambda;
#if FULL_NBIT
    Int    SHIFT_QP = 12 + 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
#else
    Int    SHIFT_QP = 12;
#endif
    
    // set frame lambda
    if (m_pcCfg->getGOPSize() > 1)
    {
        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
    }
    else
    {
        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
    }
    m_pcRdCost      ->setFrameLambda(dFrameLambda);
    
    // for each QP candidate
    for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
    {
        pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
#if ADAPTIVE_QP_SELECTION
        pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
#endif
        setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);
        
        // try compress
        compressSlice   ( rpcPic );
        
        Double dPicRdCost;
        UInt64 uiPicDist        = m_uiPicDist;
        UInt64 uiALFBits        = 0;
        
        m_pcGOPEncoder->preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
        
        // compute RD cost and choose the best
        dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);
        
        if ( dPicRdCost < dPicRdCostBest )
        {
            uiQpIdxBest    = uiQpIdx;
            dPicRdCostBest = dPicRdCost;
        }
    }
    
    // set best values
    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
#if ADAPTIVE_QP_SELECTION
    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
#endif
    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);
}

Void TEncSlice::calCostSliceI(TComPic*& rpcPic)
{
    UInt    uiCUAddr;
    UInt    uiStartCUAddr;
    UInt    uiBoundingCUAddr;
    Int     iSumHad, shift = g_bitDepth[CHANNEL_TYPE_LUMA]-8, offset = (shift>0)?(1<<(shift-1)):0;;
    Double  iSumHadSlice = 0;
    
    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
    
    UInt uiEncCUOrder;
    uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        // initialize CU encoder
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        pcCU->initCU( rpcPic, uiCUAddr );
        
        Int height  = min( pcSlice->getSPS()->getMaxCUHeight(),pcSlice->getSPS()->getPicHeightInLumaSamples() - uiCUAddr / rpcPic->getFrameWidthInCU() * pcSlice->getSPS()->getMaxCUHeight() );
        Int width   = min( pcSlice->getSPS()->getMaxCUWidth(),pcSlice->getSPS()->getPicWidthInLumaSamples() - uiCUAddr % rpcPic->getFrameWidthInCU() * pcSlice->getSPS()->getMaxCUWidth() );
        
        iSumHad = m_pcCuEncoder->updateLCUDataISlice(pcCU, uiCUAddr, width, height);
        
        (m_pcRateCtrl->getRCPic()->getLCU(uiCUAddr)).m_costIntra=(iSumHad+offset)>>shift;
        iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(uiCUAddr)).m_costIntra;
        
    }
    m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

static const int fourTwoZeroScale = 4;
// Hossam: Dump the residuals to a YUV file
Void TEncSlice::xDumpResiduals(TComSlice* pcSlice)
{
    
    cout << "xDumppppppppppppp Noura Rawan Amanyyyy :X :X :X" << endl;
    Char *pFileName = "hello.yuv";
    Bool bAdd = true;
    
    
    //        outPicYuv->dump(pFileName, isAdd);
    
    FILE* pFile;
    if (!bAdd)
    {
        pFile = fopen (pFileName, "wb");
    }
    else
    {
        pFile = fopen (pFileName, "ab");
    }
    
    int n_y =    pcSlice->getSPS()->getPicHeightInLumaSamples() * pcSlice->getSPS()->getPicWidthInLumaSamples();
    
    // Hossam: XXXX assuming 420
    int n_u = n_y/fourTwoZeroScale;
    int n_v = n_y/fourTwoZeroScale;
    
    
    
    // Dump Y
    Int chan = 0;
    const ComponentID  ch     = ComponentID(chan);
    const Int          shift  = g_bitDepth[toChannelType(ch)] - 8;
    const Int          offset = (shift>0)?(1<<(shift-1)):0;
    
    // Let is point to the address of the Y
    const Pel         *pi     = m_pcPredSearch->allData_Y;
    const Int          stride = 8;
    //    const Int          height = 240;
    //    const Int          width  = 416;
    const Int          height = 416;
    const Int          width  = 240;
    
    
    //    const Int          height = getHeight(ch);
    //    const Int          width  = getWidth(ch);
    
    //    for (Int y = 0; y < height; y++ )
    //    {
    //        for (Int x = 0; x < width; x++ )
    //        {
    //            UChar uc = (UChar)Clip3<Pel>(0, 255, (pi[x]+offset)>>shift);
    //
    //            cout << "Value Write " << Int(uc) << endl;
    //            fwrite( &uc, sizeof(UChar), 1, pFile );
    //        }
    //        pi += stride;
    //    }
    
    for (int i = 0; i < width*height; i++) {
        UChar uc = pi[i];
        
        cout << "Value Write " << Int(uc) << endl;
        fwrite( &uc, sizeof(UChar), 1, pFile );
    }
    
    
    // Destroy Y
    delete [] m_pcPredSearch->allData_Y;
    
    ////    delete [] m_pcCuEncoder->allData_U;
    ////    delete [] m_pcCuEncoder->allData_V;
    
    
    
}


//static const int fourTwoZeroScale = 4;
//// Hossam: Dump the residuals to a YUV file
//Void TEncSlice::xDumpResiduals(TComSlice* pcSlice)
//{
//
//    cout << "xDumppppppppppppp Noura Rawan Amanyyyy :X :X :X" << endl;
//    Char *pFileName = "hello.yuv";
//    Bool bAdd = true;
////    TComList<TComYuv*>::iterator iterCUs   = m_pcCuEncoder->m_scResidualFrame.begin();
//
//
////    TComPicYuv *& outPicYuv = m_pcCuEncoder->m_scResidualFrame;
//    Bool isAdd = true;
////
////    outPicYuv->dump(pFileName, isAdd);
//
//    FILE* pFile;
//    if (!bAdd)
//    {
//        pFile = fopen (pFileName, "wb");
//    }
//    else
//    {
//        pFile = fopen (pFileName, "ab");
//    }
//
//    int n_y =    pcSlice->getSPS()->getPicHeightInLumaSamples() * pcSlice->getSPS()->getPicWidthInLumaSamples();
//
//    // Hossam: XXXX assuming 420
//    int n_u = n_y/fourTwoZeroScale;
//    int n_v = n_y/fourTwoZeroScale;
//
//
////    cout << "Number of Y samples " << n_y << endl;
//
//
//    // Dump Y
////    Int chan = 0;
////      const ComponentID  ch     = COMPONENT_Y;
////    const Int          shift  = g_bitDepth[toChannelType(ch)] - 8;
////    const Int          offset = (shift>0)?(1<<(shift-1)):0;
////
////    for (Int i = 0; i < n_y; i++) {
//////        UChar uc = (UChar)Clip3<Pel>(0, 255, (m_pcCuEncoder->allData_Y[i]+offset)>>shift);
//////        fwrite( &uc, sizeof(UChar), 1, pFile );
////
////        UChar uc = m_pcCuEncoder->allData_Y[i];
////        fwrite( &uc, sizeof(UChar), 1, pFile );
////    }
////
////
////    // Dump U
////    chan = 1;
////    const ComponentID  ch2     = COMPONENT_Cb;
////    const Int          shift2  = g_bitDepth[toChannelType(ch2)] - 8;
////    const Int          offset2 = (shift>0)?(1<<(shift-1)):0;
////
////    for (Int i = 0; i < n_u; i++) {
//////        UChar uc = (UChar)Clip3<Pel>(0, 255, (m_pcCuEncoder->allData_U[i]+offset2)>>shift2);
//////        fwrite( &uc, sizeof(UChar), 1, pFile );
////
////        UChar uc = m_pcCuEncoder->allData_U[i];
////        fwrite( &uc, sizeof(UChar), 1, pFile );
////    }
////
////
////    // Dump V
////    chan = 2;
////    const ComponentID  ch3     = COMPONENT_Cr;
////    const Int          shift3  = g_bitDepth[toChannelType(ch3)] - 8;
////    const Int          offset3 = (shift>0)?(1<<(shift-1)):0;
////
////    for (Int i = 0; i < n_v; i++) {
//////        UChar uc = (UChar)Clip3<Pel>(0, 255, (m_pcCuEncoder->allData_V[i]+offset3)>>shift3);
//////        fwrite( &uc, sizeof(UChar), 1, pFile );
////
////        UChar uc = m_pcCuEncoder->allData_V[i];
////        fwrite( &uc, sizeof(UChar), 1, pFile );
////    }
////
////    fclose(pFile);
////    /////
////
////
////    // destroy the frame
//    cout << "xDumppppppppppppp nestelroooy el Destroyyyy  :X :X :X" << endl;
////    m_pcCuEncoder->m_scResidualFrame->destroy();
//
//
////
////    delete [] m_pcCuEncoder->allData_Y;
////    delete [] m_pcCuEncoder->allData_U;
////    delete [] m_pcCuEncoder->allData_V;
////
//
//    //    TComList<TComPic*>::iterator iterPic   =   m_pcGOPEncoder->getListPic()->begin();
//
//
////    TComYuv* current_cu = *(iterCUs);
//////    TComPic* current_frame = *(iterPic);
////
////    TComPicYuv* current_frame;
//
//
//
//
////    for (UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
////    {
////        if (m_outputBitDepth[channelType] == 0) m_outputBitDepth[channelType] = g_bitDepth[channelType];
////    }
////    m_pcCuEncoder->m_scResidualFrame.clear();
//
////    TComPicYuv* outPic = new TComPicYuv;
////      outPic->create(m_iSourceWidth , m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
//
//
//}

// Hossam: Scene change
Void TEncSlice:: xCreateSCResidualsBuffer(TComPic *rpcPic, TComSlice *pcSlice)
{
    
    cout << "xCreateSCResidualsBuffer  Create the residuals " << endl;
    //    Int iPicWidth  = 416;
    //    Int iPicHeight = 240;
    
    //    UInt uiMaxCUWidth  = 7;
    //    UInt uiMaxCUHeight = 4;
    
    Int iPicWidth  = 416;
    Int iPicHeight = 240;
    
    
    
    UInt uiMaxCUWidth  =  rpcPic->getPicSym()->getFrameWidthInCU();
    UInt uiMaxCUHeight =  rpcPic->getPicSym()->getFrameHeightInCU();
    UInt uiMaxCUDepth  = SC_MAX_DEPTH;
    
    
    //    m_pcCuEncoder->m_scResidualFrame = new TComPicYuv;
    //    m_pcCuEncoder->m_scResidualFrame->create(iPicWidth, iPicHeight, CHROMA_420, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth);
    
    //    m_pcCuEncoder->allData_Y = new Int[iPicWidth*iPicHeight];
    //    m_pcCuEncoder->allData_U = new Int[iPicWidth/2*iPicHeight/2];
    //    m_pcCuEncoder->allData_V = new Int[iPicWidth/2*iPicHeight/2];
    
    //    m_pcCuEncoder->allData_Y = new Pel[iPicWidth*iPicHeight];
    //    m_pcCuEncoder->allData_U = new Pel[iPicWidth/2*iPicHeight/2];
    //    m_pcCuEncoder->allData_V = new Pel[iPicWidth/2*iPicHeight/2];
    
    //    m_pcCuEncoder->allData = new Int;
    
    m_pcPredSearch->allData_Y = new Pel[iPicWidth*iPicHeight];
    
    
}


// Hossam: Scene change: Compress Slice Org
/** \param rpcPic   picture class
 */

// Hossam: Changed the TComPic&* rpcPic to TComPic* rpcPic to be able to work on a copy
// Hossam: No need since I already think I am taking copies in SC engine
// and not the Original
Void TEncSlice::compressSliceNewOrg( TComPic*& rpcPic, Int which_reference )
{
    UInt   uiStartCUAddr;
    UInt   uiBoundingCUAddr;
    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
    TEncBinCABAC* pppcRDSbacCoder = NULL;
    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
    
    // Determine the start and bounding CU address
    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
    
    // initialize cost values
    m_uiPicTotalBits  = 0;
    m_dPicRdCost      = 0;
    m_uiPicDist       = 0;
    
    
    
    
//    cout << "Monna: CompressSlice New ORGGGGGGGGG \n" << endl;
    //    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
    
    // set entropy coder
    m_pcSbacCoder->init( m_pcBinCABAC );
    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
    m_pcEntropyCoder->resetEntropy      ();
    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
    pppcRDSbacCoder->setBinCountingEnableFlag( false );
    pppcRDSbacCoder->setBinsCoded( 0 );
    
    //------------------------------------------------------------------------------
    //  Weighted Prediction parameters estimation.
    //------------------------------------------------------------------------------
    // Hossam: Weighted prediction is not visited so far --
    // calculate AC/DC values for current picture
    if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
    {
        cout << " Do I visit this case at all! " << endl;
        xCalcACDCParamSlice(pcSlice);
    }
    
    Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
    
    if ( bWp_explicit )
    {
        
        //      cout << " Do I visit this case at all! (2) " << endl;
        //------------------------------------------------------------------------------
        //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
        //------------------------------------------------------------------------------
        // Hossam: WP is not visites so far XXXXX
        if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
        {
            //      printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
        }
        
        xEstimateWPParamSlice( pcSlice );
        pcSlice->initWpScaling();
        
        // check WP on/off
        xCheckWPEnable( pcSlice );
    }
    
#if ADAPTIVE_QP_SELECTION
    if( m_pcCfg->getUseAdaptQpSelect() )
    {
        m_pcTrQuant->clearSliceARLCnt();
        if(pcSlice->getSliceType()!=I_SLICE)
        {
            Int qpBase = pcSlice->getSliceQpBase();
            pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
        }
    }
#endif
    
    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
    delete[] m_pcBufferSbacCoders;
    delete[] m_pcBufferBinCoderCABACs;
    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    }
    
    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
    {
        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
    }
    delete[] m_pcBufferLowLatSbacCoders;
    delete[] m_pcBufferLowLatBinCoderCABACs;
    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    
    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
    UInt      uiTileCol               = 0;
    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
    if( depSliceSegmentsEnabled )
    {
        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
        {
            UInt uiSubStrm=0;
            if( m_pcCfg->getWaveFrontsynchro() )
            {
                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
                if ( pCurrentTile->getTileWidth() < 2)
                {
                    CTXMem[0]->loadContexts(m_pcSbacCoder);
                }
            }
            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
        }
        else
        {
            if(m_pcCfg->getWaveFrontsynchro())
            {
                CTXMem[1]->loadContexts(m_pcSbacCoder);
            }
            CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
    }
    
    
    // for every CU in slice
    UInt uiEncCUOrder;
    
    
    
    // Hossam: Scene Change
    // xCreate the YUV residuals buffer
    //    xCreateSCResidualsBuffer(rpcPic, pcSlice);
    
    
    //    cout << "Hanteera wedding Start address: " << uiStartCUAddr/rpcPic->getNumPartInCU() << endl; // --> 0
    //    cout << "Hanteera wedding End address: " << (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU() << endl; // --> 28
    
    
    //    cout << "Best Plan Ticket: Width in CUs: " << uiWidthInLCUs << endl; // 6.5 CUs rounded up to 7
    //    cout << "Best Plan Ticket: Height in CUs: " << rpcPic->getPicSym()->getFrameHeightInCU() << endl; // 3.75 up to 4
    
    
    // 256 partitions cause we have 4x4 blocks
    //    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        // initialize CU encoder
        
#if SC_ENABLE_PRINT
        // Problem here
        cout << "uiCUAddr: " << uiCUAddr << endl;
        
#endif
        
        
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        
        
//       cout << "TEncSlice: CompressSlice SC: pcCu: " <<  pcCU->getCUMvField(RefPicList(0))->getRefIdx(0) << endl;

        //      cout << "isNil: " << (pcCU == NULL) << endl;
        //        pcCU->initCU( rpcPic, uiCUAddr );
//        pcCU->initCUNew( rpcPic, uiCUAddr );
        
        pcCU->initCUNewOrg( rpcPic, uiCUAddr );
        
//        cout << "TEncSlice: CompressSlice SC: pcCu: " <<  pcCU->getCUMvField(RefPicList(0))->getRefIdx(0) << endl;

        
        
        
        // inherit from TR if necessary, select substream to use.
        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
        UInt uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
        UInt uiCol     = uiCUAddr % uiWidthInLCUs;
        UInt uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
        
        if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
        {
            // We'll sync if the TR is available.
            TComDataCU *pcCUUp = pcCU->getCUAbove();
            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
            TComDataCU *pcCUTR = NULL;
            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
            {
                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
            }
            if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
                  (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
                  ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
                  )
                )
            {
                // TR not available.
            }
            else
            {
                // TR is available, we use it.
                ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
            }
        }
        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code
        
        // reset the entropy coder
        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
        {
            SliceType sliceType = pcSlice->getSliceType();
            if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
            {
                sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
            }
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
            m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
            m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
        }
        
        // set go-on entropy coder
        m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        
        ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);
        
        Double oldLambda = m_pcRdCost->getLambda();
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int estQP        = pcSlice->getSliceQp();
            Double estLambda = -1.0;
            Double bpp       = -1.0;
            
            if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
            {
                estQP = pcSlice->getSliceQp();
            }
            else
            {
                bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
                if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
                }
                else
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
                    estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
                }
                
                estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );
                
                m_pcRdCost->setLambda(estLambda);
                
#if RDOQ_CHROMA_LAMBDA
                // set lambda for RDOQ
                const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
                const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
                m_pcTrQuant->setLambdas( lambdaArray );
#else
                m_pcTrQuant->setLambda( estLambda );
#endif
            }
            
            m_pcRateCtrl->setRCQP( estQP );
#if ADAPTIVE_QP_SELECTION
            pcCU->getSlice()->setSliceQpBase( estQP );
#endif
        }
        
        // run CU encoder
            m_pcCuEncoder->compressCUNewOrg( pcCU, which_reference );
        //        m_pcCuEncoder->compressCU( pcCU );
//        m_pcCuEncoder->compressCUNew( pcCU );
        
        // Hossam: Save the CU residual data in the Residual frame buffer
        // Hopefully this does not casue problems SEG faults..etc
        
        //        TComPicYuv*             m_apcPicYuvResi;                      ///< residual picture buffer
        
#if SC_ENABLE_PRINT
        cout << "(((((Run the CU encoder: " << uiEncCUOrder << ", pcCU->getAddr()" << pcCU->getAddr() << " )))))" << endl;
#endif
       
        
        /// Copy the residual
        m_pcCuEncoder->scBestResidual[SC_BEST_BUFFER_IND]->copyToPicYuv(m_apcPicYuvResi, pcCU->getAddr(), pcCU->getZorderIdxInCU());
        
        
        
//        //-----Debug-----// failed the test
//                cout <<    pcCU->getAddr() << ", " << pcCU->getZorderIdxInCU() <<  ", After CompressSC Reference: " << endl;
//                pcCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//        
//                cout << ", After CompressSC SC: " << endl;
//                pcCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//        //-----Debug-----
        
        //          cout << "(((((Run the CU encoder: " << uiEncCUOrder << " )))))" << endl;
        
        
        // restore entropy coder to an initial stage
        m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
        m_pcBitCounter = &pcBitCounters[uiSubStrm];
        pppcRDSbacCoder->setBinCountingEnableFlag( true );
        m_pcBitCounter->resetBits();
        pppcRDSbacCoder->setBinsCoded( 0 );
        
        
        // Don't encode the CU!
        //        m_pcCuEncoder->encodeCU( pcCU );
        
        pppcRDSbacCoder->setBinCountingEnableFlag( false );
        if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
        {
            pcSlice->setNextSlice( true );
            break;
        }
        if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSliceSegment( true );
            break;
        }
        
        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
        
        //Store probabilties of second LCU in line into buffer
        if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
        {
            m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
        }
        
        // Hossam: Scene change: Rate control is disabled
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int actualQP        = g_RCInvalidQPValue;
            Double actualLambda = m_pcRdCost->getLambda();
            Int actualBits      = pcCU->getTotalBits();
            Int numberOfEffectivePixels    = 0;
            for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
            {
                if ( pcCU->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pcCU->isSkipped( idx ) ) )
                {
                    numberOfEffectivePixels = numberOfEffectivePixels + 16;
                    break;
                }
            }
            
            if ( numberOfEffectivePixels == 0 )
            {
                actualQP = g_RCInvalidQPValue;
            }
            else
            {
                actualQP = pcCU->getQP( 0 );
            }
            
//            cout << "OLD LAMBDA IN COMpress SLICe: " << oldLambda << endl;
            // Hossam: Scene change: OlD lambda ORG
            m_pcRdCost->setLambda(oldLambda);
            m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                                     pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
        }
        
        m_uiPicTotalBits += pcCU->getTotalBits();
        m_dPicRdCost     += pcCU->getTotalCost();
        m_uiPicDist      += pcCU->getTotalDistortion();
    }
    if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
    {
        pcSlice->setNextSlice( true );
    }
    if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
    {
        if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSlice( true );
        }
        else
        {
            pcSlice->setNextSliceSegment( true );
        }
    }
    if( depSliceSegmentsEnabled )
    {
        if (m_pcCfg->getWaveFrontsynchro())
        {
            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
        }
        CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
    }
    xRestoreWPparam( pcSlice );
    
    
    
    // Hossam: Scene Change
    //        xDumpResiduals(pcSlice);
    
    //    scBestResidual[0]->mockPrintFromSlice(rpcCU->getZorderIdxInCU());
    //    m_apcPicYuvResi->mockPrintComponent(COMPONENT_Y, 0);
    
    
    //    m_apcPicYuvResi->mockPrintComponentLineBlk(COMPONENT_Y, 0, m_apcPicYuvResi->getWidth(COMPONENT_Y), m_apcPicYuvResi->getHeight(COMPONENT_Y), true, 400);
    
    //        cout << "EXIT " << endl;
    //        if(pcSlice->getPOC() == 2)
//                exit (EXIT_FAILURE);
    
    
} // end compressSliceNewORG


// Hossam: Scene change
/** \param rpcPic   picture class
 */

// Hossam: Changed the TComPic&* rpcPic to TComPic* rpcPic to be able to work on a copy
// Hossam: No need since I already think I am taking copies in SC engine
// and not the Original
Void TEncSlice::compressSliceNew( TComPic*& rpcPic )
{
    UInt   uiStartCUAddr;
    UInt   uiBoundingCUAddr;
    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
    TEncBinCABAC* pppcRDSbacCoder = NULL;
    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
    
    // Determine the start and bounding CU address
    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
    
    // initialize cost values
    m_uiPicTotalBits  = 0;
    m_dPicRdCost      = 0;
    m_uiPicDist       = 0;
    
    //    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
    
    // set entropy coder
    m_pcSbacCoder->init( m_pcBinCABAC );
    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
    m_pcEntropyCoder->resetEntropy      ();
    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
    pppcRDSbacCoder->setBinCountingEnableFlag( false );
    pppcRDSbacCoder->setBinsCoded( 0 );
    
    //------------------------------------------------------------------------------
    //  Weighted Prediction parameters estimation.
    //------------------------------------------------------------------------------
    // Hossam: Weighted prediction is not visited so far --
    // calculate AC/DC values for current picture
    if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
    {
        cout << " Do I visit this case at all! " << endl;
        xCalcACDCParamSlice(pcSlice);
    }
    
    Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
    
    if ( bWp_explicit )
    {
        
        //      cout << " Do I visit this case at all! (2) " << endl;
        //------------------------------------------------------------------------------
        //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
        //------------------------------------------------------------------------------
        // Hossam: WP is not visites so far XXXXX
        if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
        {
            //      printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
        }
        
        xEstimateWPParamSlice( pcSlice );
        pcSlice->initWpScaling();
        
        // check WP on/off
        xCheckWPEnable( pcSlice );
    }
    
#if ADAPTIVE_QP_SELECTION
    if( m_pcCfg->getUseAdaptQpSelect() )
    {
        m_pcTrQuant->clearSliceARLCnt();
        if(pcSlice->getSliceType()!=I_SLICE)
        {
            Int qpBase = pcSlice->getSliceQpBase();
            pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
        }
    }
#endif
    
    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
    delete[] m_pcBufferSbacCoders;
    delete[] m_pcBufferBinCoderCABACs;
    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    }
    
    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
    {
        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
    }
    delete[] m_pcBufferLowLatSbacCoders;
    delete[] m_pcBufferLowLatBinCoderCABACs;
    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    
    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
    UInt      uiTileCol               = 0;
    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
    if( depSliceSegmentsEnabled )
    {
        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
        {
            UInt uiSubStrm=0;
            if( m_pcCfg->getWaveFrontsynchro() )
            {
                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
                if ( pCurrentTile->getTileWidth() < 2)
                {
                    CTXMem[0]->loadContexts(m_pcSbacCoder);
                }
            }
            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
        }
        else
        {
            if(m_pcCfg->getWaveFrontsynchro())
            {
                CTXMem[1]->loadContexts(m_pcSbacCoder);
            }
            CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
    }
    
    
    // for every CU in slice
    UInt uiEncCUOrder;
    
    
    
    // Hossam: Scene Change
    // xCreate the YUV residuals buffer
    //    xCreateSCResidualsBuffer(rpcPic, pcSlice);
    
    
    //    cout << "Hanteera wedding Start address: " << uiStartCUAddr/rpcPic->getNumPartInCU() << endl; // --> 0
    //    cout << "Hanteera wedding End address: " << (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU() << endl; // --> 28
    
    
    //    cout << "Best Plan Ticket: Width in CUs: " << uiWidthInLCUs << endl; // 6.5 CUs rounded up to 7
    //    cout << "Best Plan Ticket: Height in CUs: " << rpcPic->getPicSym()->getFrameHeightInCU() << endl; // 3.75 up to 4
    
    
    // 256 partitions
    //    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        // initialize CU encoder
        
#if SC_ENABLE_PRINT
        // Problem here
        cout << "uiCUAddr: " << uiCUAddr << endl;
        
#endif
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        
        
        
        //      cout << "isNil: " << (pcCU == NULL) << endl;
        //        pcCU->initCU( rpcPic, uiCUAddr );
        pcCU->initCUNew( rpcPic, uiCUAddr );
        
        
        
        // inherit from TR if necessary, select substream to use.
        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
        UInt uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
        UInt uiCol     = uiCUAddr % uiWidthInLCUs;
        UInt uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
        
        if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
        {
            // We'll sync if the TR is available.
            TComDataCU *pcCUUp = pcCU->getCUAbove();
            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
            TComDataCU *pcCUTR = NULL;
            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
            {
                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
            }
            if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
                  (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
                  ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
                  )
                )
            {
                // TR not available.
            }
            else
            {
                // TR is available, we use it.
                ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
            }
        }
        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code
        
        // reset the entropy coder
        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
        {
            SliceType sliceType = pcSlice->getSliceType();
            if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
            {
                sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
            }
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
            m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
            m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
        }
        
        // set go-on entropy coder
        m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        
        ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);
        
        Double oldLambda = m_pcRdCost->getLambda();
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int estQP        = pcSlice->getSliceQp();
            Double estLambda = -1.0;
            Double bpp       = -1.0;
            
            if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
            {
                estQP = pcSlice->getSliceQp();
            }
            else
            {
                bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
                if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
                }
                else
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
                    estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
                }
                
                estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );
                
                m_pcRdCost->setLambda(estLambda);
                
#if RDOQ_CHROMA_LAMBDA
                // set lambda for RDOQ
                const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
                const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
                m_pcTrQuant->setLambdas( lambdaArray );
#else
                m_pcTrQuant->setLambda( estLambda );
#endif
            }
            
            m_pcRateCtrl->setRCQP( estQP );
#if ADAPTIVE_QP_SELECTION
            pcCU->getSlice()->setSliceQpBase( estQP );
#endif
        }
        
        // run CU encoder
        //        m_pcCuEncoder->compressCU( pcCU );
        m_pcCuEncoder->compressCUNew( pcCU );
        
        // Hossam: Save the CU residual data in the Residual frame buffer
        // Hopefully this does not casue problems SEG faults..etc
        
        //        TComPicYuv*             m_apcPicYuvResi;                      ///< residual picture buffer
        
#if SC_ENABLE_PRINT
        cout << "(((((Run the CU encoder: " << uiEncCUOrder << ", pcCU->getAddr()" << pcCU->getAddr() << " )))))" << endl;
#endif
        m_pcCuEncoder->scBestResidual[SC_BEST_BUFFER_IND]->copyToPicYuv(m_apcPicYuvResi, pcCU->getAddr(), pcCU->getZorderIdxInCU());
        
        
        
        //          cout << "(((((Run the CU encoder: " << uiEncCUOrder << " )))))" << endl;
        
        
        // restore entropy coder to an initial stage
        m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
        m_pcBitCounter = &pcBitCounters[uiSubStrm];
        pppcRDSbacCoder->setBinCountingEnableFlag( true );
        m_pcBitCounter->resetBits();
        pppcRDSbacCoder->setBinsCoded( 0 );
        
        
        // Don't encode the CU!
        //        m_pcCuEncoder->encodeCU( pcCU );
        
        pppcRDSbacCoder->setBinCountingEnableFlag( false );
        if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
        {
            pcSlice->setNextSlice( true );
            break;
        }
        if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSliceSegment( true );
            break;
        }
        
        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
        
        //Store probabilties of second LCU in line into buffer
        if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
        {
            m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
        }
        
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int actualQP        = g_RCInvalidQPValue;
            Double actualLambda = m_pcRdCost->getLambda();
            Int actualBits      = pcCU->getTotalBits();
            Int numberOfEffectivePixels    = 0;
            for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
            {
                if ( pcCU->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pcCU->isSkipped( idx ) ) )
                {
                    numberOfEffectivePixels = numberOfEffectivePixels + 16;
                    break;
                }
            }
            
            if ( numberOfEffectivePixels == 0 )
            {
                actualQP = g_RCInvalidQPValue;
            }
            else
            {
                actualQP = pcCU->getQP( 0 );
            }
            m_pcRdCost->setLambda(oldLambda);
            m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                                     pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
        }
        
        m_uiPicTotalBits += pcCU->getTotalBits();
        m_dPicRdCost     += pcCU->getTotalCost();
        m_uiPicDist      += pcCU->getTotalDistortion();
    }
    if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
    {
        pcSlice->setNextSlice( true );
    }
    if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
    {
        if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSlice( true );
        }
        else
        {
            pcSlice->setNextSliceSegment( true );
        }
    }
    if( depSliceSegmentsEnabled )
    {
        if (m_pcCfg->getWaveFrontsynchro())
        {
            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
        }
        CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
    }
    xRestoreWPparam( pcSlice );
    
    
    
    // Hossam: Scene Change
    //        xDumpResiduals(pcSlice);
    
    //    scBestResidual[0]->mockPrintFromSlice(rpcCU->getZorderIdxInCU());
    //    m_apcPicYuvResi->mockPrintComponent(COMPONENT_Y, 0);
    
    
    //    m_apcPicYuvResi->mockPrintComponentLineBlk(COMPONENT_Y, 0, m_apcPicYuvResi->getWidth(COMPONENT_Y), m_apcPicYuvResi->getHeight(COMPONENT_Y), true, 400);
    
    //        cout << "EXIT " << endl;
    //        if(pcSlice->getPOC() == 2)
    //            exit (EXIT_FAILURE);
    
} // end compressSliceNew

//Void TEncSlice::compressSliceNew( TComPic*& rpcPic )
//{
//    UInt   uiStartCUAddr;
//    UInt   uiBoundingCUAddr;
//
//
//    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
//    TEncBinCABAC* pppcRDSbacCoder = NULL;
//
//
//    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
//    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
//
//    // initialize cost values
//    m_uiPicTotalBits  = 0;
//    m_dPicRdCost      = 0;
//    m_uiPicDist       = 0;
//
//    // Up-till here we can move the encoder to almost the same performance 1/
//    //    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
//
//    // Hossam: Removed the entropy card parts XXX
//
//    // set entropy coder
//    m_pcSbacCoder->init( m_pcBinCABAC );
//    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
//    m_pcEntropyCoder->resetEntropy      ();
//    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
//    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
//    pppcRDSbacCoder->setBinCountingEnableFlag( false );
//    pppcRDSbacCoder->setBinsCoded( 0 );
//
//    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
//    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
//    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
//    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
//    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
//    delete[] m_pcBufferSbacCoders;
//    delete[] m_pcBufferBinCoderCABACs;
//    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
//    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
//    for (Int ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
//    }
//    for (UInt ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
//    }
//
//    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
//    {
//        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
//    }
//    delete[] m_pcBufferLowLatSbacCoders;
//    delete[] m_pcBufferLowLatBinCoderCABACs;
//    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
//    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
//    for (Int ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
//    }
//    for (UInt ui = 0; ui < uiTilesAcross; ui++)
//        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
//
//
//
//
//    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
//    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
//    UInt      uiTileCol               = 0;
//    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
//    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
//    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
//    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
//
//    //    // Hossam: According to Nan, it's always false XXXX
//    //    if( depSliceSegmentsEnabled )
//    //    {
//    //        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
//    //        {
//    //            UInt uiSubStrm=0;
//    //            if( m_pcCfg->getWaveFrontsynchro() )
//    //            {
//    //                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
//    //                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
//    //                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    //                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//    //                if ( pCurrentTile->getTileWidth() < 2)
//    //                {
//    //                    CTXMem[0]->loadContexts(m_pcSbacCoder);
//    //                }
//    //            }
//    //            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//    //            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//    //        }
//    //        else
//    //        {
//    //            if(m_pcCfg->getWaveFrontsynchro())
//    //            {
//    //                CTXMem[1]->loadContexts(m_pcSbacCoder);
//    //            }
//    //            CTXMem[0]->loadContexts(m_pcSbacCoder);
//    //        }
//    //    }
//    //------------------------------------------------------------------------------
//    // Hossam: Actual CU encoding
//
//    // for every CU in slice
//    UInt uiEncCUOrder;
//
//
//    // Hossam: 256 mazboot
//    //    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
//    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
//        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
//        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
//    {
//        cout << " uiCUAddr " << uiCUAddr << endl;
//
//        TComDataCU*& pcCU = rpcPic->getCU(uiCUAddr);
//        pcCU->initCUNew( rpcPic, uiCUAddr );
//
//        cout << "Init CU done  " << endl;
//        // run CU encoder
//        m_pcCuEncoder->compressCUNew( pcCU );
//
//
//        cout << "Done with CU DonEEEEEEEEEE Erb street: " << uiEncCUOrder << endl;
//        // Hossam: Removed the steps after that XXXX
//
//        // Added the bits and RD cost calculations
//        m_uiPicTotalBits += pcCU->getTotalBits();
//        m_dPicRdCost     += pcCU->getTotalCost();
//        m_uiPicDist      += pcCU->getTotalDistortion();
//
//    }
//
//
//} // end compressSliceNew

//Void TEncSlice::compressSliceNew( TComPic*& rpcPic )
//{
//    UInt   uiStartCUAddr;
//    UInt   uiBoundingCUAddr;
//
//
//    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
//    TEncBinCABAC* pppcRDSbacCoder = NULL;
//
//
//    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
//    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
//
//    // initialize cost values
//    m_uiPicTotalBits  = 0;
//    m_dPicRdCost      = 0;
//    m_uiPicDist       = 0;
//
//    // Up-till here we can move the encoder to almost the same performance 1/
////    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
//
//   // Hossam: Removed the entropy card parts XXX
//
//    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
//    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
//    UInt      uiTileCol               = 0;
//    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
//    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
//    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
//    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
//
////    // Hossam: According to Nan, it's always false
////    if( depSliceSegmentsEnabled )
////    {
////        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
////        {
////            UInt uiSubStrm=0;
////            if( m_pcCfg->getWaveFrontsynchro() )
////            {
////                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
////                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
////                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
////                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
////                if ( pCurrentTile->getTileWidth() < 2)
////                {
////                    CTXMem[0]->loadContexts(m_pcSbacCoder);
////                }
////            }
////            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
////            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
////        }
////        else
////        {
////            if(m_pcCfg->getWaveFrontsynchro())
////            {
////                CTXMem[1]->loadContexts(m_pcSbacCoder);
////            }
////            CTXMem[0]->loadContexts(m_pcSbacCoder);
////        }
////    }
//    //------------------------------------------------------------------------------
//    // Hossam: Actual CU encoding
//
//    // for every CU in slice
//    UInt uiEncCUOrder;
//
//
//    // Hossam: 256 mazboot
////    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
//    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
//        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
//        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
//    {
//        cout << " uiCUAddr " << uiCUAddr << endl;
//
//        TComDataCU*& pcCU = rpcPic->getCU(uiCUAddr);
//        pcCU->initCUNew( rpcPic, uiCUAddr );
//
//        cout << "Init CU done  " << endl;
//        // run CU encoder
//        m_pcCuEncoder->compressCUNew( pcCU );
//
//
//        cout << "Done with CU DonEEEEEEEEEE Erb street: " << uiEncCUOrder << endl;
//        // Hossam: Removed the steps after that XXXX
//
//        // Added the bits and RD cost calculations
//        m_uiPicTotalBits += pcCU->getTotalBits();
//        m_dPicRdCost     += pcCU->getTotalCost();
//        m_uiPicDist      += pcCU->getTotalDistortion();
//
//    }
//
//
//} // end compressSliceNew


// Hossam: Scene change
Void TEncSlice::xExtractSliceInfo(TComPic *&rpcPic)
{
    UInt   uiStartCUAddr;
    UInt   uiBoundingCUAddr;
    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
 //   TEncBinCABAC* pppcRDSbacCoder = NULL;
    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
    
    
//    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
//    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
//    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
//    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
//    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
//    
//    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
//    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
//    UInt      uiTileCol               = 0;
//    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
//    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
//    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
    
    
//    cout << "Slice# " << pcSlice->getPOC() << ", Intra Count: " << intra_modes << endl;
    
    // Init those for the slice
    pcSlice->intra_modes       = 0;
    pcSlice->slice_total_parts = 0;
    
    // Init the reference utilization
    for(int i = 0 ; i < 4 ; i++)
    {
        reference_utilization_counts[i] = 0;
    }
    
    // Per frame reference utilization -- Init reference utitlizations per frame
    // Push back a new dummy item in the reference utitilization rates of your frame, and then modify it
    reference1_utilization_rates_perFrame.push_back(0); reference2_utilization_rates_perFrame.push_back(0);
    reference3_utilization_rates_perFrame.push_back(0); reference4_utilization_rates_perFrame.push_back(0);

    
    // for every CU in slice
    UInt uiEncCUOrder;
    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        
        
        
        // Fetch the CU
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        
//         XXXHossam: when you call this function twice, pcCU has accumulated intra modes. set it to 0 for now
        pcCU->intra_modes = 0;
        
        // Call the CU encoder to do the recursion
        m_pcCuEncoder->xExtractCUInfo(pcCU, 0, 0, 0);
        
//        cout << "CU# " << uiEncCUOrder << ", " << pcCU->intra_modes << endl;
        
        // Accumlate the intra modes count from the CU
        intra_modes += pcCU->intra_modes;
        
        // Accumlate the total slice parts
        pcSlice->slice_total_parts += pcCU->cu_total_parts;
        
        
        if (pcSlice->getPOC() != 0)
        {
            // Accumlate the reference utitlization (the number of utilization in 8x8 blocks over the number of 8x8 per CTU)
            reference_utilization_counts[0] += pcCU->reference_utitilization_counts[0];
            reference_utilization_counts[1] += pcCU->reference_utitilization_counts[1];
            reference_utilization_counts[2] += pcCU->reference_utitilization_counts[2];
            reference_utilization_counts[3] += pcCU->reference_utitilization_counts[3];
            
            UInt rPOC = pcSlice->getPOC() % 4;
            reference_utitilization_rates[rPOC][0] += pcCU->reference_utitilization_counts[0];
            reference_utitilization_rates[rPOC][1] += pcCU->reference_utitilization_counts[1];
            reference_utitilization_rates[rPOC][2] += pcCU->reference_utitilization_counts[2];
            reference_utitilization_rates[rPOC][3] += pcCU->reference_utitilization_counts[3];
            
            // Update the reference utilization of this frame
            reference1_utilization_rates_perFrame.back() += pcCU->reference_utitilization_counts[0];
            reference2_utilization_rates_perFrame.back() += pcCU->reference_utitilization_counts[1];
            reference3_utilization_rates_perFrame.back() += pcCU->reference_utitilization_counts[2];
            reference4_utilization_rates_perFrame.back() += pcCU->reference_utitilization_counts[3];

            
            // Reset the untilization for each CU after you use it
            pcCU->resetReferenceUtitlization();
            
            
        } // end if (pcsSlice->getPOC() != 0)
        
        
//        if(pcCU->intra_modes > 0) cout << "CU# " << uiEncCUOrder << ", " << pcCU->intra_modes  << ", tot: " << pcCU->cu_total_parts << endl;
        
//            cout << "------> CU# " << uiEncCUOrder << ", " << pcCU->intra_modes  << ", tot: " << pcCU->cu_total_parts << endl;
        
//        cout << "Cu total parts: " << pcCU->cu_total_parts << endl;
        
    } // end for loop
    
    
#if IS_DEBUG_CALC_INTERDEP_REF_UTILIZATION
    printReferenceUtitilization(pcSlice->getPOC());
#endif
    
    // Fetch the intra modes from the Slice Encoder
    pcSlice->intra_modes = this->intra_modes;
    
    // Reset the intra modes of the Slice Encoder
    this->intra_modes = 0;
    
    // Reset the reference utitilization
    // Init the reference utilization
    for(int i = 0 ; i < 4 ; i++)
    {
        reference_utilization_counts[i] = 0;
    }
    
    
    
    // cout << "Slice# " << pcSlice->getPOC() << ", Intra Count: " << pcSlice->intra_modes << endl;
    const UInt uiFrameWidth  = pcSlice->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Y); // get the residual frame
    const UInt uiFrameHeight = pcSlice->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Y); // get the residual frame
    Double max_8x8 = ceil((1.0*uiFrameWidth * uiFrameHeight) / (8 * 8));
    Double percentage = 100.0*pcSlice->intra_modes / max_8x8;
    cout << "Total number of intra modes in slice " << pcSlice->getPOC() << ", is: " << pcSlice->intra_modes << "/" << max_8x8 <<  ", percentage: " << percentage << endl;
    string fileName = "";
    std::ostringstream oss;
    oss << "Gen//Seq-TXT//" << g_input_FileName << "_intra" << g_qpInit << ".txt";
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    FILE* sastre_pFile = fopen (pYUVFileName, "at");
    fprintf(sastre_pFile, "%f\n", percentage);
    fclose(sastre_pFile);
    
    // Quick test
//    cout << "Quick test assertion " << (reference_utitilization_rates[pcSlice->getPOC() % 4][0] == reference1_utilization_rates_perFrame.back()) << endl;
//    cout << "Quick test assertion " << (reference_utitilization_rates[pcSlice->getPOC() % 4][1] == reference2_utilization_rates_perFrame.back()) << endl;
//    cout << "Quick test assertion " << (reference_utitilization_rates[pcSlice->getPOC() % 4][2] == reference3_utilization_rates_perFrame.back()) << endl;
//    cout << "Quick test assertion " << (reference_utitilization_rates[pcSlice->getPOC() % 4][3] == reference4_utilization_rates_perFrame.back()) << endl;
    
    
//    cout << "Quick test org assertion1 " << (reference_utitilization_rates[pcSlice->getPOC() % 4][0]) << endl;
//    cout << "Quick test org assertion2 " << (reference_utitilization_rates[pcSlice->getPOC() % 4][1]) << endl;
//    cout << "Quick test org assertion3 " << (reference_utitilization_rates[pcSlice->getPOC() % 4][2]) << endl;
//    cout << "Quick test org assertion4 " << (reference_utitilization_rates[pcSlice->getPOC() % 4][3]) << endl;
    
//    cout << "\nQuick test new assertion1 " << (reference1_utilization_rates_perFrame.back()) << endl;
//    cout << "Quick test new assertion2 " << (reference2_utilization_rates_perFrame.back()) << endl;
//    cout << "Quick test new assertion3 " << (reference3_utilization_rates_perFrame.back()) << endl;
//    cout << "Quick test new assertion4 " << (reference4_utilization_rates_perFrame.back()) << endl;
    //
    
    
    // Calculate the percentage of reference utitilization weights
    reference1_utilization_rates_perFrame.back() = 100.0*reference1_utilization_rates_perFrame.back() / (max_8x8);
    reference2_utilization_rates_perFrame.back() = 100.0*reference2_utilization_rates_perFrame.back() / (max_8x8);
    reference3_utilization_rates_perFrame.back() = 100.0*reference3_utilization_rates_perFrame.back() / (max_8x8);
    reference4_utilization_rates_perFrame.back() = 100.0*reference4_utilization_rates_perFrame.back() / (max_8x8);
    
    

    /*
     for(int i = 0; i < 4; ++i)
    {
        oss.clear(); oss.str("");
        oss << "Gen//Seq-TXT//" << g_input_FileName << "-Util-" << (i+1) << ".txt";
        fileName = oss.str();
        pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
        sastre_pFile = fopen (pYUVFileName, "at");
        switch (i) {
            case 0:
                fprintf(sastre_pFile, "%f\n", reference1_utilization_rates_perFrame.back());
                break;
            case 1:
                fprintf(sastre_pFile, "%f\n", reference2_utilization_rates_perFrame.back());

                break;
            case 2:
                fprintf(sastre_pFile, "%f\n", reference3_utilization_rates_perFrame.back());

                break;
            case 3:
                fprintf(sastre_pFile, "%f\n", reference4_utilization_rates_perFrame.back());

                break;
                
            default:
                fprintf(sastre_pFile, "%f\n", reference1_utilization_rates_perFrame.back());
                break;
        }
        fclose(sastre_pFile);
    }
    */
    
    // Quick test
//    Double test_sum = reference1_utilization_rates_perFrame.back() + reference2_utilization_rates_perFrame.back() + reference3_utilization_rates_perFrame.back() + reference4_utilization_rates_perFrame.back();
//    cout << "Weights for frame " << pcSlice->getPOC() << " are " << reference1_utilization_rates_perFrame.back()
//    << ", " << reference2_utilization_rates_perFrame.back() << ", " << reference3_utilization_rates_perFrame.back() <<
//    ", " << reference4_utilization_rates_perFrame.back() << ", sum: " << test_sum << endl;
    //
    
}

#if IS_YAO_SCD
static Bool yao_update_lock = true;
static UInt yao_clock = 1;
static FILE* yao_pFile = NULL;
Bool TEncSlice::isSceneChangeYao(TComSlice* pcSlice, TComPic* pcPic)
{
    if (pcSlice->getPOC() == 0) {
        return false;
    }
    
    Bool isSceneChange = false;
    
    // Fetch the intra modes
    UInt intra_modes = pcSlice->intra_modes;

    
    // Calculate the mean
    if (yao_update_lock) {
        yao_intra_prev_mean    = yao_intra_current_mean;
        yao_update_lock = false;
    }

    yao_intra_current_mean = ( 1.0*(yao_clock-1) / yao_clock ) * yao_intra_prev_mean + ( ( 1.0*intra_modes) / yao_clock);
    
    
    //        Int n       = ycArray.size();
    //        Double yc_n = ycArray.back();
    //        variance    = pow( yc_n - mean_n , 2) / (n - 1)
    //        + ( ( 1.0 * (n - 1) )  / n ) * prev_variance;
    //
    
    
    yao_intra_prev_std    =  yao_intra_current_std;
    
    if(yao_clock > 1)
    {
        yao_intra_current_std = sqrt(
                                     pow(intra_modes - yao_intra_current_mean, 2) / (yao_clock-1)
                                     + ( ( 1.0 * (yao_clock - 1) )  / yao_clock ) * pow(yao_intra_prev_std , 2)
                                     );
    }
    else{
        yao_intra_current_std = 0;
    }
    
    
    // Calculate D(k, i)
    yao_content_variation = intra_modes - yao_intra_prev_mean;
    
    // total Number of CUs
    //         Double nTotal = pcPic-> getFrameWidthInCU() * pcPic->getFrameHeightInCU();
    //        Double nTotal  = pcPic-> getNumCUsInFrame();
    
    // Hossam-> this should be fixed ---
    Double nTotal = pcSlice -> slice_total_parts;

    
    // Calculate t-up and t-down
    yao_tUP   = 0.95*nTotal;
    yao_tDown = 0.50*nTotal;
    
    // Calculate average QP so far
    yao_average_QP = ( 1.0*(yao_clock-1) / yao_clock ) * yao_average_QP + ( ( 1.0*pcSlice->getSliceQp()) / yao_clock);
    
    yao_threshold = yao_intra_prev_std * (4 + 0.5 * log2( (176.0*144) / ( pcPic->getPicYuvOrg()->getWidth(COMPONENT_Y) * pcPic->getPicYuvOrg()->getHeight(COMPONENT_Y)) )
                                          + ceil(yao_average_QP/32)
                                          );
    
    //        yao_threshold = yao_intra_prev_std * (4 + 0.5 * log2( (416.0*240) / ( pcPic->getPicYuvOrg()->getWidth(COMPONENT_Y) * pcPic->getPicYuvOrg()->getHeight(COMPONENT_Y)) )
    //                                              + ceil(yao_average_QP/32)
    //                                              );
    
    
    //        cout << "inner log: " << (176.0*144) / ( pcPic->getPicYuvOrg()->getWidth(COMPONENT_Y) * pcPic->getPicYuvOrg()->getHeight(COMPONENT_Y)) << endl;
    //        cout << "log2: " << log2( 1.0*(416*240) / ( pcPic->getPicYuvOrg()->getWidth(COMPONENT_Y) * pcPic->getPicYuvOrg()->getHeight(COMPONENT_Y)) ) << endl;
    //        cout << "ceil: " << ceil(yao_average_QP/32) << endl;
    
    
    // Will use the threshold
    yao_thresholdFinal = max( yao_tDown - yao_intra_prev_mean,  min(yao_threshold, yao_tUP - yao_intra_prev_mean)  );
    //        yao_thresholdFinal = yao_threshold; // Use non-refined threshold
    
    
    
   
    
    //        cout << "frame rate: " << m_pcCfg->getFrameRate() << ", bit rate: " << m_pcCfg->getTargetBitrate() << endl;
    
    //        cout << "Number of CUs: " << pcPic-> getNumCUsInFrame() << " parts h: " << pcPic->getNumPartInHeight() << ", w: " <<  pcPic->getNumPartInWidth() << endl;
    //        cout << "-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=" << endl;
    
    
    // Update the clock --> After the frame encoding
//    yao_clock = yao_clock + 1;
    
//    if (pcSlice->getPOC() == 2 || pcSlice->getPOC()==1) {
//    if (pcSlice->getPOC() == 2) {
//        cout << "Yao variables--> " << yao_clock << endl;
//        cout << "Intra prev mean: " << yao_intra_prev_mean << endl;
//        cout << "Intra curr mean: " << yao_intra_current_mean << endl;
//        cout << "Intra prev std: " << yao_intra_prev_std << endl;
//        cout << "Intra curr std: " << yao_intra_current_std << endl;
//        cout << "Intra yao threshold: " << yao_threshold << endl;
//        cout << "Intra yao threshold FINAL: " << yao_thresholdFinal << ", content variation: " << yao_content_variation << endl;
//
//    }
//    cout << "Yao variables--> " << yao_clock << endl;
//    cout << "Intra prev mean: " << yao_intra_prev_mean << endl;
//    cout << "Intra curr mean: " << yao_intra_current_mean << endl;
//    cout << "Intra prev std: " << yao_intra_prev_std << endl;
//    cout << "Intra curr std: " << yao_intra_current_std << endl;
//    cout << "Intra yao threshold: " << yao_threshold << endl;
//    cout << "Intra yao threshold FINAL: " << yao_thresholdFinal << ", content variation: " << yao_content_variation << endl;

    
    // Scene change check
    //        if(yao_thresholdFinal != 0 && yao_content_variation > yao_thresholdFinal)
    if(yao_clock > 5 && yao_content_variation > yao_thresholdFinal)
    {
        isSceneChange = true;
        
        // reset everything
        
        // Reset the clock
        yao_clock = 1;
        
        // Reset Yao variables
        yao_intra_prev_mean = yao_intra_current_mean = yao_intra_prev_std = yao_intra_current_std
        = yao_content_variation = yao_threshold = yao_tUP = yao_tDown = yao_thresholdFinal = 0;
    }
    
#if GEN_YAO_SCs
    if (isSceneChange) {
        string fileName = "";
        std::ostringstream oss;
        oss << "Gen//Seq-SCs//" << g_input_FileName << "_" << "_yao" << g_skipInterval << ".txt";
        fileName = oss.str();
        Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
        
        yao_pFile = fopen (pYUVFileName, "at");
//        fprintf(yao_pFile, "%d\n",    pcSlice->getPOC());
        fprintf(yao_pFile, "%d,",    pcSlice->getPOC());
        fclose(yao_pFile);
        
    }
#endif
    
    return isSceneChange;
}
#endif

#if IS_SASTRE_SCD
static UInt sastre_clock = 1;
static Bool sastre_update_lock = true;
Bool TEncSlice::isSceneChangeSastre(TComSlice* pcSlice, TComPic* pcPic)
{
    if (pcSlice->getPOC() == 0) {
        return false;
    }
    
    Bool isSceneChange = false;
    
    sastre_alpha = 0.25;
    sastre_S     = 500;
    
    if (sastre_update_lock) {
        // Compute m(k)
        sastre_avg_tillK = sastre_avg_tillK*(1-sastre_alpha) + sastre_intra_count_tillK*sastre_alpha;
    }
    
    // Compute n
    sastre_span = m_pcCfg->getFrameRate()*sastre_S/1000.0;
    
    
    // Fetch the intra modes - MB(k+1)
    UInt mb_kPlus1 = pcSlice->intra_modes;
    
    // Fetch the total number of CUs
    UInt nTotal = pcSlice->slice_total_parts;
    //        UInt nTotal = pcPic-> getNumCUsInFrame();;
    
    // Compute Ta
    // Selected 50% for now
    sastre_Ta   = 0.50*nTotal; // made it nTotal and not Ta
    
    // Compute Tf
    sastre_Tf   = 0.98*nTotal;
    
    // Compute Tlim
    sastre_Tlim = 0.95*nTotal;
    
    // Select the threshold
    Double threshold = (sastre_clock < sastre_span)? sastre_Tf : min(sastre_avg_tillK + sastre_Ta, sastre_Tlim);
    
    // Print sastre variables
    cout << "k: " << sastre_clock << ", n: " << sastre_span << endl;
    cout << "nTotal: " << nTotal << ", current intra: " << mb_kPlus1 << " ratio: " << 1.0*mb_kPlus1/nTotal << endl;
    //
    //        cout << "m(k): " << sastre_avg_tillK << ", Mb(k): " << sastre_intra_count_tillK << endl;
    //        cout << "Tf: " << sastre_Tf << ", Ta: " << sastre_Ta  << ", Tlim: " << sastre_Tlim << endl;
    
    
    cout << "*******Sastreeeee MB(k+1): " << mb_kPlus1 << ", threshold: " << threshold << endl;
    //        cout << "Tf: " << sastre_Tf  << " n: "  << sastre_span  << ", total num of parts: " << nTotal << endl;
    
    // Scene change check
    if(mb_kPlus1 >= threshold)
    {
        isSceneChange = true;
        
        // Reset the clock
        sastre_clock = 0;
        
        // Reset the variables...
        sastre_avg_tillK = 0;
        
    }
    
    // Update the clock
//    sastre_clock = sastre_clock + 1;
    
//    // Update the weighted average! Now, the last encoded frame is (k+1)
//    sastre_intra_count_tillK = mb_kPlus1;
    
#if GEN_SASTRE_SCs
    if (isSceneChange) {
        string fileName = "";
        std::ostringstream oss;
//        oss << "Gen//Seq-SCs//" << g_input_FileName << "_" << "_sastre" << ".txt";
        
        oss << "Gen//Seq-SCs//" << g_input_FileName << "_" << "_sastre" << g_skipInterval << ".txt";
        fileName = oss.str();
        Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
        
        FILE* sastre_pFile = fopen (pYUVFileName, "at");
//        fprintf(sastre_pFile, "%d\n",    pcSlice->getPOC());
        
        fprintf(sastre_pFile, "%d,",    pcSlice->getPOC());
        fclose(sastre_pFile);
        
    }
#endif
    
    return isSceneChange;
}
#endif

/** \param rpcPic   picture class
 */
// Used for bench marking of SCD
Bool TEncSlice::compressSliceBench( TComPic*& rpcPic )
{
    UInt   uiStartCUAddr;
    UInt   uiBoundingCUAddr;
    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
    TEncBinCABAC* pppcRDSbacCoder = NULL;
    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
    
    // initialize cost values
    m_uiPicTotalBits  = 0;
    m_dPicRdCost      = 0;
    m_uiPicDist       = 0;
    
    
    // set entropy coder
    m_pcSbacCoder->init( m_pcBinCABAC );
    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
    m_pcEntropyCoder->resetEntropy      ();
    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
    pppcRDSbacCoder->setBinCountingEnableFlag( false );
    pppcRDSbacCoder->setBinsCoded( 0 );
    
    //------------------------------------------------------------------------------
    //  Weighted Prediction parameters estimation.
    //------------------------------------------------------------------------------
    // calculate AC/DC values for current picture
    if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
    {
        xCalcACDCParamSlice(pcSlice);
    }
    
    Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
    
    if ( bWp_explicit )
    {
        //------------------------------------------------------------------------------
        //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
        //------------------------------------------------------------------------------
        if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
        {
            printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
        }
        
        xEstimateWPParamSlice( pcSlice );
        pcSlice->initWpScaling();
        
        // check WP on/off
        xCheckWPEnable( pcSlice );
    }
    
#if ADAPTIVE_QP_SELECTION
    if( m_pcCfg->getUseAdaptQpSelect() )
    {
        m_pcTrQuant->clearSliceARLCnt();
        if(pcSlice->getSliceType()!=I_SLICE)
        {
            Int qpBase = pcSlice->getSliceQpBase();
            pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
        }
    }
#endif
    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
    delete[] m_pcBufferSbacCoders;
    delete[] m_pcBufferBinCoderCABACs;
    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    }
    
    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
    {
        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
    }
    delete[] m_pcBufferLowLatSbacCoders;
    delete[] m_pcBufferLowLatBinCoderCABACs;
    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    
    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
    UInt      uiTileCol               = 0;
    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
    if( depSliceSegmentsEnabled )
    {
        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
        {
            UInt uiSubStrm=0;
            if( m_pcCfg->getWaveFrontsynchro() )
            {
                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
                if ( pCurrentTile->getTileWidth() < 2)
                {
                    CTXMem[0]->loadContexts(m_pcSbacCoder);
                }
            }
            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
        }
        else
        {
            if(m_pcCfg->getWaveFrontsynchro())
            {
                CTXMem[1]->loadContexts(m_pcSbacCoder);
            }
            CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
    }
    
    // ============SCD Work==================
    Bool isSceneChange = false;
    // Init those for the slice
    pcSlice->intra_modes       = 0;
    pcSlice->slice_total_parts = 0;
    // ============SCD Work==================
    
    
    // for every CU in slice
    UInt uiEncCUOrder;
    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        
        // initialize CU encoder
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        
        
        pcCU->initCU( rpcPic, uiCUAddr );
        

        // inherit from TR if necessary, select substream to use.
        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
        UInt uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
        UInt uiCol     = uiCUAddr % uiWidthInLCUs;
        UInt uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
        
        if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
        {
            // We'll sync if the TR is available.
            TComDataCU *pcCUUp = pcCU->getCUAbove();
            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
            TComDataCU *pcCUTR = NULL;
            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
            {
                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
            }
            if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
                  (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
                  ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
                  )
                )
            {
                // TR not available.
            }
            else
            {
                // TR is available, we use it.
                ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
            }
        }
        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code
        
        // reset the entropy coder
        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
        {
            SliceType sliceType = pcSlice->getSliceType();
            if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
            {
                sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
            }
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
            m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
            m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
        }
        
        // set go-on entropy coder
        m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        
        ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);
        
        Double oldLambda = m_pcRdCost->getLambda();
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int estQP        = pcSlice->getSliceQp();
            Double estLambda = -1.0;
            Double bpp       = -1.0;
            
            if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
            {
                estQP = pcSlice->getSliceQp();
            }
            else
            {
                bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
                if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
                }
                else
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
                    estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
                }
                
                estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );
                
                m_pcRdCost->setLambda(estLambda);
                
#if RDOQ_CHROMA_LAMBDA
                // set lambda for RDOQ
                const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
                const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
                m_pcTrQuant->setLambdas( lambdaArray );
#else
                m_pcTrQuant->setLambda( estLambda );
#endif
            }
            
            m_pcRateCtrl->setRCQP( estQP );
#if ADAPTIVE_QP_SELECTION
            pcCU->getSlice()->setSliceQpBase( estQP );
#endif
        }
        
        
        // run CU encoder
        m_pcCuEncoder->compressCU( pcCU );
        
        // ============SCD Work==================
        // Call the CU encoder to do the recursion CTU by CTU
        m_pcCuEncoder->xExtractCUInfo(pcCU, 0, 0, 0);
        
        pcSlice->intra_modes += pcCU->intra_modes;
        
        // Accumlate the total slice parts
        pcSlice->slice_total_parts += pcCU->cu_total_parts;
        
        
#if IS_YAO_SCD
        isSceneChange = isSceneChangeYao(pcSlice, rpcPic);
        
        // Stop execution if it's a scene change
        if (isSceneChange) {
            break;
        }

#elif IS_SASTRE_SCD
        isSceneChange = isSceneChangeSastre(pcSlice, rpcPic);
        
        if (isSceneChange) {
            break;
        }
#endif
        // ============SCD Work==================

        
        // restore entropy coder to an initial stage
        m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
        m_pcBitCounter = &pcBitCounters[uiSubStrm];
        pppcRDSbacCoder->setBinCountingEnableFlag( true );
        m_pcBitCounter->resetBits();
        pppcRDSbacCoder->setBinsCoded( 0 );
        m_pcCuEncoder->encodeCU( pcCU );
        
        pppcRDSbacCoder->setBinCountingEnableFlag( false );
        if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
        {
            pcSlice->setNextSlice( true );
            break;
        }
        if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSliceSegment( true );
            break;
        }
        
        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
        
        //Store probabilties of second LCU in line into buffer
        if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
        {
            m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
        }
        
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int actualQP        = g_RCInvalidQPValue;
            Double actualLambda = m_pcRdCost->getLambda();
            Int actualBits      = pcCU->getTotalBits();
            Int numberOfEffectivePixels    = 0;
            for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
            {
                if ( pcCU->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pcCU->isSkipped( idx ) ) )
                {
                    numberOfEffectivePixels = numberOfEffectivePixels + 16;
                    break;
                }
            }
            
            if ( numberOfEffectivePixels == 0 )
            {
                actualQP = g_RCInvalidQPValue;
            }
            else
            {
                actualQP = pcCU->getQP( 0 );
            }
            m_pcRdCost->setLambda(oldLambda);
            m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                                     pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
        }
        
        m_uiPicTotalBits += pcCU->getTotalBits();
        m_dPicRdCost     += pcCU->getTotalCost();
        m_uiPicDist      += pcCU->getTotalDistortion();
    }
    if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
    {
        pcSlice->setNextSlice( true );
    }
    if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
    {
        if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSlice( true );
        }
        else
        {
            pcSlice->setNextSliceSegment( true );
        }
    }
    if( depSliceSegmentsEnabled )
    {
        if (m_pcCfg->getWaveFrontsynchro())
        {
            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
        }
        CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
    }
    xRestoreWPparam( pcSlice );
    
    // ============SCD Work==================
    // Reset the intra modes of the Slice Encoder
    this->intra_modes = 0;
    
#if IS_YAO_SCD
    
    // Print out the Yao_variables
    cout << "Yao variables--> " << yao_clock << endl;
//            cout << "Intra prev mean: " << yao_intra_prev_mean << endl;
//            cout << "Intra curr mean: " << yao_intra_current_mean << endl;
//            cout << "Intra prev std: " << yao_intra_prev_std << endl;
//            cout << "Intra curr std: " << yao_intra_current_std << endl;
//            cout << "Intra yao threshold: " << yao_threshold << endl;
    cout << "Intra yao threshold FINAL: " << yao_thresholdFinal << ", content variation: " << yao_content_variation << endl;
    
//    cout << "Intra yao Controllers Intra, count FINAL: " << pcSlice->intra_modes << ", count: " << pcSlice->slice_total_parts << endl;

    // Update the clock
    yao_clock = yao_clock + 1;
    
    // Free the update lock for the previous value
    yao_update_lock = true;
    
#elif IS_SASTRE_SCD
    
    //Update the clock
    sastre_clock = sastre_clock + 1;
    
//    cout << "k: " << sastre_clock << ", n: " << sastre_span << endl;
//    cout << "nTotal: " << pcSlice->slice_total_parts << ", current intra: " << pcSlice->intra_modes << " ratio: " << 1.0*pcSlice->intra_modes/pcSlice->slice_total_parts << endl;
//    //
//    //        cout << "m(k): " << sastre_avg_tillK << ", Mb(k): " << sastre_intra_count_tillK << endl;
//    //        cout << "Tf: " << sastre_Tf << ", Ta: " << sastre_Ta  << ", Tlim: " << sastre_Tlim << endl;
//    
//    
//    cout << "*******Sastreeeee MB(k+1): " << pcSlice->intr << ", threshold: " << threshold << endl;
    //        cout << "Tf: " << sastre_Tf  << " n: "  << sastre_span  << ", total num of parts: " << nTotal << endl;

    
    
#endif
    
    
    return isSceneChange;
    // ============SCD Work==================
}


/** \param rpcPic   picture class
 */
Void TEncSlice::compressSlice( TComPic*& rpcPic )
{
    UInt   uiStartCUAddr;
    UInt   uiBoundingCUAddr;
    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
    TEncBinCABAC* pppcRDSbacCoder = NULL;
    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
    
    // initialize cost values
    m_uiPicTotalBits  = 0;
    m_dPicRdCost      = 0;
    m_uiPicDist       = 0;
    
    
// Hossam: init the bits for each partition of reference
    m_uiPicTotalBitsRef1 = m_uiPicTotalBitsRef2 = 0;
//  Hossam: Push a new item in the reference bits
    reference1_bits_perFrame.push_back(0); reference2_bits_perFrame.push_back(0);

    
    // set entropy coder
    m_pcSbacCoder->init( m_pcBinCABAC );
    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
    m_pcEntropyCoder->resetEntropy      ();
    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
    pppcRDSbacCoder->setBinCountingEnableFlag( false );
    pppcRDSbacCoder->setBinsCoded( 0 );
    
    //------------------------------------------------------------------------------
    //  Weighted Prediction parameters estimation.
    //------------------------------------------------------------------------------
    // calculate AC/DC values for current picture
    if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
    {
        xCalcACDCParamSlice(pcSlice);
    }
    
    Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
    
    if ( bWp_explicit )
    {
        //------------------------------------------------------------------------------
        //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
        //------------------------------------------------------------------------------
        if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
        {
            printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
        }
        
        xEstimateWPParamSlice( pcSlice );
        pcSlice->initWpScaling();
        
        // check WP on/off
        xCheckWPEnable( pcSlice );
    }
    
#if ADAPTIVE_QP_SELECTION
    if( m_pcCfg->getUseAdaptQpSelect() )
    {
        m_pcTrQuant->clearSliceARLCnt();
        if(pcSlice->getSliceType()!=I_SLICE)
        {
            Int qpBase = pcSlice->getSliceQpBase();
            pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
        }
    }
#endif
    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
    delete[] m_pcBufferSbacCoders;
    delete[] m_pcBufferBinCoderCABACs;
    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    }
    
    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
    {
        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
    }
    delete[] m_pcBufferLowLatSbacCoders;
    delete[] m_pcBufferLowLatBinCoderCABACs;
    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
    for (Int ui = 0; ui < uiTilesAcross; ui++)
    {
        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
    }
    for (UInt ui = 0; ui < uiTilesAcross; ui++)
        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
    
    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
    UInt      uiTileCol               = 0;
    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
    if( depSliceSegmentsEnabled )
    {
        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
        {
            UInt uiSubStrm=0;
            if( m_pcCfg->getWaveFrontsynchro() )
            {
                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
                if ( pCurrentTile->getTileWidth() < 2)
                {
                    CTXMem[0]->loadContexts(m_pcSbacCoder);
                }
            }
            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
        }
        else
        {
            if(m_pcCfg->getWaveFrontsynchro())
            {
                CTXMem[1]->loadContexts(m_pcSbacCoder);
            }
            CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
    }
    // for every CU in slice
    UInt uiEncCUOrder;
    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        
        
        
        //        cout << "Hello for every CU" << endl;
        /// XXX problem is here: the things are null
        // initialize CU encoder
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        
//        cout << "TEncSlice: CompressSlice: pcCu: " <<  pcCU->getCUMvField(RefPicList(0))->getRefIdx(0) << endl;

//        /-----Debug-----// failed the test
//        cout <<    pcCU->getAddr() << ", " << pcCU->getZorderIdxInCU() <<  ", [Before Init Reference] Result Before Termination: " << endl;
//        pcCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//        
//        cout << ", [Before Init Reference] Final SC Result Before Termination: " << endl;
//        pcCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//        /-----Debug-----
        
        
//                cout << "My problem is in the init of the pcCU BEFORE " << endl;
        pcCU->initCU( rpcPic, uiCUAddr );
        
        
        //        /-----Debug-----// failed the test
//                cout <<    pcCU->getAddr() << ", " << pcCU->getZorderIdxInCU() <<  ", [After Init Reference] Result Before Termination: " << endl;
//                pcCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//        
//                cout << ", [After Init Reference] Final SC Result Before Termination: " << endl;
//                pcCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
        //        /-----Debug-----
        
//        cout << "TEncSlice: CompressSlice: pcCu: " <<  pcCU->getCUMvField(RefPicList(0))->getRefIdx(0) << endl;

        //        cout << "My problem is in the init of the pcCU " << endl;
        
        // inherit from TR if necessary, select substream to use.
        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
        UInt uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
        UInt uiCol     = uiCUAddr % uiWidthInLCUs;
        UInt uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
        
        if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
        {
            // We'll sync if the TR is available.
            TComDataCU *pcCUUp = pcCU->getCUAbove();
            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
            TComDataCU *pcCUTR = NULL;
            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
            {
                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
            }
            if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
                  (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
                  ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
                  )
                )
            {
                // TR not available.
            }
            else
            {
                // TR is available, we use it.
                ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
            }
        }
        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code
        
        // reset the entropy coder
        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
        {
            SliceType sliceType = pcSlice->getSliceType();
            if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
            {
                sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
            }
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
            m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
            m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
        }
        
        // set go-on entropy coder
        m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        
        ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);
        
        Double oldLambda = m_pcRdCost->getLambda();
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int estQP        = pcSlice->getSliceQp();
            Double estLambda = -1.0;
            Double bpp       = -1.0;
            
            if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
            {
                estQP = pcSlice->getSliceQp();
            }
            else
            {
                bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
                if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
                }
                else
                {
                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
                    estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
                }
                
                estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );
                
                m_pcRdCost->setLambda(estLambda);
                
#if RDOQ_CHROMA_LAMBDA
                // set lambda for RDOQ
                const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
                const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
                m_pcTrQuant->setLambdas( lambdaArray );
#else
                m_pcTrQuant->setLambda( estLambda );
#endif
            }
            
            m_pcRateCtrl->setRCQP( estQP );
#if ADAPTIVE_QP_SELECTION
            pcCU->getSlice()->setSliceQpBase( estQP );
#endif
        }
        
        // Motion Estimation
        //      cout  << pcCU -> getSlice() -> getPOC() << ") Ruba: From the heart of Motion Estimation: " << pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0) << endl;
        
        //        cout << "Run CU encoder " << endl;
        // run CU encoder
        m_pcCuEncoder->compressCU( pcCU );
        
   
//        if(pcCU->getPic()->getPOC() == 1 && uiEncCUOrder > 1) { cout << "EXIT BEFORE ENCODE CU : " << pcCU->m_uiTotalBitsRefI[0] << ", tot: " << pcCU->getTotalBits() << endl; exit(0); }

        
        // restore entropy coder to an initial stage
        m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
        m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
        m_pcBitCounter = &pcBitCounters[uiSubStrm];
        pppcRDSbacCoder->setBinCountingEnableFlag( true );
        m_pcBitCounter->resetBits();
        pppcRDSbacCoder->setBinsCoded( 0 );
        m_pcCuEncoder->encodeCU( pcCU );
        
        pppcRDSbacCoder->setBinCountingEnableFlag( false );
        if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
        {
            pcSlice->setNextSlice( true );
            break;
        }
        if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSliceSegment( true );
            break;
        }
        
        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
        
        //Store probabilties of second LCU in line into buffer
        if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
        {
            m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
        }
        
        if ( m_pcCfg->getUseRateCtrl() )
        {
            Int actualQP        = g_RCInvalidQPValue;
            Double actualLambda = m_pcRdCost->getLambda();
            Int actualBits      = pcCU->getTotalBits();
            Int numberOfEffectivePixels    = 0;
            for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
            {
                if ( pcCU->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pcCU->isSkipped( idx ) ) )
                {
                    numberOfEffectivePixels = numberOfEffectivePixels + 16;
                    break;
                }
            }
            
            if ( numberOfEffectivePixels == 0 )
            {
                actualQP = g_RCInvalidQPValue;
            }
            else
            {
                actualQP = pcCU->getQP( 0 );
            }
            m_pcRdCost->setLambda(oldLambda);
            m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                                     pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
        }
        
        m_uiPicTotalBits += pcCU->getTotalBits();
        m_dPicRdCost     += pcCU->getTotalCost();
        m_uiPicDist      += pcCU->getTotalDistortion();
        
        
        // Hossam: accumulate bits for ref1 and ref2
        m_uiPicTotalBitsRef1 += pcCU->m_uiTotalBitsRefI[0];
        m_uiPicTotalBitsRef2 += pcCU->m_uiTotalBitsRefI[1];
        
    //    cout << "\nAccumlating bits for the slice: " << m_uiPicTotalBits << endl;
    }
    
    
    // Hossam: update your bits stats
    updateRef1Ref2Bits();
    
    Double check_per = 100.0*m_uiPicTotalBitsRef1/m_uiPicTotalBits;
    cout << "\nAccumlating bits for the slice: " << m_uiPicTotalBits << ", " << check_per << ", greater than hashtaka: " << (check_per > 100) << "\n------------\n" << endl;
    
    if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
    {
        pcSlice->setNextSlice( true );
    }
    if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
    {
        if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
        {
            pcSlice->setNextSlice( true );
        }
        else
        {
            pcSlice->setNextSliceSegment( true );
        }
    }
    if( depSliceSegmentsEnabled )
    {
        if (m_pcCfg->getWaveFrontsynchro())
        {
            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
        }
        CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
    }
    xRestoreWPparam( pcSlice );
}

/**
 \param  rpcPic        picture class
 \retval rpcBitstream  bitstream class
 */
Void TEncSlice::encodeSlice   ( TComPic*& rpcPic, TComOutputBitstream* pcSubstreams )
{
    UInt       uiCUAddr;
    UInt       uiStartCUAddr;
    UInt       uiBoundingCUAddr;
    TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
    
    
    m_uiPicTotalBits_hevc = 0;
    
    
//     Hossam: init the bits for each partition of reference
//    m_uiPicTotalBitsRef1 = m_uiPicTotalBitsRef2 = 0;
    
//     Hossam: Push a new item in the reference bits
//    reference1_bits_perFrame.push_back(0); reference2_bits_perFrame.push_back(0);
    
    uiStartCUAddr=pcSlice->getSliceSegmentCurStartCUAddr();
    uiBoundingCUAddr=pcSlice->getSliceSegmentCurEndCUAddr();
    // choose entropy coder
    {
        m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
        m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
    }
    
    m_pcCuEncoder->setBitCounter( NULL );
    m_pcBitCounter = NULL;
    // Appropriate substream bitstream is switched later.
    // for every CU
#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceEnable;
#endif
    DTRACE_CABAC_VL( g_nSymbolCounter++ );
    DTRACE_CABAC_T( "\tPOC: " );
    DTRACE_CABAC_V( rpcPic->getPOC() );
    DTRACE_CABAC_T( "\n" );
#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceDisable;
#endif
    
    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
    TEncSbac* pcSbacCoders = pcEncTop->getSbacCoders(); //coder for each substream
    const Int iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
    UInt uiBitsOriginallyInSubstreams = 0;
    {
        UInt uiTilesAcross = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
        for (UInt ui = 0; ui < uiTilesAcross; ui++)
        {
            m_pcBufferSbacCoders[ui].load(m_pcSbacCoder); //init. state
        }
        
        for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
        {
            uiBitsOriginallyInSubstreams += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
        }
        
        for (UInt ui = 0; ui < uiTilesAcross; ui++)
        {
            m_pcBufferLowLatSbacCoders[ui].load(m_pcSbacCoder);  //init. state
        }
    }
    
    UInt uiWidthInLCUs  = rpcPic->getPicSym()->getFrameWidthInCU();
    UInt uiCol=0, uiSubStrm=0;
    UInt uiTileCol      = 0;
    UInt uiTileStartLCU = 0;
    UInt uiTileLCUX     = 0;
    Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
    uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());  /* for tiles, uiStartCUAddr is NOT the real raster scan address, it is actually
                                                                                               an encoding order index, so we need to convert the index (uiStartCUAddr)
                                                                                               into the real raster scan address (uiCUAddr) via the CUOrderMap */
    UInt currentTileIdx=rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
    TComTile *pCurrentTile=rpcPic->getPicSym()->getTComTile(currentTileIdx);
    uiTileStartLCU = pCurrentTile->getFirstCUAddr();
    if( depSliceSegmentsEnabled )
    {
        if( pcSlice->isNextSlice()|| uiCUAddr == uiTileStartLCU)
        {
            if(m_pcCfg->getWaveFrontsynchro())
            {
                CTXMem[1]->loadContexts(m_pcSbacCoder);
            }
            CTXMem[0]->loadContexts(m_pcSbacCoder);
        }
        else
        {
            if( m_pcCfg->getWaveFrontsynchro() )
            {
                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
                uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
                if ( pCurrentTile->getTileWidth() < 2)
                {
                    CTXMem[0]->loadContexts(m_pcSbacCoder);
                }
            }
            pcSbacCoders[uiSubStrm].loadContexts( CTXMem[0] );
        }
    }
    
//    if(rpcPic->getPOC() == 1)
//    {
//     cout << "Enc) pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//    }
    
    // bits for SAO for slice
    Int bitsAccumlatorSAO = 0;
    
    UInt uiEncCUOrder;
    for( uiEncCUOrder = uiStartCUAddr /rpcPic->getNumPartInCU();
        uiEncCUOrder < (uiBoundingCUAddr+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU();
        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
    {
        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
        uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
        uiCol     = uiCUAddr % uiWidthInLCUs;
        uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
        
        
//        if(rpcPic->getPOC() == 1) {
//           cout <<  uiEncCUOrder << ") " << uiTileCol << ", " << uiTileStartLCU << ", " << uiTileLCUX << ", \t"
//         << uiCol << ", " << uiSubStrm << endl;
//        }
        
        m_pcEntropyCoder->setBitstream( &pcSubstreams[uiSubStrm] );
        
        
//        if(rpcPic->getPOC() == 1)
//       {
//            cout << uiEncCUOrder << "-Enc) before BEFORE encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//        }
        
        // Synchronize cabac probabilities with upper-right LCU if it's available and we're at the start of a line.
        if (((iNumSubstreams > 1) || depSliceSegmentsEnabled) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
        {
            // We'll sync if the TR is available.
            TComDataCU *pcCUUp = rpcPic->getCU( uiCUAddr )->getCUAbove();
            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
            TComDataCU *pcCUTR = NULL;
            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
            {
                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
            }
            if ( (true/*bEnforceSliceRestriction*/ &&
                  ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
                   (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
                   ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
                   ))
                )
            {
                // TR not available.
            }
            else
            {
                // TR is available, we use it.
                pcSbacCoders[uiSubStrm].loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
            }
        }
        
//        if(rpcPic->getPOC() == 1)
//       {
//            cout << uiEncCUOrder << "-Enc) before BEFORE00 encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//        }
        
        m_pcSbacCoder->load(&pcSbacCoders[uiSubStrm]);  //this load is used to simplify the code (avoid to change all the call to m_pcSbacCoder)
        
        
//        if(rpcPic->getPOC() == 1)
//       {
//            cout << uiEncCUOrder << "-Enc) before BEFORE11001 encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//        }
        
        // reset the entropy coder
        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
        {
            
//            if(rpcPic->getPOC() == 1)
//            {
//                cout << uiEncCUOrder << "-Enc) before BEFORE0011 encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//            }
            
            // We're crossing into another tile, tiles are independent.
            // When tiles are independent, we have "substreams per tile".  Each substream has already been terminated, and we no longer
            // have to perform it here.
            if (iNumSubstreams <= 1)
            {
                SliceType sliceType  = pcSlice->getSliceType();
                if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
                {
                    sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
                }
                m_pcEntropyCoder->updateContextTables( sliceType, pcSlice->getSliceQp() );
                
                // Byte-alignment in slice_data() when new tile
                pcSubstreams[uiSubStrm].writeByteAlignment();
            }
            
            {
                UInt numStartCodeEmulations = pcSubstreams[uiSubStrm].countStartCodeEmulations();
                UInt uiAccumulatedSubstreamLength = 0;
                for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
                {
                    uiAccumulatedSubstreamLength += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
                }
                // add bits coded in previous dependent slices + bits coded so far
                // add number of emulation prevention byte count in the tile
                pcSlice->addTileLocation( ((pcSlice->getTileOffstForMultES() + uiAccumulatedSubstreamLength - uiBitsOriginallyInSubstreams) >> 3) + numStartCodeEmulations );
            }
        }
        
//        if(rpcPic->getPOC() == 1)
//        {
//            cout << uiEncCUOrder << "-Enc) before BEFORE1111 encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//        }
        
        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
        
        if ( pcSlice->getSPS()->getUseSAO() )
        {
//            if(rpcPic->getPOC() == 1)
//            {
//                cout << uiEncCUOrder << "-Enc) before BEFORE1111 222 encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//            }
            
            Bool bIsSAOSliceEnabled = false;
            Bool sliceEnabled[MAX_NUM_COMPONENT];
            for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
            {
                ComponentID compId=ComponentID(comp);
                sliceEnabled[compId] = pcSlice->getSaoEnabledFlag(toChannelType(compId)) && (comp < rpcPic->getNumberValidComponents());
                
//                if(rpcPic->getPOC() == 1)
//                {
//                   cout << uiEncCUOrder << "-Enc) before BEFORE BANF getSaoEnabledFlag " << pcSlice->getSaoEnabledFlag(toChannelType(compId)) << (comp < rpcPic->getNumberValidComponents()) << endl;
//                }
                
                
                if (sliceEnabled[compId]) bIsSAOSliceEnabled=true;
            }
            
//            if(rpcPic->getPOC() == 1)
//            {
//                cout << uiEncCUOrder << "-Enc) before BEFORE1111 555 encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//            }
            
            if (bIsSAOSliceEnabled)
            {
                SAOBlkParam& saoblkParam = (rpcPic->getPicSym()->getSAOBlkParam())[uiCUAddr];
                
                Bool leftMergeAvail = false;
                Bool aboveMergeAvail= false;
                //merge left condition
                Int rx = (uiCUAddr % uiWidthInLCUs);
                if(rx > 0)
                {
                    leftMergeAvail = rpcPic->getSAOMergeAvailability(uiCUAddr, uiCUAddr-1);
                }
                
                //merge up condition
                Int ry = (uiCUAddr / uiWidthInLCUs);
                if(ry > 0)
                {
                    aboveMergeAvail = rpcPic->getSAOMergeAvailability(uiCUAddr, uiCUAddr-uiWidthInLCUs);
                }
                
                
                UInt bits_prev_state = m_pcEntropyCoder -> getNumberOfWrittenBits();
                m_pcEntropyCoder->encodeSAOBlkParam(saoblkParam, sliceEnabled, leftMergeAvail, aboveMergeAvail);
                bitsAccumlatorSAO = m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state;
                // cout << "Slice bits SAO bitsAccumlatorSAO: " << bitsAccumlatorSAO << endl;
                
            } // end if (bIsSAOSliceEnabled)
            

        }
        
        
//        if(rpcPic->getPOC() == 1)
//        {
//            cout << uiEncCUOrder << "-Enc) before encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//        }
#if ENC_DEC_TRACE
        g_bJustDoIt = g_bEncDecTraceEnable;
#endif
        if ( (m_pcCfg->getSliceMode()!=0 || m_pcCfg->getSliceSegmentMode()!=0) &&
            uiCUAddr == rpcPic->getPicSym()->getCUOrderMap((uiBoundingCUAddr+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU()-1) )
        {
            m_pcCuEncoder->encodeCU( pcCU );
        }
        else
        {
            m_pcCuEncoder->encodeCU( pcCU );
        }
        
        // Hossam: accumulate bits
//        m_uiPicTotalBitsRef1 += pcCU->m_uiTotalBitsRefI[0];
 //       m_uiPicTotalBitsRef2 += pcCU->m_uiTotalBitsRefI[1];
        
        // m_uiPicTotalBits_hevc += m_pcCuEncoder->bitsAccumulator + bitsAccumlatorSAO;
        // m_uiPicTotalBits_hevc += m_pcCuEncoder->bitsAccumulator;
        m_uiPicTotalBits_hevc += pcCU->getTotalBits();

        
#if ENC_DEC_TRACE
        g_bJustDoIt = g_bEncDecTraceDisable;
#endif
        pcSbacCoders[uiSubStrm].load(m_pcSbacCoder);   //load back status of the entropy coder after encoding the LCU into relevant bitstream entropy coder
        
        //Store probabilties of second LCU in line into buffer
        if ( (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && (uiCol == uiTileLCUX+1) && m_pcCfg->getWaveFrontsynchro())
        {
            m_pcBufferSbacCoders[uiTileCol].loadContexts( &pcSbacCoders[uiSubStrm] );
        }
        
//        if(rpcPic->getPOC() == 1)
//        {
//            cout << uiEncCUOrder << "-Enc) After encodeCU pcSubstreamsOut[0].getNumberOfWrittenBits(): " << pcSubstreams[0].getNumberOfWrittenBits() << endl;
//        }
    } // end for loop on CUs
    
    // Hossam: update your bits stats
//    reference1_bits_perFrame.back() = m_uiPicTotalBitsRef1; reference2_bits_perFrame.back() = m_uiPicTotalBitsRef2;
    
    if( depSliceSegmentsEnabled )
    {
        if (m_pcCfg->getWaveFrontsynchro())
        {
            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
        }
        CTXMem[0]->loadContexts( m_pcSbacCoder );//ctx end of dep.slice
    }
#if ADAPTIVE_QP_SELECTION
    if( m_pcCfg->getUseAdaptQpSelect() )
    {
        m_pcTrQuant->storeSliceQpNext(pcSlice);
    }
#endif
    if (pcSlice->getPPS()->getCabacInitPresentFlag())
    {
        if (pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
        {
            pcSlice->getPPS()->setEncCABACTableIdx( pcSlice->getSliceType() );
        }
        else
        {
            m_pcEntropyCoder->determineCabacInitIdx();
        }
    }
    
    // cout << "Total bits in a slice is " << m_uiPicTotalBits_hevc << endl;
    
} // end encodeSlice

Void TEncSlice::calculateBoundingCUAddrForSlice(UInt &uiStartCUAddrSlice, UInt &uiBoundingCUAddrSlice, Bool &bReachedTileBoundary, TComPic*& rpcPic, Bool bEncodeSlice, Int sliceMode, Int sliceArgument, UInt uiSliceCurEndCUAddr)
{
    TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
    UInt uiNumberOfCUsInFrame = rpcPic->getNumCUsInFrame();
    const UInt scaleCUAddr = rpcPic->getNumPartInCU(); // due to fine granularity slices all addresses are scaled.
    uiBoundingCUAddrSlice=0;
    bReachedTileBoundary=false;
    
    switch (sliceMode)
    {
        case FIXED_NUMBER_OF_LCU:
        {
            UInt uiCUAddrIncrement    = sliceArgument;
            uiBoundingCUAddrSlice     = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*scaleCUAddr) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*scaleCUAddr;
        }
            break;
        case FIXED_NUMBER_OF_BYTES:
            if (bEncodeSlice)
                uiBoundingCUAddrSlice     = uiSliceCurEndCUAddr;
            else
                uiBoundingCUAddrSlice     = uiNumberOfCUsInFrame*scaleCUAddr;
            break;
        case FIXED_NUMBER_OF_TILES:
        {
            UInt tileIdx              = rpcPic->getPicSym()->getTileIdxMap( rpcPic->getPicSym()->getCUOrderMap(uiStartCUAddrSlice/scaleCUAddr) );
            UInt uiCUAddrIncrement    = 0;
            UInt tileTotalCount       = (rpcPic->getPicSym()->getNumColumnsMinus1()+1) * (rpcPic->getPicSym()->getNumRowsMinus1()+1);
            
            for(UInt tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)
            {
                if((tileIdx + tileIdxIncrement) < tileTotalCount)
                {
                    UInt tileWidthInLcu   = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidth();
                    UInt tileHeightInLcu  = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeight();
                    uiCUAddrIncrement    += (tileWidthInLcu * tileHeightInLcu * scaleCUAddr);
                }
            }
            
            uiBoundingCUAddrSlice     = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*scaleCUAddr) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*scaleCUAddr;
        }
            break;
        default:
            uiBoundingCUAddrSlice       = uiNumberOfCUsInFrame*scaleCUAddr;
            break;
    }
    
    // Adjust for tiles and wavefronts.
    if ((sliceMode == FIXED_NUMBER_OF_LCU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
        (m_pcCfg->getNumRowsMinus1() > 0 || m_pcCfg->getNumColumnsMinus1() > 0))
    {
        const UInt lcuEncAddrStart = (uiStartCUAddrSlice+scaleCUAddr-1)/scaleCUAddr;
        const UInt lcuAddr = rpcPic->getPicSym()->getCUOrderMap(lcuEncAddrStart);
        const UInt startTileIdx = rpcPic->getPicSym()->getTileIdxMap(lcuAddr);
        const Bool bWavefrontsEnabled = m_pcCfg->getWaveFrontsynchro();
        
        TComTile *pStartingTile = rpcPic->getPicSym()->getTComTile(startTileIdx);
        const UInt uiTileStartLCUEncAddr      = rpcPic->getPicSym()->getInverseCUOrderMap(pStartingTile->getFirstCUAddr());
        const UInt uiTileStartWidth           = pStartingTile->getTileWidth();
        const UInt uiTileStartHeight          = pStartingTile->getTileHeight();
        const UInt uiTileLastLCUEncAddr_excl  = uiTileStartLCUEncAddr + uiTileStartWidth*uiTileStartHeight;
        const UInt tileBoundingCUAddrSlice    = uiTileLastLCUEncAddr_excl * scaleCUAddr;
        
        const UInt lcuColumnOfStartingTile=((lcuEncAddrStart-uiTileStartLCUEncAddr)%uiTileStartWidth);
        if (bWavefrontsEnabled && lcuColumnOfStartingTile!=0)
        {
            // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
            const UInt numberOfLCUsToEndOfRow=uiTileStartWidth-lcuColumnOfStartingTile;
            const UInt wavefrontTileBoundingCUAddrSlice = (lcuEncAddrStart+numberOfLCUsToEndOfRow)*scaleCUAddr;
            if (wavefrontTileBoundingCUAddrSlice < uiBoundingCUAddrSlice)
            {
                uiBoundingCUAddrSlice = wavefrontTileBoundingCUAddrSlice;
            }
        }
        
        if (tileBoundingCUAddrSlice < uiBoundingCUAddrSlice)
        {
            uiBoundingCUAddrSlice = tileBoundingCUAddrSlice;
            bReachedTileBoundary = true;
        }
    }
    else if (pcSlice->getPPS()->getNumSubstreams() > 1 && (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*scaleCUAddr) != 0))
    {
        // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
        uiBoundingCUAddrSlice = min(uiBoundingCUAddrSlice, uiStartCUAddrSlice - (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*scaleCUAddr)) + (rpcPic->getFrameWidthInCU()*scaleCUAddr));
    }
    
    //calculate real slice start address (fine granularity slices)
    {
        UInt uiInternalAddress = rpcPic->getPicSym()->getPicSCUAddr(uiStartCUAddrSlice) % scaleCUAddr;
        UInt uiExternalAddress = rpcPic->getPicSym()->getPicSCUAddr(uiStartCUAddrSlice) / scaleCUAddr;
        UInt uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
        UInt uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
        UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
        UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
        while((uiPosX>=uiWidth||uiPosY>=uiHeight)&&!(uiPosX>=uiWidth&&uiPosY>=uiHeight))
        {
            uiInternalAddress++;
            if(uiInternalAddress>=scaleCUAddr)
            {
                uiInternalAddress=0;
                uiExternalAddress = rpcPic->getPicSym()->getCUOrderMap(rpcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
            }
            uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
            uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
        }
        UInt uiRealStartAddress = rpcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*scaleCUAddr+uiInternalAddress);
        
        uiStartCUAddrSlice=uiRealStartAddress;
    }
}

/** Determines the starting and bounding LCU address of current slice / dependent slice
 * \param bEncodeSlice Identifies if the calling function is compressSlice() [false] or encodeSlice() [true]
 * \returns Updates uiStartCUAddr, uiBoundingCUAddr with appropriate LCU address
 */
Void TEncSlice::xDetermineStartAndBoundingCUAddr  ( UInt& startCUAddr, UInt& boundingCUAddr, TComPic*& rpcPic, Bool bEncodeSlice )
{
    TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
    
    // Non-dependent slice
    UInt uiStartCUAddrSlice   = pcSlice->getSliceCurStartCUAddr();
    Bool bTileBoundarySlice   = false;
    UInt uiBoundingCUAddrSlice;
    calculateBoundingCUAddrForSlice(uiStartCUAddrSlice, uiBoundingCUAddrSlice, bTileBoundarySlice, rpcPic, bEncodeSlice, m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument(), pcSlice->getSliceCurEndCUAddr());
    pcSlice->setSliceCurEndCUAddr( uiBoundingCUAddrSlice );
    pcSlice->setSliceCurStartCUAddr(uiStartCUAddrSlice);
    
    // Dependent slice
    UInt startCUAddrSliceSegment    = pcSlice->getSliceSegmentCurStartCUAddr();
    Bool bTileBoundarySliceSegment  = false;
    UInt boundingCUAddrSliceSegment;
    calculateBoundingCUAddrForSlice(startCUAddrSliceSegment, boundingCUAddrSliceSegment, bTileBoundarySliceSegment, rpcPic, bEncodeSlice, m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument(), pcSlice->getSliceSegmentCurEndCUAddr());
    pcSlice->setSliceSegmentCurEndCUAddr( boundingCUAddrSliceSegment );
    pcSlice->setSliceSegmentCurStartCUAddr(startCUAddrSliceSegment);
    
    // Make a joint decision based on reconstruction and dependent slice bounds
    startCUAddr    = max(uiStartCUAddrSlice   , startCUAddrSliceSegment   );
    boundingCUAddr = min(uiBoundingCUAddrSlice, boundingCUAddrSliceSegment);
    
    
    if (!bEncodeSlice)
    {
        // For fixed number of LCU within an entropy and reconstruction slice we already know whether we will encounter end of entropy and/or reconstruction slice
        // first. Set the flags accordingly.
        if ( (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
            || (m_pcCfg->getSliceMode()==0 && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
            || (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU && m_pcCfg->getSliceSegmentMode()==0)
            || (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
            || (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceSegmentMode()==0)
            || (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceMode()==0)
            || bTileBoundarySlice || bTileBoundarySliceSegment )
        {
            if (uiBoundingCUAddrSlice < boundingCUAddrSliceSegment)
            {
                pcSlice->setNextSlice       ( true );
                pcSlice->setNextSliceSegment( false );
            }
            else if (uiBoundingCUAddrSlice > boundingCUAddrSliceSegment)
            {
                pcSlice->setNextSlice       ( false );
                pcSlice->setNextSliceSegment( true );
            }
            else
            {
                pcSlice->setNextSlice       ( true );
                pcSlice->setNextSliceSegment( true );
            }
        }
        else
        {
            pcSlice->setNextSlice       ( false );
            pcSlice->setNextSliceSegment( false );
        }
    }
}

Double TEncSlice::xGetQPValueAccordingToLambda ( Double lambda )
{
    return 4.2005*log(lambda) + 13.7122;
}

//! \}

///// PASTTTTT/// PASTTTTT/// PASTTTTT/// PASTTTTT/// PASTTTTT/// PASTTTTT/// PASTTTTT/// PASTTTTT
///* The copyright in this software is being made available under the BSD
// * License, included below. This software may be subject to other third party
// * and contributor rights, including patent rights, and no such rights are
// * granted under this license.
// *
// * Copyright (c) 2010-2014, ITU/ISO/IEC
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// *  * Redistributions of source code must retain the above copyright notice,
// *    this list of conditions and the following disclaimer.
// *  * Redistributions in binary form must reproduce the above copyright notice,
// *    this list of conditions and the following disclaimer in the documentation
// *    and/or other materials provided with the distribution.
// *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
// *    be used to endorse or promote products derived from this software without
// *    specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// * THE POSSIBILITY OF SUCH DAMAGE.
// */
//
///** \file     TEncSlice.cpp
// \brief    slice encoder class
// */
//
//#include "TEncTop.h"
//#include "TEncSlice.h"
//#include <math.h>
//
////! \ingroup TLibEncoder
////! \{
//
//// ====================================================================================================================
//// Constructor / destructor / create / destroy
//// ====================================================================================================================
//
//TEncSlice::TEncSlice()
//{
//    m_apcPicYuvPred = NULL;
//    m_apcPicYuvResi = NULL;
//
//    m_pdRdPicLambda = NULL;
//    m_pdRdPicQp     = NULL;
//    m_piRdPicQp     = NULL;
//    m_pcBufferSbacCoders    = NULL;
//    m_pcBufferBinCoderCABACs  = NULL;
//    m_pcBufferLowLatSbacCoders    = NULL;
//    m_pcBufferLowLatBinCoderCABACs  = NULL;
//}
//
//TEncSlice::~TEncSlice()
//{
//    for (std::vector<TEncSbac*>::iterator i = CTXMem.begin(); i != CTXMem.end(); i++)
//    {
//        delete (*i);
//    }
//}
//
//Void TEncSlice::initCtxMem(  UInt i )
//{
//    for (std::vector<TEncSbac*>::iterator j = CTXMem.begin(); j != CTXMem.end(); j++)
//    {
//        delete (*j);
//    }
//    CTXMem.clear();
//    CTXMem.resize(i);
//}
//
//Void TEncSlice::create( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )
//{
//    // create prediction picture
//    if ( m_apcPicYuvPred == NULL )
//    {
//        m_apcPicYuvPred  = new TComPicYuv;
//        m_apcPicYuvPred->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
//    }
//
//    // create residual picture
//    if( m_apcPicYuvResi == NULL )
//    {
//        m_apcPicYuvResi  = new TComPicYuv;
//        m_apcPicYuvResi->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth );
//    }
//}
//
//Void TEncSlice::destroy()
//{
//    // destroy prediction picture
//    if ( m_apcPicYuvPred )
//    {
//        m_apcPicYuvPred->destroy();
//        delete m_apcPicYuvPred;
//        m_apcPicYuvPred  = NULL;
//    }
//
//    // destroy residual picture
//    if ( m_apcPicYuvResi )
//    {
//        m_apcPicYuvResi->destroy();
//        delete m_apcPicYuvResi;
//        m_apcPicYuvResi  = NULL;
//    }
//
//    // free lambda and QP arrays
//    if ( m_pdRdPicLambda ) { xFree( m_pdRdPicLambda ); m_pdRdPicLambda = NULL; }
//    if ( m_pdRdPicQp     ) { xFree( m_pdRdPicQp     ); m_pdRdPicQp     = NULL; }
//    if ( m_piRdPicQp     ) { xFree( m_piRdPicQp     ); m_piRdPicQp     = NULL; }
//
//    if ( m_pcBufferSbacCoders )
//    {
//        delete[] m_pcBufferSbacCoders;
//    }
//    if ( m_pcBufferBinCoderCABACs )
//    {
//        delete[] m_pcBufferBinCoderCABACs;
//    }
//    if ( m_pcBufferLowLatSbacCoders )
//        delete[] m_pcBufferLowLatSbacCoders;
//    if ( m_pcBufferLowLatBinCoderCABACs )
//        delete[] m_pcBufferLowLatBinCoderCABACs;
//}
//
//Void TEncSlice::init( TEncTop* pcEncTop )
//{
//    m_pcCfg             = pcEncTop;
//    m_pcListPic         = pcEncTop->getListPic();
//
//    m_pcGOPEncoder      = pcEncTop->getGOPEncoder();
//    m_pcCuEncoder       = pcEncTop->getCuEncoder();
//    m_pcPredSearch      = pcEncTop->getPredSearch();
//
//    m_pcEntropyCoder    = pcEncTop->getEntropyCoder();
//    m_pcCavlcCoder      = pcEncTop->getCavlcCoder();
//    m_pcSbacCoder       = pcEncTop->getSbacCoder();
//    m_pcBinCABAC        = pcEncTop->getBinCABAC();
//    m_pcTrQuant         = pcEncTop->getTrQuant();
//
//    m_pcBitCounter      = pcEncTop->getBitCounter();
//    m_pcRdCost          = pcEncTop->getRdCost();
//    m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
//    m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();
//
//    // create lambda and QP arrays
//    m_pdRdPicLambda     = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
//    m_pdRdPicQp         = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
//    m_piRdPicQp         = (Int*   )xMalloc( Int,    m_pcCfg->getDeltaQpRD() * 2 + 1 );
//    m_pcRateCtrl        = pcEncTop->getRateCtrl();
//}
//
//
//
//Void
//TEncSlice::setUpLambda(TComSlice* slice, const Double dLambda, Int iQP)
//{
//    // store lambda
//    m_pcRdCost ->setLambda( dLambda );
//
//    // for RDO
//    // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
//    Double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
//    for(UInt compIdx=1; compIdx<MAX_NUM_COMPONENT; compIdx++)
//    {
//        const ComponentID compID=ComponentID(compIdx);
//        Int chromaQPOffset = slice->getPPS()->getQpOffset(compID) + slice->getSliceChromaQpDelta(compID);
//        Int qpc=(iQP + chromaQPOffset < 0) ? iQP : getScaledChromaQP(iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc());
//        Double tmpWeight = pow( 2.0, (iQP-qpc)/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
//        m_pcRdCost->setDistortionWeight(compID, tmpWeight);
//        dLambdas[compIdx]=dLambda/tmpWeight;
//    }
//
//#if RDOQ_CHROMA_LAMBDA
//    // for RDOQ
//    m_pcTrQuant->setLambdas( dLambdas );
//#else
//    m_pcTrQuant->setLambda( dLambda );
//#endif
//
//    // For SAO
//    slice   ->setLambdas( dLambdas );
//}
//
//
//
///**
// - non-referenced frame marking
// - QP computation based on temporal structure
// - lambda computation based on QP
// - set temporal layer ID and the parameter sets
// .
// \param pcPic         picture class
// \param pocLast      POC of last picture
// \param pocCurr     current POC
// \param iNumPicRcvd   number of received pictures
// \param iTimeOffset   POC offset for hierarchical structure
// \param iDepth        temporal layer depth
// \param rpcSlice      slice header class
// \param pSPS          SPS associated with the slice
// \param pPPS          PPS associated with the slice
// */
//
//// Cathy
//static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};
//static int Qpoff[] ={3, 2, 3, 1};
//static int track = 0;
//static bool isSc = false;
//
//Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc )
//{
//    Double dQP;
//    Double dLambda;
//
//    rpcSlice = pcPic->getSlice(0);
//    rpcSlice->setSPS( pSPS );
//    rpcSlice->setPPS( pPPS );
//    rpcSlice->setSliceBits(0);
//    rpcSlice->setPic( pcPic );
//    rpcSlice->initSlice();
//    rpcSlice->setPicOutputFlag( true );
//    rpcSlice->setPOC( pocCurr );
//
//    // depth computation based on GOP size
//    Int depth;
//    {
//#if FIX_FIELD_DEPTH
//        Int poc = rpcSlice->getPOC();
//        if(isField)
//        {
//            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
//        }
//        else
//        {
//            poc = poc % m_pcCfg->getGOPSize();
//        }
//#else
//        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
//#endif
//
//        if ( poc == 0 )
//        {
//            depth = 0;
//        }
//        else
//        {
//            Int step = m_pcCfg->getGOPSize();
//            depth    = 0;
//            for( Int i=step>>1; i>=1; i>>=1 )
//            {
//                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
//                {
//                    if ( j == poc )
//                    {
//                        i=0;
//                        break;
//                    }
//                }
//                step >>= 1;
//                depth++;
//            }
//        }
//
//#if FIX_FIELD_DEPTH
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        if(poc != 0)
//        {
//#endif
//            if (isField && ((rpcSlice->getPOC() % 2) == 1))
//            {
//                depth ++;
//            }
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        }
//#endif
//#endif
//    }
//
//
//    // slice type
//    SliceType eSliceType;
//
//    eSliceType=B_SLICE;
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif
//
//    rpcSlice->setSliceType    ( eSliceType );
//
//
//    // ------------------------------------------------------------------------------------------------------------------
//    // Non-referenced frame marking
//    // ------------------------------------------------------------------------------------------------------------------
//
//    if(pocLast == 0)
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(false);
//    }
//    else
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
//    }
//    rpcSlice->setReferenced(true);
//
//    // ------------------------------------------------------------------------------------------------------------------
//    // QP setting
//    // ------------------------------------------------------------------------------------------------------------------
//
////    cout << "Indie QP modifyyyyy  "  << boolalpha << isSceneChange<< endl;
//    if (isSceneChange) {
//
//        // Fix QP by track --> Next step
////        track = 3;
////        -12 bad
//
////        cout << "Indie QP modifyyyyy  " << endl;
//        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(SC_QP_OFFSET);
//
////        Int handleCaseQpMismatch = rpcSlice->getPOC();
////
////        if (handleCaseQpMismatch % 4 == 3)
////           m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(0);
////        else
////            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(SC_QP_OFFSET);
//
//
////                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-6);
////                        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-12);
////      Fix: //  m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
//
//        track = 0;
//        isSc = true;
//        eSliceType = I_SLICE;
//        rpcSlice->setSliceType(I_SLICE);
//    }
//    else
//    {
//////        cout << "\n Cathy: TEncSlice: LAST SCCCCCCC! " << lastSc;
//         if( (rpcSlice->getPOC() != 0) && lastSc != -1)
//         {
//             // Fix QP by track --> Next step
////             track = 1;
//             m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//             m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
//             track = (track+1)%m_pcCfg->getGOPSize();
//         }
//
//    }
//
//
//
//    dQP = m_pcCfg->getQP();
//
//
//
//    if(eSliceType!=I_SLICE)
//    {
//        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
//        {
//            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//        }
//    }
//
//    // Hossam: Scene change
//    if(isSceneChange)
//    {
//         dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//    }
//
//    // modify QP
//    Int* pdQPs = m_pcCfg->getdQPs();
//    if ( pdQPs )
//    {
//        dQP += pdQPs[ rpcSlice->getPOC() ];
//    }
//
//    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
//    {
//        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
//        m_pcCfg->setDeltaQpRD(0);
//    }
//
//    // ------------------------------------------------------------------------------------------------------------------
//    // Lambda computation
//    // ------------------------------------------------------------------------------------------------------------------
//
//    Int iQP;
//    Double dOrigQP = dQP;
//
//    // pre-compute lambda and QP values for all possible QP candidates
//    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
//    {
//        // compute QP value
//        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
//
//        // compute lambda value
//        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
//        Int    SHIFT_QP = 12;
//
//        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
//
//#if FULL_NBIT
//        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//        Int    bitdepth_luma_qp_scale = 0;
//#endif
//        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
//#if FULL_NBIT
//        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
//#endif
//
//        // Hossam: Computing eSliceType I or P or B
//        // Case #1: I or P-slices (key-frame)
//        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
//        if ( eSliceType==I_SLICE )
//        {
////            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQpFactor: " << dQPFactor << "\n" << endl;
////            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
//
//            dQPFactor=0.57*dLambda_scale;
//        }
//        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
//
////        cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
////        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
//        //      getchar();
//
//        if ( depth>0 )
//        {
//#if FULL_NBIT
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#else
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#endif
//        }
//
//        // if hadamard is used in ME process
//        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
//        {
//            dLambda *= 0.95;
//        }
//
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//
//        m_pdRdPicLambda[iDQpIdx] = dLambda;
//        m_pdRdPicQp    [iDQpIdx] = dQP;
//        m_piRdPicQp    [iDQpIdx] = iQP;
//    }
//
//    // obtain dQP = 0 case
//    dLambda = m_pdRdPicLambda[0];
//    dQP     = m_pdRdPicQp    [0];
//    iQP     = m_piRdPicQp    [0];
//
//    if( rpcSlice->getSliceType( ) != I_SLICE )
//    {
//        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    }
//
//    setUpLambda(rpcSlice, dLambda, iQP);
//
//#if HB_LAMBDA_FOR_LDC
//    // restore original slice type
//
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif // EFFICIENT_FIELD_IRAP
//
//
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
//    rpcSlice->setSliceType        ( eSliceType );
//#endif
//
//    //*************************************************************************************
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
//
//    // Hossam: Code Tweaking -- Second trial start
//
//    Int sc = rpcSlice->getPOC();
//    Int n = 2; /// 3rd frame
//    n = 5;
//
////    if(n == sc)
//    if(isSceneChange)
//    {
//        cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//        rpcSlice->setSliceType(I_SLICE);
//
//    }
//
//
//
//    // rpcSlice -> getSliceQp()
//    // rpcSlice -> setSliceQp(<#Int i#>)
//    //    cout << "\nYang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//    //    getchar();
//    //    if(sc == n)
//    // Sequence 1
//    /*
//     if(
//     sc == 13 ||
//     sc == 54 ||
//     sc == 110 ||
//     sc == 130 ||
//     sc == 165 ||
//     sc == 180 ||
//     sc == 213 ||
//     sc == 277
//     )
//     /*
//     // Sequence 2
//     if(
//     sc == 30 ||
//     sc == 84 ||
//     sc == 165 ||
//     sc == 240 ||
//     sc == 300
//     )
//
//     {
//     cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//     rpcSlice->setSliceType(I_SLICE);
//     //        getchar();
//     }
//     */
//    // Hossam Code Tweaking -- Second trial end
//    //*************************************************************************************
//
//    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
//    {
//        dQP = xGetQPValueAccordingToLambda( dLambda );
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//    }
//
//    rpcSlice->setSliceQp           ( iQP );
//#if ADAPTIVE_QP_SELECTION
//    rpcSlice->setSliceQpBase       ( iQP );
//#endif
//    rpcSlice->setSliceQpDelta      ( 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
//    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//
//    if ( m_pcCfg->getDeblockingFilterMetric() )
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag(true);
//        rpcSlice->setDeblockingFilterDisable(false);
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
//    {
//        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
//        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
//        if ( !rpcSlice->getDeblockingFilterDisable())
//        {
//            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//            }
//            else
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//            }
//        }
//    }
//    else
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag( false );
//        rpcSlice->setDeblockingFilterDisable( false );
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//
//    rpcSlice->setDepth            ( depth );
//
//    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    if(eSliceType==I_SLICE)
//    {
//        pcPic->setTLayer(0);
//    }
//    rpcSlice->setTLayer( pcPic->getTLayer() );
//
//    assert( m_apcPicYuvPred );
//    assert( m_apcPicYuvResi );
//
//    pcPic->setPicYuvPred( m_apcPicYuvPred );
//    pcPic->setPicYuvResi( m_apcPicYuvResi );
//
//    // Hossam: Settting Slice Mode
//    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
//    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
//    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
//    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
//    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
//    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
//}
//
//
//Void TEncSlice::initEncSlice( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField )
//{
//    Double dQP;
//    Double dLambda;
//
//    rpcSlice = pcPic->getSlice(0);
//    rpcSlice->setSPS( pSPS );
//    rpcSlice->setPPS( pPPS );
//    rpcSlice->setSliceBits(0);
//    rpcSlice->setPic( pcPic );
//    rpcSlice->initSlice();
//    rpcSlice->setPicOutputFlag( true );
//    rpcSlice->setPOC( pocCurr );
//
//    // depth computation based on GOP size
//    Int depth;
//    {
//#if FIX_FIELD_DEPTH
//        Int poc = rpcSlice->getPOC();
//        if(isField)
//        {
//            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
//        }
//        else
//        {
//            poc = poc % m_pcCfg->getGOPSize();
//        }
//#else
//        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
//#endif
//
//        if ( poc == 0 )
//        {
//            depth = 0;
//        }
//        else
//        {
//            Int step = m_pcCfg->getGOPSize();
//            depth    = 0;
//            for( Int i=step>>1; i>=1; i>>=1 )
//            {
//                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
//                {
//                    if ( j == poc )
//                    {
//                        i=0;
//                        break;
//                    }
//                }
//                step >>= 1;
//                depth++;
//            }
//        }
//
//#if FIX_FIELD_DEPTH
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        if(poc != 0)
//        {
//#endif
//            if (isField && ((rpcSlice->getPOC() % 2) == 1))
//            {
//                depth ++;
//            }
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        }
//#endif
//#endif
//    }
//
//
//    // slice type
//    SliceType eSliceType;
//
//    eSliceType=B_SLICE;
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif
//
//    rpcSlice->setSliceType    ( eSliceType );
//
//
//    // ------------------------------------------------------------------------------------------------------------------
//    // Non-referenced frame marking
//    // ------------------------------------------------------------------------------------------------------------------
//
//    if(pocLast == 0)
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(false);
//    }
//    else
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
//    }
//    rpcSlice->setReferenced(true);
//
//    // ------------------------------------------------------------------------------------------------------------------
//    // QP setting
//    // ------------------------------------------------------------------------------------------------------------------
//
//
//    dQP = m_pcCfg->getQP();
//
//
//
//    if(eSliceType!=I_SLICE)
//    {
//        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
//        {
//            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//        }
//    }
//
//    // modify QP
//    Int* pdQPs = m_pcCfg->getdQPs();
//    if ( pdQPs )
//    {
//        dQP += pdQPs[ rpcSlice->getPOC() ];
//    }
//
//    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
//    {
//        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
//        m_pcCfg->setDeltaQpRD(0);
//    }
//
//    // ------------------------------------------------------------------------------------------------------------------
//    // Lambda computation
//    // ------------------------------------------------------------------------------------------------------------------
//
//    Int iQP;
//    Double dOrigQP = dQP;
//
//    // pre-compute lambda and QP values for all possible QP candidates
//    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
//    {
//        // compute QP value
//        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
//
//        // compute lambda value
//        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
//        Int    SHIFT_QP = 12;
//
//        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
//
//#if FULL_NBIT
//        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//        Int    bitdepth_luma_qp_scale = 0;
//#endif
//        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
//#if FULL_NBIT
//        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
//#endif
//
//        // Hossam: Computing eSliceType I or P or B
//        // Case #1: I or P-slices (key-frame)
//        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
//        if ( eSliceType==I_SLICE )
//        {
//            dQPFactor=0.57*dLambda_scale;
//        }
//        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
//
////        cout << "\nCathy: TEncSlice: LAMDA WITH dLambda " << dLambda << "\n" << endl;
//        //      getchar();
//
//        if ( depth>0 )
//        {
//#if FULL_NBIT
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#else
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#endif
//        }
//
//        // if hadamard is used in ME process
//        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
//        {
//            dLambda *= 0.95;
//        }
//
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//
//        m_pdRdPicLambda[iDQpIdx] = dLambda;
//        m_pdRdPicQp    [iDQpIdx] = dQP;
//        m_piRdPicQp    [iDQpIdx] = iQP;
//    }
//
//    // obtain dQP = 0 case
//    dLambda = m_pdRdPicLambda[0];
//    dQP     = m_pdRdPicQp    [0];
//    iQP     = m_piRdPicQp    [0];
//
//    if( rpcSlice->getSliceType( ) != I_SLICE )
//    {
//        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    }
//
//    setUpLambda(rpcSlice, dLambda, iQP);
//
//#if HB_LAMBDA_FOR_LDC
//    // restore original slice type
//
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif // EFFICIENT_FIELD_IRAP
//
//
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
//    rpcSlice->setSliceType        ( eSliceType );
//#endif
//
//    //*************************************************************************************
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
//
//    // Hossam: Code Tweaking -- Second trial start
//
//    Int sc = rpcSlice->getPOC();
//    Int n = 2; /// 3rd frame
//
//    // rpcSlice -> getSliceQp()
//    // rpcSlice -> setSliceQp(<#Int i#>)
//    //    cout << "\nYang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//    //    getchar();
//    //    if(sc == n)
//    // Sequence 1
//    /*
//     if(
//     sc == 13 ||
//     sc == 54 ||
//     sc == 110 ||
//     sc == 130 ||
//     sc == 165 ||
//     sc == 180 ||
//     sc == 213 ||
//     sc == 277
//     )
//     /*
//     // Sequence 2
//     if(
//     sc == 30 ||
//     sc == 84 ||
//     sc == 165 ||
//     sc == 240 ||
//     sc == 300
//     )
//
//     {
//     cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//     rpcSlice->setSliceType(I_SLICE);
//     //        getchar();
//     }
//     */
//    // Hossam Code Tweaking -- Second trial end
//    //*************************************************************************************
//
//    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
//    {
//        dQP = xGetQPValueAccordingToLambda( dLambda );
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//    }
//
//    rpcSlice->setSliceQp           ( iQP );
//#if ADAPTIVE_QP_SELECTION
//    rpcSlice->setSliceQpBase       ( iQP );
//#endif
//    rpcSlice->setSliceQpDelta      ( 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
//    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//
//    if ( m_pcCfg->getDeblockingFilterMetric() )
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag(true);
//        rpcSlice->setDeblockingFilterDisable(false);
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
//    {
//        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
//        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
//        if ( !rpcSlice->getDeblockingFilterDisable())
//        {
//            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//            }
//            else
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//            }
//        }
//    }
//    else
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag( false );
//        rpcSlice->setDeblockingFilterDisable( false );
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//
//    rpcSlice->setDepth            ( depth );
//
//    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    if(eSliceType==I_SLICE)
//    {
//        pcPic->setTLayer(0);
//    }
//    rpcSlice->setTLayer( pcPic->getTLayer() );
//
//    assert( m_apcPicYuvPred );
//    assert( m_apcPicYuvResi );
//
//    pcPic->setPicYuvPred( m_apcPicYuvPred );
//    pcPic->setPicYuvResi( m_apcPicYuvResi );
//
//    // Hossam: Settting Slice Mode
//    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
//    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
//    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
//    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
//    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
//    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
//}
//
//
//Void TEncSlice::resetQP( TComPic* pic, Int sliceQP, Double lambda )
//{
//    TComSlice* slice = pic->getSlice(0);
//
//    // store lambda
//    slice->setSliceQp( sliceQP );
//#if ADAPTIVE_QP_SELECTION
//    slice->setSliceQpBase ( sliceQP );
//#endif
//    setUpLambda(slice, lambda, sliceQP);
//}
//
//// ====================================================================================================================
//// Public member functions
//// ====================================================================================================================
//
//Void TEncSlice::setSearchRange( TComSlice* pcSlice )
//{
//    Int iCurrPOC = pcSlice->getPOC();
//    Int iRefPOC;
//    Int iGOPSize = m_pcCfg->getGOPSize();
//    Int iOffset = (iGOPSize >> 1);
//    Int iMaxSR = m_pcCfg->getSearchRange();
//    Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;
//
//    for (Int iDir = 0; iDir <= iNumPredDir; iDir++)
//    {
//        //RefPicList e = (RefPicList)iDir;
//        RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
//        for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
//        {
//            iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
//            Int iNewSR = Clip3(8, iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
//            m_pcPredSearch->setAdaptiveSearchRange(iDir, iRefIdx, iNewSR);
//        }
//    }
//}
//
//
///**
// - multi-loop slice encoding for different slice QP
// .
// \param rpcPic    picture class
// */
//Void TEncSlice::precompressSliceNew( TComPic*& rpcPic )
//{
//    // Hossam: this step can be overlooked now: DeltaQpRd is 0 now
//    // if deltaQP RD is not used, simply return
//    if ( m_pcCfg->getDeltaQpRD() == 0 )
//    {
//        return;
//    }
//
//    if ( m_pcCfg->getUseRateCtrl() )
//    {
//        printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
//        assert(0);
//    }
//
//    TComSlice* pcSlice        = rpcPic->getSlice(getSliceIdx());
//    Double     dPicRdCostBest = MAX_DOUBLE;
//    UInt       uiQpIdxBest = 0;
//
//    Double dFrameLambda;
//#if FULL_NBIT
//    Int    SHIFT_QP = 12 + 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//    Int    SHIFT_QP = 12;
//#endif
//
//    // set frame lambda
//    if (m_pcCfg->getGOPSize() > 1)
//    {
//        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
//    }
//    else
//    {
//        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
//    }
//    m_pcRdCost      ->setFrameLambda(dFrameLambda);
//
//    // for each QP candidate
//    for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
//    {
//        pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
//#if ADAPTIVE_QP_SELECTION
//        pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
//#endif
//        setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);
//
//        // try compress
//        // Hossam: Change to compressSlice new
//        compressSliceNew   ( rpcPic );
////        compressSlice   ( rpcPic );
//
//        Double dPicRdCost;
//        UInt64 uiPicDist        = m_uiPicDist;
//        UInt64 uiALFBits        = 0;
//
//        m_pcGOPEncoder->preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
//
//        // compute RD cost and choose the best
//        dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);
//
//        if ( dPicRdCost < dPicRdCostBest )
//        {
//            uiQpIdxBest    = uiQpIdx;
//            dPicRdCostBest = dPicRdCost;
//        }
//    }
//
//    // set best values
//    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
//#if ADAPTIVE_QP_SELECTION
//    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
//#endif
//    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);
//}
//
//
///**
// - multi-loop slice encoding for different slice QP
// .
// \param rpcPic    picture class
// */
//Void TEncSlice::precompressSlice( TComPic*& rpcPic )
//{
//    // if deltaQP RD is not used, simply return
//    if ( m_pcCfg->getDeltaQpRD() == 0 )
//    {
//        return;
//    }
//
//    if ( m_pcCfg->getUseRateCtrl() )
//    {
//        printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
//        assert(0);
//    }
//
//    TComSlice* pcSlice        = rpcPic->getSlice(getSliceIdx());
//    Double     dPicRdCostBest = MAX_DOUBLE;
//    UInt       uiQpIdxBest = 0;
//
//    Double dFrameLambda;
//#if FULL_NBIT
//    Int    SHIFT_QP = 12 + 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//    Int    SHIFT_QP = 12;
//#endif
//
//    // set frame lambda
//    if (m_pcCfg->getGOPSize() > 1)
//    {
//        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
//    }
//    else
//    {
//        dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
//    }
//    m_pcRdCost      ->setFrameLambda(dFrameLambda);
//
//    // for each QP candidate
//    for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
//    {
//        pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );
//#if ADAPTIVE_QP_SELECTION
//        pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
//#endif
//        setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);
//
//        // try compress
//        compressSlice   ( rpcPic );
//
//        Double dPicRdCost;
//        UInt64 uiPicDist        = m_uiPicDist;
//        UInt64 uiALFBits        = 0;
//
//        m_pcGOPEncoder->preLoopFilterPicAll( rpcPic, uiPicDist, uiALFBits );
//
//        // compute RD cost and choose the best
//        dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits + uiALFBits, uiPicDist, true, DF_SSE_FRAME);
//
//        if ( dPicRdCost < dPicRdCostBest )
//        {
//            uiQpIdxBest    = uiQpIdx;
//            dPicRdCostBest = dPicRdCost;
//        }
//    }
//
//    // set best values
//    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );
//#if ADAPTIVE_QP_SELECTION
//    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
//#endif
//    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);
//}
//
//Void TEncSlice::calCostSliceI(TComPic*& rpcPic)
//{
//    UInt    uiCUAddr;
//    UInt    uiStartCUAddr;
//    UInt    uiBoundingCUAddr;
//    Int     iSumHad, shift = g_bitDepth[CHANNEL_TYPE_LUMA]-8, offset = (shift>0)?(1<<(shift-1)):0;;
//    Double  iSumHadSlice = 0;
//
//    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
//    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
//    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
//
//    UInt uiEncCUOrder;
//    uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
//        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
//        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
//    {
//        // initialize CU encoder
//        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
//        pcCU->initCU( rpcPic, uiCUAddr );
//
//        Int height  = min( pcSlice->getSPS()->getMaxCUHeight(),pcSlice->getSPS()->getPicHeightInLumaSamples() - uiCUAddr / rpcPic->getFrameWidthInCU() * pcSlice->getSPS()->getMaxCUHeight() );
//        Int width   = min( pcSlice->getSPS()->getMaxCUWidth(),pcSlice->getSPS()->getPicWidthInLumaSamples() - uiCUAddr % rpcPic->getFrameWidthInCU() * pcSlice->getSPS()->getMaxCUWidth() );
//
//        iSumHad = m_pcCuEncoder->updateLCUDataISlice(pcCU, uiCUAddr, width, height);
//
//        (m_pcRateCtrl->getRCPic()->getLCU(uiCUAddr)).m_costIntra=(iSumHad+offset)>>shift;
//        iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(uiCUAddr)).m_costIntra;
//
//    }
//    m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
//}
//
//static const int fourTwoZeroScale = 4;
//// Hossam: Dump the residuals to a YUV file
//Void TEncSlice::xDumpResiduals(TComSlice* pcSlice)
//{
//
//    cout << "xDumppppppppppppp Noura Rawan Amanyyyy :X :X :X" << endl;
//    Char *pFileName = "hello.yuv";
//    Bool bAdd = true;
//
//
////        outPicYuv->dump(pFileName, isAdd);
//
//    FILE* pFile;
//    if (!bAdd)
//    {
//        pFile = fopen (pFileName, "wb");
//    }
//    else
//    {
//        pFile = fopen (pFileName, "ab");
//    }
//
//    int n_y =    pcSlice->getSPS()->getPicHeightInLumaSamples() * pcSlice->getSPS()->getPicWidthInLumaSamples();
//
//    // Hossam: XXXX assuming 420
//    int n_u = n_y/fourTwoZeroScale;
//    int n_v = n_y/fourTwoZeroScale;
//
//
//
//    // Dump Y
//    Int chan = 0;
//    const ComponentID  ch     = ComponentID(chan);
//    const Int          shift  = g_bitDepth[toChannelType(ch)] - 8;
//    const Int          offset = (shift>0)?(1<<(shift-1)):0;
//
//    // Let is point to the address of the Y
//    const Pel         *pi     = m_pcPredSearch->allData_Y;
//    const Int          stride = 8;
////    const Int          height = 240;
////    const Int          width  = 416;
//    const Int          height = 416;
//    const Int          width  = 240;
//
//
////    const Int          height = getHeight(ch);
////    const Int          width  = getWidth(ch);
//
////    for (Int y = 0; y < height; y++ )
////    {
////        for (Int x = 0; x < width; x++ )
////        {
////            UChar uc = (UChar)Clip3<Pel>(0, 255, (pi[x]+offset)>>shift);
////
////            cout << "Value Write " << Int(uc) << endl;
////            fwrite( &uc, sizeof(UChar), 1, pFile );
////        }
////        pi += stride;
////    }
//
//    for (int i = 0; i < width*height; i++) {
//        UChar uc = pi[i];
//
//        cout << "Value Write " << Int(uc) << endl;
//        fwrite( &uc, sizeof(UChar), 1, pFile );
//    }
//
//
//    // Destroy Y
//    delete [] m_pcPredSearch->allData_Y;
//
//    ////    delete [] m_pcCuEncoder->allData_U;
//    ////    delete [] m_pcCuEncoder->allData_V;
//
//
//
//}
//
//
////static const int fourTwoZeroScale = 4;
////// Hossam: Dump the residuals to a YUV file
////Void TEncSlice::xDumpResiduals(TComSlice* pcSlice)
////{
////
////    cout << "xDumppppppppppppp Noura Rawan Amanyyyy :X :X :X" << endl;
////    Char *pFileName = "hello.yuv";
////    Bool bAdd = true;
//////    TComList<TComYuv*>::iterator iterCUs   = m_pcCuEncoder->m_scResidualFrame.begin();
////
////
//////    TComPicYuv *& outPicYuv = m_pcCuEncoder->m_scResidualFrame;
////    Bool isAdd = true;
//////
//////    outPicYuv->dump(pFileName, isAdd);
////
////    FILE* pFile;
////    if (!bAdd)
////    {
////        pFile = fopen (pFileName, "wb");
////    }
////    else
////    {
////        pFile = fopen (pFileName, "ab");
////    }
////
////    int n_y =    pcSlice->getSPS()->getPicHeightInLumaSamples() * pcSlice->getSPS()->getPicWidthInLumaSamples();
////
////    // Hossam: XXXX assuming 420
////    int n_u = n_y/fourTwoZeroScale;
////    int n_v = n_y/fourTwoZeroScale;
////
////
//////    cout << "Number of Y samples " << n_y << endl;
////
////
////    // Dump Y
//////    Int chan = 0;
//////      const ComponentID  ch     = COMPONENT_Y;
//////    const Int          shift  = g_bitDepth[toChannelType(ch)] - 8;
//////    const Int          offset = (shift>0)?(1<<(shift-1)):0;
//////
//////    for (Int i = 0; i < n_y; i++) {
////////        UChar uc = (UChar)Clip3<Pel>(0, 255, (m_pcCuEncoder->allData_Y[i]+offset)>>shift);
////////        fwrite( &uc, sizeof(UChar), 1, pFile );
//////
//////        UChar uc = m_pcCuEncoder->allData_Y[i];
//////        fwrite( &uc, sizeof(UChar), 1, pFile );
//////    }
//////
//////
//////    // Dump U
//////    chan = 1;
//////    const ComponentID  ch2     = COMPONENT_Cb;
//////    const Int          shift2  = g_bitDepth[toChannelType(ch2)] - 8;
//////    const Int          offset2 = (shift>0)?(1<<(shift-1)):0;
//////
//////    for (Int i = 0; i < n_u; i++) {
////////        UChar uc = (UChar)Clip3<Pel>(0, 255, (m_pcCuEncoder->allData_U[i]+offset2)>>shift2);
////////        fwrite( &uc, sizeof(UChar), 1, pFile );
//////
//////        UChar uc = m_pcCuEncoder->allData_U[i];
//////        fwrite( &uc, sizeof(UChar), 1, pFile );
//////    }
//////
//////
//////    // Dump V
//////    chan = 2;
//////    const ComponentID  ch3     = COMPONENT_Cr;
//////    const Int          shift3  = g_bitDepth[toChannelType(ch3)] - 8;
//////    const Int          offset3 = (shift>0)?(1<<(shift-1)):0;
//////
//////    for (Int i = 0; i < n_v; i++) {
////////        UChar uc = (UChar)Clip3<Pel>(0, 255, (m_pcCuEncoder->allData_V[i]+offset3)>>shift3);
////////        fwrite( &uc, sizeof(UChar), 1, pFile );
//////
//////        UChar uc = m_pcCuEncoder->allData_V[i];
//////        fwrite( &uc, sizeof(UChar), 1, pFile );
//////    }
//////
//////    fclose(pFile);
//////    /////
//////
//////
//////    // destroy the frame
////    cout << "xDumppppppppppppp nestelroooy el Destroyyyy  :X :X :X" << endl;
//////    m_pcCuEncoder->m_scResidualFrame->destroy();
////
////
//////
//////    delete [] m_pcCuEncoder->allData_Y;
//////    delete [] m_pcCuEncoder->allData_U;
//////    delete [] m_pcCuEncoder->allData_V;
//////
////
////    //    TComList<TComPic*>::iterator iterPic   =   m_pcGOPEncoder->getListPic()->begin();
////
////
//////    TComYuv* current_cu = *(iterCUs);
////////    TComPic* current_frame = *(iterPic);
//////
//////    TComPicYuv* current_frame;
////
////
////
////
//////    for (UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
//////    {
//////        if (m_outputBitDepth[channelType] == 0) m_outputBitDepth[channelType] = g_bitDepth[channelType];
//////    }
//////    m_pcCuEncoder->m_scResidualFrame.clear();
////
//////    TComPicYuv* outPic = new TComPicYuv;
//////      outPic->create(m_iSourceWidth , m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
////
////
////}
//
//// Hossam: Scene change
//Void TEncSlice:: xCreateSCResidualsBuffer(TComPic *rpcPic, TComSlice *pcSlice)
//{
//
//    cout << "xCreateSCResidualsBuffer  Create the residuals " << endl;
////    Int iPicWidth  = 416;
////    Int iPicHeight = 240;
//
////    UInt uiMaxCUWidth  = 7;
////    UInt uiMaxCUHeight = 4;
//
//    Int iPicWidth  = 416;
//    Int iPicHeight = 240;
//
//
//
//    UInt uiMaxCUWidth  =  rpcPic->getPicSym()->getFrameWidthInCU();
//    UInt uiMaxCUHeight =  rpcPic->getPicSym()->getFrameHeightInCU();
//    UInt uiMaxCUDepth  = SC_MAX_DEPTH;
//
//
////    m_pcCuEncoder->m_scResidualFrame = new TComPicYuv;
////    m_pcCuEncoder->m_scResidualFrame->create(iPicWidth, iPicHeight, CHROMA_420, uiMaxCUWidth, uiMaxCUHeight, uiMaxCUDepth);
//
////    m_pcCuEncoder->allData_Y = new Int[iPicWidth*iPicHeight];
////    m_pcCuEncoder->allData_U = new Int[iPicWidth/2*iPicHeight/2];
////    m_pcCuEncoder->allData_V = new Int[iPicWidth/2*iPicHeight/2];
//
////    m_pcCuEncoder->allData_Y = new Pel[iPicWidth*iPicHeight];
////    m_pcCuEncoder->allData_U = new Pel[iPicWidth/2*iPicHeight/2];
////    m_pcCuEncoder->allData_V = new Pel[iPicWidth/2*iPicHeight/2];
//
////    m_pcCuEncoder->allData = new Int;
//
//    m_pcPredSearch->allData_Y = new Pel[iPicWidth*iPicHeight];
//
//
//}
//
//// Hossam: Scene change
///** \param rpcPic   picture class
// */
//
//// Hossam: Changed the TComPic&* rpcPic to TComPic* rpcPic to be able to work on a copy
//// Hossam: No need since I already think I am taking copies in SC engine
//// and not the Original
//Void TEncSlice::compressSliceNew( TComPic*& rpcPic )
//{
//    UInt   uiStartCUAddr;
//    UInt   uiBoundingCUAddr;
//    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
//    TEncBinCABAC* pppcRDSbacCoder = NULL;
//    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
//
//    // Determine the start and bounding CU address
//    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
//
//    // initialize cost values
//    m_uiPicTotalBits  = 0;
//    m_dPicRdCost      = 0;
//    m_uiPicDist       = 0;
//
//    //    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
//
//    // set entropy coder
//    m_pcSbacCoder->init( m_pcBinCABAC );
//    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
//    m_pcEntropyCoder->resetEntropy      ();
//    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
//    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
//    pppcRDSbacCoder->setBinCountingEnableFlag( false );
//    pppcRDSbacCoder->setBinsCoded( 0 );
//
//    //------------------------------------------------------------------------------
//    //  Weighted Prediction parameters estimation.
//    //------------------------------------------------------------------------------
//    // Hossam: Weighted prediction is not visited so far --
//    // calculate AC/DC values for current picture
//    if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
//    {
//        cout << " Do I visit this case at all! " << endl;
//        xCalcACDCParamSlice(pcSlice);
//    }
//
//    Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
//
//    if ( bWp_explicit )
//    {
//
//        //      cout << " Do I visit this case at all! (2) " << endl;
//        //------------------------------------------------------------------------------
//        //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
//        //------------------------------------------------------------------------------
//        // Hossam: WP is not visites so far XXXXX
//        if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
//        {
//            //      printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
//        }
//
//        xEstimateWPParamSlice( pcSlice );
//        pcSlice->initWpScaling();
//
//        // check WP on/off
//        xCheckWPEnable( pcSlice );
//    }
//
//#if ADAPTIVE_QP_SELECTION
//    if( m_pcCfg->getUseAdaptQpSelect() )
//    {
//        m_pcTrQuant->clearSliceARLCnt();
//        if(pcSlice->getSliceType()!=I_SLICE)
//        {
//            Int qpBase = pcSlice->getSliceQpBase();
//            pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
//        }
//    }
//#endif
//
//    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
//    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
//    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
//    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
//    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
//    delete[] m_pcBufferSbacCoders;
//    delete[] m_pcBufferBinCoderCABACs;
//    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
//    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
//    for (Int ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
//    }
//    for (UInt ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
//    }
//
//    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
//    {
//        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
//    }
//    delete[] m_pcBufferLowLatSbacCoders;
//    delete[] m_pcBufferLowLatBinCoderCABACs;
//    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
//    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
//    for (Int ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
//    }
//    for (UInt ui = 0; ui < uiTilesAcross; ui++)
//        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
//
//    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
//    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
//    UInt      uiTileCol               = 0;
//    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
//    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
//    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
//    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
//    if( depSliceSegmentsEnabled )
//    {
//        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
//        {
//            UInt uiSubStrm=0;
//            if( m_pcCfg->getWaveFrontsynchro() )
//            {
//                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
//                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
//                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//                if ( pCurrentTile->getTileWidth() < 2)
//                {
//                    CTXMem[0]->loadContexts(m_pcSbacCoder);
//                }
//            }
//            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//        }
//        else
//        {
//            if(m_pcCfg->getWaveFrontsynchro())
//            {
//                CTXMem[1]->loadContexts(m_pcSbacCoder);
//            }
//            CTXMem[0]->loadContexts(m_pcSbacCoder);
//        }
//    }
//
//
//    // for every CU in slice
//    UInt uiEncCUOrder;
//
//
//
//    // Hossam: Scene Change
//    // xCreate the YUV residuals buffer
////    xCreateSCResidualsBuffer(rpcPic, pcSlice);
//
//
//    //    cout << "Hanteera wedding Start address: " << uiStartCUAddr/rpcPic->getNumPartInCU() << endl; // --> 0
//    //    cout << "Hanteera wedding End address: " << (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU() << endl; // --> 28
//
//
//    //    cout << "Best Plan Ticket: Width in CUs: " << uiWidthInLCUs << endl; // 6.5 CUs rounded up to 7
//    //    cout << "Best Plan Ticket: Height in CUs: " << rpcPic->getPicSym()->getFrameHeightInCU() << endl; // 3.75 up to 4
//
//
//    // 256 partitions
//    //    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
//    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
//        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
//        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
//    {
//        // initialize CU encoder
//
//#if SC_ENABLE_PRINT
//        // Problem here
//        cout << "uiCUAddr: " << uiCUAddr << endl;
//
//#endif
//        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
//
//        //      cout << "isNil: " << (pcCU == NULL) << endl;
////        pcCU->initCU( rpcPic, uiCUAddr );
//          pcCU->initCUNew( rpcPic, uiCUAddr );
//
//
//
//        // inherit from TR if necessary, select substream to use.
//        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
//        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
//        UInt uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
//        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
//        UInt uiCol     = uiCUAddr % uiWidthInLCUs;
//        UInt uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//
//        if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
//        {
//            // We'll sync if the TR is available.
//            TComDataCU *pcCUUp = pcCU->getCUAbove();
//            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
//            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
//            TComDataCU *pcCUTR = NULL;
//            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
//            {
//                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
//            }
//            if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
//                  (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
//                  ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
//                  )
//                )
//            {
//                // TR not available.
//            }
//            else
//            {
//                // TR is available, we use it.
//                ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
//            }
//        }
//        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code
//
//        // reset the entropy coder
//        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
//           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
//           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
//           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
//        {
//            SliceType sliceType = pcSlice->getSliceType();
//            if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
//            {
//                sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
//            }
//            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
//            m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
//            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
//            m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
//        }
//
//        // set go-on entropy coder
//        m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
//        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
//
//        ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);
//
//        Double oldLambda = m_pcRdCost->getLambda();
//        if ( m_pcCfg->getUseRateCtrl() )
//        {
//            Int estQP        = pcSlice->getSliceQp();
//            Double estLambda = -1.0;
//            Double bpp       = -1.0;
//
//            if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
//            {
//                estQP = pcSlice->getSliceQp();
//            }
//            else
//            {
//                bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
//                if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
//                {
//                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
//                }
//                else
//                {
//                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
//                    estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
//                }
//
//                estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );
//
//                m_pcRdCost->setLambda(estLambda);
//
//#if RDOQ_CHROMA_LAMBDA
//                // set lambda for RDOQ
//                const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
//                const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
//                m_pcTrQuant->setLambdas( lambdaArray );
//#else
//                m_pcTrQuant->setLambda( estLambda );
//#endif
//            }
//
//            m_pcRateCtrl->setRCQP( estQP );
//#if ADAPTIVE_QP_SELECTION
//            pcCU->getSlice()->setSliceQpBase( estQP );
//#endif
//        }
//
//        // run CU encoder
////        m_pcCuEncoder->compressCU( pcCU );
//          m_pcCuEncoder->compressCUNew( pcCU );
//
//         // Hossam: Save the CU residual data in the Residual frame buffer
//        // Hopefully this does not casue problems SEG faults..etc
//
////        TComPicYuv*             m_apcPicYuvResi;                      ///< residual picture buffer
//
//#if SC_ENABLE_PRINT
//        cout << "(((((Run the CU encoder: " << uiEncCUOrder << ", pcCU->getAddr()" << pcCU->getAddr() << " )))))" << endl;
//#endif
//        m_pcCuEncoder->scBestResidual[SC_BEST_BUFFER_IND]->copyToPicYuv(m_apcPicYuvResi, pcCU->getAddr(), pcCU->getZorderIdxInCU());
//
//
//
////          cout << "(((((Run the CU encoder: " << uiEncCUOrder << " )))))" << endl;
//
//
//        // restore entropy coder to an initial stage
//        m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
//        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
//        m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
//        m_pcBitCounter = &pcBitCounters[uiSubStrm];
//        pppcRDSbacCoder->setBinCountingEnableFlag( true );
//        m_pcBitCounter->resetBits();
//        pppcRDSbacCoder->setBinsCoded( 0 );
//
//
//        // Don't encode the CU!
////        m_pcCuEncoder->encodeCU( pcCU );
//
//        pppcRDSbacCoder->setBinCountingEnableFlag( false );
//        if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
//        {
//            pcSlice->setNextSlice( true );
//            break;
//        }
//        if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
//        {
//            pcSlice->setNextSliceSegment( true );
//            break;
//        }
//
//        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
//
//        //Store probabilties of second LCU in line into buffer
//        if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
//        {
//            m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
//        }
//
//        if ( m_pcCfg->getUseRateCtrl() )
//        {
//            Int actualQP        = g_RCInvalidQPValue;
//            Double actualLambda = m_pcRdCost->getLambda();
//            Int actualBits      = pcCU->getTotalBits();
//            Int numberOfEffectivePixels    = 0;
//            for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
//            {
//                if ( pcCU->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pcCU->isSkipped( idx ) ) )
//                {
//                    numberOfEffectivePixels = numberOfEffectivePixels + 16;
//                    break;
//                }
//            }
//
//            if ( numberOfEffectivePixels == 0 )
//            {
//                actualQP = g_RCInvalidQPValue;
//            }
//            else
//            {
//                actualQP = pcCU->getQP( 0 );
//            }
//            m_pcRdCost->setLambda(oldLambda);
//            m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
//                                                     pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
//        }
//
//        m_uiPicTotalBits += pcCU->getTotalBits();
//        m_dPicRdCost     += pcCU->getTotalCost();
//        m_uiPicDist      += pcCU->getTotalDistortion();
//    }
//    if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
//    {
//        pcSlice->setNextSlice( true );
//    }
//    if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
//    {
//        if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
//        {
//            pcSlice->setNextSlice( true );
//        }
//        else
//        {
//            pcSlice->setNextSliceSegment( true );
//        }
//    }
//    if( depSliceSegmentsEnabled )
//    {
//        if (m_pcCfg->getWaveFrontsynchro())
//        {
//            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
//        }
//        CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
//    }
//    xRestoreWPparam( pcSlice );
//
//
//
//        // Hossam: Scene Change
////        xDumpResiduals(pcSlice);
//
////    scBestResidual[0]->mockPrintFromSlice(rpcCU->getZorderIdxInCU());
////    m_apcPicYuvResi->mockPrintComponent(COMPONENT_Y, 0);
//
//
////    m_apcPicYuvResi->mockPrintComponentLineBlk(COMPONENT_Y, 0, m_apcPicYuvResi->getWidth(COMPONENT_Y), m_apcPicYuvResi->getHeight(COMPONENT_Y), true, 400);
//
////        cout << "EXIT " << endl;
////        if(pcSlice->getPOC() == 2)
////            exit (EXIT_FAILURE);
//
//} // end compressSliceNew
//
////Void TEncSlice::compressSliceNew( TComPic*& rpcPic )
////{
////    UInt   uiStartCUAddr;
////    UInt   uiBoundingCUAddr;
////
////
////    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
////    TEncBinCABAC* pppcRDSbacCoder = NULL;
////
////
////    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
////    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
////
////    // initialize cost values
////    m_uiPicTotalBits  = 0;
////    m_dPicRdCost      = 0;
////    m_uiPicDist       = 0;
////
////    // Up-till here we can move the encoder to almost the same performance 1/
////    //    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
////
////    // Hossam: Removed the entropy card parts XXX
////
////    // set entropy coder
////    m_pcSbacCoder->init( m_pcBinCABAC );
////    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
////    m_pcEntropyCoder->resetEntropy      ();
////    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
////    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
////    pppcRDSbacCoder->setBinCountingEnableFlag( false );
////    pppcRDSbacCoder->setBinsCoded( 0 );
////
////    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
////    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
////    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
////    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
////    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
////    delete[] m_pcBufferSbacCoders;
////    delete[] m_pcBufferBinCoderCABACs;
////    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
////    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
////    for (Int ui = 0; ui < uiTilesAcross; ui++)
////    {
////        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
////    }
////    for (UInt ui = 0; ui < uiTilesAcross; ui++)
////    {
////        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
////    }
////
////    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
////    {
////        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
////    }
////    delete[] m_pcBufferLowLatSbacCoders;
////    delete[] m_pcBufferLowLatBinCoderCABACs;
////    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
////    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
////    for (Int ui = 0; ui < uiTilesAcross; ui++)
////    {
////        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
////    }
////    for (UInt ui = 0; ui < uiTilesAcross; ui++)
////        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
////
////
////
////
////    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
////    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
////    UInt      uiTileCol               = 0;
////    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
////    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
////    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
////    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
////    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
////
////    //    // Hossam: According to Nan, it's always false XXXX
////    //    if( depSliceSegmentsEnabled )
////    //    {
////    //        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
////    //        {
////    //            UInt uiSubStrm=0;
////    //            if( m_pcCfg->getWaveFrontsynchro() )
////    //            {
////    //                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
////    //                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
////    //                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
////    //                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
////    //                if ( pCurrentTile->getTileWidth() < 2)
////    //                {
////    //                    CTXMem[0]->loadContexts(m_pcSbacCoder);
////    //                }
////    //            }
////    //            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
////    //            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
////    //        }
////    //        else
////    //        {
////    //            if(m_pcCfg->getWaveFrontsynchro())
////    //            {
////    //                CTXMem[1]->loadContexts(m_pcSbacCoder);
////    //            }
////    //            CTXMem[0]->loadContexts(m_pcSbacCoder);
////    //        }
////    //    }
////    //------------------------------------------------------------------------------
////    // Hossam: Actual CU encoding
////
////    // for every CU in slice
////    UInt uiEncCUOrder;
////
////
////    // Hossam: 256 mazboot
////    //    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
////    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
////        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
////        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
////    {
////        cout << " uiCUAddr " << uiCUAddr << endl;
////
////        TComDataCU*& pcCU = rpcPic->getCU(uiCUAddr);
////        pcCU->initCUNew( rpcPic, uiCUAddr );
////
////        cout << "Init CU done  " << endl;
////        // run CU encoder
////        m_pcCuEncoder->compressCUNew( pcCU );
////
////
////        cout << "Done with CU DonEEEEEEEEEE Erb street: " << uiEncCUOrder << endl;
////        // Hossam: Removed the steps after that XXXX
////
////        // Added the bits and RD cost calculations
////        m_uiPicTotalBits += pcCU->getTotalBits();
////        m_dPicRdCost     += pcCU->getTotalCost();
////        m_uiPicDist      += pcCU->getTotalDistortion();
////
////    }
////
////
////} // end compressSliceNew
//
////Void TEncSlice::compressSliceNew( TComPic*& rpcPic )
////{
////    UInt   uiStartCUAddr;
////    UInt   uiBoundingCUAddr;
////
////
////    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
////    TEncBinCABAC* pppcRDSbacCoder = NULL;
////
////
////    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
////    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
////
////    // initialize cost values
////    m_uiPicTotalBits  = 0;
////    m_dPicRdCost      = 0;
////    m_uiPicDist       = 0;
////
////    // Up-till here we can move the encoder to almost the same performance 1/
//////    cout << "uiStartAddr: " << uiStartCUAddr << " uiBoundingAddr " << uiBoundingCUAddr << endl;
////
////   // Hossam: Removed the entropy card parts XXX
////
////    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
////    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
////    UInt      uiTileCol               = 0;
////    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
////    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
////    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
////    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
////    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
////
//////    // Hossam: According to Nan, it's always false
//////    if( depSliceSegmentsEnabled )
//////    {
//////        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
//////        {
//////            UInt uiSubStrm=0;
//////            if( m_pcCfg->getWaveFrontsynchro() )
//////            {
//////                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
//////                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
//////                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//////                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//////                if ( pCurrentTile->getTileWidth() < 2)
//////                {
//////                    CTXMem[0]->loadContexts(m_pcSbacCoder);
//////                }
//////            }
//////            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//////            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//////        }
//////        else
//////        {
//////            if(m_pcCfg->getWaveFrontsynchro())
//////            {
//////                CTXMem[1]->loadContexts(m_pcSbacCoder);
//////            }
//////            CTXMem[0]->loadContexts(m_pcSbacCoder);
//////        }
//////    }
////    //------------------------------------------------------------------------------
////    // Hossam: Actual CU encoding
////
////    // for every CU in slice
////    UInt uiEncCUOrder;
////
////
////    // Hossam: 256 mazboot
//////    cout << "Number of partitions: " << rpcPic->getNumPartInCU() << endl;
////    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
////        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
////        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
////    {
////        cout << " uiCUAddr " << uiCUAddr << endl;
////
////        TComDataCU*& pcCU = rpcPic->getCU(uiCUAddr);
////        pcCU->initCUNew( rpcPic, uiCUAddr );
////
////        cout << "Init CU done  " << endl;
////        // run CU encoder
////        m_pcCuEncoder->compressCUNew( pcCU );
////
////
////        cout << "Done with CU DonEEEEEEEEEE Erb street: " << uiEncCUOrder << endl;
////        // Hossam: Removed the steps after that XXXX
////
////        // Added the bits and RD cost calculations
////        m_uiPicTotalBits += pcCU->getTotalBits();
////        m_dPicRdCost     += pcCU->getTotalCost();
////        m_uiPicDist      += pcCU->getTotalDistortion();
////
////    }
////
////
////} // end compressSliceNew
//
//
//
//
///** \param rpcPic   picture class
// */
//Void TEncSlice::compressSlice( TComPic*& rpcPic )
//{
//    UInt   uiStartCUAddr;
//    UInt   uiBoundingCUAddr;
//    rpcPic->getSlice(getSliceIdx())->setSliceSegmentBits(0);
//    TEncBinCABAC* pppcRDSbacCoder = NULL;
//    TComSlice* pcSlice            = rpcPic->getSlice(getSliceIdx());
//    xDetermineStartAndBoundingCUAddr ( uiStartCUAddr, uiBoundingCUAddr, rpcPic, false );
//
//    // initialize cost values
//    m_uiPicTotalBits  = 0;
//    m_dPicRdCost      = 0;
//    m_uiPicDist       = 0;
//
//
//    // set entropy coder
//    m_pcSbacCoder->init( m_pcBinCABAC );
//    m_pcEntropyCoder->setEntropyCoder   ( m_pcSbacCoder, pcSlice );
//    m_pcEntropyCoder->resetEntropy      ();
//    m_pppcRDSbacCoder[0][CI_CURR_BEST]->load(m_pcSbacCoder);
//    pppcRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
//    pppcRDSbacCoder->setBinCountingEnableFlag( false );
//    pppcRDSbacCoder->setBinsCoded( 0 );
//
//    //------------------------------------------------------------------------------
//    //  Weighted Prediction parameters estimation.
//    //------------------------------------------------------------------------------
//    // calculate AC/DC values for current picture
//    if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
//    {
//        xCalcACDCParamSlice(pcSlice);
//    }
//
//    Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());
//
//    if ( bWp_explicit )
//    {
//        //------------------------------------------------------------------------------
//        //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
//        //------------------------------------------------------------------------------
//        if ( pcSlice->getSliceMode()==2 || pcSlice->getSliceSegmentMode()==2 )
//        {
//            printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
//        }
//
//        xEstimateWPParamSlice( pcSlice );
//        pcSlice->initWpScaling();
//
//        // check WP on/off
//        xCheckWPEnable( pcSlice );
//    }
//
//#if ADAPTIVE_QP_SELECTION
//    if( m_pcCfg->getUseAdaptQpSelect() )
//    {
//        m_pcTrQuant->clearSliceARLCnt();
//        if(pcSlice->getSliceType()!=I_SLICE)
//        {
//            Int qpBase = pcSlice->getSliceQpBase();
//            pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
//        }
//    }
//#endif
//    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
//    TEncSbac**** ppppcRDSbacCoders    = pcEncTop->getRDSbacCoders();
//    TComBitCounter* pcBitCounters     = pcEncTop->getBitCounters();
//    const Int  iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
//    const UInt uiTilesAcross  = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
//    delete[] m_pcBufferSbacCoders;
//    delete[] m_pcBufferBinCoderCABACs;
//    m_pcBufferSbacCoders     = new TEncSbac    [uiTilesAcross];
//    m_pcBufferBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
//    for (Int ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferSbacCoders[ui].init( &m_pcBufferBinCoderCABACs[ui] );
//    }
//    for (UInt ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
//    }
//
//    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ ) //init all sbac coders for RD optimization
//    {
//        ppppcRDSbacCoders[ui][0][CI_CURR_BEST]->load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
//    }
//    delete[] m_pcBufferLowLatSbacCoders;
//    delete[] m_pcBufferLowLatBinCoderCABACs;
//    m_pcBufferLowLatSbacCoders     = new TEncSbac    [uiTilesAcross];
//    m_pcBufferLowLatBinCoderCABACs = new TEncBinCABAC[uiTilesAcross];
//    for (Int ui = 0; ui < uiTilesAcross; ui++)
//    {
//        m_pcBufferLowLatSbacCoders[ui].init( &m_pcBufferLowLatBinCoderCABACs[ui] );
//    }
//    for (UInt ui = 0; ui < uiTilesAcross; ui++)
//        m_pcBufferLowLatSbacCoders[ui].load(m_pppcRDSbacCoder[0][CI_CURR_BEST]);  //init. state
//
//    UInt      uiWidthInLCUs           = rpcPic->getPicSym()->getFrameWidthInCU();
//    // UInt   uiHeightInLCUs          = rpcPic->getPicSym()->getFrameHeightInCU();
//    UInt      uiTileCol               = 0;
//    Bool      depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
//    UInt      uiCUAddr                = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//    UInt      currentTileIdx          = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
//    TComTile *pCurrentTile            = rpcPic->getPicSym()->getTComTile(currentTileIdx);
//    UInt      uiTileStartLCU          = pCurrentTile->getFirstCUAddr();
//    if( depSliceSegmentsEnabled )
//    {
//        if((pcSlice->getSliceSegmentCurStartCUAddr()!= pcSlice->getSliceCurStartCUAddr())&&(uiCUAddr != uiTileStartLCU))
//        {
//            UInt uiSubStrm=0;
//            if( m_pcCfg->getWaveFrontsynchro() )
//            {
//                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
//                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
//                //uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//                if ( pCurrentTile->getTileWidth() < 2)
//                {
//                    CTXMem[0]->loadContexts(m_pcSbacCoder);
//                }
//            }
//            m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//            ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( CTXMem[0] );
//        }
//        else
//        {
//            if(m_pcCfg->getWaveFrontsynchro())
//            {
//                CTXMem[1]->loadContexts(m_pcSbacCoder);
//            }
//            CTXMem[0]->loadContexts(m_pcSbacCoder);
//        }
//    }
//    // for every CU in slice
//    UInt uiEncCUOrder;
//    for( uiEncCUOrder = uiStartCUAddr/rpcPic->getNumPartInCU();
//        uiEncCUOrder < (uiBoundingCUAddr+(rpcPic->getNumPartInCU()-1))/rpcPic->getNumPartInCU();
//        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
//    {
//
////        cout << "Hello for every CU" << endl;
//        /// XXX problem is here: the things are null
//        // initialize CU encoder
//        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
//
////        cout << "My problem is in the init of the pcCU BEFORE " << endl;
//        pcCU->initCU( rpcPic, uiCUAddr );
//
////        cout << "My problem is in the init of the pcCU " << endl;
//
//        // inherit from TR if necessary, select substream to use.
//        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
//        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
//        UInt uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
//        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
//        UInt uiCol     = uiCUAddr % uiWidthInLCUs;
//        UInt uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//
//        if ( ((iNumSubstreams > 1) || depSliceSegmentsEnabled ) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
//        {
//            // We'll sync if the TR is available.
//            TComDataCU *pcCUUp = pcCU->getCUAbove();
//            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
//            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
//            TComDataCU *pcCUTR = NULL;
//            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
//            {
//                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
//            }
//            if ( ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
//                  (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
//                  ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
//                  )
//                )
//            {
//                // TR not available.
//            }
//            else
//            {
//                // TR is available, we use it.
//                ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
//            }
//        }
//        m_pppcRDSbacCoder[0][CI_CURR_BEST]->load( ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST] ); //this load is used to simplify the code
//
//        // reset the entropy coder
//        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
//           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
//           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
//           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
//        {
//            SliceType sliceType = pcSlice->getSliceType();
//            if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
//            {
//                sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
//            }
//            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp(), false );
//            m_pcEntropyCoder->setEntropyCoder     ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
//            m_pcEntropyCoder->updateContextTables ( sliceType, pcSlice->getSliceQp() );
//            m_pcEntropyCoder->setEntropyCoder     ( m_pcSbacCoder, pcSlice );
//        }
//
//        // set go-on entropy coder
//        m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder, pcSlice );
//        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
//
//        ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);
//
//        Double oldLambda = m_pcRdCost->getLambda();
//        if ( m_pcCfg->getUseRateCtrl() )
//        {
//            Int estQP        = pcSlice->getSliceQp();
//            Double estLambda = -1.0;
//            Double bpp       = -1.0;
//
//            if ( ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )
//            {
//                estQP = pcSlice->getSliceQp();
//            }
//            else
//            {
//                bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());
//                if ( rpcPic->getSlice( 0 )->getSliceType() == I_SLICE)
//                {
//                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
//                }
//                else
//                {
//                    estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );
//                    estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
//                }
//
//                estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );
//
//                m_pcRdCost->setLambda(estLambda);
//
//#if RDOQ_CHROMA_LAMBDA
//                // set lambda for RDOQ
//                const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
//                const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
//                m_pcTrQuant->setLambdas( lambdaArray );
//#else
//                m_pcTrQuant->setLambda( estLambda );
//#endif
//            }
//
//            m_pcRateCtrl->setRCQP( estQP );
//#if ADAPTIVE_QP_SELECTION
//            pcCU->getSlice()->setSliceQpBase( estQP );
//#endif
//        }
//
//        // Motion Estimation
//        //      cout  << pcCU -> getSlice() -> getPOC() << ") Ruba: From the heart of Motion Estimation: " << pcCU->getSlice()->getNumRefIdx(REF_PIC_LIST_0) << endl;
//
////        cout << "Run CU encoder " << endl;
//        // run CU encoder
//        m_pcCuEncoder->compressCU( pcCU );
//
//        // restore entropy coder to an initial stage
//        m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST], pcSlice );
//        m_pcEntropyCoder->setBitstream( &pcBitCounters[uiSubStrm] );
//        m_pcCuEncoder->setBitCounter( &pcBitCounters[uiSubStrm] );
//        m_pcBitCounter = &pcBitCounters[uiSubStrm];
//        pppcRDSbacCoder->setBinCountingEnableFlag( true );
//        m_pcBitCounter->resetBits();
//        pppcRDSbacCoder->setBinsCoded( 0 );
//        m_pcCuEncoder->encodeCU( pcCU );
//
//        pppcRDSbacCoder->setBinCountingEnableFlag( false );
//        if (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES && ( ( pcSlice->getSliceBits() + m_pcEntropyCoder->getNumberOfWrittenBits() ) ) > m_pcCfg->getSliceArgument()<<3)
//        {
//            pcSlice->setNextSlice( true );
//            break;
//        }
//        if (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+m_pcEntropyCoder->getNumberOfWrittenBits() > (m_pcCfg->getSliceSegmentArgument() << 3) &&pcSlice->getSliceCurEndCUAddr()!=pcSlice->getSliceSegmentCurEndCUAddr())
//        {
//            pcSlice->setNextSliceSegment( true );
//            break;
//        }
//
//        ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
//
//        //Store probabilties of second LCU in line into buffer
//        if ( ( uiCol == uiTileLCUX+1) && (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && m_pcCfg->getWaveFrontsynchro())
//        {
//            m_pcBufferSbacCoders[uiTileCol].loadContexts(ppppcRDSbacCoders[uiSubStrm][0][CI_CURR_BEST]);
//        }
//
//        if ( m_pcCfg->getUseRateCtrl() )
//        {
//            Int actualQP        = g_RCInvalidQPValue;
//            Double actualLambda = m_pcRdCost->getLambda();
//            Int actualBits      = pcCU->getTotalBits();
//            Int numberOfEffectivePixels    = 0;
//            for ( Int idx = 0; idx < rpcPic->getNumPartInCU(); idx++ )
//            {
//                if ( pcCU->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pcCU->isSkipped( idx ) ) )
//                {
//                    numberOfEffectivePixels = numberOfEffectivePixels + 16;
//                    break;
//                }
//            }
//
//            if ( numberOfEffectivePixels == 0 )
//            {
//                actualQP = g_RCInvalidQPValue;
//            }
//            else
//            {
//                actualQP = pcCU->getQP( 0 );
//            }
//            m_pcRdCost->setLambda(oldLambda);
//            m_pcRateCtrl->getRCPic()->updateAfterLCU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
//                                                     pcCU->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );
//        }
//
//        m_uiPicTotalBits += pcCU->getTotalBits();
//        m_dPicRdCost     += pcCU->getTotalCost();
//        m_uiPicDist      += pcCU->getTotalDistortion();
//    }
//    if ((iNumSubstreams > 1) && !depSliceSegmentsEnabled)
//    {
//        pcSlice->setNextSlice( true );
//    }
//    if(m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_BYTES || m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES)
//    {
//        if(pcSlice->getSliceCurEndCUAddr()<=pcSlice->getSliceSegmentCurEndCUAddr())
//        {
//            pcSlice->setNextSlice( true );
//        }
//        else
//        {
//            pcSlice->setNextSliceSegment( true );
//        }
//    }
//    if( depSliceSegmentsEnabled )
//    {
//        if (m_pcCfg->getWaveFrontsynchro())
//        {
//            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
//        }
//        CTXMem[0]->loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice
//    }
//    xRestoreWPparam( pcSlice );
//}
//
///**
// \param  rpcPic        picture class
// \retval rpcBitstream  bitstream class
// */
//Void TEncSlice::encodeSlice   ( TComPic*& rpcPic, TComOutputBitstream* pcSubstreams )
//{
//    UInt       uiCUAddr;
//    UInt       uiStartCUAddr;
//    UInt       uiBoundingCUAddr;
//    TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
//
//    uiStartCUAddr=pcSlice->getSliceSegmentCurStartCUAddr();
//    uiBoundingCUAddr=pcSlice->getSliceSegmentCurEndCUAddr();
//    // choose entropy coder
//    {
//        m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
//        m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
//    }
//
//    m_pcCuEncoder->setBitCounter( NULL );
//    m_pcBitCounter = NULL;
//    // Appropriate substream bitstream is switched later.
//    // for every CU
//#if ENC_DEC_TRACE
//    g_bJustDoIt = g_bEncDecTraceEnable;
//#endif
//    DTRACE_CABAC_VL( g_nSymbolCounter++ );
//    DTRACE_CABAC_T( "\tPOC: " );
//    DTRACE_CABAC_V( rpcPic->getPOC() );
//    DTRACE_CABAC_T( "\n" );
//#if ENC_DEC_TRACE
//    g_bJustDoIt = g_bEncDecTraceDisable;
//#endif
//
//    TEncTop* pcEncTop = (TEncTop*) m_pcCfg;
//    TEncSbac* pcSbacCoders = pcEncTop->getSbacCoders(); //coder for each substream
//    const Int iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
//    UInt uiBitsOriginallyInSubstreams = 0;
//    {
//        UInt uiTilesAcross = rpcPic->getPicSym()->getNumColumnsMinus1()+1;
//        for (UInt ui = 0; ui < uiTilesAcross; ui++)
//        {
//            m_pcBufferSbacCoders[ui].load(m_pcSbacCoder); //init. state
//        }
//
//        for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
//        {
//            uiBitsOriginallyInSubstreams += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
//        }
//
//        for (UInt ui = 0; ui < uiTilesAcross; ui++)
//        {
//            m_pcBufferLowLatSbacCoders[ui].load(m_pcSbacCoder);  //init. state
//        }
//    }
//
//    UInt uiWidthInLCUs  = rpcPic->getPicSym()->getFrameWidthInCU();
//    UInt uiCol=0, uiSubStrm=0;
//    UInt uiTileCol      = 0;
//    UInt uiTileStartLCU = 0;
//    UInt uiTileLCUX     = 0;
//    Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
//    uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());  /* for tiles, uiStartCUAddr is NOT the real raster scan address, it is actually
//                                                                                               an encoding order index, so we need to convert the index (uiStartCUAddr)
//                                                                                               into the real raster scan address (uiCUAddr) via the CUOrderMap */
//    UInt currentTileIdx=rpcPic->getPicSym()->getTileIdxMap(uiCUAddr);
//    TComTile *pCurrentTile=rpcPic->getPicSym()->getTComTile(currentTileIdx);
//    uiTileStartLCU = pCurrentTile->getFirstCUAddr();
//    if( depSliceSegmentsEnabled )
//    {
//        if( pcSlice->isNextSlice()|| uiCUAddr == uiTileStartLCU)
//        {
//            if(m_pcCfg->getWaveFrontsynchro())
//            {
//                CTXMem[1]->loadContexts(m_pcSbacCoder);
//            }
//            CTXMem[0]->loadContexts(m_pcSbacCoder);
//        }
//        else
//        {
//            if( m_pcCfg->getWaveFrontsynchro() )
//            {
//                uiTileCol = currentTileIdx % (rpcPic->getPicSym()->getNumColumnsMinus1()+1);
//                m_pcBufferSbacCoders[uiTileCol].loadContexts( CTXMem[1] );
//                uiCUAddr = rpcPic->getPicSym()->getCUOrderMap( uiStartCUAddr /rpcPic->getNumPartInCU());
//                uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//                if ( pCurrentTile->getTileWidth() < 2)
//                {
//                    CTXMem[0]->loadContexts(m_pcSbacCoder);
//                }
//            }
//            pcSbacCoders[uiSubStrm].loadContexts( CTXMem[0] );
//        }
//    }
//
//    UInt uiEncCUOrder;
//    for( uiEncCUOrder = uiStartCUAddr /rpcPic->getNumPartInCU();
//        uiEncCUOrder < (uiBoundingCUAddr+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU();
//        uiCUAddr = rpcPic->getPicSym()->getCUOrderMap(++uiEncCUOrder) )
//    {
//        uiTileCol = rpcPic->getPicSym()->getTileIdxMap(uiCUAddr) % (rpcPic->getPicSym()->getNumColumnsMinus1()+1); // what column of tiles are we in?
//        uiTileStartLCU = rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr();
//        uiTileLCUX = uiTileStartLCU % uiWidthInLCUs;
//        //UInt uiSliceStartLCU = pcSlice->getSliceCurStartCUAddr();
//        uiCol     = uiCUAddr % uiWidthInLCUs;
//        uiSubStrm=rpcPic->getSubstreamForLCUAddr(uiCUAddr, true, pcSlice);
//
//        m_pcEntropyCoder->setBitstream( &pcSubstreams[uiSubStrm] );
//        // Synchronize cabac probabilities with upper-right LCU if it's available and we're at the start of a line.
//        if (((iNumSubstreams > 1) || depSliceSegmentsEnabled) && (uiCol == uiTileLCUX) && m_pcCfg->getWaveFrontsynchro())
//        {
//            // We'll sync if the TR is available.
//            TComDataCU *pcCUUp = rpcPic->getCU( uiCUAddr )->getCUAbove();
//            UInt uiWidthInCU = rpcPic->getFrameWidthInCU();
//            UInt uiMaxParts = 1<<(pcSlice->getSPS()->getMaxCUDepth()<<1);
//            TComDataCU *pcCUTR = NULL;
//            if ( pcCUUp && ((uiCUAddr%uiWidthInCU+1) < uiWidthInCU)  )
//            {
//                pcCUTR = rpcPic->getCU( uiCUAddr - uiWidthInCU + 1 );
//            }
//            if ( (true/*bEnforceSliceRestriction*/ &&
//                  ((pcCUTR==NULL) || (pcCUTR->getSlice()==NULL) ||
//                   (pcCUTR->getSCUAddr()+uiMaxParts-1 < pcSlice->getSliceCurStartCUAddr()) ||
//                   ((rpcPic->getPicSym()->getTileIdxMap( pcCUTR->getAddr() ) != rpcPic->getPicSym()->getTileIdxMap(uiCUAddr)))
//                   ))
//                )
//            {
//                // TR not available.
//            }
//            else
//            {
//                // TR is available, we use it.
//                pcSbacCoders[uiSubStrm].loadContexts( &m_pcBufferSbacCoders[uiTileCol] );
//            }
//        }
//        m_pcSbacCoder->load(&pcSbacCoders[uiSubStrm]);  //this load is used to simplify the code (avoid to change all the call to m_pcSbacCoder)
//
//        // reset the entropy coder
//        if( uiCUAddr == rpcPic->getPicSym()->getTComTile(rpcPic->getPicSym()->getTileIdxMap(uiCUAddr))->getFirstCUAddr() &&                                   // must be first CU of tile
//           uiCUAddr!=0 &&                                                                                                                                    // cannot be first CU of picture
//           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceSegmentCurStartCUAddr())/rpcPic->getNumPartInCU() &&
//           uiCUAddr!=rpcPic->getPicSym()->getPicSCUAddr(rpcPic->getSlice(rpcPic->getCurrSliceIdx())->getSliceCurStartCUAddr())/rpcPic->getNumPartInCU())     // cannot be first CU of slice
//        {
//            // We're crossing into another tile, tiles are independent.
//            // When tiles are independent, we have "substreams per tile".  Each substream has already been terminated, and we no longer
//            // have to perform it here.
//            if (iNumSubstreams <= 1)
//            {
//                SliceType sliceType  = pcSlice->getSliceType();
//                if (!pcSlice->isIntra() && pcSlice->getPPS()->getCabacInitPresentFlag() && pcSlice->getPPS()->getEncCABACTableIdx()!=I_SLICE)
//                {
//                    sliceType = (SliceType) pcSlice->getPPS()->getEncCABACTableIdx();
//                }
//                m_pcEntropyCoder->updateContextTables( sliceType, pcSlice->getSliceQp() );
//
//                // Byte-alignment in slice_data() when new tile
//                pcSubstreams[uiSubStrm].writeByteAlignment();
//            }
//
//            {
//                UInt numStartCodeEmulations = pcSubstreams[uiSubStrm].countStartCodeEmulations();
//                UInt uiAccumulatedSubstreamLength = 0;
//                for (Int iSubstrmIdx=0; iSubstrmIdx < iNumSubstreams; iSubstrmIdx++)
//                {
//                    uiAccumulatedSubstreamLength += pcSubstreams[iSubstrmIdx].getNumberOfWrittenBits();
//                }
//                // add bits coded in previous dependent slices + bits coded so far
//                // add number of emulation prevention byte count in the tile
//                pcSlice->addTileLocation( ((pcSlice->getTileOffstForMultES() + uiAccumulatedSubstreamLength - uiBitsOriginallyInSubstreams) >> 3) + numStartCodeEmulations );
//            }
//        }
//
//        TComDataCU*& pcCU = rpcPic->getCU( uiCUAddr );
//
//        if ( pcSlice->getSPS()->getUseSAO() )
//        {
//            Bool bIsSAOSliceEnabled = false;
//            Bool sliceEnabled[MAX_NUM_COMPONENT];
//            for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
//            {
//                ComponentID compId=ComponentID(comp);
//                sliceEnabled[compId] = pcSlice->getSaoEnabledFlag(toChannelType(compId)) && (comp < rpcPic->getNumberValidComponents());
//                if (sliceEnabled[compId]) bIsSAOSliceEnabled=true;
//            }
//            if (bIsSAOSliceEnabled)
//            {
//                SAOBlkParam& saoblkParam = (rpcPic->getPicSym()->getSAOBlkParam())[uiCUAddr];
//
//                Bool leftMergeAvail = false;
//                Bool aboveMergeAvail= false;
//                //merge left condition
//                Int rx = (uiCUAddr % uiWidthInLCUs);
//                if(rx > 0)
//                {
//                    leftMergeAvail = rpcPic->getSAOMergeAvailability(uiCUAddr, uiCUAddr-1);
//                }
//
//                //merge up condition
//                Int ry = (uiCUAddr / uiWidthInLCUs);
//                if(ry > 0)
//                {
//                    aboveMergeAvail = rpcPic->getSAOMergeAvailability(uiCUAddr, uiCUAddr-uiWidthInLCUs);
//                }
//
//                m_pcEntropyCoder->encodeSAOBlkParam(saoblkParam, sliceEnabled, leftMergeAvail, aboveMergeAvail);
//            }
//        }
//
//#if ENC_DEC_TRACE
//        g_bJustDoIt = g_bEncDecTraceEnable;
//#endif
//        if ( (m_pcCfg->getSliceMode()!=0 || m_pcCfg->getSliceSegmentMode()!=0) &&
//            uiCUAddr == rpcPic->getPicSym()->getCUOrderMap((uiBoundingCUAddr+rpcPic->getNumPartInCU()-1)/rpcPic->getNumPartInCU()-1) )
//        {
//            m_pcCuEncoder->encodeCU( pcCU );
//        }
//        else
//        {
//            m_pcCuEncoder->encodeCU( pcCU );
//        }
//#if ENC_DEC_TRACE
//        g_bJustDoIt = g_bEncDecTraceDisable;
//#endif
//        pcSbacCoders[uiSubStrm].load(m_pcSbacCoder);   //load back status of the entropy coder after encoding the LCU into relevant bitstream entropy coder
//
//        //Store probabilties of second LCU in line into buffer
//        if ( (depSliceSegmentsEnabled || (iNumSubstreams > 1)) && (uiCol == uiTileLCUX+1) && m_pcCfg->getWaveFrontsynchro())
//        {
//            m_pcBufferSbacCoders[uiTileCol].loadContexts( &pcSbacCoders[uiSubStrm] );
//        }
//    }
//    if( depSliceSegmentsEnabled )
//    {
//        if (m_pcCfg->getWaveFrontsynchro())
//        {
//            CTXMem[1]->loadContexts( &m_pcBufferSbacCoders[uiTileCol] );//ctx 2.LCU
//        }
//        CTXMem[0]->loadContexts( m_pcSbacCoder );//ctx end of dep.slice
//    }
//#if ADAPTIVE_QP_SELECTION
//    if( m_pcCfg->getUseAdaptQpSelect() )
//    {
//        m_pcTrQuant->storeSliceQpNext(pcSlice);
//    }
//#endif
//    if (pcSlice->getPPS()->getCabacInitPresentFlag())
//    {
//        if (pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
//        {
//            pcSlice->getPPS()->setEncCABACTableIdx( pcSlice->getSliceType() );
//        }
//        else
//        {
//            m_pcEntropyCoder->determineCabacInitIdx();
//        }
//    }
//}
//
//Void TEncSlice::calculateBoundingCUAddrForSlice(UInt &uiStartCUAddrSlice, UInt &uiBoundingCUAddrSlice, Bool &bReachedTileBoundary, TComPic*& rpcPic, Bool bEncodeSlice, Int sliceMode, Int sliceArgument, UInt uiSliceCurEndCUAddr)
//{
//    TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
//    UInt uiNumberOfCUsInFrame = rpcPic->getNumCUsInFrame();
//    const UInt scaleCUAddr = rpcPic->getNumPartInCU(); // due to fine granularity slices all addresses are scaled.
//    uiBoundingCUAddrSlice=0;
//    bReachedTileBoundary=false;
//
//    switch (sliceMode)
//    {
//        case FIXED_NUMBER_OF_LCU:
//        {
//            UInt uiCUAddrIncrement    = sliceArgument;
//            uiBoundingCUAddrSlice     = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*scaleCUAddr) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*scaleCUAddr;
//        }
//            break;
//        case FIXED_NUMBER_OF_BYTES:
//            if (bEncodeSlice)
//                uiBoundingCUAddrSlice     = uiSliceCurEndCUAddr;
//            else
//                uiBoundingCUAddrSlice     = uiNumberOfCUsInFrame*scaleCUAddr;
//            break;
//        case FIXED_NUMBER_OF_TILES:
//        {
//            UInt tileIdx              = rpcPic->getPicSym()->getTileIdxMap( rpcPic->getPicSym()->getCUOrderMap(uiStartCUAddrSlice/scaleCUAddr) );
//            UInt uiCUAddrIncrement    = 0;
//            UInt tileTotalCount       = (rpcPic->getPicSym()->getNumColumnsMinus1()+1) * (rpcPic->getPicSym()->getNumRowsMinus1()+1);
//
//            for(UInt tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)
//            {
//                if((tileIdx + tileIdxIncrement) < tileTotalCount)
//                {
//                    UInt tileWidthInLcu   = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidth();
//                    UInt tileHeightInLcu  = rpcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeight();
//                    uiCUAddrIncrement    += (tileWidthInLcu * tileHeightInLcu * scaleCUAddr);
//                }
//            }
//
//            uiBoundingCUAddrSlice     = ((uiStartCUAddrSlice + uiCUAddrIncrement) < uiNumberOfCUsInFrame*scaleCUAddr) ? (uiStartCUAddrSlice + uiCUAddrIncrement) : uiNumberOfCUsInFrame*scaleCUAddr;
//        }
//            break;
//        default:
//            uiBoundingCUAddrSlice       = uiNumberOfCUsInFrame*scaleCUAddr;
//            break;
//    }
//
//    // Adjust for tiles and wavefronts.
//    if ((sliceMode == FIXED_NUMBER_OF_LCU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
//        (m_pcCfg->getNumRowsMinus1() > 0 || m_pcCfg->getNumColumnsMinus1() > 0))
//    {
//        const UInt lcuEncAddrStart = (uiStartCUAddrSlice+scaleCUAddr-1)/scaleCUAddr;
//        const UInt lcuAddr = rpcPic->getPicSym()->getCUOrderMap(lcuEncAddrStart);
//        const UInt startTileIdx = rpcPic->getPicSym()->getTileIdxMap(lcuAddr);
//        const Bool bWavefrontsEnabled = m_pcCfg->getWaveFrontsynchro();
//
//        TComTile *pStartingTile = rpcPic->getPicSym()->getTComTile(startTileIdx);
//        const UInt uiTileStartLCUEncAddr      = rpcPic->getPicSym()->getInverseCUOrderMap(pStartingTile->getFirstCUAddr());
//        const UInt uiTileStartWidth           = pStartingTile->getTileWidth();
//        const UInt uiTileStartHeight          = pStartingTile->getTileHeight();
//        const UInt uiTileLastLCUEncAddr_excl  = uiTileStartLCUEncAddr + uiTileStartWidth*uiTileStartHeight;
//        const UInt tileBoundingCUAddrSlice    = uiTileLastLCUEncAddr_excl * scaleCUAddr;
//
//        const UInt lcuColumnOfStartingTile=((lcuEncAddrStart-uiTileStartLCUEncAddr)%uiTileStartWidth);
//        if (bWavefrontsEnabled && lcuColumnOfStartingTile!=0)
//        {
//            // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
//            const UInt numberOfLCUsToEndOfRow=uiTileStartWidth-lcuColumnOfStartingTile;
//            const UInt wavefrontTileBoundingCUAddrSlice = (lcuEncAddrStart+numberOfLCUsToEndOfRow)*scaleCUAddr;
//            if (wavefrontTileBoundingCUAddrSlice < uiBoundingCUAddrSlice)
//            {
//                uiBoundingCUAddrSlice = wavefrontTileBoundingCUAddrSlice;
//            }
//        }
//
//        if (tileBoundingCUAddrSlice < uiBoundingCUAddrSlice)
//        {
//            uiBoundingCUAddrSlice = tileBoundingCUAddrSlice;
//            bReachedTileBoundary = true;
//        }
//    }
//    else if (pcSlice->getPPS()->getNumSubstreams() > 1 && (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*scaleCUAddr) != 0))
//    {
//        // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
//        uiBoundingCUAddrSlice = min(uiBoundingCUAddrSlice, uiStartCUAddrSlice - (uiStartCUAddrSlice % (rpcPic->getFrameWidthInCU()*scaleCUAddr)) + (rpcPic->getFrameWidthInCU()*scaleCUAddr));
//    }
//
//    //calculate real slice start address (fine granularity slices)
//    {
//        UInt uiInternalAddress = rpcPic->getPicSym()->getPicSCUAddr(uiStartCUAddrSlice) % scaleCUAddr;
//        UInt uiExternalAddress = rpcPic->getPicSym()->getPicSCUAddr(uiStartCUAddrSlice) / scaleCUAddr;
//        UInt uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
//        UInt uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
//        UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
//        UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
//        while((uiPosX>=uiWidth||uiPosY>=uiHeight)&&!(uiPosX>=uiWidth&&uiPosY>=uiHeight))
//        {
//            uiInternalAddress++;
//            if(uiInternalAddress>=scaleCUAddr)
//            {
//                uiInternalAddress=0;
//                uiExternalAddress = rpcPic->getPicSym()->getCUOrderMap(rpcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
//            }
//            uiPosX = ( uiExternalAddress % rpcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
//            uiPosY = ( uiExternalAddress / rpcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
//        }
//        UInt uiRealStartAddress = rpcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*scaleCUAddr+uiInternalAddress);
//
//        uiStartCUAddrSlice=uiRealStartAddress;
//    }
//}
//
///** Determines the starting and bounding LCU address of current slice / dependent slice
// * \param bEncodeSlice Identifies if the calling function is compressSlice() [false] or encodeSlice() [true]
// * \returns Updates uiStartCUAddr, uiBoundingCUAddr with appropriate LCU address
// */
//Void TEncSlice::xDetermineStartAndBoundingCUAddr  ( UInt& startCUAddr, UInt& boundingCUAddr, TComPic*& rpcPic, Bool bEncodeSlice )
//{
//    TComSlice* pcSlice = rpcPic->getSlice(getSliceIdx());
//
//    // Non-dependent slice
//    UInt uiStartCUAddrSlice   = pcSlice->getSliceCurStartCUAddr();
//    Bool bTileBoundarySlice   = false;
//    UInt uiBoundingCUAddrSlice;
//    calculateBoundingCUAddrForSlice(uiStartCUAddrSlice, uiBoundingCUAddrSlice, bTileBoundarySlice, rpcPic, bEncodeSlice, m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument(), pcSlice->getSliceCurEndCUAddr());
//    pcSlice->setSliceCurEndCUAddr( uiBoundingCUAddrSlice );
//    pcSlice->setSliceCurStartCUAddr(uiStartCUAddrSlice);
//
//    // Dependent slice
//    UInt startCUAddrSliceSegment    = pcSlice->getSliceSegmentCurStartCUAddr();
//    Bool bTileBoundarySliceSegment  = false;
//    UInt boundingCUAddrSliceSegment;
//    calculateBoundingCUAddrForSlice(startCUAddrSliceSegment, boundingCUAddrSliceSegment, bTileBoundarySliceSegment, rpcPic, bEncodeSlice, m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument(), pcSlice->getSliceSegmentCurEndCUAddr());
//    pcSlice->setSliceSegmentCurEndCUAddr( boundingCUAddrSliceSegment );
//    pcSlice->setSliceSegmentCurStartCUAddr(startCUAddrSliceSegment);
//
//    // Make a joint decision based on reconstruction and dependent slice bounds
//    startCUAddr    = max(uiStartCUAddrSlice   , startCUAddrSliceSegment   );
//    boundingCUAddr = min(uiBoundingCUAddrSlice, boundingCUAddrSliceSegment);
//
//
//    if (!bEncodeSlice)
//    {
//        // For fixed number of LCU within an entropy and reconstruction slice we already know whether we will encounter end of entropy and/or reconstruction slice
//        // first. Set the flags accordingly.
//        if ( (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
//            || (m_pcCfg->getSliceMode()==0 && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
//            || (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU && m_pcCfg->getSliceSegmentMode()==0)
//            || (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU)
//            || (m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceSegmentMode()==0)
//            || (m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_TILES && m_pcCfg->getSliceMode()==0)
//            || bTileBoundarySlice || bTileBoundarySliceSegment )
//        {
//            if (uiBoundingCUAddrSlice < boundingCUAddrSliceSegment)
//            {
//                pcSlice->setNextSlice       ( true );
//                pcSlice->setNextSliceSegment( false );
//            }
//            else if (uiBoundingCUAddrSlice > boundingCUAddrSliceSegment)
//            {
//                pcSlice->setNextSlice       ( false );
//                pcSlice->setNextSliceSegment( true );
//            }
//            else
//            {
//                pcSlice->setNextSlice       ( true );
//                pcSlice->setNextSliceSegment( true );
//            }
//        }
//        else
//        {
//            pcSlice->setNextSlice       ( false );
//            pcSlice->setNextSliceSegment( false );
//        }
//    }
//}
//
//Double TEncSlice::xGetQPValueAccordingToLambda ( Double lambda )
//{
//    return 4.2005*log(lambda) + 13.7122;
//}
//
////! \}


///////////////////////////////////////////////////////////////

// |*************************************************************|
// | Case 1: 0.29 dB in KB, you should keep it for the reference |
// | VERY IMPORTANT CASE ONEEEEEEEE                              |
// |*************************************************************|

// Cathy
//static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};
//static int Qpoff[] ={3, 2, 3, 1};
//static int depths[] ={3, 2, 3, 1, 0};
//static int track = 0;
//static bool isSc = false;
//static float lambdaArr[]  = {20.1964, 14.7968, 20.1964, 7.34014, 4.88345};
//static float lambdaArr2[] = {88.7808, 66.5506, 88.7808, 23.3035, 15.504};
//static float lambdaArr3[] = {360.156, 273.428, 360.156, 73.984, 49.2221};
//static float lambdaArr4[] = {1193.14, 946.995, 1193.14, 234.885, 156.271};
//static float lambdaArr5[] = {3787.98, 3006.52, 3787.98, 745.712, 496.128};
//
//static int trackLambda = 0;
//static Int comingFromSmooth = 0;// 0, 1, 2
//
////Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc )
//
//Void TEncSlice::initEncSliceNew( TComPic* pcPic, Int pocLast, Int pocCurr, Int iNumPicRcvd, Int iGOPid, TComSlice*& rpcSlice, TComSPS* pSPS, TComPPS *pPPS, Bool isField, Bool isSceneChange, Int lastSc, Int scState, Bool isSmooth )
//{
//    Double dQP;
//    Double dLambda;
//    
//    // Hossam: Scene change
//    Double lambdaHelper = 0;
//    
//    rpcSlice = pcPic->getSlice(0);
//    rpcSlice->setSPS( pSPS );
//    rpcSlice->setPPS( pPPS );
//    rpcSlice->setSliceBits(0);
//    rpcSlice->setPic( pcPic );
//    rpcSlice->initSlice();
//    rpcSlice->setPicOutputFlag( true );
//    rpcSlice->setPOC( pocCurr );
//    
//    // depth computation based on GOP size
//    Int depth;
//    {
//#if FIX_FIELD_DEPTH
//        Int poc = rpcSlice->getPOC();
//        if(isField)
//        {
//            poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
//        }
//        else
//        {
//            poc = poc % m_pcCfg->getGOPSize();
//        }
//#else
//        Int poc = rpcSlice->getPOC()%m_pcCfg->getGOPSize();
//#endif
//        
//        if ( poc == 0 )
//        {
//            depth = 0;
//        }
//        else
//        {
//            Int step = m_pcCfg->getGOPSize();
//            depth    = 0;
//            for( Int i=step>>1; i>=1; i>>=1 )
//            {
//                for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
//                {
//                    if ( j == poc )
//                    {
//                        i=0;
//                        break;
//                    }
//                }
//                step >>= 1;
//                depth++;
//            }
//        }
//        
//#if FIX_FIELD_DEPTH
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        if(poc != 0)
//        {
//#endif
//            if (isField && ((rpcSlice->getPOC() % 2) == 1))
//            {
//                depth ++;
//            }
//#if HARMONIZE_GOP_FIRST_FIELD_COUPLE
//        }
//#endif
//#endif
//    }
//    
//    //    // Hossam: Scene Change Depth modification
//    //    if(isSceneChange)
//    //    {
//    //        depth = 0;
//    //    }
//    //    // Hossam: Scene change to make it higher lambda for non smooth frames (20.xx)
//    //    if (pocCurr == lastSc + 1) {
//    //        if (!isSmooth) {
//    //            depth = 1;
//    //        }
//    //    }
//    
//    
//    // slice type
//    SliceType eSliceType;
//    
//    eSliceType=B_SLICE;
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif
//    
//    rpcSlice->setSliceType    ( eSliceType );
//    
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // Non-referenced frame marking
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    if(pocLast == 0)
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(false);
//    }
//    else
//    {
//        rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
//    }
//    rpcSlice->setReferenced(true);
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // QP setting
//    // ------------------------------------------------------------------------------------------------------------------
//    // ------------------------------------------------------------------------------------------------------------------
//    // Depth ADJUSMENT
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    
//    //    cout << "Indie QP modifyyyyy  "  << boolalpha << isSceneChange<< endl;
//    if (isSceneChange) {
//        
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//        //                m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//        //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
//        //            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+2);
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+0);
//        //        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+2);
//        //        lambdaHelper = getLambda(1);
//        
//        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//        m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
//        depth = depths[3];
//        
//        lambdaHelper = getLambda(3);
//        
//    }
//    else
//    {
//        
//        if( (rpcSlice->getPOC() != 0) && lastSc != -1)
//        {
//            //            if(scState == 4)
//            if(rpcSlice->getPOC() == lastSc + 1)
//            {
//                if(isSmooth)
//                {
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    
//                    switch (m_pcCfg->getQP()) {
//                        case 22:
//                            //                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(SC_QP_OFFSET);
//                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-2);
//                            break;
//                        case 27:
//                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-2);
//                            break;
//                        case 32:
//                        case 37:
//                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-4);
//                            break;
//                        case 42:
//                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(-6);
//                            break;
//                        default:
//                            m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(0);
//                            break;
//                    }
//                    
//                    lambdaHelper = getLambda(4);
//                    depth = depths[3];
//                    
//                    //                    // Reset the track
//                    //                    track = 0;
//                    
//                    // Smooth
//                    comingFromSmooth = 1;
//                    
//                }
//                else
//                {
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[2]);
//                    
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
//                    //                    lambdaHelper = getLambda(3);
//                    
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[3]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+3);
//                    //                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(+1);
//                    depth = depths[0];
//                    
//                    //                    // Reset the track
//                    //                    track = 1;
//                    
//                    // Not Smooth
//                    comingFromSmooth = 2;
//                }
//                
//                // Reset the track lambda
//                trackLambda = 0;
//                
//                //                // Reset the track
//                track = 0;
//                
//                // Force the I frame
//                //                eSliceType = I_SLICE;
//                //                rpcSlice->setSliceType(I_SLICE);
//                
//            }// end if(scState==4)
//            else
//            {
//                
//                //                cout << "COMING FROM SMOOTH: " << comingFromSmooth << endl;
//                if (comingFromSmooth == 2) {
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[1]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[1]);
//                    depth = depths[1];
//                    
//                }
//                else{
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPFac(QpFact[track]);
//                    m_pcCfg -> getGOPEntryNew(iGOPid) -> setQPOffset(Qpoff[track]);
//                    depth = depths[track];
//                    
//                }
//                lambdaHelper = getLambda(trackLambda);
//                
//                track = (track+1)%m_pcCfg->getGOPSize(); trackLambda = (trackLambda+1) % m_pcCfg->getGOPSize();
//                
//            }// end normal case
//            
//        }// end entering the normal case
//        
//    }// end adjusting the QP and lambda
//    
//    
//    
//    dQP = m_pcCfg->getQP();
//    
//    
//    
//    if(eSliceType!=I_SLICE)
//    {
//        if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))
//        {
//            dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//        }
//    }
//    
//    // modify QP
//    // Hossam: Scene Change
//    Int* pdQPs = m_pcCfg->getdQPs();
//    if ( pdQPs)
//    {
//        dQP += pdQPs[ rpcSlice->getPOC() ];
//    }
//    
//    if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
//    {
//        dQP=RExt__LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
//        m_pcCfg->setDeltaQpRD(0);
//    }
//    
//    
//    // Hossam: Scene Change
//    if(isSceneChange || rpcSlice->getPOC() == lastSc + 1)
//    {
//        dQP = m_pcCfg->getQP() + m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
//    }
//    
//    // ------------------------------------------------------------------------------------------------------------------
//    // Lambda computation
//    // ------------------------------------------------------------------------------------------------------------------
//    
//    //     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    
//    Int iQP;
//    Double dOrigQP = dQP;
//    
//    // pre-compute lambda and QP values for all possible QP candidates
//    for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
//    {
//        // compute QP value
//        dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
//        
//        // compute lambda value
//        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
//        Int    SHIFT_QP = 12;
//        
//        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );
//        
//        //         cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda_scale " << dLambda_scale << endl;
//        
//#if FULL_NBIT
//        Int    bitdepth_luma_qp_scale = 6 * (g_bitDepth[CHANNEL_TYPE_LUMA] - 8);
//#else
//        Int    bitdepth_luma_qp_scale = 0;
//#endif
//        Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
//#if FULL_NBIT
//        Double qp_temp_orig = (Double) dQP - SHIFT_QP;
//#endif
//        
//        //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp " << qp_temp << endl;
//        
//        // Hossam: Computing eSliceType I or P or B
//        // Case #1: I or P-slices (key-frame)
//        Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
//        if ( eSliceType==I_SLICE )
//            //        if ( eSliceType==I_SLICE || (pocCurr == lastSc + 1 && isSmooth))
//        {
//            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQpFactor: " << dQPFactor << "\n" << endl;
//            //            cout << "\nCathy: TEncSlice: I FRAMEEEEEEeEEEEEEEE dQp: " << dQP << "\n" << endl;
//            
//            dQPFactor=0.57*dLambda_scale;
//        }
//        
//        cout << "\nPARAMS OF THE EQUATN: " << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dQPFactor2222 " << dQPFactor << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH qp_temp2222 " << qp_temp << endl;
//        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH depthhh2222 " << depth << endl;
//        
//        dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
//        
//        //        cout << "\nCathy: TEncSlice:  FRAMEEEEEEeEEEEEEEE dQp: " << dQP << ", QpFact: " << dQPFactor << ", dLambda: " << dLambda << "\n" << endl;
//        cout << "\nCathy: TEncSlice: AFTER PARAMS LAMDA WITH dLambda " << dLambda << "\n" << endl;
//        //        cout << "m_pcCfg->getDeltaQpRD(): " << m_pcCfg->getDeltaQpRD() << "\n" << endl;// 0
//        
//        //      getchar();
//        
//        
//        if ( depth>0 )
//        {
//            
//            
//#if FULL_NBIT
//            
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#else
//            //            cout << depth << " INSIDE IF STATEMENT: "  << ", qp_temp: " << qp_temp << endl;
//            dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
//#endif
//        }
//        
//        //        cout << "Before  !m_pcCfg->getUseHADME() : " << dLambda << endl;
//        // if hadamard is used in ME process
//        if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
//        {
//            dLambda *= 0.95;
//        }
//        //        cout << "After  !m_pcCfg->getUseHADME() : " << dLambda << endl;
//        
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//        
//        m_pdRdPicLambda[iDQpIdx] = dLambda;
//        m_pdRdPicQp    [iDQpIdx] = dQP;
//        m_piRdPicQp    [iDQpIdx] = iQP;
//    }
//    
//    // obtain dQP = 0 case
//    dLambda = m_pdRdPicLambda[0];
//    dQP     = m_pdRdPicQp    [0];
//    iQP     = m_piRdPicQp    [0];
//    
//    //    cout << "Before  rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
//    if( rpcSlice->getSliceType( ) != I_SLICE )
//    {
//        dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );// equals 0
//    }
//    //    cout << "After  rpcSlice->getSliceType( ) != I_SLICE: " << dLambda << endl;
//    
//    //    // Hossam: Scene Change
//    //    if(rpcSlice->getPOC() == lastSc + 1)
//    //    {
//    //        if(isSmooth)
//    //        {
//    ////            cout << "QP ORGGGGGGGGGGG1111: " << m_pcCfg->getQP() << endl;
//    ////            iQP = m_pcCfg->getQP() + SC_QP_OFFSET;
//    ////            dQP = +1;
//    //            iQP = m_pcCfg->getQP() + SC_QP_OFFSET;
//    //            dQP = SC_QP_OFFSET;
//    //
//    //        }
//    //        else
//    //        {
//    ////            cout << "QP ORGGGGGGGGGGG2222: " << m_pcCfg->getQP() << endl;
//    //            iQP = m_pcCfg->getQP() + 1;
//    //            dQP = +1;
//    //        }
//    //
//    //    }
//    
//    cout << "\nCathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH dLambda " << dLambda  << endl;
//    //     cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH lambdaHelper " << lambdaHelper << endl;
//    //        cout << "Cathy: TEncSlice: ATTEMPT 2 ADJUSTTTTTT WITH QPFactor " << m_pcCfg->getGOPEntry(iGOPid).m_QPFactor  << endl;
//    
//    // Hossam: Scene Change: Ignore the dlamda calculation and take it from the table for now
//    //     dLambda = lambdaHelper; // New change: leave it calculated programitically!
//    
//    //    if(isSceneChange)
//    //        dLambda = min(dLambda, lambdaHelper); // New change: leave it calculated programitically!
//    //    else if (pocCurr == lastSc + 1) {
//    //        dLambda = isSmooth? min(dLambda, lambdaHelper): max(dLambda, lambdaHelper);
//    //    }
//    //    else
//    //        dLambda = lambdaHelper;
//    
//    setUpLambda(rpcSlice, dLambda, iQP);
//    
//    
//    //    cout << pocCurr << "Cathy: TEncSlice: ATTEMPT 2 MODIFYYY LAMDA WITH dLambda " << dLambda << "\n" << endl;
//    
//#if HB_LAMBDA_FOR_LDC
//    // restore original slice type
//    
//#if EFFICIENT_FIELD_IRAP
//    if(!(isField && pocLast == 1))
//    {
//#endif // EFFICIENT_FIELD_IRAP
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if(m_pcCfg->getDecodingRefreshType() == 3)
//        {
//            eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//        }
//        else
//        {
//#endif
//            eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        }
//#endif
//#if EFFICIENT_FIELD_IRAP
//    }
//#endif // EFFICIENT_FIELD_IRAP
//    
//    
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case B
//    rpcSlice->setSliceType        ( eSliceType );
//#endif
//    
//    //*************************************************************************************
//    // Hossam: Setting the slice type to the computed Slice type to the rpcSlice -- Case A
//    
//    // Hossam: Code Tweaking -- Second trial start
//    
//    Int sc = rpcSlice->getPOC();
//    Int n = 2; /// 3rd frame
//    n = 5;
//    
//    //    if(n == sc)
//    //    if(isSceneChange)
//    //    {
//    //        cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//    //        rpcSlice->setSliceType(I_SLICE);
//    //
//    //    }
//    
//    //    if(scState == 4)
//    
//    // Modify
//    //    if(rpcSlice->getPOC() == lastSc + 1)
//    //    {
//    //        cout << "Yang: TEncSlice: initEncSlice: Forcing an CONCEPTUAL I frame XXXX " << sc << "\n" << endl;
//    //        rpcSlice->setSliceType(I_SLICE);
//    //
//    //    }
//    
//    if(isSceneChange)
//    {
//        cout << "Yang: TEncSlice: initEncSlice: Forcing an CONCEPTUAL I frame XXXX " << sc << "\n" << endl;
//        rpcSlice->setSliceType(I_SLICE);
//        
//    }
//    
//    
//    // rpcSlice -> getSliceQp()
//    // rpcSlice -> setSliceQp(<#Int i#>)
//    //    cout << "\nYang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//    //    getchar();
//    //    if(sc == n)
//    // Sequence 1
//    /*
//     if(
//     sc == 13 ||
//     sc == 54 ||
//     sc == 110 ||
//     sc == 130 ||
//     sc == 165 ||
//     sc == 180 ||
//     sc == 213 ||
//     sc == 277
//     )
//     /*
//     // Sequence 2
//     if(
//     sc == 30 ||
//     sc == 84 ||
//     sc == 165 ||
//     sc == 240 ||
//     sc == 300
//     )
//     
//     {
//     cout << "Yang: TEncSlice: initEncSlice: Forcing an I frame XXXX " << sc << "\n" << endl;
//     rpcSlice->setSliceType(I_SLICE);
//     //        getchar();
//     }
//     */
//    // Hossam Code Tweaking -- Second trial end
//    //*************************************************************************************
//    
//    if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
//    {
//        dQP = xGetQPValueAccordingToLambda( dLambda );
//        iQP = max( -pSPS->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );
//    }
//    
//    rpcSlice->setSliceQp           ( iQP );
//#if ADAPTIVE_QP_SELECTION
//    rpcSlice->setSliceQpBase       ( iQP );
//#endif
//    rpcSlice->setSliceQpDelta      ( 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
//    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
//    rpcSlice->setUseChromaQpAdj( pPPS->getChromaQpAdjTableSize() > 0 );
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
//    
//    if ( m_pcCfg->getDeblockingFilterMetric() )
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag(true);
//        rpcSlice->setDeblockingFilterDisable(false);
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
//    {
//        rpcSlice->getPPS()->setDeblockingFilterOverrideEnabledFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->setDeblockingFilterOverrideFlag( !m_pcCfg->getLoopFilterOffsetInPPS() );
//        rpcSlice->getPPS()->setPicDisableDeblockingFilterFlag( m_pcCfg->getLoopFilterDisable() );
//        rpcSlice->setDeblockingFilterDisable( m_pcCfg->getLoopFilterDisable() );
//        if ( !rpcSlice->getDeblockingFilterDisable())
//        {
//            if ( !m_pcCfg->getLoopFilterOffsetInPPS() && eSliceType!=I_SLICE)
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
//            }
//            else
//            {
//                rpcSlice->getPPS()->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->getPPS()->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//                rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
//                rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
//            }
//        }
//    }
//    else
//    {
//        rpcSlice->setDeblockingFilterOverrideFlag( false );
//        rpcSlice->setDeblockingFilterDisable( false );
//        rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
//        rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
//    }
//    
//    rpcSlice->setDepth            ( depth );
//    
//    pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );
//    if(eSliceType==I_SLICE)
//    {
//        pcPic->setTLayer(0);
//    }
//    rpcSlice->setTLayer( pcPic->getTLayer() );
//    
//    assert( m_apcPicYuvPred );
//    assert( m_apcPicYuvResi );
//    
//    pcPic->setPicYuvPred( m_apcPicYuvPred );
//    pcPic->setPicYuvResi( m_apcPicYuvResi );
//    
//    // Hossam: Settting Slice Mode
//    rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
//    rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
//    rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
//    rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
//    rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );
//    xStoreWPparam( pPPS->getUseWP(), pPPS->getWPBiPred() );
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



