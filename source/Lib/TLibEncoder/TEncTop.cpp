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

/** \file     TEncTop.cpp
 \brief    encoder class
 */

#include "TLibCommon/CommonDef.h"
#include "TEncTop.h"
#include "TEncPic.h"
#include "TLibCommon/TComChromaFormat.h"
#if FAST_BIT_EST
#include "TLibCommon/ContextModel.h"
#endif

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncTop::TEncTop()
{
    
    // Hossam: look ahead
    m_iPOCLastLast = -1;
    m_iNumPicRvdLast = 0;
    
    m_iPOCLast          = -1;
    m_iNumPicRcvd       =  0;
    m_uiNumAllPicCoded  =  0;
    m_pppcRDSbacCoder   =  NULL;
    m_pppcBinCoderCABAC =  NULL;
    m_cRDGoOnSbacCoder.init( &m_cRDGoOnBinCoderCABAC );
#if ENC_DEC_TRACE
    if (g_hTrace == NULL)
    {
        g_hTrace = fopen( "TraceEnc_RExt.txt", "wb" );
    }
    g_bJustDoIt = g_bEncDecTraceDisable;
    g_nSymbolCounter = 0;
#endif
    
    m_iMaxRefPicNum     = 0;
    
#if FAST_BIT_EST
    ContextModel::buildNextStateTable();
#endif
    
    m_pcSbacCoders           = NULL;
    m_pcBinCoderCABACs       = NULL;
    m_ppppcRDSbacCoders      = NULL;
    m_ppppcBinCodersCABAC    = NULL;
    m_pcRDGoOnSbacCoders     = NULL;
    m_pcRDGoOnBinCodersCABAC = NULL;
    m_pcBitCounters          = NULL;
    m_pcRdCosts              = NULL;
}

TEncTop::~TEncTop()
{
#if ENC_DEC_TRACE
    if (g_hTrace != stdout)
    {
        fclose( g_hTrace );
    }
#endif
}


Void TEncTop::create ()
{
    // initialize global variables
    initROM();
    
    
    cout << "Yang: TEncTop: create: I'm creating the GOP, Slice, CU Encoder " << "\n" << endl;
    //    getchar();
    
    // create processing unit classes
    m_cGOPEncoder.        create( );
    m_cSliceEncoder.      create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth );
    m_cCuEncoder.         create( g_uiMaxCUDepth, g_uiMaxCUWidth, g_uiMaxCUHeight, m_chromaFormatIDC );
    
    // create SC processing unit
    m_cSCEncoder.    create();
    
    if (m_bUseSAO)
    {
        m_cEncSAO.create( getSourceWidth(), getSourceHeight(), m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_saoOffsetBitShift[CHANNEL_TYPE_LUMA], m_saoOffsetBitShift[CHANNEL_TYPE_CHROMA] );
#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
        m_cEncSAO.createEncData(getSaoLcuBoundary());
#else
        m_cEncSAO.createEncData();
#endif
    }
#if ADAPTIVE_QP_SELECTION
    if (m_bUseAdaptQpSelect)
    {
        m_cTrQuant.initSliceQpDelta();
    }
#endif
    
    m_cLoopFilter.create( g_uiMaxCUDepth );
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.init( m_framesToBeEncoded, m_RCTargetBitrate, m_iFrameRate, m_iGOPSize, m_iSourceWidth, m_iSourceHeight,
                         g_uiMaxCUWidth, g_uiMaxCUHeight, m_RCKeepHierarchicalBit, m_RCUseLCUSeparateModel, m_GOPList );
    }
    
    m_pppcRDSbacCoder = new TEncSbac** [g_uiMaxCUDepth+1];
#if FAST_BIT_EST
    m_pppcBinCoderCABAC = new TEncBinCABACCounter** [g_uiMaxCUDepth+1];
#else
    m_pppcBinCoderCABAC = new TEncBinCABAC** [g_uiMaxCUDepth+1];
#endif
    
    for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
    {
        m_pppcRDSbacCoder[iDepth] = new TEncSbac* [CI_NUM];
#if FAST_BIT_EST
        m_pppcBinCoderCABAC[iDepth] = new TEncBinCABACCounter* [CI_NUM];
#else
        m_pppcBinCoderCABAC[iDepth] = new TEncBinCABAC* [CI_NUM];
#endif
        
        for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
        {
            m_pppcRDSbacCoder[iDepth][iCIIdx] = new TEncSbac;
#if FAST_BIT_EST
            m_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABACCounter;
#else
            m_pppcBinCoderCABAC [iDepth][iCIIdx] = new TEncBinCABAC;
#endif
            m_pppcRDSbacCoder   [iDepth][iCIIdx]->init( m_pppcBinCoderCABAC [iDepth][iCIIdx] );
        }
    }
}

/**
 - Allocate coders required for wavefront for the nominated number of substreams.
 .
 \param iNumSubstreams Determines how much information to allocate.
 */
Void TEncTop::createWPPCoders(Int iNumSubstreams)
{
    if (m_pcSbacCoders != NULL)
    {
        return; // already generated.
    }
    
    m_iNumSubstreams         = iNumSubstreams;
    m_pcSbacCoders           = new TEncSbac       [iNumSubstreams];
    m_pcBinCoderCABACs       = new TEncBinCABAC   [iNumSubstreams];
    m_pcRDGoOnSbacCoders     = new TEncSbac       [iNumSubstreams];
    m_pcRDGoOnBinCodersCABAC = new TEncBinCABAC   [iNumSubstreams];
    m_pcBitCounters          = new TComBitCounter [iNumSubstreams];
    m_pcRdCosts              = new TComRdCost     [iNumSubstreams];
    
    for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
    {
        m_pcRDGoOnSbacCoders[ui].init( &m_pcRDGoOnBinCodersCABAC[ui] );
        m_pcSbacCoders[ui].init( &m_pcBinCoderCABACs[ui] );
    }
    m_ppppcRDSbacCoders      = new TEncSbac***    [iNumSubstreams];
    m_ppppcBinCodersCABAC    = new TEncBinCABAC***[iNumSubstreams];
    for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ )
    {
        m_ppppcRDSbacCoders[ui]  = new TEncSbac** [g_uiMaxCUDepth+1];
        m_ppppcBinCodersCABAC[ui]= new TEncBinCABAC** [g_uiMaxCUDepth+1];
        
        for ( Int iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
        {
            m_ppppcRDSbacCoders[ui][iDepth]  = new TEncSbac*     [CI_NUM];
            m_ppppcBinCodersCABAC[ui][iDepth]= new TEncBinCABAC* [CI_NUM];
            
            for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
            {
                m_ppppcRDSbacCoders  [ui][iDepth][iCIIdx] = new TEncSbac;
                m_ppppcBinCodersCABAC[ui][iDepth][iCIIdx] = new TEncBinCABAC;
                m_ppppcRDSbacCoders  [ui][iDepth][iCIIdx]->init( m_ppppcBinCodersCABAC[ui][iDepth][iCIIdx] );
            }
        }
    }
}

Void TEncTop::destroy ()
{
    // destroy processing unit classes
    m_cGOPEncoder.        destroy();
    m_cSliceEncoder.      destroy();
    m_cCuEncoder.         destroy();
    
    // destroy the Scenechange processing unit
    m_cSCEncoder.   destroy();
    
    if (m_cSPS.getUseSAO())
    {
        m_cEncSAO.destroyEncData();
        m_cEncSAO.destroy();
    }
    m_cLoopFilter.        destroy();
    m_cRateCtrl.          destroy();
    Int iDepth;
    for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
    {
        for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
        {
            delete m_pppcRDSbacCoder[iDepth][iCIIdx];
            delete m_pppcBinCoderCABAC[iDepth][iCIIdx];
        }
    }
    
    for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
    {
        delete [] m_pppcRDSbacCoder[iDepth];
        delete [] m_pppcBinCoderCABAC[iDepth];
    }
    
    delete [] m_pppcRDSbacCoder;
    delete [] m_pppcBinCoderCABAC;
    
    for ( UInt ui = 0; ui < m_iNumSubstreams; ui++ )
    {
        for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
        {
            for (Int iCIIdx = 0; iCIIdx < CI_NUM; iCIIdx ++ )
            {
                delete m_ppppcRDSbacCoders  [ui][iDepth][iCIIdx];
                delete m_ppppcBinCodersCABAC[ui][iDepth][iCIIdx];
            }
        }
        
        for ( iDepth = 0; iDepth < g_uiMaxCUDepth+1; iDepth++ )
        {
            delete [] m_ppppcRDSbacCoders  [ui][iDepth];
            delete [] m_ppppcBinCodersCABAC[ui][iDepth];
        }
        delete[] m_ppppcRDSbacCoders  [ui];
        delete[] m_ppppcBinCodersCABAC[ui];
    }
    delete[] m_ppppcRDSbacCoders;
    delete[] m_ppppcBinCodersCABAC;
    delete[] m_pcSbacCoders;
    delete[] m_pcBinCoderCABACs;
    delete[] m_pcRDGoOnSbacCoders;
    delete[] m_pcRDGoOnBinCodersCABAC;
    delete[] m_pcBitCounters;
    delete[] m_pcRdCosts;
    
    // destroy ROM
    destroyROM();
    
    return;
}

Void TEncTop::init(Bool isFieldCoding)
{
    // initialize SPS
    xInitSPS();
    
    // Hossam: GOP_STR_TYPE
    //    m_cGOPEncoder.m_inputFileName = m_inputFileName;
    //    m_cGOPEncoder.m_gop_str_type = m_gop_str_type;
    
    
    //    cout << " I srsly love you sexy :D " << m_cGOPEncoder.m_inputFileName << " " << m_cGOPEncoder.m_gop_str_type;
    // getchar();
    
    // set the VPS profile information
    *m_cVPS.getPTL() = *m_cSPS.getPTL();
    m_cVPS.getTimingInfo()->setTimingInfoPresentFlag       ( false );
    
    m_cRdCost.setCostMode(m_costMode);
    
    // initialize PPS
    m_cPPS.setSPS(&m_cSPS);
    xInitPPS();
    xInitRPS(isFieldCoding);
    
    cout << "xInitRPS Inside TEncTop after the INIT: " << endl;
    
    xInitPPSforTiles();
    
    // initialize processing unit classes
    m_cGOPEncoder.  init( this );
    m_cSliceEncoder.init( this );
    m_cCuEncoder.   init( this );
    
    // Initialize the SC processing unit class
    m_cSCEncoder.init(this);
    
    
    // initialize transform & quantization class
    m_pcCavlcCoder = getCavlcCoder();
    
    m_cTrQuant.init( 1 << m_uiQuadtreeTULog2MaxSize,
                    m_useRDOQ,
                    m_useRDOQTS,
                    true
                    ,m_useTransformSkipFast
#if ADAPTIVE_QP_SELECTION
                    ,m_bUseAdaptQpSelect
#endif
                    );
    
    // initialize encoder search class
    m_cSearch.init( this, &m_cTrQuant, m_iSearchRange, m_bipredSearchRange, m_iFastSearch, 0, &m_cEntropyCoder, &m_cRdCost, getRDSbacCoder(), getRDGoOnSbacCoder() );
    
    m_iMaxRefPicNum = 0;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncTop::deletePicBuffer()
{
    TComList<TComPic*>::iterator iterPic = m_cListPic.begin();
    Int iSize = Int( m_cListPic.size() );
    
    for ( Int i = 0; i < iSize; i++ )
    {
        TComPic* pcPic = *(iterPic++);
        
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
    }
}

// Hossam: Scene Change
Void TEncTop::deleteOrgPicBuffer()
{
    TComList<TComPic*>::iterator iterPic = m_cListOrgPic.begin();
    Int iSize = Int( m_cListOrgPic.size() );
    
    for ( Int i = 0; i < iSize; i++ )
    {
        TComPic* pcPic = *(iterPic++);
        
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
    }
}


Void TEncTop::deletePicBufferLookAhead()
{
    TComList<TComPic*>::iterator iterPic = m_cListPicLookAhead.begin();
    Int iSize = Int( m_cListPicLookAhead.size() );
    
    for ( Int i = 0; i < iSize; i++ )
    {
        TComPic* pcPic = *(iterPic++);
        
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
    }
}


/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  rcListBitstreamOut  list of output bitstreams
 \retval  iNumEncoded         number of encoded pictures
 */
// Hossam: Reads every GOP and encodes it
Void TEncTop::encodeNew( Bool flush, TComPicYuv* pcPicYuvOrg, TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded )
{
    
//        m_cTEncTop.encodeNew( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
     ///rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference


    if (pcPicYuvOrg != NULL)
    {
        
        // Hossam: Scene change: I guess it gets the original picture buffer and not the whole thing
        // get original YUV
        TComPic* pcPicCurr = NULL;
        
        xGetNewPicBuffer( pcPicCurr );
        pcPicYuvOrg->copyToPic( pcPicCurr->getPicYuvOrg() );
        
//         cout << "PICLast EncodeNew Working : " << m_iPOCLast << endl;
        // TRUE_ORG is the input file without any pre-encoder colour space conversion (but with possible bit depth increment)
        pcPicYuvTrueOrg->copyToPic( pcPicCurr->getPicYuvTrueOrg() );
        
        // Hossam: Scene change: By here, I guess he should have inserted a new TComPic in the buffer be kol 7agaa
        // Hossam: Scene change: This step happens every GOP (4 frames)
//            cout <<  " Monna Yes I am everytime coming here to copy the PIC YUV org to the rcLIst buffer " << endl;
        
        // compute image characteristics
        if ( getUseAdaptiveQP() )
        {
            m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcPicCurr ) );
        }
        
        // BUFFER PROBLEM HERE XXXXXXXX
        //        // Hossam: Scene Change XXXX I guess I need to get the buffer here
        //        xGetNewPicBufferOrg(pcPicCurr);

    }
    
//    cout << "m_iNumPicRcvd " << m_iNumPicRcvd << ", " << ", flush " << flush << ", POCLast " << m_iPOCLast << ", GOPsize " << m_iGOPSize << endl;
    
    if ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0)))
    {
        iNumEncoded = 0;
        return;
    }
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }
    
    // cout << m_iNumPicRcvd << ")  YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iNumPicRcvd) << endl;
    
    //    UInt pocCurr = m_iPOCLast - m_iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - (( true && m_iGOPSize>1) ? 1:0)
    //    cout << m_iNumPicRcvd << ")  m_iNumPicRcvd YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iNumPicRcvd) << endl;
    //    cout << m_iPOCLast << ")  m_iPOCLast YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iPOCLast) << endl;
    
    // Hossam
    //    Int pocCurr;
    //    Int m_iGopSize = m_cGOPEncoder.getGOPSize();
    //    Bool isField = false; // default value from cfg is false!
    //    for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
    //    {
    //        // determine actual POC
    //        if(m_iPOCLast == 0) //case first frame or first top field
    //        {
    //            pocCurr=0;
    //        }
    //        else if(m_iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    //        {
    //            pocCurr = 1;
    //        }
    //        else
    //        {
    //            // Change m_Cfg into public
    //            pocCurr = m_iPOCLast - m_iNumPicRcvd + m_cGOPEncoder.m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
    //        }
    //    }
    
    //    cout << pocCurr << ")  pocCurr YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iNumPicRcvd) << endl;
    
    //    if (pocCurr == 10) {
    //            xInitRPSNew(isField);
    //    }
    
    // compress GOP
//    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
    // Hossam:
     // rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference
    // m_cListOrgPic: My List
    // m_cListPic: the one I'm guessing it's the original frames buffer
    
#if IS_YAO_SCD || IS_SASTRE_SCD // Skip encoding is within
        m_cGOPEncoder.compressGOPNewBench(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
//    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);

#else
//    cout << "Compress GOP EncodeNew Working: " << m_iPOCLast << ", " << m_iNumPicRcvd << endl;
     m_cGOPEncoder.compressGOPNew(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#endif

    
    // Hossam: Scene Change
//    m_cGOPEncoder.compressGOPNew(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, m_cListOrgPic, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);

    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.destroyRCGOP();
    }
    
    iNumEncoded         = m_iNumPicRcvd;
    m_iNumPicRcvd       = 0;
    m_uiNumAllPicCoded += iNumEncoded;
}

///////////// P frames //////////



Void TEncTop::encodeNewInter( Bool flush, TComPicYuv* pcPicYuvOrg, TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded )
{
    
    if (pcPicYuvOrg != NULL)
    {
        
        // Hossam: Scene change: I guess it gets the original picture buffer and not the whole thing
        // get original YUV
        TComPic* pcPicCurr = NULL;
        
        xGetNewPicBuffer( pcPicCurr );
        pcPicYuvOrg->copyToPic( pcPicCurr->getPicYuvOrg() );
        
        //         cout << "PICLast EncodeNew Working : " << m_iPOCLast << endl;
        // TRUE_ORG is the input file without any pre-encoder colour space conversion (but with possible bit depth increment)
        pcPicYuvTrueOrg->copyToPic( pcPicCurr->getPicYuvTrueOrg() );
        
        // Hossam: Scene change: By here, I guess he should have inserted a new TComPic in the buffer be kol 7agaa
        // Hossam: Scene change: This step happens every GOP (4 frames)
        //            cout <<  " Monna Yes I am everytime coming here to copy the PIC YUV org to the rcLIst buffer " << endl;
        
        // compute image characteristics
        if ( getUseAdaptiveQP() )
        {
            m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcPicCurr ) );
        }
        
        // BUFFER PROBLEM HERE XXXXXXXX
        //        // Hossam: Scene Change XXXX I guess I need to get the buffer here
        //        xGetNewPicBufferOrg(pcPicCurr);
        
    }
    
    //    cout << "m_iNumPicRcvd " << m_iNumPicRcvd << ", " << ", flush " << flush << ", POCLast " << m_iPOCLast << ", GOPsize " << m_iGOPSize << endl;
    
    if ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0)))
    {
        iNumEncoded = 0;
        return;
    }
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }
    
    // Hossam:
    // rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference
    // m_cListOrgPic: My List
    // m_cListPic: the one I'm guessing it's the original frames buffer
    
#if IS_DING_SCD || IS_YAO_SCD || IS_SASTRE_SCD // Skip encoding is within
    
// if you actually want to run the adaptive QP
#if INTER_RUN_YAO_SASTRE
    // if you want to run the adaptive QP with SCD
    #if IS_INTER_DEP_WITH_SC
        // if epsilon or mu
        #if IS_EPSILON_PRED_METHOD
            m_cGOPEncoder.compressGOPNewInterSC(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
        #else
            m_cGOPEncoder.compressGOPNewInterMuSC(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
        #endif
    
        #else
            m_cGOPEncoder.compressGOPNewBench(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    #endif // end if IS_INTER_DEP_WITH_SC
#else
    m_cGOPEncoder.compressGOPNewBench(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
#endif
    
    
#else
    
#if IS_EPSILON_PRED_METHOD
    
#if IS_INTER_DEP_WITH_SC
    
    // use this for SC algorithms
        m_cGOPEncoder.compressGOPNewInterSC(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#else
    //    cout << "Compress GOP EncodeNew Working: " << m_iPOCLast << ", " << m_iNumPicRcvd << endl;
    m_cGOPEncoder.compressGOPNewInter(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#endif // end if IS_INTER_DEP_WITH_SC
    
#else
    
#if IS_INTER_DEP_MULTIPLE_REF // multiple reference for mu
//    m_cGOPEncoder.compressGOPNewInterMuMult(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    //    cout << "Compress GOP EncodeNew Working: " << m_iPOCLast << ", " << m_iNumPicRcvd << endl;
    m_cGOPEncoder.compressGOPNewInterMu(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#else
    
#if IS_INTER_DEP_WITH_SC
    m_cGOPEncoder.compressGOPNewInterMuSC(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#else
    //    cout << "Compress GOP EncodeNew Working: " << m_iPOCLast << ", " << m_iNumPicRcvd << endl;
    m_cGOPEncoder.compressGOPNewInterMu(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#endif // end if IS_INTER_DEP_WITH_SC
    

#endif // end if IS_INTERDEP_MULTIPLE_REF
    
#endif // end if IS_EPSILON_PRED_METHOD
    


    
#endif // end if IS_YAO_SCD || IS_SASTRE_SCD
    
    
    // Hossam: Scene Change
    //    m_cGOPEncoder.compressGOPNew(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, m_cListOrgPic, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.destroyRCGOP();
    }
    
    iNumEncoded         = m_iNumPicRcvd;
    m_iNumPicRcvd       = 0;
    m_uiNumAllPicCoded += iNumEncoded;
    
}



/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  rcListBitstreamOut  list of output bitstreams
 \retval  iNumEncoded         number of encoded pictures
 */
// Hossam: Reads every GOP and encodes it
Void TEncTop::encodeNewP04( Bool flush, TComPicYuv* pcPicYuvOrg, TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded )
{
    
    //        m_cTEncTop.encodeNew( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
    ///rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference
    
    
    if (pcPicYuvOrg != NULL)
    {
        
        // Hossam: Scene change: I guess it gets the original picture buffer and not the whole thing
        // get original YUV
        TComPic* pcPicCurr = NULL;
        
        xGetNewPicBuffer( pcPicCurr );
        pcPicYuvOrg->copyToPic( pcPicCurr->getPicYuvOrg() );
        
        cout << "PICLast Encode Not working : " << m_iPOCLast << endl;
        // TRUE_ORG is the input file without any pre-encoder colour space conversion (but with possible bit depth increment)
        pcPicYuvTrueOrg->copyToPic( pcPicCurr->getPicYuvTrueOrg() );
        
        // Hossam: Scene change: By here, I guess he should have inserted a new TComPic in the buffer be kol 7agaa
        // Hossam: Scene change: This step happens every GOP (4 frames)
        //            cout <<  " Monna Yes I am everytime coming here to copy the PIC YUV org to the rcLIst buffer " << endl;
        
        // compute image characteristics
        if ( getUseAdaptiveQP() )
        {
            m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcPicCurr ) );
        }
    }
    
//    cout << "encodeNewPP m_iNumPicRcvd " << m_iNumPicRcvd << ", " << ", flush " << flush << ", POCLast " << m_iPOCLast << ", GOPsize " << m_iGOPSize << endl;

    // Hossam: This if statement to write output or not
    if ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0)))
    {
        iNumEncoded = 0;
        return;
    }
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }
    
    // Hossam:
    // rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference
    // m_cListOrgPic: My List
    // m_cListPic: the one I'm guessing it's the original frames buffer
    
#if IS_YAO_SCD || IS_SASTRE_SCD // Skip encoding is within
    m_cGOPEncoder.compressGOPNewBench(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    //    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
#else
    cout << "Compress GOP EncodeNew Working: " << m_iPOCLast << ", " << m_iNumPicRcvd << endl;
    m_cGOPEncoder.compressGOPNewP04(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
#endif
    
    
    // Hossam: Scene Change
    //    m_cGOPEncoder.compressGOPNew(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, m_cListOrgPic, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.destroyRCGOP();
    }
    
    iNumEncoded         = m_iNumPicRcvd;
    m_iNumPicRcvd       = 0;
    m_uiNumAllPicCoded += iNumEncoded;
}


Void TEncTop::encodeNewP(Bool isMockEncoding, Bool isAddPSNR, Bool flush, TComPicYuv* pcPicYuvOrg, TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded )
{
    
    //        m_cTEncTop.encodeNew( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
    ///rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference
    
    
    if (pcPicYuvOrg != NULL)
    {
        
        // Hossam: Scene change: I guess it gets the original picture buffer and not the whole thing
        // get original YUV
        TComPic* pcPicCurr = NULL;
        
        if(isMockEncoding) {
            xGetNewPicBufferLookAhead( pcPicCurr );
        }
        else {
            xGetNewPicBufferLookActual( pcPicCurr );
        }
        
        pcPicYuvOrg->copyToPic( pcPicCurr->getPicYuvOrg() );
        
//        cout << "PICLast Encode Not working : " << m_iPOCLast << endl;
        // TRUE_ORG is the input file without any pre-encoder colour space conversion (but with possible bit depth increment)
        pcPicYuvTrueOrg->copyToPic( pcPicCurr->getPicYuvTrueOrg() );
        
        // Hossam: Scene change: By here, I guess he should have inserted a new TComPic in the buffer be kol 7agaa
        // Hossam: Scene change: This step happens every GOP (4 frames)
        //            cout <<  " Monna Yes I am everytime coming here to copy the PIC YUV org to the rcLIst buffer " << endl;
        
        // compute image characteristics
        if ( getUseAdaptiveQP() )
        {
            m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcPicCurr ) );
        }
    }
    
//    cout << "encodeNewPP m_iNumPicRcvd " << m_iNumPicRcvd << ", " << ", flush " << flush << ", POCLast " << m_iPOCLast << ", GOPsize " << m_iGOPSize << endl;
//    cout << "TEncTop: isMockEncoding:  " << isMockEncoding << endl;
    
    // Hossam: This if statement to write output or not
    
    
//    if (isMockEncoding && ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0))))
//    if (true && ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0))))
//    if (!isMockEncoding && ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0))))
    if (((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0))))
    {
//        cout << " [STOP EXEC] Stop Encode New Pee from execution " << endl;
        iNumEncoded = 0;
//        return;
    }
//    else
//    {
//        cout << " [PROCEED EXEC] Stop Encode New Pee from execution " << endl;
//        
//        cout << "--Setting the number of pictures received to ONE when I reach a GOP" << endl;
////        if (m_iPOCLast != 0) {
////            m_iNumPicRcvd       = 1;
////        }
//    }
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }
    
    // Hossam:
    // rcListPicYuvRecOut < list of reconstruction YUV files coming from the very top encoder main, sent by reference
    // m_cListOrgPic: My List
    // m_cListPic: the one I'm guessing it's the original frames buffer
    
#if IS_YAO_SCD || IS_SASTRE_SCD // Skip encoding is within
    m_cGOPEncoder.compressGOPNewBench(m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    //    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
#else
//    cout << "Compress GOP EncodeNew Working: " << m_iPOCLast << ", " << m_iNumPicRcvd << endl;
//    cout << "TEncTop: tell me mockEncoding is: " << isMockEncoding << endl;
    static bool is_first_frame = true;
    static bool is_first_frame_actual = true;
    
    if(isMockEncoding) {
        
//        cout << "\n\n\n******* Enter Encode  " << endl;
//        reportCounterStatus();
        
    
//        if(m_iPOCLast == 13 || m_iPOCLast == 14)
//        {
//            cout << "before compressGOPNewPMock show me the rc lists " << endl;
//            showMeRcLists();
//        }
        
        m_cGOPEncoder.compressGOPNewPMock(isMockEncoding, isAddPSNR, m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPicLookAhead, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
//        m_cGOPEncoder.compressGOPNewPMock(isMockEncoding, isAddPSNR, m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);

        if(is_first_frame)
        {
            m_iNumPicRcvd--;
            is_first_frame = false;
        }
        
//        if(is_first_frame)
//        {
//            cout << "Setting the number of pictures coded to zero after first frame " << endl;
////            m_cGOPEncoder.setNumPicCodedGOP(0);
//            is_first_frame = false;
//        }
//        else
//        {
//            if(m_iPOCLast > 1) {
//                m_cGOPEncoder.incrGOPId();
//            }
//         
//        }
        

        
    }
    else{
        
//        cout << "\n\n\n******* Enter Encode Actual  " << endl;
//        reportCounterStatus();
        
        m_cGOPEncoder.compressGOPNewP(isMockEncoding, isAddPSNR, m_cListOrgPic, m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);

        if(is_first_frame_actual)
        {
            m_iNumPicRcvd--;
            is_first_frame_actual = false;
        }
    }
#endif
    
    
    // Hossam: Scene Change
    //    m_cGOPEncoder.compressGOPNew(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, m_cListOrgPic, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.destroyRCGOP();
    }
    
    
//    cout << "XXXXXXXXXX iNumEncoded " << iNumEncoded << ", Rcvd " << m_iNumPicRcvd << ", Coded " << m_uiNumAllPicCoded << endl;
    
    iNumEncoded         = m_iNumPicRcvd;

    
//    m_iNumPicRcvd       = 0;
//    m_uiNumAllPicCoded += iNumEncoded;
//    cout << "XXXXXXXXXX111 iNumEncoded " << iNumEncoded << ", Rcvd " << m_iNumPicRcvd << ", Coded " << m_uiNumAllPicCoded << endl;
}



///////////////P Frames/////////////////



/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   pcPicYuvOrg         original YUV picture
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  rcListBitstreamOut  list of output bitstreams
 \retval  iNumEncoded         number of encoded pictures
 */
// Hossam: Reads every GOP and encodes it
Void TEncTop::encode( Bool flush, TComPicYuv* pcPicYuvOrg, TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded )
{
    cout << "Yes I read the GOP in the buffer and I'll start encoding in TEncTop " << endl;
    
    
    
    if (pcPicYuvOrg != NULL)
    {
        cout << " Monna is mad: pcPicYuvOrg != null encode TEncTop: " << endl;
        // get original YUV
        TComPic* pcPicCurr = NULL;
        
        xGetNewPicBuffer( pcPicCurr );
        pcPicYuvOrg->copyToPic( pcPicCurr->getPicYuvOrg() );
        
        // TRUE_ORG is the input file without any pre-encoder colour space conversion (but with possible bit depth increment)
        pcPicYuvTrueOrg->copyToPic( pcPicCurr->getPicYuvTrueOrg() );
        
        
//        // Hossam: Scene Change XXXX I guess I need to get the buffer here
//        xGetNewPicBufferOrg(pcPicCurr);
        
        // compute image characteristics
        if ( getUseAdaptiveQP() )
        {
            m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcPicCurr ) );
        }
    }
    
    if ((m_iNumPicRcvd == 0) || (!flush && (m_iPOCLast != 0) && (m_iNumPicRcvd != m_iGOPSize) && (m_iGOPSize != 0)))
    {
        iNumEncoded = 0;
        return;
    }
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.initRCGOP( m_iNumPicRcvd );
    }
    
    // cout << m_iNumPicRcvd << ")  YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iNumPicRcvd) << endl;
    
    //    UInt pocCurr = m_iPOCLast - m_iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - (( true && m_iGOPSize>1) ? 1:0)
    //    cout << m_iNumPicRcvd << ")  m_iNumPicRcvd YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iNumPicRcvd) << endl;
    //    cout << m_iPOCLast << ")  m_iPOCLast YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iPOCLast) << endl;
    
    // Hossam
    //    Int pocCurr;
    //    Int m_iGopSize = m_cGOPEncoder.getGOPSize();
    //    Bool isField = false; // default value from cfg is false!
    //    for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
    //    {
    //        // determine actual POC
    //        if(m_iPOCLast == 0) //case first frame or first top field
    //        {
    //            pocCurr=0;
    //        }
    //        else if(m_iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    //        {
    //            pocCurr = 1;
    //        }
    //        else
    //        {
    //            // Change m_Cfg into public
    //            pocCurr = m_iPOCLast - m_iNumPicRcvd + m_cGOPEncoder.m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
    //        }
    //    }
    
    //    cout << pocCurr << ")  pocCurr YARA 111 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(m_iNumPicRcvd) << endl;
    
    //    if (pocCurr == 10) {
    //            xInitRPSNew(isField);
    //    }
    
    // Send the buffer to the compress GOP
    // compress GOP
    m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, false, false, snrCSC, m_printFrameMSE);
    
    
    if ( m_RCEnableRateControl )
    {
        m_cRateCtrl.destroyRCGOP();
    }
    
    iNumEncoded         = m_iNumPicRcvd;
    m_iNumPicRcvd       = 0;
    m_uiNumAllPicCoded += iNumEncoded;
}

// Hossam
//Void TEncTop::xInitRPSNew(Bool isFieldCoding)
//{
//
//    TComReferencePictureSet*      rps;
//
//    m_cSPS.createRPSList(getGOPSize() + m_extraRPSs + 1);
//    TComRPSList* rpsList = m_cSPS.getRPSList();
//
//    UInt m_numRefPics = 1;
//    UInt m_numRefIdc = 2;
//    // Removed Inner loop for +ve and negative
//    Int numNeg = 1;
//    Int numPos = 0;
//    UInt m_interRPSPrediction = 1;
//    UInt m_SC = 0;
//    UInt referencePic = -1;
//    UInt m_deltaRPS = -1;
//
//
//    for (Int i = 0; i < getGOPSize() + m_extraRPSs; i++) {
////        GOPEntry ge = getGOPEntry(i);
//          GOPEntry* ge = getGOPEntryNew(i);
//
//
//         cout << "Before ge: " << ge << endl;
////        cout << "Before ge: " << &ge << endl;
//
//        cout << i << ")  YARA 000 TAREK NEGM I'M SORRY! SC!: " << endl;
//        cout << "Before ge: " << &ge << endl;
//        cout << "Before ge.m_numRefPics: " << ge -> m_numRefPics << endl;
//        cout << "Before ge.m_numRefIdc: " << ge -> m_numRefIdc << endl;
//
//
//        ge -> m_numRefPics = m_numRefPics;
//
//
//        cout << "--0 " << endl;
//
//        if (i > 0) {
//            ge -> m_numRefIdc = m_numRefIdc;
//
//
//
//            cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//            cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//
//
//            cout << "--1 " << endl;
//
//        }
//
//        for (Int j = 0 ; j < ge -> m_numRefPics; j++) {
//            ge -> m_referencePics[j] = referencePic;
//            cout << "--2A " << endl;
//        }
//        cout << "--2 " << endl;
//
//
//        //        ge.m_interRPSPrediction = m_interRPSPrediction;
//
//        if(ge->m_interRPSPrediction == 1)
//        {
//            ge->m_deltaRPS = m_deltaRPS;
//            ge->m_numRefIdc = m_numRefIdc;
//
//            cout << "--3 " << endl;
//            // set the reference IDcs without loop
//            Int k = 0;
//            ge->m_refIdc[k]   = 0;
//            ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//
//            ge->m_refIdc[++k] = 1;
//            ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//
//
//        }
//
//
//      /*
//        cout << i << ")  YARA 000 TAREK NEGM I'M SORRY! SC!: " << endl;
//        cout << "Before ge: " << &ge << endl;
//        cout << "Before ge.m_numRefPics: " << ge.m_numRefPics << endl;
//        cout << "Before ge.m_numRefIdc: " << ge.m_numRefIdc << endl;
//
//
//        ge.m_numRefPics = m_numRefPics;
//
//
//        cout << "--0 " << endl;
//
//        if (i > 0) {
//            ge.m_numRefIdc = m_numRefIdc;
//
//        cout << "After ge.m_numRefPics: " << ge.m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge.m_numRefIdc << endl;
//
//
//        cout << "--1 " << endl;
//
//        }
//
//        for (Int j = 0 ; j < ge.m_numRefPics; j++) {
//            ge.m_referencePics[j] = referencePic;
//             cout << "--2A " << endl;
//        }
//               cout << "--2 " << endl;
//
//
////        ge.m_interRPSPrediction = m_interRPSPrediction;
//
//        if(ge.m_interRPSPrediction == 1)
//        {
//            ge.m_deltaRPS = m_deltaRPS;
//            ge.m_numRefIdc = m_numRefIdc;
//
//                    cout << "--3 " << endl;
//            // set the reference IDcs without loop
//            Int k = 0;
//            ge.m_refIdc[k]   = 0;
//            ge.m_refIdc[++k] = 1;
//
//        }
//        */
//   /*
//         cout << "--4 " << endl;
//
//        rps = rpsList -> getReferencePictureSet(i);
//         cout << "--5 " << endl;
//
//        rps->setNumberOfPictures(m_numRefPics);
//        rps -> setNumRefIdc(m_numRefIdc);
//
//         cout << "--6 " << endl;
//
//        rps->setNumberOfNegativePictures(numNeg);
//        rps->setNumberOfPositivePictures(numPos);
//
//         cout << "--7 " << endl;
//        // set interprediction flag
//        */
////        cout << i << ")  YARA 111 TAREK NEGM I'M SORRY! SC!: " << endl;
//
//    }
//
////    cout << "Before INIT: " << endl;
////    GOPEntry* test =getGOPEntryNew(1);
////    cout << "Before ge: " << test << endl;
////    test = getGOPEntryNew(2);
////        cout << "Before ge: " << test << endl;
////    cout << "Before Normal Init ge.m_numRefPics: " << getGOPEntry(1).m_numRefPics << endl;
////    cout << "Before Normal Init ge.m_numRefPics: " << getGOPEntry(1).m_numRefIdc << endl;
//
////    cout << "Before Normal Init new ge.m_numRefPics: " << getGOPEntryNew(1).m_numRefPics << endl;
////    cout << "Before Normal Init new ge.m_numRefPics: " << getGOPEntryNew(1).m_numRefIdc << endl;
//
//    xInitRPS(isFieldCoding);//This fixes it!
////    xCheckGSParameters();
//    cout  << ")  Done YARA 222 TAREK NEGM I'M SORRY! SC!: " << endl;
//
//}


// Hossam

Void TEncTop::xInitRPSNew(Bool isFieldCoding, Int state)
{
    
    cout << "xInitRPSNew: SCENE Chang 3andy amal # " << state << endl;
    //    TComReferencePictureSet*      rps;
    
    m_cSPS.createRPSList(getGOPSize() + m_extraRPSs + 1);
    //    TComRPSList* rpsList = m_cSPS.getRPSList();
    //    UInt m_numRefPics = 1;
    //    UInt m_numRefIdc = 2;
    // Removed Inner loop for +ve and negative
    //    UInt m_interRPSPrediction = 1;
    //    UInt referencePic = -1;
    
    UInt m_numRefPics;
    UInt m_numRefIdc;
    UInt m_deltaRPS = -1;
    
    
    if(state == 0)
    {
        // First picture after SC
        Int i = 0;
        //        Int i = 5;
        GOPEntry* ge = getGOPEntryNew(i);
        
        m_numRefPics = 1;
        
        cout << "GROUP oof entry: " << ge << endl;
        
        // Set the number of reference pics
        ge -> m_numRefPics = m_numRefPics;
        
        cout << "--0 " << endl;
        // Set the reference pic
        ge -> m_referencePics[i] = -1;
        
        
        // Second picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 2;
        m_numRefIdc  = 2;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        Int j = 0;
        ge -> m_referencePics[j] = -1;
        ge -> m_referencePics[++j] = -2;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        Int k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        
        // Third picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 3;
        m_numRefIdc  = 3;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -3;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        
        // Fourth picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 4;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -3;
        ge -> m_referencePics[++j] = -4;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;

        
        
    }
    else if(state == 1)
    {
        // First picture after SC
        Int i = 0;
        m_numRefPics = 4;
        
        GOPEntry* ge = getGOPEntryNew(i);
        
        // Set the number of reference pics
        ge -> m_numRefPics = m_numRefPics;
        
        cout << "--0 " << endl;
        // Set the reference pics without loop
        Int m = 0;
        ge -> m_referencePics[m]   = -1;
        ge -> m_referencePics[++m] = -2;
        ge -> m_referencePics[++m] = -3;
        ge -> m_referencePics[++m] = -5;
        
        
        // Second picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        Int j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -3;
        ge -> m_referencePics[++j] = -6;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        Int k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;

        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;

        
        
        // Third picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -3;
        ge -> m_referencePics[++j] = -7;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;

        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;


        
        // Fourth picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -4;
        ge -> m_referencePics[++j] = -8;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;

    }
    else if(state == 2)
    {
        // First picture after SC
        Int i = 0;
        m_numRefPics = 4;
        
        GOPEntry* ge = getGOPEntryNew(i);
        
        // Set the number of reference pics
        ge -> m_numRefPics = m_numRefPics;
        
        cout << "--0 " << endl;
        // Set the reference pics without loop
        Int m = 0;
        ge -> m_referencePics[m]   = -1;
        ge -> m_referencePics[++m] = -2;
        ge -> m_referencePics[++m] = -5;
        ge -> m_referencePics[++m] = -9;
        
        
        // Second picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        Int j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -6;
        ge -> m_referencePics[++j] = -10;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        Int k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;

        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;

        
        // Third picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -3;
        ge -> m_referencePics[++j] = -7;
        ge -> m_referencePics[++j] = -11;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        
        // Fourth picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -4;
        ge -> m_referencePics[++j] = -8;
        ge -> m_referencePics[++j] = -12;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
    }
    else
    {
        // First picture after SC
        Int i = 0;
        m_numRefPics = 4;
        
        GOPEntry* ge = getGOPEntryNew(i);
        
        // Set the number of reference pics
        ge -> m_numRefPics = m_numRefPics;
        
        cout << "--0 " << endl;
        // Set the reference pics without loop
        Int m = 0;
        ge -> m_referencePics[m]   = -1;
        ge -> m_referencePics[++m] = -5;
        ge -> m_referencePics[++m] = -9;
        ge -> m_referencePics[++m] = -13;
        
        
        // Second picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        Int j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -2;
        ge -> m_referencePics[++j] = -6;
        ge -> m_referencePics[++j] = -10;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        Int k = 0;
        ge->m_refIdc[k]   = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        
        
        // Third picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -3;
        ge -> m_referencePics[++j] = -7;
        ge -> m_referencePics[++j] = -11;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        
        // Fourth picture after SC
        ge = getGOPEntryNew(++i);
        
        // Will be used for the rest of the frames
        m_numRefPics = 4;
        m_numRefIdc  = 5;
        
        // Set the reference IDcs and reference pics number
        ge -> m_numRefPics = m_numRefPics;
        ge -> m_numRefIdc = m_numRefIdc;
        
        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
        cout << "--1 " << endl;
        
        // Set the reference pic without loop
        j = 0;
        ge -> m_referencePics[j]   = -1;
        ge -> m_referencePics[++j] = -4;
        ge -> m_referencePics[++j] = -8;
        ge -> m_referencePics[++j] = -12;
        
        cout << "--2 " << endl;
        
        ge->m_deltaRPS = m_deltaRPS;
        ge->m_numRefIdc = m_numRefIdc;
        
        cout << "--3 " << endl;
        
        // Set the reference IDcs without loop
        k = 0;
        ge->m_refIdc[k]   = 0;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
        
        ge->m_refIdc[++k] = 1;
        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
    }
    
    cout << "xInitRPS Inside TEncTop before ADJUSMENT: " << endl;
    xInitRPS(isFieldCoding);//This fixes it!
    
    cout  << ")  Done YARA 222 TAREK NEGM I'M SORRY! SC!: " << endl;
}


//Void TEncTop::xInitRPSNew(Bool isFieldCoding, Int state)
//{
//    
//    cout << "xInitRPSNew: SCENE Chang 3andy amal # " << state << endl;
//    //    TComReferencePictureSet*      rps;
//    
//    m_cSPS.createRPSList(getGOPSize() + m_extraRPSs + 1);
//    //    TComRPSList* rpsList = m_cSPS.getRPSList();
//    //    UInt m_numRefPics = 1;
//    //    UInt m_numRefIdc = 2;
//    // Removed Inner loop for +ve and negative
//    //    UInt m_interRPSPrediction = 1;
//    //    UInt referencePic = -1;
//    
//    UInt m_numRefPics;
//    UInt m_numRefIdc;
//    UInt m_deltaRPS = -1;
//    
//    
//    if(state == 0)
//    {
//        // First picture after SC
//        Int i = 0;
//        //        Int i = 5;
//        GOPEntry* ge = getGOPEntryNew(i);
//        
//        m_numRefPics = 1;
//        
//        cout << "GROUP oof entry: " << ge << endl;
//        
//        // Set the number of reference pics
//        ge -> m_numRefPics = m_numRefPics;
//        
//        cout << "--0 " << endl;
//        // Set the reference pic
//        ge -> m_referencePics[i] = -1;
//        
//        
//        // Second picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 2;
//        m_numRefIdc  = 2;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        Int j = 0;
//        ge -> m_referencePics[j] = -1;
//        ge -> m_referencePics[++j] = -2;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        Int k = 0;
//        ge->m_refIdc[k]   = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        
//        // Third picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 2;
//        m_numRefIdc  = 3;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j] = -1;
//        ge -> m_referencePics[++j] = -3;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        
//        // Fourth picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 2;
//        m_numRefIdc  = 3;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -4;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        
//    }
//    else if(state == 1)
//    {
//        // First picture after SC
//        Int i = 0;
//        m_numRefPics = 2;
//        
//        GOPEntry* ge = getGOPEntryNew(i);
//        
//        // Set the number of reference pics
//        ge -> m_numRefPics = m_numRefPics;
//        
//        cout << "--0 " << endl;
//        // Set the reference pics without loop
//        Int m = 0;
//        ge -> m_referencePics[m]   = -1;
//        ge -> m_referencePics[++m] = -5;
//        
//        
//        // Second picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 3;
//        m_numRefIdc  = 3;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        Int j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -2;
//        ge -> m_referencePics[++j] = -6;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        Int k = 0;
//        ge->m_refIdc[k]   = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        
//        // Third picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 3;
//        m_numRefIdc  = 4;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -3;
//        ge -> m_referencePics[++j] = -7;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        
//        // Fourth picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 3;
//        m_numRefIdc  = 4;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j] = -1;
//        ge -> m_referencePics[++j] = -4;
//        ge -> m_referencePics[++j] = -8;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//    }
//    else if(state == 2)
//    {
//        // First picture after SC
//        Int i = 0;
//        m_numRefPics = 3;
//        
//        GOPEntry* ge = getGOPEntryNew(i);
//        
//        // Set the number of reference pics
//        ge -> m_numRefPics = m_numRefPics;
//        
//        cout << "--0 " << endl;
//        // Set the reference pics without loop
//        Int m = 0;
//        ge -> m_referencePics[m]   = -1;
//        ge -> m_referencePics[++m] = -5;
//        ge -> m_referencePics[++m] = -9;
//        
//        
//        // Second picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 4;
//        m_numRefIdc  = 4;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        Int j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -2;
//        ge -> m_referencePics[++j] = -6;
//        ge -> m_referencePics[++j] = -10;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        Int k = 0;
//        ge->m_refIdc[k]   = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        
//        // Third picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 4;
//        m_numRefIdc  = 5;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -3;
//        ge -> m_referencePics[++j] = -7;
//        ge -> m_referencePics[++j] = -11;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        
//        // Fourth picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 4;
//        m_numRefIdc  = 5;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -4;
//        ge -> m_referencePics[++j] = -8;
//        ge -> m_referencePics[++j] = -12;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//    }
//    else
//    {
//        // First picture after SC
//        Int i = 0;
//        m_numRefPics = 4;
//        
//        GOPEntry* ge = getGOPEntryNew(i);
//        
//        // Set the number of reference pics
//        ge -> m_numRefPics = m_numRefPics;
//        
//        cout << "--0 " << endl;
//        // Set the reference pics without loop
//        Int m = 0;
//        ge -> m_referencePics[m]   = -1;
//        ge -> m_referencePics[++m] = -5;
//        ge -> m_referencePics[++m] = -9;
//        ge -> m_referencePics[++m] = -13;
//        
//        
//        // Second picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 4;
//        m_numRefIdc  = 5;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        Int j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -2;
//        ge -> m_referencePics[++j] = -6;
//        ge -> m_referencePics[++j] = -10;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        Int k = 0;
//        ge->m_refIdc[k]   = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        
//        
//        // Third picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 4;
//        m_numRefIdc  = 5;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -3;
//        ge -> m_referencePics[++j] = -7;
//        ge -> m_referencePics[++j] = -11;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        
//        // Fourth picture after SC
//        ge = getGOPEntryNew(++i);
//        
//        // Will be used for the rest of the frames
//        m_numRefPics = 4;
//        m_numRefIdc  = 5;
//        
//        // Set the reference IDcs and reference pics number
//        ge -> m_numRefPics = m_numRefPics;
//        ge -> m_numRefIdc = m_numRefIdc;
//        
//        cout << "After ge.m_numRefPics: " << ge->m_numRefPics << endl;
//        cout << "After ge.m_numRefIdc: " << ge->m_numRefIdc << endl;
//        cout << "--1 " << endl;
//        
//        // Set the reference pic without loop
//        j = 0;
//        ge -> m_referencePics[j]   = -1;
//        ge -> m_referencePics[++j] = -4;
//        ge -> m_referencePics[++j] = -8;
//        ge -> m_referencePics[++j] = -12;
//        
//        cout << "--2 " << endl;
//        
//        ge->m_deltaRPS = m_deltaRPS;
//        ge->m_numRefIdc = m_numRefIdc;
//        
//        cout << "--3 " << endl;
//        
//        // Set the reference IDcs without loop
//        k = 0;
//        ge->m_refIdc[k]   = 0;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k] ==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//        
//        ge->m_refIdc[++k] = 1;
//        ge->m_usedByCurrPic[k] = ge->m_refIdc[k]==1;
//    }
//    
//    cout << "xInitRPS Inside TEncTop before ADJUSMENT: " << endl;
//    xInitRPS(isFieldCoding);//This fixes it!
//    
//    cout  << ")  Done YARA 222 TAREK NEGM I'M SORRY! SC!: " << endl;
//}

// Yara

/**------------------------------------------------
 Separate interlaced frame into two fields
 -------------------------------------------------**/
Void separateFields(Pel* org, Pel* dstField, UInt stride, UInt width, UInt height, Bool isTop)
{
    if (!isTop)
    {
        org += stride;
    }
    for (Int y = 0; y < height>>1; y++)
    {
        for (Int x = 0; x < width; x++)
        {
            dstField[x] = org[x];
        }
        
        dstField += stride;
        org += stride*2;
    }
    
}

// It does not come here -- Hossam
Void TEncTop::encode(Bool flush, TComPicYuv* pcPicYuvOrg, TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded, Bool isTff)
{
    
//        cout << "Yang: TEncTop: encode: Frame Encoding fun started" << "\n" << endl;
    //    getchar();
    
    
    // Dady: The fields here mean one for Y, one for U, and one for V
    iNumEncoded = 0;
    
    for (Int fieldNum=0; fieldNum<2; fieldNum++)
    {
        if (pcPicYuvOrg)
        {
            
            /* -- field initialization -- */
            const Bool isTopField=isTff==(fieldNum==0);
            
            TComPic *pcField;
            xGetNewPicBuffer( pcField );
            pcField->setReconMark (false);                     // where is this normally?
            
            if (fieldNum==1)                                   // where is this normally?
            {
                TComPicYuv* rpcPicYuvRec;
                
                // org. buffer
                if ( rcListPicYuvRecOut.size() >= (UInt)m_iGOPSize+1 ) // need to maintain field 0 in list of RecOuts while processing field 1. Hence +1 on m_iGOPSize.
                {
                    rpcPicYuvRec = rcListPicYuvRecOut.popFront();
                }
                else
                {
                    rpcPicYuvRec = new TComPicYuv;
                    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth);
                }
                rcListPicYuvRecOut.pushBack( rpcPicYuvRec );
            }
            
            pcField->getSlice(0)->setPOC( m_iPOCLast );        // superfluous?
            pcField->getPicYuvRec()->setBorderExtension(false);// where is this normally?
            
            pcField->setTopField(isTopField);                  // interlaced requirement
            
            // Dady: Components are 3 (Y, U, V)
            //         cout << "\nYang: TEncTop: encode: Number of Components ZZZZ " << (pcPicYuvOrg->getNumberValidComponents()) <<  "\n" << endl;
            // getchar();
            
            for (UInt componentIndex = 0; componentIndex < pcPicYuvOrg->getNumberValidComponents(); componentIndex++)
            {
                
                //          cout << "Yang: TEncTop: encode:Seperate Interlaced frame into two fields" << "\n" << endl;
                //          getchar();
                
                const ComponentID component = ComponentID(componentIndex);
                const UInt stride = pcPicYuvOrg->getStride(component);
                
                //        cout << "Yang: TEncTop: encode: Number of Strides YYYYY " << pcPicYuvOrg->getStride(component) <<  "\n" << endl;
                // getchar();
                
                
                separateFields((pcPicYuvOrg->getBuf(component) + pcPicYuvOrg->getMarginX(component) + (pcPicYuvOrg->getMarginY(component) * stride)),
                               pcField->getPicYuvOrg()->getAddr(component),
                               pcPicYuvOrg->getStride(component),
                               pcPicYuvOrg->getWidth(component),
                               pcPicYuvOrg->getHeight(component),
                               isTopField);
                
                separateFields((pcPicYuvTrueOrg->getBuf(component) + pcPicYuvTrueOrg->getMarginX(component) + (pcPicYuvTrueOrg->getMarginY(component) * stride)),
                               pcField->getPicYuvTrueOrg()->getAddr(component),
                               pcPicYuvTrueOrg->getStride(component),
                               pcPicYuvTrueOrg->getWidth(component),
                               pcPicYuvTrueOrg->getHeight(component),
                               isTopField);
            }
            
            // compute image characteristics
            if ( getUseAdaptiveQP() )
            {
                m_cPreanalyzer.xPreanalyze( dynamic_cast<TEncPic*>( pcField ) );
            }
        }
        
        if ( m_iNumPicRcvd && ((flush&&fieldNum==1) || (m_iPOCLast/2)==0 || m_iNumPicRcvd==m_iGOPSize ) )
        {
            
//                    cout << "Yang: TEncTop: encode: Fields Compress GOP" << "\n" << endl;
//                    getchar();
            
            //        UInt pocCurr = m_iPOCLast - m_iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - (( isField && m_iGOPSize>1) ? 1:0)
            //        UInt iGOPid = 0;
            //        UInt x = 0;
            //        UInt pocCurr = m_iPOCLast - m_iNumPicRcvd +  x - (( true && m_iGOPSize>1) ? 1:0);
            //   UInt pocCurr = m_iPOCLast - m_iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - (( true && m_iGOPSize>1) ? 1:0)
            //        cout << pocCurr << ")  YARA 222 TAREK NEGM I'M SORRY! SC!: " << m_cSCEncoder.isSceneChange(pocCurr) << endl;
            
            
            // compress GOP
            m_cGOPEncoder.compressGOP(m_iPOCLast, m_iNumPicRcvd, m_cListPic, rcListPicYuvRecOut, accessUnitsOut, true, isTff, snrCSC, m_printFrameMSE);
            
            iNumEncoded += m_iNumPicRcvd;
            m_uiNumAllPicCoded += m_iNumPicRcvd;
            m_iNumPicRcvd = 0;
        }
    }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================


/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval rpcPic obtained picture buffer
 */
Void TEncTop::xGetNewPicBuffer ( TComPic*& rpcPic )
{
    TComSlice::sortPicList(m_cListPic);
    
    // GOP size + total number of pictures in decoded picture buffer
    if (m_cListPic.size() >= (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2) )
    {
        TComList<TComPic*>::iterator iterPic  = m_cListPic.begin();
        Int iSize = Int( m_cListPic.size() );
        for ( Int i = 0; i < iSize; i++ )
        {
            rpcPic = *(iterPic++);
            
            if(rpcPic->getSlice(0)->isReferenced() == false)
            {
                break;
            }
        }
    }
    else
    {
        // Hossam: Scene Change: Every block has its QP
        if ( getUseAdaptiveQP() )
        {
            TEncPic* pcEPic = new TEncPic;
            pcEPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_cPPS.getMaxCuDQPDepth()+1, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
            rpcPic = pcEPic;
        }
        else
        {
            rpcPic = new TComPic;
            rpcPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics, false );
        }
        
        m_cListPic.pushBack( rpcPic );
    }
    
    // Hossam: Set the picture to not needed anymore in the list
    rpcPic->setReconMark (false);
    
    m_iPOCLast++;
    m_iNumPicRcvd++;
    
    rpcPic->getSlice(0)->setPOC( m_iPOCLast );
    // mark it should be extended
    rpcPic->getPicYuvRec()->setBorderExtension(false);
}


// Hossam: xGetNewPicBufferLookAhead
Void TEncTop::xGetNewPicBufferLookAhead ( TComPic*& rpcPic )
{
//    cout << "Before mock sorting " << endl;
//    showMeRcLists();
    
    TComSlice::sortPicList(m_cListPicLookAhead);
    
    // GOP size + total number of pictures in decoded picture buffer
//    if (m_cListPicLookAhead.size() >= (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2) )
//    {
    
//    UInt keep_sz = (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2  +  (1 + g_pFramesSize));
    UInt keep_sz = (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2  +  (g_pFramesSize + g_pFramesSize));

    if (m_cListPicLookAhead.size() >= (UInt)(keep_sz) )
    {
        TComList<TComPic*>::iterator iterPic  = m_cListPicLookAhead.begin();
        Int iSize = Int( m_cListPicLookAhead.size() );
        for ( Int i = 0; i < iSize; i++ )
        {
            rpcPic = *(iterPic++);
            
            if(rpcPic->getSlice(0)->isReferenced() == false)
            {
                break;
            }
        }

//        cout << "mock exceeded DPB buffer OP on it " << endl;
//        showMeRcLists();

    }
    else
    {
        // Hossam: Scene Change: Every block has its QP
        if ( getUseAdaptiveQP() )
        {
            TEncPic* pcEPic = new TEncPic;
            pcEPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_cPPS.getMaxCuDQPDepth()+1, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
            rpcPic = pcEPic;
        }
        else
        {
            rpcPic = new TComPic;
            rpcPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics, false );
        }
        
        m_cListPicLookAhead.pushBack( rpcPic );
    }
   
    
    
    // Hossam: Set the picture to not needed anymore in the list
    rpcPic->setReconMark (false);
    
    
    m_iPOCLast++;
    m_iNumPicRcvd++;
    
    
    
//    cout << "LookAhead: Update the number of pictures received for poc curr from Top to GOP " << m_iNumPicRcvd << endl;
    
    rpcPic->getSlice(0)->setPOC( m_iPOCLast );
    // mark it should be extended
    rpcPic->getPicYuvRec()->setBorderExtension(false);
    
    
//    cout << "mock did SET reconcMark to false and set POC to last  " << endl;
//    showMeRcLists();
    
}


// Hossam: look ahead -- made the DPB bigger in size to account for the case when you delete something you need
// in actually encoding the frame
Void TEncTop::xGetNewPicBufferLookActual ( TComPic*& rpcPic )
{
    
    TComSlice::sortPicList(m_cListPic);
    
    // GOP size + total number of pictures in decoded picture buffer + (1 + P)
    UInt keep_sz = (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2  +  (1 + g_pFramesSize));
    if (m_cListPic.size() >= (UInt)(keep_sz) )
    {
        
        
        TComList<TComPic*>::iterator iterPic  = m_cListPic.begin();
        Int iSize = Int( m_cListPic.size() );
        for ( Int i = 0; i < iSize; i++ )
        {
            rpcPic = *(iterPic++);
            
            if(rpcPic->getSlice(0)->isReferenced() == false)
            {
                break;
            }
        }
        
//        cout << "mock exceeded DPB buffer OP on it " << endl;
//        showMeRcLists();
        
    }
    else
    {
        // Hossam: Scene Change: Every block has its QP
        if ( getUseAdaptiveQP() )
        {
            TEncPic* pcEPic = new TEncPic;
            pcEPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_cPPS.getMaxCuDQPDepth()+1, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
            rpcPic = pcEPic;
        }
        else
        {
            rpcPic = new TComPic;
            rpcPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics, false );
        }
        
//        cout << "mock pushing a new item to DPB buffer  " << endl;
//        showMeRcLists();
        
        
        m_cListPic.pushBack( rpcPic );
    }
    
    
    
    // Hossam: Set the picture to not needed anymore in the list
    rpcPic->setReconMark (false);
    
    
    m_iPOCLast++;
    m_iNumPicRcvd++;
    
    
    
//    cout << "LookAhead: Update the number of pictures received for poc curr from Top to GOP " << m_iNumPicRcvd << endl;
    
    rpcPic->getSlice(0)->setPOC( m_iPOCLast );
    // mark it should be extended
    rpcPic->getPicYuvRec()->setBorderExtension(false);
    
    
//    cout << "mock did SET reconcMark to false and set POC to last  " << endl;
//    showMeRcLists();
    
}


// Hossam: Show me the status of the rcLists
Void TEncTop::showMeRcLists()
{
//    cout << "Show me RC Lists " << endl;
    
    cout << "\n\n******RC List ORG****** " << endl;
    
    TComPic* rpcPic;
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = m_cListPic.begin();
    while ( iterPic != m_cListPic.end())
    {
        rpcPic = *(iterPic++);
        cout << "Frame " << rpcPic->getPicSym()->getSlice(0)->getPOC() << ", " << rpcPic->getSlice(0)->isReferenced() << endl;
    }

    cout << "\n\n******RC List Mock****** " << endl;
    
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPicMock = m_cListPicLookAhead.begin();
    while ( iterPicMock != m_cListPicLookAhead.end())
    {
        rpcPic = *(iterPicMock++);
        cout << "Frame " << rpcPic->getPicSym()->getSlice(0)->getPOC() << ", " << rpcPic->getSlice(0)->isReferenced() << endl;
    }
    
    cout << "\n\n************ " << endl;
    
}

// Hossam: Restore the PicBuffer for reconstructed pictures used in referencing to before actualEncoding
//Void TEncTop::xRestorePicBufferInTop()
//{
//    //    TComSlice::sortPicList(m_cListPic);
//    
//    //    TComSlice::sortPicList(m_cListPicLookAhead);
//    cout << "xRestorePicBufferInTop Yes I will cause error after you use me for zero " << endl;
//    
//    cout << "[Before OP] LookAheadbuffer size " << m_cListPicLookAhead.size() << ", while ORG size " << m_cListPic.size() << endl;
//    
//    // Pop all from ORG
//    Int iSize = Int( m_cListPic.size() );
//    for ( Int i = 0; i < iSize; i++ )
//    {
//       m_cListPic.popBack();
//    }
//
//    // Push pictures that are less than nextPOCActual
//    UInt nextPOCActual = m_iPOCLast + 1; // xx
//    TComPic* rpcPic;
//    iSize = Int( m_cListPicLookAhead.size() );
//    for ( Int i = 0; i < iSize; i++ )
//    {
//        rpcPic = m_cListPicLookAhead.front();
//        // Push Past and Current and ignore the future
//        if(rpcPic->getPOC() <= nextPOCActual)
//        {
//            m_cListPic.pushBack(rpcPic);
//        }
//        m_cListPicLookAhead.pushBack(m_cListPicLookAhead.popFront());
//    }
//    
//    //--> modify RC list will do the referencing flags in compressGOP
//    
//    cout << "[After OP] LookAheadbuffer size " << m_cListPicLookAhead.size() << ", while ORG size " << m_cListPic.size() << endl;
//    // Copy the elements to look ahead
//    //    m_cListPicLookAhead.insert(m_cListPic.end(), m_cListPic.begin(), m_cListPic.end());
//    
//}

Void TEncTop::xRestorePicBufferInTop()
{

//    cout << "xRestorePicBufferInTop Yes I will cause error after you use me for zero " << endl;
//
//    cout << "[Before OP] LookAheadbuffer size " << m_cListPicLookAhead.size() << ", while ORG size " << m_cListPic.size() << endl;
    
    Int iSize = Int( m_cListPicLookAhead.size() );
    for ( Int i = 0; i < iSize; i++ )
    {
        m_cListPicLookAhead.popBack();
    }

    // Pop all
//    m_cListPicLookAhead.clear();
    
    Int sizeOrg = Int (m_cListPic.size());
    
    for (Int i = 0; i < sizeOrg; i++)
    {
//        cout << "Copying the elements from org to lookAhead " << i << endl;
        m_cListPicLookAhead.pushBack(m_cListPic.front());
        m_cListPic.pushBack(m_cListPic.popFront());
    }
    
//    cout << "[After OP] LookAheadbuffer size " << m_cListPicLookAhead.size() << ", while ORG size " << m_cListPic.size() << endl;    
}



// Hossam: look ahead
Void TEncTop::setLastNumPicRcvd()
{
    m_iNumPicRvdLast = m_iNumPicRcvd;
}


Void TEncTop::updateNumePicRcvd()
{
    m_iNumPicRcvd = m_iNumPicRvdLast;
}

Void TEncTop::setLastPOCLast()
{
    m_iPOCLastLast = m_iPOCLast;
}

Void TEncTop::updatePOCLast()
{
    
//    cout << "Update POC last " << m_iPOCLastLast << ", " << m_iPOCLast << endl;
    m_iPOCLast = m_iPOCLastLast;
}

///


// Hossam: Scene change xGetNewPicBuffer

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval rpcPic obtained picture buffer
 */
Void TEncTop::xGetNewPicBufferOrg ( TComPic*& rpcPic )
{
    TComSlice::sortPicList(m_cListOrgPic);
    
    // GOP size + total number of pictures in decoded picture buffer
    if (m_cListOrgPic.size() >= (UInt)(m_iGOPSize + getMaxDecPicBuffering(MAX_TLAYER-1) + 2) )
    {
        TComList<TComPic*>::iterator iterPic  = m_cListOrgPic.begin();
        Int iSize = Int( m_cListOrgPic.size() );
        for ( Int i = 0; i < iSize; i++ )
        {
            rpcPic = *(iterPic++);
            
            if(rpcPic->getSlice(0)->isReferenced() == false)
            {
                break;
            }
        }
    }
    else
    {
        // Hossam: XXXX Commented this part cause I don't want to create the picture twice!
       
        // XXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXXXXXXXXXX
        
        // Hossam: Scene Change: Every block has its QP
//        if ( getUseAdaptiveQP() )
//        {
//            TEncPic* pcEPic = new TEncPic;
//            pcEPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_cPPS.getMaxCuDQPDepth()+1, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics);
//            rpcPic = pcEPic;
//        }
//        else
//        {
//            rpcPic = new TComPic;
//            rpcPic->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, g_uiMaxCUWidth, g_uiMaxCUHeight, g_uiMaxCUDepth, m_conformanceWindow, m_defaultDisplayWindow, m_numReorderPics, false );
//        }
        
        m_cListOrgPic.pushBack( rpcPic );
    }
  
    // XXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXXXXXXXXXX
    
    // Hossam: Set the picture to not needed anymore in the list
//    rpcPic->setReconMark (false);
    
//    m_iPOCLast++;
//    m_iNumPicRcvd++;
    
//    rpcPic->getSlice(0)->setPOC( m_iPOCLast );
    // mark it should be extended
//    rpcPic->getPicYuvRec()->setBorderExtension(false);
}


Void TEncTop::xInitSPS()
{
    ProfileTierLevel& profileTierLevel = *m_cSPS.getPTL()->getGeneralPTL();
    profileTierLevel.setLevelIdc(m_level);
    profileTierLevel.setTierFlag(m_levelTier);
    profileTierLevel.setProfileIdc(m_profile);
    profileTierLevel.setProfileCompatibilityFlag(m_profile, 1);
    profileTierLevel.setProgressiveSourceFlag(m_progressiveSourceFlag);
    profileTierLevel.setInterlacedSourceFlag(m_interlacedSourceFlag);
    profileTierLevel.setNonPackedConstraintFlag(m_nonPackedConstraintFlag);
    profileTierLevel.setFrameOnlyConstraintFlag(m_frameOnlyConstraintFlag);
    profileTierLevel.setBitDepthConstraint(m_bitDepthConstraintValue);
    profileTierLevel.setChromaFormatConstraint(m_chromaFormatConstraintValue);
    profileTierLevel.setIntraConstraintFlag(m_intraConstraintFlag);
    profileTierLevel.setLowerBitRateConstraintFlag(m_lowerBitRateConstraintFlag);
    
    if ((m_profile == Profile::MAIN10) && (g_bitDepth[CHANNEL_TYPE_LUMA] == 8) && (g_bitDepth[CHANNEL_TYPE_CHROMA] == 8))
    {
        /* The above constraint is equal to Profile::MAIN */
        profileTierLevel.setProfileCompatibilityFlag(Profile::MAIN, 1);
    }
    if (m_profile == Profile::MAIN)
    {
        /* A Profile::MAIN10 decoder can always decode Profile::MAIN */
        profileTierLevel.setProfileCompatibilityFlag(Profile::MAIN10, 1);
    }
    /* XXX: should Main be marked as compatible with still picture? */
    /* XXX: may be a good idea to refactor the above into a function
     * that chooses the actual compatibility based upon options */
    
    m_cSPS.setPicWidthInLumaSamples         ( m_iSourceWidth      );
    m_cSPS.setPicHeightInLumaSamples        ( m_iSourceHeight     );
    m_cSPS.setConformanceWindow             ( m_conformanceWindow );
    m_cSPS.setMaxCUWidth    ( g_uiMaxCUWidth      );
    m_cSPS.setMaxCUHeight   ( g_uiMaxCUHeight     );
    m_cSPS.setMaxCUDepth    ( g_uiMaxCUDepth      );
    m_cSPS.setChromaFormatIdc( m_chromaFormatIDC);
    
    Int minCUSize = m_cSPS.getMaxCUWidth() >> ( m_cSPS.getMaxCUDepth()-g_uiAddCUDepth );
    Int log2MinCUSize = 0;
    while(minCUSize > 1)
    {
        minCUSize >>= 1;
        log2MinCUSize++;
    }
    
    m_cSPS.setLog2MinCodingBlockSize(log2MinCUSize);
    m_cSPS.setLog2DiffMaxMinCodingBlockSize(m_cSPS.getMaxCUDepth()-g_uiAddCUDepth-getMaxCUDepthOffset(m_cSPS.getChromaFormatIdc(), m_cSPS.getQuadtreeTULog2MinSize()));
    
    m_cSPS.setPCMLog2MinSize (m_uiPCMLog2MinSize);
    m_cSPS.setUsePCM        ( m_usePCM           );
    m_cSPS.setPCMLog2MaxSize( m_pcmLog2MaxSize  );
    
    m_cSPS.setQuadtreeTULog2MaxSize( m_uiQuadtreeTULog2MaxSize );
    m_cSPS.setQuadtreeTULog2MinSize( m_uiQuadtreeTULog2MinSize );
    m_cSPS.setQuadtreeTUMaxDepthInter( m_uiQuadtreeTUMaxDepthInter    );
    m_cSPS.setQuadtreeTUMaxDepthIntra( m_uiQuadtreeTUMaxDepthIntra    );
    
    m_cSPS.setTMVPFlagsPresent(false);
    
    m_cSPS.setMaxTrSize   ( 1 << m_uiQuadtreeTULog2MaxSize );
    
    Int i;
    
    for (i = 0; i < g_uiMaxCUDepth-g_uiAddCUDepth; i++ )
    {
        m_cSPS.setAMPAcc( i, m_useAMP );
        //m_cSPS.setAMPAcc( i, 1 );
    }
    
    m_cSPS.setUseAMP ( m_useAMP );
    
    for (i = g_uiMaxCUDepth-g_uiAddCUDepth; i < g_uiMaxCUDepth; i++ )
    {
        m_cSPS.setAMPAcc(i, 0);
    }
    
    
    for (UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++)
    {
        m_cSPS.setBitDepth    (ChannelType(channelType), g_bitDepth[channelType]            );
        m_cSPS.setQpBDOffset  (ChannelType(channelType), (6 * (g_bitDepth[channelType] - 8)));
        m_cSPS.setPCMBitDepth (ChannelType(channelType), g_PCMBitDepth[channelType]         );
    }
    
    m_cSPS.setUseExtendedPrecision(m_useExtendedPrecision);
    m_cSPS.setUseHighPrecisionPredictionWeighting(m_useHighPrecisionPredictionWeighting);
    
    m_cSPS.setUseSAO( m_bUseSAO );
    m_cSPS.setUseResidualRotation(m_useResidualRotation);
    m_cSPS.setUseSingleSignificanceMapContext(m_useSingleSignificanceMapContext);
    m_cSPS.setUseGolombRiceParameterAdaptation(m_useGolombRiceParameterAdaptation);
    m_cSPS.setAlignCABACBeforeBypass(m_alignCABACBeforeBypass);
    
    for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
    {
        m_cSPS.setUseResidualDPCM(RDPCMSignallingMode(signallingModeIndex), m_useResidualDPCM[signallingModeIndex]);
    }
    
    m_cSPS.setMaxTLayers( m_maxTempLayer );
    m_cSPS.setTemporalIdNestingFlag( ( m_maxTempLayer == 1 ) ? true : false );
    
    for ( i = 0; i < min(m_cSPS.getMaxTLayers(),(UInt) MAX_TLAYER); i++ )
    {
        m_cSPS.setMaxDecPicBuffering(m_maxDecPicBuffering[i], i);
        m_cSPS.setNumReorderPics(m_numReorderPics[i], i);
    }
    
    m_cSPS.setPCMFilterDisableFlag  ( m_bPCMFilterDisableFlag );
    m_cSPS.setDisableIntraReferenceSmoothing( m_disableIntraReferenceSmoothing );
    m_cSPS.setScalingListFlag ( (m_useScalingListId == 0) ? 0 : 1 );
    m_cSPS.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
    m_cSPS.setVuiParametersPresentFlag(getVuiParametersPresentFlag());
    
    if (m_cSPS.getVuiParametersPresentFlag())
    {
        TComVUI* pcVUI = m_cSPS.getVuiParameters();
        pcVUI->setAspectRatioInfoPresentFlag(getAspectRatioInfoPresentFlag());
        pcVUI->setAspectRatioIdc(getAspectRatioIdc());
        pcVUI->setSarWidth(getSarWidth());
        pcVUI->setSarHeight(getSarHeight());
        pcVUI->setOverscanInfoPresentFlag(getOverscanInfoPresentFlag());
        pcVUI->setOverscanAppropriateFlag(getOverscanAppropriateFlag());
        pcVUI->setVideoSignalTypePresentFlag(getVideoSignalTypePresentFlag());
        pcVUI->setVideoFormat(getVideoFormat());
        pcVUI->setVideoFullRangeFlag(getVideoFullRangeFlag());
        pcVUI->setColourDescriptionPresentFlag(getColourDescriptionPresentFlag());
        pcVUI->setColourPrimaries(getColourPrimaries());
        pcVUI->setTransferCharacteristics(getTransferCharacteristics());
        pcVUI->setMatrixCoefficients(getMatrixCoefficients());
        pcVUI->setChromaLocInfoPresentFlag(getChromaLocInfoPresentFlag());
        pcVUI->setChromaSampleLocTypeTopField(getChromaSampleLocTypeTopField());
        pcVUI->setChromaSampleLocTypeBottomField(getChromaSampleLocTypeBottomField());
        pcVUI->setNeutralChromaIndicationFlag(getNeutralChromaIndicationFlag());
        pcVUI->setDefaultDisplayWindow(getDefaultDisplayWindow());
        pcVUI->setFrameFieldInfoPresentFlag(getFrameFieldInfoPresentFlag());
        pcVUI->setFieldSeqFlag(false);
        pcVUI->setHrdParametersPresentFlag(false);
        pcVUI->getTimingInfo()->setPocProportionalToTimingFlag(getPocProportionalToTimingFlag());
        pcVUI->getTimingInfo()->setNumTicksPocDiffOneMinus1   (getNumTicksPocDiffOneMinus1()   );
        pcVUI->setBitstreamRestrictionFlag(getBitstreamRestrictionFlag());
        pcVUI->setTilesFixedStructureFlag(getTilesFixedStructureFlag());
        pcVUI->setMotionVectorsOverPicBoundariesFlag(getMotionVectorsOverPicBoundariesFlag());
        pcVUI->setMinSpatialSegmentationIdc(getMinSpatialSegmentationIdc());
        pcVUI->setMaxBytesPerPicDenom(getMaxBytesPerPicDenom());
        pcVUI->setMaxBitsPerMinCuDenom(getMaxBitsPerMinCuDenom());
        pcVUI->setLog2MaxMvLengthHorizontal(getLog2MaxMvLengthHorizontal());
        pcVUI->setLog2MaxMvLengthVertical(getLog2MaxMvLengthVertical());
    }
}

Void TEncTop::xInitPPS()
{
    m_cPPS.setConstrainedIntraPred( m_bUseConstrainedIntraPred );
    Bool bUseDQP = (getMaxCuDQPDepth() > 0)? true : false;
    
    if((getMaxDeltaQP() != 0 )|| getUseAdaptiveQP())
    {
        bUseDQP = true;
    }
    
    if (m_costMode==COST_SEQUENCE_LEVEL_LOSSLESS || m_costMode==COST_LOSSLESS_CODING) bUseDQP=false;
    
    if(bUseDQP)
    {
        m_cPPS.setUseDQP(true);
        m_cPPS.setMaxCuDQPDepth( m_iMaxCuDQPDepth );
        m_cPPS.setMinCuDQPSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuDQPDepth()) );
    }
    else
    {
        m_cPPS.setUseDQP(false);
        m_cPPS.setMaxCuDQPDepth( 0 );
        m_cPPS.setMinCuDQPSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuDQPDepth()) );
    }
    
    if ( m_maxCUChromaQpAdjustmentDepth >= 0 )
    {
        m_cPPS.setMaxCuChromaQpAdjDepth(m_maxCUChromaQpAdjustmentDepth);
        m_cPPS.setChromaQpAdjTableAt(1, 6, 6);
        /* todo, insert table entries from command line (NB, 0 should not be touched) */
    }
    else
    {
        m_cPPS.setMaxCuChromaQpAdjDepth(0);
        m_cPPS.clearChromaQpAdjTable();
    }
    
    if ( m_RCEnableRateControl )
    {
        m_cPPS.setUseDQP(true);
        m_cPPS.setMaxCuDQPDepth( 0 );
        m_cPPS.setMinCuDQPSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuDQPDepth()) );
    }
    
    m_cPPS.setMinCuChromaQpAdjSize( m_cPPS.getSPS()->getMaxCUWidth() >> ( m_cPPS.getMaxCuChromaQpAdjDepth()) );
    
    m_cPPS.setQpOffset(COMPONENT_Cb, m_chromaCbQpOffset );
    m_cPPS.setQpOffset(COMPONENT_Cr, m_chromaCrQpOffset );
    
    m_cPPS.setNumSubstreams(m_iWaveFrontSubstreams);
    m_cPPS.setEntropyCodingSyncEnabledFlag( m_iWaveFrontSynchro > 0 );
    m_cPPS.setTilesEnabledFlag( (m_iNumColumnsMinus1 > 0 || m_iNumRowsMinus1 > 0) );
    m_cPPS.setUseWP( m_useWeightedPred );
    m_cPPS.setWPBiPred( m_useWeightedBiPred );
    m_cPPS.setUseCrossComponentPrediction(m_useCrossComponentPrediction);
    m_cPPS.setSaoOffsetBitShift(CHANNEL_TYPE_LUMA,   m_saoOffsetBitShift[CHANNEL_TYPE_LUMA  ]);
    m_cPPS.setSaoOffsetBitShift(CHANNEL_TYPE_CHROMA, m_saoOffsetBitShift[CHANNEL_TYPE_CHROMA]);
    m_cPPS.setOutputFlagPresentFlag( false );
    m_cPPS.setSignHideFlag(getSignHideFlag());
    if ( getDeblockingFilterMetric() )
    {
        m_cPPS.setDeblockingFilterControlPresentFlag (true);
        m_cPPS.setDeblockingFilterOverrideEnabledFlag(true);
        m_cPPS.setPicDisableDeblockingFilterFlag(false);
        m_cPPS.setDeblockingFilterBetaOffsetDiv2(0);
        m_cPPS.setDeblockingFilterTcOffsetDiv2(0);
    }
    else
    {
        m_cPPS.setDeblockingFilterControlPresentFlag (m_DeblockingFilterControlPresent );
    }
    m_cPPS.setLog2ParallelMergeLevelMinus2   (m_log2ParallelMergeLevelMinus2 );
    m_cPPS.setCabacInitPresentFlag(CABAC_INIT_PRESENT_FLAG);
    m_cPPS.setLoopFilterAcrossSlicesEnabledFlag( m_bLFCrossSliceBoundaryFlag );
    
    Int histogram[MAX_NUM_REF + 1];
    for( Int i = 0; i <= MAX_NUM_REF; i++ )
    {
        histogram[i]=0;
    }
    for( Int i = 0; i < getGOPSize(); i++)
    {
        assert(getGOPEntry(i).m_numRefPicsActive >= 0 && getGOPEntry(i).m_numRefPicsActive <= MAX_NUM_REF);
        histogram[getGOPEntry(i).m_numRefPicsActive]++;
    }
    
    Int maxHist=-1;
    Int bestPos=0;
    for( Int i = 0; i <= MAX_NUM_REF; i++ )
    {
        if(histogram[i]>maxHist)
        {
            maxHist=histogram[i];
            bestPos=i;
        }
    }
    assert(bestPos <= 15);
    m_cPPS.setNumRefIdxL0DefaultActive(bestPos);
    m_cPPS.setNumRefIdxL1DefaultActive(bestPos);
    m_cPPS.setTransquantBypassEnableFlag(getTransquantBypassEnableFlag());
    m_cPPS.setUseTransformSkip( m_useTransformSkip );
    m_cPPS.setTransformSkipLog2MaxSize( m_transformSkipLog2MaxSize  );
    
    if (m_sliceSegmentMode)
    {
        m_cPPS.setDependentSliceSegmentsEnabledFlag( true );
    }
    
    if( m_cPPS.getDependentSliceSegmentsEnabledFlag() )
    {
        Int NumCtx = m_cPPS.getEntropyCodingSyncEnabledFlag()?2:1;
        m_cSliceEncoder.initCtxMem( NumCtx );
        for ( UInt st = 0; st < NumCtx; st++ )
        {
            TEncSbac* ctx = NULL;
            ctx = new TEncSbac;
            ctx->init( &m_cBinCoderCABAC );
            m_cSliceEncoder.setCtxMem( ctx, st );
        }
    }
}

//Function for initializing m_RPSList, a list of TComReferencePictureSet, based on the GOPEntry objects read from the config file.
Void TEncTop::xInitRPS(Bool isFieldCoding)
{
    
    TComReferencePictureSet*      rps;
    
    // Hossam: Destroys the old RPS list and creates a new one with the new size
    // getGopsize + extra + 1
    m_cSPS.createRPSList(getGOPSize() + m_extraRPSs + 1);
    TComRPSList* rpsList = m_cSPS.getRPSList();
    
    // Outer loop
    for( Int i = 0; i < getGOPSize()+m_extraRPSs; i++)
    {
        GOPEntry ge = getGOPEntry(i);
        rps = rpsList->getReferencePictureSet(i);
        rps->setNumberOfPictures(ge.m_numRefPics);
        rps->setNumRefIdc(ge.m_numRefIdc);
        Int numNeg = 0;
        Int numPos = 0;
        
//        cout << "\n" << endl;
//        cout << "Group of picture Entry POC " << ge.m_POC  << ", i " << i << endl;
//        cout << "Version II ge.m_numRefPics: " << ge.m_numRefPics << endl;
//        cout << "Version II ge.m_numRefIdc: " << ge.m_numRefIdc << endl;

        
        
        // Inner loop
        for( Int j = 0; j < ge.m_numRefPics; j++)
        {
            rps->setDeltaPOC(j,ge.m_referencePics[j]);
            rps->setUsed(j,ge.m_usedByCurrPic[j]);
            if(ge.m_referencePics[j]>0)
            {
                numPos++;
            }
            else
            {
                numNeg++;
            }
        }
        rps->setNumberOfNegativePictures(numNeg);
        rps->setNumberOfPositivePictures(numPos);
        
//        cout << "Print delta POC CFG generated: ";
//        rps -> printDeltaPOC();
        
        
        // handle inter RPS intialization from the config file.
#if AUTO_INTER_RPS
        rps->setInterRPSPrediction(ge.m_interRPSPrediction > 0);  // not very clean, converting anything > 0 to true.
        rps->setDeltaRIdxMinus1(0);                               // index to the Reference RPS is always the previous one.
        TComReferencePictureSet*     RPSRef = rpsList->getReferencePictureSet(i-1);  // get the reference RPS
        
        if (ge.m_interRPSPrediction == 2)  // Automatic generation of the inter RPS idc based on the RIdx provided.
        {
            Int deltaRPS = getGOPEntry(i-1).m_POC - ge.m_POC;  // the ref POC - current POC
            Int numRefDeltaPOC = RPSRef->getNumberOfPictures();
            
            rps->setDeltaRPS(deltaRPS);           // set delta RPS
            rps->setNumRefIdc(numRefDeltaPOC+1);  // set the numRefIdc to the number of pictures in the reference RPS + 1.
            Int count=0;
            for (Int j = 0; j <= numRefDeltaPOC; j++ ) // cycle through pics in reference RPS.
            {
                Int RefDeltaPOC = (j<numRefDeltaPOC)? RPSRef->getDeltaPOC(j): 0;  // if it is the last decoded picture, set RefDeltaPOC = 0
                rps->setRefIdc(j, 0);
                for (Int k = 0; k < rps->getNumberOfPictures(); k++ )  // cycle through pics in current RPS.
                {
                    if (rps->getDeltaPOC(k) == ( RefDeltaPOC + deltaRPS))  // if the current RPS has a same picture as the reference RPS.
                    {
                        rps->setRefIdc(j, (rps->getUsed(k)?1:2));
                        count++;
                        break;
                    }
                }
            }
            if (count != rps->getNumberOfPictures())
            {
                printf("Warning: Unable fully predict all delta POCs using the reference RPS index given in the config file.  Setting Inter RPS to false for this RPS.\n");
                rps->setInterRPSPrediction(0);
            }
        }
        else if (ge.m_interRPSPrediction == 1)  // inter RPS idc based on the RefIdc values provided in config file.
        {
            rps->setDeltaRPS(ge.m_deltaRPS);
            rps->setNumRefIdc(ge.m_numRefIdc);
            for (Int j = 0; j < ge.m_numRefIdc; j++ )
            {
                rps->setRefIdc(j, ge.m_refIdc[j]);
            }
#if WRITE_BACK
            // the folowing code overwrite the deltaPOC and Used by current values read from the config file with the ones
            // computed from the RefIdc.  A warning is printed if they are not identical.
            numNeg = 0;
            numPos = 0;
            TComReferencePictureSet      RPSTemp;  // temporary variable
            
            for (Int j = 0; j < ge.m_numRefIdc; j++ )
            {
                if (ge.m_refIdc[j])
                {
                    Int deltaPOC = ge.m_deltaRPS + ((j < RPSRef->getNumberOfPictures())? RPSRef->getDeltaPOC(j) : 0);
                    RPSTemp.setDeltaPOC((numNeg+numPos),deltaPOC);
                    RPSTemp.setUsed((numNeg+numPos),ge.m_refIdc[j]==1?1:0);
                    
                    if (deltaPOC<0)
                    {
                        numNeg++;
                    }
                    else
                    {
                        numPos++;
                    }
                }
            }
//            cout << "Print delta POC C++ generated: ";
//            RPSTemp.printDeltaPOC();
//            
//            // Hossam: warning:
//            cout <<         ge.m_POC <<  ") Before WARNING -ve RESOLVED: calc: " << numNeg  << ", rps: " << rps->getNumberOfNegativePictures() << endl;
//            cout <<         ge.m_POC << ") Before WARNING +ve RESOLVED: calc: " << numPos  << ", rps: " << rps->getNumberOfPositivePictures() << endl;
            
            if (numNeg != rps->getNumberOfNegativePictures())
            {
                printf("Warning: number of negative pictures in RPS is different between intra and inter RPS specified in the config file.\n");
                rps->setNumberOfNegativePictures(numNeg);
                rps->setNumberOfPictures(numNeg+numPos);
            }
            if (numPos != rps->getNumberOfPositivePictures())
            {
                printf("Warning: number of positive pictures in RPS is different between intra and inter RPS specified in the config file.\n");
                rps->setNumberOfPositivePictures(numPos);
                rps->setNumberOfPictures(numNeg+numPos);
            }
            RPSTemp.setNumberOfPictures(numNeg+numPos);
            RPSTemp.setNumberOfNegativePictures(numNeg);
            RPSTemp.sortDeltaPOC();     // sort the created delta POC before comparing
            // check if Delta POC and Used are the same
            // print warning if they are not.
            for (Int j = 0; j < ge.m_numRefIdc; j++ )
            {
                if (RPSTemp.getDeltaPOC(j) != rps->getDeltaPOC(j))
                {
                    printf("Warning: delta POC is different between intra RPS and inter RPS specified in the config file.\n");
                    
//                    cout << "2nd Debug WARNING Entry#: " << j  << ", m_numRefIdc: " << ge.m_numRefIdc << ", Config delta: " << rps->getDeltaPOC(j) << ", RPSTemp.getDeltaPOC(j): " << RPSTemp.getDeltaPOC(j) << endl;
                    rps->setDeltaPOC(j,RPSTemp.getDeltaPOC(j));
                }
                if (RPSTemp.getUsed(j) != rps->getUsed(j))
                {
                    printf("Warning: Used by Current in RPS is different between intra and inter RPS specified in the config file.\n");
                    rps->setUsed(j,RPSTemp.getUsed(j));
                }
            }
#endif
        }
#else
        rps->setInterRPSPrediction(ge.m_interRPSPrediction);
        if (ge.m_interRPSPrediction)
        {
            rps->setDeltaRIdxMinus1(0);
            rps->setDeltaRPS(ge.m_deltaRPS);
            rps->setNumRefIdc(ge.m_numRefIdc);
            for (Int j = 0; j < ge.m_numRefIdc; j++ )
            {
                rps->setRefIdc(j, ge.m_refIdc[j]);
            }
#if WRITE_BACK
            // the folowing code overwrite the deltaPOC and Used by current values read from the config file with the ones
            // computed from the RefIdc.  This is not necessary if both are identical. Currently there is no check to see if they are identical.
            numNeg = 0;
            numPos = 0;
            TComReferencePictureSet*     RPSRef = m_RPSList.getReferencePictureSet(i-1);
            
            for (Int j = 0; j < ge.m_numRefIdc; j++ )
            {
                if (ge.m_refIdc[j])
                {
                    Int deltaPOC = ge.m_deltaRPS + ((j < RPSRef->getNumberOfPictures())? RPSRef->getDeltaPOC(j) : 0);
                    rps->setDeltaPOC((numNeg+numPos),deltaPOC);
                    rps->setUsed((numNeg+numPos),ge.m_refIdc[j]==1?1:0);
                    if (deltaPOC<0)
                    {
                        numNeg++;
                    }
                    else
                    {
                        numPos++;
                    }
                }
            }
            rps->setNumberOfNegativePictures(numNeg);
            rps->setNumberOfPositivePictures(numPos);
            rps->sortDeltaPOC();
#endif
        }
#endif //INTER_RPS_AUTO
    }
    //In case of field coding, we need to set special parameters for the first bottom field of the sequence, since it is not specified in the cfg file.
    //The position = GOPSize + extraRPSs which is (a priori) unused is reserved for this field in the RPS.
    if (isFieldCoding)
    {
        rps = rpsList->getReferencePictureSet(getGOPSize()+m_extraRPSs);
        rps->setNumberOfPictures(1);
        rps->setNumberOfNegativePictures(1);
        rps->setNumberOfPositivePictures(0);
        rps->setNumberOfLongtermPictures(0);
        rps->setDeltaPOC(0,-1);
        rps->setPOC(0,0);
        rps->setUsed(0,true);
        rps->setInterRPSPrediction(false);
        rps->setDeltaRIdxMinus1(0);
        rps->setDeltaRPS(0);
        rps->setNumRefIdc(0);
    }
    
//    cout << "InitRPS DONEeeeeeeeeeeeeeeeeeeeee ";
    
}


// This is a function that
// determines what Reference Picture Set to use
// for a specific slice (with POC = POCCurr)
Void TEncTop::selectReferencePictureSet(TComSlice* slice, Int POCCurr, Int GOPid )
{
    
    cout << "selectReferencePictureSet selectReferencePictureSet GOP # " << GOPid << endl;
        cout << "SEG FAULT " << endl;
    slice->setRPSidx(GOPid);
        cout << "SEG FAULT " << endl;
    for(Int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
    {
        if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
        {
            Int POCIndex = POCCurr%m_uiIntraPeriod;
            if(POCIndex == 0)
            {
                POCIndex = m_uiIntraPeriod;
            }
            if(POCIndex == m_GOPList[extraNum].m_POC)
            {
                slice->setRPSidx(extraNum);
            }
        }
        else
        {
            if(POCCurr==m_GOPList[extraNum].m_POC)
            {
                slice->setRPSidx(extraNum);
            }
        }
    }
    
    if(POCCurr == 1 && slice->getPic()->isField())
    {
        slice->setRPSidx(m_iGOPSize+m_extraRPSs);
    }
    
    
    
    //     cout << "*****SELECT selectReferencePictureSet RPS Before " << endl;
    //    slice->getRPS()->printDeltaPOC();
    
    //    if(slice->getPOC() == 6)
    //        slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(0));
    //    else
    //        if(slice->getPOC() == 7)
    //            slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(1));
    //        else
    //            if(slice->getPOC() == 8)
    //                slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(2));
    
    cout << "SEG FAULT " << endl;
    
    slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(slice->getRPSidx()));
    
        cout << "SEG FAULT final" << endl;
    slice->getRPS()->setNumberOfPictures(slice->getRPS()->getNumberOfNegativePictures()+slice->getRPS()->getNumberOfPositivePictures());
      cout << "SEG FAULT final 2" << endl;
    
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 0" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(0)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 1" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(1)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 2" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(2)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 3" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(3)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 4" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(4)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 5" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(5)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet Set # 6" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(6)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 7" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(7)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 8" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(8)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 9" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(9)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 10" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(10)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 11" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(11)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 12" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(12)->printDeltaPOC();
//    cout << "SELECT selectReferencePictureSet selectReferencePictureSet GOP # 13" << endl;
//    getSPS()->getRPSList()->getReferencePictureSet(13)->printDeltaPOC();
//    cout << "*****SELECT selectReferencePictureSet selectReferencePictureSet INDEX # "<< slice->getRPSidx() << endl;
//    cout << "\n " << endl;
//
//    cout << "*****SELECT selectReferencePictureSet RPS Before " << endl;
//    slice->getRPS()->printDeltaPOC();
    
//    if (slice->getPOC()==6) {
//        slice->setRPSidx(0);
//    }
//    else if(slice->getPOC()==7)
//    {
//        slice->setRPSidx(1);
//    }
//    else if(slice->getPOC()==8)
//    {
//        slice->setRPSidx(2);
//    }
//    
//    if (slice->getPOC()==9) {
//        slice->setRPSidx(3);
//    }
//    else if(slice->getPOC()==10)
//    {
//        slice->setRPSidx(0);
//    }
//    else if(slice->getPOC()==11)
//    {
//        slice->setRPSidx(1);
//    }
//    else if(slice->getPOC()==12)
//    {
//        slice->setRPSidx(2);
//    }
    
//    cout << "*****SELECT selectReferencePictureSet RPS AFTER " << endl;
//    slice->getRPS()->printDeltaPOC();
    
}



Int TEncTop::getCaseNumber(Int POCCurr)
{
    Int caseNumber;
    Int pocHelper = POCCurr - 1;
    
    if(pocHelper % 4 == 0)
    {
        caseNumber = 4;
    }
    else if(pocHelper % 4 == 1)
    {
        caseNumber = 1;
    }
    else if(pocHelper % 4 == 2)
    {
        caseNumber = 2;
    }
    else if(pocHelper % 4 == 3)
    {
        caseNumber = 3;
    }
    else
    {
        caseNumber = -1;
        cout << "ERROR ERROR ERROR In get Case Number() " << endl;
    }
    
    cout << "ERROR ERROR XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  Returning a hardcoded value in getCASENUMBER " << endl;
//    return caseNumber;
      return 2;
}

Int TEncTop::getSelectIdx(Int caseNumber, Bool isSmooth)
{
    Int select = -1;
    
    switch (caseNumber) {
        case 4: // checked
            select = 5;
            break;
        case 1: // checked
            select = isSmooth? 4:5;
            break;
        case 2:
            select = isSmooth? 4:5;
            break;
        case 3:
            select = isSmooth? 4:5;
            break;
            
        default:
            
            cout << "ERROR ERROR in getSelectIdx() " << endl;
            break;
    }
    
    return select;
}


// Hossam
// This is a function that
// determines what Reference Picture Set to use
// for a specific slice (with POC = POCCurr)
static Int withinState = 0;
static Int select = 4;
static Int width = 12; // GOP size + extra size = Gop list size
static Int state = -1;
static const Int emptyReference = 13;
static Bool isFstTime = true;
//Void TEncTop::selectReferencePictureSetNew(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSc )
Void TEncTop::selectReferencePictureSetNew(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSc, Bool isSmooth )

{
    
//    cout << "selectReferencePictureSet NEW MODIFY selectReferencePictureSet <state, GOP> # " << state <<  ", " << GOPid  << ", " << select << endl;
    
//    for (int z = 0; z < m_extraRPSs+m_iGOPSize; z++) {
//        
//        cout << "Gop entry " << m_GOPList[z].m_POC << endl;
//    }
    
//    if (state == -1 || (state == 0 && isFstTime) ) {
        if (state == -1) {
    
        slice->setRPSidx(GOPid);
        
        
        for(Int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
        {
            if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
            {
                Int POCIndex = POCCurr%m_uiIntraPeriod;
                if(POCIndex == 0)
                {
                    POCIndex = m_uiIntraPeriod;
                }
                if(POCIndex == m_GOPList[extraNum].m_POC)
                {
                    slice->setRPSidx(extraNum);
                }
            }
            else
            {
                if(POCCurr==m_GOPList[extraNum].m_POC)
                {
                    slice->setRPSidx(extraNum);
                }
            }
        }
        
        if(POCCurr == 1 && slice->getPic()->isField())
        {
            slice->setRPSidx(m_iGOPSize+m_extraRPSs);
        }
        
//        if(state == 0)
//        {
//            if(isFstTime)
//                isFstTime = false;
//        }
        
    }
    
    else
    {

        
#if DELAY_SC_I==1
        
//        if(POCCurr == lastSc)
//        {
//            slice->setRPSidx(4);// previous frame only!
//        }
        // Conceptual I frame
//   else if(POCCurr == lastSc + 1)

        // I'm here the modify before encoding and I'm not first time
        if(POCCurr == lastSc)
        {
            // Choose the one designated to you!
            slice -> setRPSidx(emptyReference);
            
            
            
        }
        else if(POCCurr == lastSc + 1)
        {
//            slice->setRPSidx(emptyReference);
            slice->setRPSidx(4);
            
            Int caseNumber = getCaseNumber(POCCurr);
            select = getSelectIdx(caseNumber, isSmooth);
            
//            select = isSmooth? 4:5;
//            cout << "SELECTTTTTTT22222:    XXXXXX " << select << endl;


//            select = isSmooth? 4:5;
//            select = 4;
        }
        else
        {
            if(state == 4)
            {
                slice->setRPSidx(select);

                select = select + 1;
//                if(POCCurr != lastSc)
//                    select = select + 1;
                
                if(select > 7)
                {
                    state = 1;
                }

            }
            else if(state == 1)
            {
                slice->setRPSidx(select);

                    select = select + 1;
//                if(POCCurr != lastSc)
//                    select = select + 1;
                
                if (select > 11) {
                    state = 2;
                }
            }
            else if(state == 2)
            {
                slice->setRPSidx(select);

                   select = select + 1;
//                if(POCCurr != lastSc)
//                    select = select + 1;

                if(select > 12)
                    select = 1;
                
                if(select > 3)
                {
                    select = 0;
                    state = 3;
                }
            }
            else if(state == 3)
            {
                slice->setRPSidx(select);
                
                select = (select + 1) % 4;// Gop size
//                 select = select + 1;
//                if(POCCurr != lastSc)
//                    select = select + 1;
            }
        }// else of SC and other frames

#else
        // Update the within state
        // Maybe bug when we go to 8 frames GOP size
        
        //        if(state !=0)
        if (lastSc == POCCurr) {
            //            slice->setRPSidx(0);
            slice->setRPSidx(emptyReference);
            withinState = 0;// reset the withinState
            select = 4;
            //                        select = 8;
            state = 0;
        }
        else{
            
            // Coming from a SC
            // Next four frames
            if (state == 0) {
                
//                                cout << "Adeeela Select: " << select << endl;
                
                slice->setRPSidx(select);
                select = select + 1;
                
                if(select > 7)
                {
                    state = 1;
                }
                
            }
            else if(state == 1)
            {
//                               cout << "Adeeela Select: " << select << endl;
                slice->setRPSidx(select);
                select = select + 1;
                
                if (select > 11) {
                    state = 2;
                }
            }
            else if(state == 2)
            {
//                                cout << "Adeeela Select: " << select << endl;
                slice->setRPSidx(select);
                select = select + 1;
                
                if(select > 12)
                    select = 1;
                
                if(select > 3)
                {
                    select = 0;
                    state = 3;
                }
            }
            else if(state == 3)
            {
//                               cout << "Adeeela Select: " << select << endl;
                slice->setRPSidx(select);
                select = (select + 1)%4;// Gop size
            }
            
        }
        
#endif
        
    }// SC existed, Adjust the references!
    
   

    slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(slice->getRPSidx()));
    slice->getRPS()->setNumberOfPictures(slice->getRPS()->getNumberOfNegativePictures()+slice->getRPS()->getNumberOfPositivePictures());
    
    // Hossam: Scene change
//    return state;
    
//    cout << "*****SELECT selectReferencePictureSet MODIFY RPS AFTER " << endl;
//    slice->getRPS()->printDeltaPOC();
    
}


static Int select2 = 4;
//static Int width2 = 12; // GOP size + extra size = Gop list size
static Int state2 = 0;
static Bool isFstTime2 = true;
Void TEncTop::selectReferencePictureSetNewAttempt(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSc, Bool isSmooth )
{
    
//        cout << "selectReferencePictureSet NEW ATTEMPT selectReferencePictureSet <state, GOP> # " << state2 <<  ", " << GOPid << endl;
//    cout << "selectReferencePictureSet NEW ATTEMPT selectReferencePictureSet <state, GOP> # " << state2  <<  ", " << GOPid  << ", " << select2 << endl;
    
//    cout << "selectReferencePictureSet NEW ATTEMPT selectReferencePictureSet <state, GOP> # " << state  <<  ", " << GOPid  << ", state2: " << state2 <<  ", Select: " << select2 << endl;


//
//        for (int z = 0; z < m_extraRPSs+m_iGOPSize; z++) {
//    
//            cout << "Gop entry " << m_GOPList[z].m_POC << endl;
//        }

#if DELAY_SC_I==1
    if (state == -1 || (state == 0 && isFstTime2)) {
#else
    if (state == -1) {
#endif
        
        slice->setRPSidx(GOPid);
        
        for(Int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
        {
            if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
            {
                Int POCIndex = POCCurr%m_uiIntraPeriod;
                if(POCIndex == 0)
                {
                    POCIndex = m_uiIntraPeriod;
                }
                if(POCIndex == m_GOPList[extraNum].m_POC)
                {
                    slice->setRPSidx(extraNum);
                }
            }
            else
            {
                if(POCCurr==m_GOPList[extraNum].m_POC)
                {
                    slice->setRPSidx(extraNum);
                }
            }
        }
        
        if(POCCurr == 1 && slice->getPic()->isField())
        {
            slice->setRPSidx(m_iGOPSize+m_extraRPSs);
        }

#if DELAY_SC_I==1
        if(state==0)
        {
            if(isFstTime2)
                isFstTime2 = false;
        }
#endif
        
    }//end first if
    
    else
    {
        
        
        // Update the within state
        // Maybe bug when we go to 8 frames GOP size

        //        if(state !=0)
//       if (lastSc + 1 == POCCurr) {
//      if (POCCurr == lastSc + 1) {

#if DELAY_SC_I==1        
        // Do what you are doing 
        if (POCCurr == lastSc && POCCurr % 4 != 0) {
            slice->setRPSidx(select2);
        }
        else if (POCCurr == lastSc + 1) {
            //            slice->setRPSidx(0);
//            slice->setRPSidx(emptyReference);
            slice->setRPSidx(4);// choose previous one to calculate Yc
            
            
            Int caseNumber = getCaseNumber(POCCurr);
            select2        = getSelectIdx(caseNumber, isSmooth);
            
//            select2 = isSmooth? 4:5;
//            select2 = 4;
            state2 = 0;
            
//            cout << "SELECTTTTTTT1111:    XXXXXX " << select2 << endl;

        }
        else{
            
            // Coming from a SC
            // Next four frames
            if (state2 == 0) {
                
                //                cout << "Adeeela Select: " << select2 << endl;
                
                slice->setRPSidx(select2);
                select2 = select2 + 1;
                
                if(select2 > 7)
                {
                    state2 = 1;
                }
                
            }
            else if(state2 == 1)
            {
                //                cout << "Adeeela Select: " << select2 << endl;
                slice->setRPSidx(select2);
                select2 = select2 + 1;
                
                if (select2 > 11) {
                    state2 = 2;
                }
            }
            else if(state2 == 2)
            {
                //                cout << "Adeeela Select2 attempt before: " << select2 << endl;
                slice->setRPSidx(select2);
                select2 = select2 + 1;
                
                //             cout << "Adeeela Select2 attempt after: " << select2 << endl;
                if(select2 > 12)
                    //                if(select2 >= width)
                {
                    select2 = 1;
                }
                
                if(select2 > 3)
                {
                    select2 = 0;
                    state2 = 3;
                }
            }
            else if(state2 == 3)
            {
                //                cout << "Adeeela Select: " << select2 << endl;
                slice->setRPSidx(select2);
                select2 = (select2 + 1)%4;// Gop size
            }
            
        }
        
        
    }

#else
        // Update the within state
        // Maybe bug when we go to 8 frames GOP size
        
        //  if(state !=0)
        if (lastSc == POCCurr) {
            //            slice->setRPSidx(0);
            slice->setRPSidx(emptyReference);
            select2 = 4;
            state2 = 0;
            
//            cout << "SELECTTTTTTT1111:    XXXXXX " << select2 << endl;
        }
        else{
            
            // Coming from a SC
            // Next four frames
            if (state2 == 0) {
                
//                                cout << "Adeeela Select: " << select2 << endl;
                
                slice->setRPSidx(select2);
                select2 = select2 + 1;
                
                if(select2 > 7)
                {
                    state2 = 1;
                }
                
//                cout << "SELECTTTTTTT2222:    XXXXXX " << select2 << endl;
                
            }
            else if(state2 == 1)
            {
//                                cout << "Adeeela Select: " << select2 << endl;
                slice->setRPSidx(select2);
                select2 = select2 + 1;
                
                if (select2 > 11) {
                    state2 = 2;
                }
            }
            else if(state2 == 2)
            {
//                                cout << "Adeeela Select2 attempt before: " << select2 << endl;
                slice->setRPSidx(select2);
                select2 = select2 + 1;
                
                //             cout << "Adeeela Select2 attempt after: " << select2 << endl;
                if(select2 > 12)
                    //                if(select2 >= width)
                {
                    select2 = 1;
                }
                
                if(select2 > 3)
                {
                    select2 = 0;
                    state2 = 3;
                }
            }
            else if(state2 == 3)
            {
//                               cout << "Adeeela Select: " << select2 << endl;
                slice->setRPSidx(select2);
                select2 = (select2 + 1)%4;// Gop size
            }
            
        }
        
        
    }
    
#endif
    
#if !IS_ENABLE_ACTUAL_LD
    // Hossam: Change the referencing to only previous
    if (state == -1)
    {
        if(POCCurr == 0)
            slice->setRPSidx(emptyReference);
        else
            slice->setRPSidx(4);
        cout << "Yes I KNOW THAT I AM DOING ONLY PREV REFERENCE FOR ANALYSIS: find me in Top ;) " << endl;
    }
#endif
    
//            cout << "GOP Id RPS " << slice->getRPSidx() << endl;
    slice->setRPS(getSPS()->getRPSList()->getReferencePictureSet(slice->getRPSidx()));
    slice->getRPS()->setNumberOfPictures(slice->getRPS()->getNumberOfNegativePictures()+slice->getRPS()->getNumberOfPositivePictures());
    
    
//        cout << "*****SELECT ATTEMPT selectReferencePictureSet RPS AFTER " << endl;
//        slice->getRPS()->printDeltaPOC();
    
}

// Reset the Select process
Void TEncTop::resetSelectAttemptProcess(Bool isSmooth, Int POCCurr)
{
    
//    cout << "Attempt process is RESETTTTTTTTT " << endl;

    state2 = 0;
    
    Int caseNumber = getCaseNumber(POCCurr);

    
    // Edited this part to make sure that the code is working, offset calculator will be used later
    select2 = isSmooth? 4:5;
    
    // Remove the use of offset calculator
//    select2 = getSelectIdx(caseNumber, isSmooth);
}

Int TEncTop::getReferencePictureSetIdxForSOP(TComSlice* slice, Int POCCurr, Int GOPid )
{
    Int rpsIdx = GOPid;
    
    for(Int extraNum=m_iGOPSize; extraNum<m_extraRPSs+m_iGOPSize; extraNum++)
    {
        if(m_uiIntraPeriod > 0 && getDecodingRefreshType() > 0)
        {
            Int POCIndex = POCCurr%m_uiIntraPeriod;
            if(POCIndex == 0)
            {
                POCIndex = m_uiIntraPeriod;
            }
            if(POCIndex == m_GOPList[extraNum].m_POC)
            {
                rpsIdx = extraNum;
            }
        }
        else
        {
            if(POCCurr==m_GOPList[extraNum].m_POC)
            {
                rpsIdx = extraNum;
            }
        }
    }
    
    return rpsIdx;
}

Void  TEncTop::xInitPPSforTiles()
{
    m_cPPS.setTileUniformSpacingFlag( m_tileUniformSpacingFlag );
    m_cPPS.setNumTileColumnsMinus1( m_iNumColumnsMinus1 );
    m_cPPS.setNumTileRowsMinus1( m_iNumRowsMinus1 );
    if( !m_tileUniformSpacingFlag )
    {
        m_cPPS.setTileColumnWidth( m_tileColumnWidth );
        m_cPPS.setTileRowHeight( m_tileRowHeight );
    }
    m_cPPS.setLoopFilterAcrossTilesEnabledFlag( m_loopFilterAcrossTilesEnabledFlag );
    
    // # substreams is "per tile" when tiles are independent.
    if (m_iWaveFrontSynchro )
    {
        m_cPPS.setNumSubstreams(m_iWaveFrontSubstreams * (m_iNumColumnsMinus1+1));
    }
}

Void  TEncCfg::xCheckGSParameters()
{
    
    Int   iWidthInCU = ( m_iSourceWidth%g_uiMaxCUWidth ) ? m_iSourceWidth/g_uiMaxCUWidth + 1 : m_iSourceWidth/g_uiMaxCUWidth;
    Int   iHeightInCU = ( m_iSourceHeight%g_uiMaxCUHeight ) ? m_iSourceHeight/g_uiMaxCUHeight + 1 : m_iSourceHeight/g_uiMaxCUHeight;
    UInt  uiCummulativeColumnWidth = 0;
    UInt  uiCummulativeRowHeight = 0;
    
    //check the column relative parameters
    if( m_iNumColumnsMinus1 >= (1<<(LOG2_MAX_NUM_COLUMNS_MINUS1+1)) )
    {
        printf( "The number of columns is larger than the maximum allowed number of columns.\n" );
        exit( EXIT_FAILURE );
    }
    
    if( m_iNumColumnsMinus1 >= iWidthInCU )
    {
        printf( "The current picture can not have so many columns.\n" );
        exit( EXIT_FAILURE );
    }
    
    if( m_iNumColumnsMinus1 && !m_tileUniformSpacingFlag )
    {
        for(Int i=0; i<m_iNumColumnsMinus1; i++)
        {
            uiCummulativeColumnWidth += m_tileColumnWidth[i];
        }
        
        if( uiCummulativeColumnWidth >= iWidthInCU )
        {
            printf( "The width of the column is too large.\n" );
            exit( EXIT_FAILURE );
        }
    }
    
    //check the row relative parameters
    if( m_iNumRowsMinus1 >= (1<<(LOG2_MAX_NUM_ROWS_MINUS1+1)) )
    {
        printf( "The number of rows is larger than the maximum allowed number of rows.\n" );
        exit( EXIT_FAILURE );
    }
    
    if( m_iNumRowsMinus1 >= iHeightInCU )
    {
        printf( "The current picture can not have so many rows.\n" );
        exit( EXIT_FAILURE );
    }
    
    if( m_iNumRowsMinus1 && !m_tileUniformSpacingFlag )
    {
        for(Int i=0; i<m_iNumRowsMinus1; i++)
            uiCummulativeRowHeight += m_tileRowHeight[i];
        
        if( uiCummulativeRowHeight >= iHeightInCU )
        {
            printf( "The height of the row is too large.\n" );
            exit( EXIT_FAILURE );
        }
    }
}
//! \}
