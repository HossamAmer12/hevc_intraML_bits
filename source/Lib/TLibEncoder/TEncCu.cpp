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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"
#include "TLibCommon/Debug.h"

#include <cmath>
#include <algorithm>
using namespace std;


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uiTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight, ChromaFormat chromaFormat)
{
  Int i;

  m_uhTotalDepth   = uhTotalDepth + 1;
  m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
  m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];

  m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];

  // Hossam: Scene change
   // Hossam: Create the residual buffers
    scTempResidual = new TComYuv*[m_uhTotalDepth-1];
    scBestResidual = new TComYuv*[m_uhTotalDepth-1];
    
    
  UInt uiNumPartitions;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );
    UInt uiWidth  = uiMaxWidth  >> i;
    UInt uiHeight = uiMaxHeight >> i;

      
//      cout << "Create the residuals buffer " << uiWidth << ", " << uiHeight << endl;
    m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( chromaFormat, uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( chromaFormat, uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );

    m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);

    m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);

    m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight, chromaFormat);
      
      // Hossam: Scene change
      // Hossam: Create the residual buffers
//      scTempResidual[i] = new TComYuv; scTempResidual[i]->create(uiWidth, uiHeight, chromaFormat);
//      scBestResidual[i] = new TComYuv; scBestResidual[i]->create(uiWidth, uiHeight, chromaFormat);
      
  }

    //Hossam: Scene change
    for( i=0 ; i<m_uhTotalDepth-1 ; i++)
    {
        UInt uiWidth  = uiMaxWidth  >> 0;
        UInt uiHeight = uiMaxHeight >> 0;
      
        // Hossam: Scene change
        // Hossam: Create the residual buffers
        scTempResidual[i] = new TComYuv; scTempResidual[i]->create(uiWidth, uiHeight, chromaFormat);
        scBestResidual[i] = new TComYuv; scBestResidual[i]->create(uiWidth, uiHeight, chromaFormat);
        
    }
    
    
    // Hossam: Create the temp and best residual buffers --> Not the best practice to access it from outside thu XXXX
//    Int shift = SC_MAX_DEPTH;
//    m_pcPredSearch->m_tempResidual  = new TComYuv; m_pcPredSearch->m_tempResidual->create(uiWidth, uiHeight, chromaFormat);
    
 

  m_bEncodeDQP          = false;
  m_CodeChromaQpAdjFlag = false;
  m_ChromaQpAdjIdc      = 0;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );

  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );
}

Void TEncCu::destroy()
{
  Int i;

  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    if(m_ppcBestCU[i])
    {
      m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
    }
    if(m_ppcTempCU[i])
    {
      m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
    }
    if(m_ppcPredYuvBest[i])
    {
      m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
    }
    if(m_ppcResiYuvBest[i])
    {
      m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
    }
    if(m_ppcRecoYuvBest[i])
    {
      m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
    }
    if(m_ppcPredYuvTemp[i])
    {
      m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
    }
    if(m_ppcResiYuvTemp[i])
    {
      m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
    }
    if(m_ppcRecoYuvTemp[i])
    {
      m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
    }
    if(m_ppcOrigYuv[i])
    {
      m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
    }
    
      // Hossam: Scene change
      // Destroy the temp -- Best residual buffers
      // Temp:
      if(scTempResidual[i])
      {
          scTempResidual[i]->destroy(); delete scTempResidual[i]; scTempResidual[i] = NULL;
      }
      
      // Best:
      if(scBestResidual[i])
      {
          scBestResidual[i]->destroy(); delete scBestResidual[i]; scBestResidual[i] = NULL;
      }
      
  }
  if(m_ppcBestCU)
  {
    delete [] m_ppcBestCU;
    m_ppcBestCU = NULL;
  }
  if(m_ppcTempCU)
  {
    delete [] m_ppcTempCU;
    m_ppcTempCU = NULL;
  }

  if(m_ppcPredYuvBest)
  {
    delete [] m_ppcPredYuvBest;
    m_ppcPredYuvBest = NULL;
  }
  if(m_ppcResiYuvBest)
  {
    delete [] m_ppcResiYuvBest;
    m_ppcResiYuvBest = NULL;
  }
  if(m_ppcRecoYuvBest)
  {
    delete [] m_ppcRecoYuvBest;
    m_ppcRecoYuvBest = NULL;
  }
  if(m_ppcPredYuvTemp)
  {
    delete [] m_ppcPredYuvTemp;
    m_ppcPredYuvTemp = NULL;
  }
  if(m_ppcResiYuvTemp)
  {
    delete [] m_ppcResiYuvTemp;
    m_ppcResiYuvTemp = NULL;
  }
  if(m_ppcRecoYuvTemp)
  {
    delete [] m_ppcRecoYuvTemp;
    m_ppcRecoYuvTemp = NULL;
  }
  if(m_ppcOrigYuv)
  {
    delete [] m_ppcOrigYuv;
    m_ppcOrigYuv = NULL;
  }
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init( TEncTop* pcEncTop )
{
  m_pcEncCfg           = pcEncTop;
  m_pcPredSearch       = pcEncTop->getPredSearch();
  m_pcTrQuant          = pcEncTop->getTrQuant();
  m_pcBitCounter       = pcEncTop->getBitCounter();
  m_pcRdCost           = pcEncTop->getRdCost();

  m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
  m_pcCavlcCoder       = pcEncTop->getCavlcCoder();
  m_pcSbacCoder        = pcEncTop->getSbacCoder();
  m_pcBinCABAC         = pcEncTop->getBinCABAC();

  m_pppcRDSbacCoder    = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder  = pcEncTop->getRDGoOnSbacCoder();

  m_pcRateCtrl         = pcEncTop->getRateCtrl();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  rpcCU pointer of CU data class
 */
static int cu_done = 0;
Void TEncCu::compressCUNewOrg( TComDataCU*& rpcCU, Int which_reference )
{
    
    // Hossam: XXXXX do they need initCU or initCU new?
    //      cout << "compressCU method says hi from start" << endl;
    // initialize CU data
    //    m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
    //    m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
    
//    m_ppcBestCU[0]->initCUNew( rpcCU->getPic(), rpcCU->getAddr() );
//    m_ppcTempCU[0]->initCUNew( rpcCU->getPic(), rpcCU->getAddr() );
    
    // Hossam: Scene Change: Switched them to Original 
    m_ppcBestCU[0]->initCUNewOrg( rpcCU->getPic(), rpcCU->getAddr() );
    m_ppcTempCU[0]->initCUNewOrg( rpcCU->getPic(), rpcCU->getAddr() );

    
    
    //    cout << "compressCU method says hi " << endl;
    // analysis of CU
    DEBUG_STRING_NEW(sDebug)
    
    // Hossam: Scene change: Update the compressCU to true
    //    setCompressCUFirstTime(true);
    
    // Save the prev_dep
    //    Int prev_dep = g_uiMaxCUDepth;
    //    g_uiMaxCUDepth = SC_MAX_DEPTH;
    
    // Starts with Depth = 0
    //    xCompressCUNew( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );
    
    // Run CU compression at Depth ORG 
    xCompressCURunAtDepthOrg(m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug), NUMBER_OF_PART_SIZES, which_reference);
//    xCompressCURunAtDepth(m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug));
    
    // Restore the depth value
    //    g_uiMaxCUDepth = prev_dep;
    
    
    //    xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );
    
    DEBUG_STRING_OUTPUT(std::cout, sDebug)
    
#if ADAPTIVE_QP_SELECTION
    if( m_pcEncCfg->getUseAdaptQpSelect() )
    {
        if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
        {
            xLcuCollectARLStats( rpcCU);
        }
    }
#endif
    
//            cu_done++;
//           cout << "TEncCu: compress CU: DOONEEEEEEEEEEEEEEEE " << cu_done << endl;
//        cout << "EXIT " << endl;
//        exit (EXIT_FAILURE);
    
#if SC_ENABLE_PRINT
    //    cout << "TEncCu: compress CU: DOONEEEEEEEEEEEEEEEE " << cu_done << endl;
    cout << "=============DONE CU!!!!!!! ===========" << endl;
    //    cout << "Addr of the final CU: " << rpcCU << endl;
    cout << "Number of FINAL partitions: " << Int(rpcCU->getNumPartitions()) << endl;
    cout << "rpcCU->getPic()->getNumPartInCU(): " << Int(rpcCU->getPic()->getNumPartInCU()) << endl; // 256 --> the depth thing per CU in 4x4 block
    cout << "rpcCU->getPic()->getNumCUsInFrame()(): " << Int(rpcCU->getPic()->getNumCUsInFrame()) << endl; // 28 total number of CUs in the actual frame
    
    
    cout << "Partition Size 0: " << Int(rpcCU->getPartitionSize(0)) << endl;
    cout << "Partition Dimensions 0: " << Int(rpcCU->getWidth(0)) << ", " <<  Int(rpcCU->getHeight(0)) << endl;
    cout << "Partition Dimensions 1: " << Int(rpcCU->getWidth(1)) << ", " <<  Int(rpcCU->getHeight(1)) << endl;
    cout << "Partition Dimensions 2: " << Int(rpcCU->getWidth(2)) << ", " <<  Int(rpcCU->getHeight(2)) << endl;
    cout << "Depth 0: " << Int(rpcCU->getDepth(0)) << endl;
    
#endif
    
    //    scBestResidual[rpcCU->getDepth(0)]->printComponent(ComponentID(COMPONENT_Y), 0);
    
    
    //    cout << "Mock # of partiotions: " << Int(rpcCU->getZorderIdxInCU()) << endl;
    
    
    //    scBestResidual[0]->mockPrintComponent(COMPONENT_Y, rpcCU->getZorderIdxInCU(), scBestResidual[0]->getWidth(COMPONENT_Y), scBestResidual[0]->getHeight(COMPONENT_Y));
    
    // Hossam: This is how it's supposed to be printed
    //    scBestResidual[0]->mockPrintFromSlice(rpcCU->getZorderIdxInCU());
    
    
    //    TComPicYuv* x = new TComPicYuv;
    //    x ->create(416, 240, CHROMA_420, 64, 64, 3);
    //
    //    scBestResidual[0]->copyToPicYuv(x, rpcCU->getAddr(), rpcCU->getZorderIdxInCU());
    //
    //    Char* pFileName = "hello2.yuv";
    //    x->dump(pFileName);
    //
    ////    x->mockPrint(0);
    //    x->destroy();
    //    exit (EXIT_FAILURE);
    
    //    if(rpcCU->getSlice()->getPOC() == 1)
    //        exit(EXIT_FAILURE);
    
    
}


/** \param  rpcCU pointer of CU data class
 */
//static int cu_done = 0;
Void TEncCu::compressCUNew( TComDataCU*& rpcCU )
{
    
    // Hossam: XXXXX do they need initCU or initCU new?
    //      cout << "compressCU method says hi from start" << endl;
    // initialize CU data
//    m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
//    m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );

    m_ppcBestCU[0]->initCUNew( rpcCU->getPic(), rpcCU->getAddr() );
    m_ppcTempCU[0]->initCUNew( rpcCU->getPic(), rpcCU->getAddr() );
    
    
    //    cout << "compressCU method says hi " << endl;
    // analysis of CU
    DEBUG_STRING_NEW(sDebug)
    
    // Hossam: Scene change: Update the compressCU to true
//    setCompressCUFirstTime(true);
    
    // Save the prev_dep
//    Int prev_dep = g_uiMaxCUDepth;
//    g_uiMaxCUDepth = SC_MAX_DEPTH;
    
    // Starts with Depth = 0
//    xCompressCUNew( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );

    // Run CU compression at Depth
    xCompressCURunAtDepth(m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug));
    
    // Restore the depth value
//    g_uiMaxCUDepth = prev_dep;
    
    
//    xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );

    DEBUG_STRING_OUTPUT(std::cout, sDebug)
    
#if ADAPTIVE_QP_SELECTION
    if( m_pcEncCfg->getUseAdaptQpSelect() )
    {
        if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
        {
            xLcuCollectARLStats( rpcCU);
        }
    }
#endif
    
//        cu_done++;
//        cout << "TEncCu: compress CU: DOONEEEEEEEEEEEEEEEE " << cu_done << endl;
//    cout << "EXIT " << endl;
    
#if SC_ENABLE_PRINT
    //    cout << "TEncCu: compress CU: DOONEEEEEEEEEEEEEEEE " << cu_done << endl;
    cout << "=============DONE CU!!!!!!! ===========" << endl;
    //    cout << "Addr of the final CU: " << rpcCU << endl;
    cout << "Number of FINAL partitions: " << Int(rpcCU->getNumPartitions()) << endl;
    cout << "rpcCU->getPic()->getNumPartInCU(): " << Int(rpcCU->getPic()->getNumPartInCU()) << endl; // 256 --> the depth thing per CU in 4x4 block
    cout << "rpcCU->getPic()->getNumCUsInFrame()(): " << Int(rpcCU->getPic()->getNumCUsInFrame()) << endl; // 28 total number of CUs in the actual frame
    
    
    cout << "Partition Size 0: " << Int(rpcCU->getPartitionSize(0)) << endl;
    cout << "Partition Dimensions 0: " << Int(rpcCU->getWidth(0)) << ", " <<  Int(rpcCU->getHeight(0)) << endl;
    cout << "Partition Dimensions 1: " << Int(rpcCU->getWidth(1)) << ", " <<  Int(rpcCU->getHeight(1)) << endl;
    cout << "Partition Dimensions 2: " << Int(rpcCU->getWidth(2)) << ", " <<  Int(rpcCU->getHeight(2)) << endl;
    cout << "Depth 0: " << Int(rpcCU->getDepth(0)) << endl;
    
#endif
    
//    scBestResidual[rpcCU->getDepth(0)]->printComponent(ComponentID(COMPONENT_Y), 0);
    
    
//    cout << "Mock # of partiotions: " << Int(rpcCU->getZorderIdxInCU()) << endl;

    
//    scBestResidual[0]->mockPrintComponent(COMPONENT_Y, rpcCU->getZorderIdxInCU(), scBestResidual[0]->getWidth(COMPONENT_Y), scBestResidual[0]->getHeight(COMPONENT_Y));

    // Hossam: This is how it's supposed to be printed
//    scBestResidual[0]->mockPrintFromSlice(rpcCU->getZorderIdxInCU());
    
    
//    TComPicYuv* x = new TComPicYuv;
//    x ->create(416, 240, CHROMA_420, 64, 64, 3);
//    
//    scBestResidual[0]->copyToPicYuv(x, rpcCU->getAddr(), rpcCU->getZorderIdxInCU());
//    
//    Char* pFileName = "hello2.yuv";
//    x->dump(pFileName);
//    
////    x->mockPrint(0);
//    x->destroy();
//    exit (EXIT_FAILURE);
    
//    if(rpcCU->getSlice()->getPOC() == 1)
//        exit(EXIT_FAILURE);

   
}

/** \param  rpcCU pointer of CU data class
 */
Void TEncCu::compressCU( TComDataCU*& rpcCU )
{
    
//    cout << "REMOVE ME I AM THE BITS ACCUMULATOR XXXX " << endl;
//    bitsAccumulator = 0;
    
//    cout << "compressCU method says hi from start" << endl;

    // initialize CU data
  m_ppcBestCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );
  m_ppcTempCU[0]->initCU( rpcCU->getPic(), rpcCU->getAddr() );

    
//    cout << "compressCU method says hi " << endl;
  // analysis of CU
  DEBUG_STRING_NEW(sDebug)

  xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );
  DEBUG_STRING_OUTPUT(std::cout, sDebug)

//  cout << "AFTER XCOMPRESS CU " << m_ppcBestCU[0] -> m_uiTotalBitsRefI[0] << ", -> " << rpcCU->m_uiTotalBitsRefI[0] << ", tots: " << m_ppcBestCU[0]->getTotalBits() << ", " << rpcCU->getTotalBits() << endl;
    
//    Double check_per = 100.0*m_ppcBestCU[0] -> m_uiTotalBitsRefI[0]/m_ppcBestCU[0]->getTotalBits();
//    cout << "AFTER XCOMPRESS CU " << m_ppcBestCU[0] -> m_uiTotalBitsRefI[0] << ", tots: " << m_ppcBestCU[0]->getTotalBits() << ", ratio: " << check_per << endl;

    
#if ADAPTIVE_QP_SELECTION
  if( m_pcEncCfg->getUseAdaptQpSelect() )
  {
    if(rpcCU->getSlice()->getSliceType()!=I_SLICE) //IIII
    {
      xLcuCollectARLStats( rpcCU);
    }
  }
#endif
    
    
}
/** \param  pcCU  pointer of CU data class
 */
//static int vv = 0;
Void TEncCu::encodeCU ( TComDataCU* pcCU )
{
  if ( pcCU->getSlice()->getPPS()->getUseDQP() )
  {
    setdQPFlag(true);
  }

  if ( pcCU->getSlice()->getUseChromaQpAdj() )
  {
    setCodeChromaQpAdjFlag(true);
  }
    
    bitsAccumulator = 0;
    bitsAccumulatorRes = 0;
    bitsAccumulatorPred = 0;
    
    bitsSnapEntropyBeforePerCu = bitsSnapEntropyAfterPerCu = 0;
    bitsAccumulateEntropyBits = 0;
    
    
    bitsAccumulatorSplitFlag = 0;
    bitsAccumulatorPredMode = 0;
    bitsAccumulatorPartSize = 0;
    bitsAccumulatorIPCMFlag = 0;
    bitsAccumulatorPredInfo = 0;
    bitsAccumulatorCoeff = 0;
    bitsAccumulatorTerminateInfo = 0;
    bitsAccumulatorCUTransquantBypassFlag = 0;
    bitsAccumulatorSkipFlag = 0;
    bitsAccumulatorMergIndex = 0;
    
    
    // cout << "\n \n NEW CTU BITS BITS " << endl;
    
    
    
    
  // Encode CU data
  xEncodeCU( pcCU, 0, 0 );
    
   
//    cout << "TOTAL " << bitsAccumulator << endl;
//    cout << "\n0) bitsAccumulatorPredMode: " << bitsAccumulatorPredMode << ", bitsAccumulatorPartSize: " << bitsAccumulatorPartSize << ", bitsAccumulatorIPCMFlag: " << bitsAccumulatorIPCMFlag << ", bitsPredInfo: " << bitsAccumulatorPredInfo << ", bitsCoeff: " << bitsAccumulatorCoeff << ", bitsAccumulatorTerminateInfo: " << bitsAccumulatorTerminateInfo << ",  bitsAccumulatorCUTransquantBypassFlag: " <<  bitsAccumulatorCUTransquantBypassFlag << ", bitsAccumulatorSkipFlag: "<<  bitsAccumulatorSkipFlag << ", bitsAccumulatorMergIndex: " << bitsAccumulatorMergIndex <<  endl;
    
    // Do the final math.
    bitsAccumulatorRes  = bitsAccumulatorCoeff;
    bitsAccumulatorPred = bitsAccumulatorPredInfo + bitsAccumulatorPredMode + bitsAccumulatorPartSize;
    bitsAccumulator = bitsAccumulatorRes + bitsAccumulatorPred + bitsAccumulatorSplitFlag;
    
//    cout << "\n0) bitsPred: " << bitsAccumulatorPred << ", bitsRes: " << bitsAccumulatorRes <<  ", bitsAccumulatorSplitFlag: " << bitsAccumulatorSplitFlag << endl;
//    cout << "bits bits: " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", " << pcCU->getTotalBits() << endl;
    
    
//    cout << "Bits For this CTU: " << bitsAccumulator << ", " <<  pcCU->getTotalBits()  << endl;
//    vv++;
//    if (vv == 3)
//        exit(0);
//    exit(0);
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Derive small set of test modes for AMP encoder speed-up
 *\param   rpcBestCU
 *\param   eParentPartSize
 *\param   bTestAMP_Hor
 *\param   bTestAMP_Ver
 *\param   bTestMergeAMP_Hor
 *\param   bTestMergeAMP_Ver
 *\returns Void
*/
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *&rpcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{
    if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
    {
//        cout << "geego 1" << endl;
        bTestAMP_Hor = true;
    }
    else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
//        cout << "geego 2" << endl;
        bTestAMP_Ver = true;
    }
    else if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->getMergeFlag(0) == false && rpcBestCU->isSkipped(0) == false )
    {
//        cout << "geego 3" << endl;
        bTestAMP_Hor = true;
        bTestAMP_Ver = true;
    }
    
#if AMP_MRG
    //! Utilizing the partition size of parent PU
    if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
    {
//        cout << "geego 4" << endl;
        bTestMergeAMP_Hor = true;
        bTestMergeAMP_Ver = true;
    }
    
    if ( eParentPartSize == NUMBER_OF_PART_SIZES ) //! if parent is intra
    {
//        cout << "geego 5" << endl;
        if ( rpcBestCU->getPartitionSize(0) == SIZE_2NxN )
        {
//            cout << "geego 6" << endl;
            bTestMergeAMP_Hor = true;
        }
        else if ( rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
        {
//            cout << "geego 7" << endl;
            bTestMergeAMP_Ver = true;
        }
    }
    
    if ( rpcBestCU->getPartitionSize(0) == SIZE_2Nx2N && rpcBestCU->isSkipped(0) == false )
    {
//        cout << "geego 8" << endl;
        bTestMergeAMP_Hor = true;
        bTestMergeAMP_Ver = true;
    }
    
    if ( rpcBestCU->getWidth(0) == 64 )
    {
//        cout << "geego 9" << endl;
        bTestAMP_Hor = false;
        bTestAMP_Ver = false;
    }
#else
    //! Utilizing the partition size of parent PU
    if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
    {
//        cout << "geego 10" << endl;
        bTestAMP_Hor = true;
        bTestAMP_Ver = true;
    }
    
    if ( eParentPartSize == SIZE_2Nx2N )
    {
//        cout << "geego 11" << endl;
        bTestAMP_Hor = false;
        bTestAMP_Ver = false;
    }
#endif
}
#endif



// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
 */
// xCompressCUNew: Trial to limit the depth
// static int call = 2082;
//#if AMP_ENC_SPEEDUP
//Void TEncCu::xCompressCUNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize )
//#else
//Void TEncCu::xCompressCUNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
//#endif
//{
//    TComPic* pcPic = rpcBestCU->getPic();
//    DEBUG_STRING_NEW(sDebug)
//    
//#if SC_ENABLE_PRINT
//    cout << "========================" << endl;
//    //        cout << " \n\nxCOmpressCUNew says hi from start " << uiDepth << endl;
//    cout << " \n\nxCOmpressCUNew says hi from start " << (++call) << ", dep: " << uiDepth  << endl;
//    cout << "========================" << endl;
//#endif
//    
//    // get Original YUV data from picture
//    m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );
//    
//    
//    // variable for Early CU determination
//    Bool    bSubBranch = true;
//    
//    // variable for Cbf fast mode PU decision
//    Bool    doNotBlockPu = true;
//    Bool    earlyDetectionSkipMode = false;
//    
//    Bool bBoundary = false;
//    UInt uiLPelX   = rpcBestCU->getCUPelX();
//    UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
//    UInt uiTPelY   = rpcBestCU->getCUPelY();
//    UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;
//    
//    // XXXX Hossam: I don't need use adaptive QP XXXXX No need -- Just get!
//    //    Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
//    Int iBaseQP = rpcBestCU->getSlice()->getSliceQp();
//    Int iMinQP;
//    Int iMaxQP;
//    Bool isAddLowestQP = false;
//    
//    const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();
//    
//    if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
//    {
//        Int idQP = m_pcEncCfg->getMaxDeltaQP();
//        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
//        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
//    }
//    else
//    {
//        iMinQP = rpcTempCU->getQP(0);
//        iMaxQP = rpcTempCU->getQP(0);
//    }
//    
//    // Hossam: I think I need to remove that!!!! XXXXXX
//    //    if ( m_pcEncCfg->getUseRateCtrl() )
//    //    {
//    //        iMinQP = m_pcRateCtrl->getRCQP();
//    //        iMaxQP = m_pcRateCtrl->getRCQP();
//    //    }
//    
//    // transquant-bypass (TQB) processing loop variable initialisation ---
//    
//    const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.
//    
//#if SC_ENABLE_PRINT
//    cout << "xCompressCUNew transequent flag " << (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) << endl;
//#endif
//    
//    // Hossam: Transequent flag is false XXXXX need to remove it!!!!
//    if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
//    {
//        isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
//        iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
//        if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
//        {
//            iMaxQP = iMinQP;
//        }
//    }
//    
//    // If slice start or slice end is within this cu...
//    TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
//    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
//    Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
//    Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
//    
//    // Hossam: He splits only on these conditions
//    // We need to split, so don't try these modes.
//    // If there's no slice start, slice end in this picture
//    if(!bSliceEnd && !bSliceStart && bInsidePicture )
//    {
//        for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
//        {
//            
//            //        cout << " xCOmpressCU says hi " << endl;
//            const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);
//            
//            if (bIsLosslessMode)
//            {
//                iQP = lowestQP;
//            }
//            
//            m_ChromaQpAdjIdc = 0;
//            if (pcSlice->getUseChromaQpAdj())
//            {
//                /* Pre-estimation of chroma QP based on input block activity may be performed
//                 * here, using for example m_ppcOrigYuv[uiDepth] */
//                /* To exercise the current code, the index used for adjustment is based on
//                 * block position
//                 */
//                Int lgMinCuSize = pcSlice->getSPS()->getLog2MinCodingBlockSize();
//                m_ChromaQpAdjIdc = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pcSlice->getPPS()->getChromaQpAdjTableSize() + 1);
//            }
//            
//            /** initialize prediction data with enabling sub-LCU-level delta QP
//             *\param  uiDepth  depth of the current CU
//             *\param  qp     qp for the current CU
//             *- set CU width and CU height according to depth
//             *- set qp value according to input qp
//             *- set last-coded qp value according to input last-coded qp
//             */
//#if SC_ENABLE_PRINT
//            cout << " xCompressCU New (!bSliceEnd && !bSliceStart && bInsidePicture ) calls initEstData " << endl;
//#endif
//            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//            
//            // do inter modes, SKIP and 2Nx2N
//            if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
//            {
//                //                cout << "I'm Inter CU ppl :) " << endl;
//                
//                //                if(call==2087)
//                //                {
//                //                    cout << "2087 Bits: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
//                //                }
//                
//#if SC_ENABLE_PRINT
//                // It's false!
//                cout << "xCompressCUNew m_pcEncCfg->getUseEarlySkipDetection() flag is if true remove!: " << m_pcEncCfg->getUseEarlySkipDetection() << endl; // false
//#endif
//                
//                
//                // 2Nx2N
//                if(m_pcEncCfg->getUseEarlySkipDetection())
//                {
//                    // Check RD cost Inter
//                    //                    xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                    
//#if SC_ENABLE_PRINT
//                    // Hossam: check RD cost Inter new XXXX
//                    cout << "\n xCompressCU New will call xCheckRDCostInterNew 1" << endl;
//#endif
//                    xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                    
//                    //                    cout << "Will Call initEstData 2Nx2N " << endl;
//                    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
//                    //                    cout << "ACK initEstData " << endl;
//                }
//                // SKIP
//                //                xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
//                
//#if SC_ENABLE_PRINT
//                //                cout << "xCheckRDCostMerge2Nx2NNew  1" << endl;
//                cout << "\nxCheckRDCostMerge2Nx2NNew  1 cost " << rpcTempCU->getTotalCost() << endl;
//#endif
//                
//                //                if(call==2087)
//                //                {
//                //                    cout << "2087 Bits 2: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
//                //                }
//                
//                
//                xCheckRDCostMerge2Nx2NNew( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
//                
//                //                cout << "Will Call initEstData " << endl;
//                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                //                cout << "Call initEstData All Done! " << endl;
//                //                cout << "initEstData  " << endl;
//                
//                //                cout << "ACK initEstData " << endl;
//                //                cout << "xCompressCUNew m_pcEncCfg->getUseEarlySkipDetection() flag is if true remove!: " << m_pcEncCfg->getUseEarlySkipDetection() << endl;
//                // Hossam: No need to check this case for now -- unsatisfied condition
//                if(!m_pcEncCfg->getUseEarlySkipDetection())
//                {
//#if SC_ENABLE_PRINT
//                    cout << "\n xCompressCU New will call xCheckRDCostInterNew  Inter Inside Not Early:  " << endl;
//#endif
//                    // 2Nx2N, NxN
//                    // Hossam: New version of the xCheckRDCostInter new
//                    //                    xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                    xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                    
//                    
//                    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                    if(m_pcEncCfg->getUseCbfFastMode())
//                    {
//                        doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                    }
//                }
//            }// end if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
//            
//            //======================================================================
//            
//            if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
//            {
//                iQP = iMinQP;
//            }
//        }
//        
//        //        cout << " earlyDetectionSkipMode: Should be false to enter: " << earlyDetectionSkipMode << endl;
//        // Hossam: This case is not visited so far!
//        if(!earlyDetectionSkipMode)
//        {
//            for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
//            {
//                const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP); // If lossless, then iQP is irrelevant for subsequent modules.
//                
//                if (bIsLosslessMode)
//                {
//                    iQP = lowestQP;
//                }
//                
//                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                
//                // do inter modes, NxN, 2NxN, and Nx2N
//                if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
//                {
//                    // 2Nx2N, NxN
//                    if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
//                    {
////                        if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
//                        if( uiDepth == sc_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
//                        {
//                            //                            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
//                            
//                            
//#if SC_ENABLE_PRINT
//                            cout << " \nxCompressCU New will call xCheckRDCostInterNew 2" << endl;
//#endif
//                            // Hossam: New version of the xCheckRDCostInterNew XXX
//                            xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
//                            
//                            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        }
//                    }
//                    
//                    if(doNotBlockPu)
//                    {
//                        //                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );
//                        
//#if SC_ENABLE_PRINT
//                        cout << " \nxCompressCU New will call xCheckRDCostInterNew 3" << endl;
//#endif
//                        // Hossam: New version of the xCheckRDCostInter XXXX
//                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
//                        {
//                            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                        }
//                    }
//                    if(doNotBlockPu)
//                    {
//#if SC_ENABLE_PRINT
//                        // Hossam: Nev version of the xCheckRDCostInter XXXX
//                        cout << " \nxCompressCU New will call xCheckRDCostInterNew 4" << endl;
//#endif
//                        //                        xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );
//                        xCheckRDCostInterNew      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
//                        {
//                            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                        }
//                    }
//                    
//                    //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
//                    if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
//                    {
//#if AMP_ENC_SPEEDUP
//                        Bool bTestAMP_Hor = false, bTestAMP_Ver = false;
//                        
//#if AMP_MRG
//                        Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;
//                        
//                        deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
//#else
//                        deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
//#endif
//                        
//                        //! Do horizontal AMP
//                        if ( bTestAMP_Hor )
//                        {
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 5" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
//                                {
//                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                                }
//                            }
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 6" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
//                                {
//                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                                }
//                            }
//                        }
//#if AMP_MRG
//                        else if ( bTestMergeAMP_Hor )
//                        {
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 7" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
//                                {
//                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                                }
//                            }
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 8" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
//                                {
//                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                                }
//                            }
//                        }
//#endif
//                        
//#if SC_ENABLE_PRINT
//                        //                        cout << "Vertical and horizontal flags: " << boolalpha << bTestAMP_Ver << ", " << bTestAMP_Hor <<  endl;
//                        cout << "Vertical MERGE AMP FLAG : " << boolalpha << bTestMergeAMP_Ver << ", rpcTempCU " << rpcTempCU->getTotalCost()<<  endl;
//#endif
//                        
//                        //! Do horizontal AMP
//                        if ( bTestAMP_Ver )
//                        {
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 9" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
//                                {
//                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                                }
//                            }
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 10" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                            }
//                        }
//#if AMP_MRG
//                        
//                        else if ( bTestMergeAMP_Ver )
//                        {
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 11" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
//                                {
//                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
//                                }
//                            }
//                            if(doNotBlockPu)
//                            {
//#if SC_ENABLE_PRINT
//                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 12" << endl;
//#endif
//                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
//                                //                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
//                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                            }
//                        }
//#endif
//                        
//#else
//                        
//#if SC_ENABLE_PRINT
//                        cout << " \nxCompressCU New will call xCheckRDCostInterNew 13 A lot" << endl;
//#endif
//                        // Hossam: New version of the xCheckRDCostInter XXXX
//                        //                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
//                        //                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        //                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
//                        //                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        //                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
//                        //                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        //
//                        //                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
//                        //                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnU );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnD );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nLx2N );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        
//                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nRx2N );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        
//                        
//#endif
//                    }
//                }
//                
//#if SC_ENABLE_INTRA_MODE
//                // do normal intra modes
//                // speedup for inter frames
//                Double intraCost = 0.0;
//                
//                if((rpcBestCU->getSlice()->getSliceType() == I_SLICE)                                     ||
//                   (rpcBestCU->getCbf( 0, COMPONENT_Y  ) != 0)                                            ||
//                   ((rpcBestCU->getCbf( 0, COMPONENT_Cb ) != 0) && (numberValidComponents > COMPONENT_Cb)) ||
//                   ((rpcBestCU->getCbf( 0, COMPONENT_Cr ) != 0) && (numberValidComponents > COMPONENT_Cr))  ) // avoid very complex intra if it is unlikely
//                {
//                    cout << "Check INTRA 1 haleeeeeem" << endl;
//                    
//                    xCheckRDCostIntraNew( rpcBestCU, rpcTempCU, intraCost, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                    
//                    //                    xCheckRDCostIntra( rpcBestCU, rpcTempCU, intraCost, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
//                    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                    // Hossam: 6 - 0
////                    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
//                    if( uiDepth == sc_uiMaxCUDepth - g_uiAddCUDepth )
//                    {
//                        if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
//                        {
//                            cout << "Check INTRA 2 " << endl;
//                            Double tmpIntraCost;
//                            xCheckRDCostIntraNew( rpcBestCU, rpcTempCU, tmpIntraCost, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
//                            
//                            
//                            //                            xCheckRDCostIntra( rpcBestCU, rpcTempCU, tmpIntraCost, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
//                            intraCost = std::min(intraCost, tmpIntraCost);
//                            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        }
//                    }
//                }
//#endif
//                
//                
//                // Hossam: Probably not needed XXXXX
//                // test PCM
//                if(pcPic->getSlice(0)->getSPS()->getUsePCM()
//                   && rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
//                   && rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
//                {
//                    UInt uiRawBits = getTotalBits(rpcBestCU->getWidth(0), rpcBestCU->getHeight(0), rpcBestCU->getPic()->getChromaFormat(), g_bitDepth);
//                    UInt uiBestBits = rpcBestCU->getTotalBits();
//                    if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
//                    {
//                        cout << "xCheckIntraPCM has a load buffer from Cabac " << endl;
//                        xCheckIntraPCM (rpcBestCU, rpcTempCU);
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                    }
//                }
//                
//                if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
//                {
//                    iQP = iMinQP;
//                }
//            }
//        }
//        
//        // Hossam:  XXXX I think this one might not be needed
//        m_pcEntropyCoder->resetBits();
//        m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
//        rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
//        rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
//        rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );
//        
//        //        cout << "wof no where <bits, cost>: " << "<" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">" << endl;
//        
//        // Early CU determination
//        if( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) )
//        {
//            bSubBranch = false;
//        }
//        else
//        {
//            bSubBranch = true;
//        }
//    }
//    else if(!(bSliceEnd && bInsidePicture))
//    {
//        bBoundary = true;
//    }
//    
//    // Hossam: XXXX I think I might not need that XXXXX
//    // copy orginal YUV samples to PCM buffer
//    
//    //    cout << "I never enter the xFillPCMBuffer case " << endl;
//    if( rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
//    {
//        //        cout << "xFillPCMBuffer" << endl;
//        xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
//    }
//    
//    //    cout << "About to check g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() " << endl;
//    
//    if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
//    {
//        //        cout << "g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
//        Int idQP = m_pcEncCfg->getMaxDeltaQP();
//        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
//        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
//    }
//    else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
//    {
//        //         cout << "g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
//        iMinQP = iBaseQP;
//        iMaxQP = iBaseQP;
//    }
//    else
//    {
//        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << endl;
//        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU << endl;
//        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic << endl;
//        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic->getCU(rpcTempCU->getAddr()) << endl;
//        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU->getZorderIdxInCU() << endl;
//#if SC_ENABLE_PRINT
//        cout << "Flafeelo Should be equal!??? " << (pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr()) << endl;
//#endif
//        
//        Int iStartQP;
//        if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
//        {
//            
//            //            cout << "Mini If part " << endl;
//            iStartQP = rpcTempCU->getQP(0);
//            //            cout << "Mini If part done" << endl;
//        }
//        else
//        {
//            //            cout << "Mini else part 1" << endl;
//            UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
//            
//            //            cout << "Mini else part 2 " << (rpcTempCU->m_phQP==NULL) << ", " << uiCurSliceStartPartIdx << endl;
//            //            cout << "Mini else part 2 " << (rpcTempCU->m_phQP[uiCurSliceStartPartIdx]) << endl;
//            
//            iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
//            //            iStartQP = 22;
//            //            cout << "Mini else part done " << endl;
//        }
//        iMinQP = iStartQP;
//        iMaxQP = iStartQP;
//        
//        //        cout << "Flafeelo END cPic->getCU( rpcTempCU->getAddr() ) " << endl;
//    }
//    
//    //    cout << "About to check m_pcEncCfg->getUseRateCtrl() " << endl;
//    // Hossam: XXXX I think no need for that
//    if ( m_pcEncCfg->getUseRateCtrl() )
//    {
//        iMinQP = m_pcRateCtrl->getRCQP();
//        iMaxQP = m_pcRateCtrl->getRCQP();
//    }
//    
//    //    cout << "About to check m_pcEncCfg->getCUTransquantBypassFlagForceValue() " << endl;
//    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
//    {
//        iMaxQP = iMinQP; // If all TUs are forced into using transquant bypass, do not loop here.
//    }
//    
//    //    cout << "More splitting! " << endl;
//    // Hossam: More modes -- Splitting
//    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
//    {
//        const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
//        
//        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//        
//        // Hossam: This has to be changed to a limited SC depth
//        // further split
////        if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
//         if( bSubBranch && uiDepth < sc_uiMaxCUDepth - g_uiAddCUDepth )
//        {
//            //            cout << "I'm about to split and go into the next depth " << endl;
//            // Hossam: Increment Depth
//            UChar       uhNextDepth         = uiDepth+1;
//            TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
//            TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
//            DEBUG_STRING_NEW(sTempDebug)
//            
//#if SC_ENABLE_PRINT
//            cout << "////////Next Depth Probe Marry me: \\\\\\\\" << Int(uhNextDepth) << endl;
//#endif
//            for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
//            {
//                pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
//                pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
//                
//                Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
//                if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
//                {
//                    if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
//                    {
//                        //                        cout << "REMOVED m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load( part id = 0" << endl;
//                        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
//                    }
//                    else
//                    {
//                        //                        cout << " REMOVED m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load( part id = 1" << endl;
//                        
//                        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
//                    }
//                    
//#if AMP_ENC_SPEEDUP
//                    DEBUG_STRING_NEW(sChild)
//                    if ( !rpcBestCU->isInter(0) )
//                    {
//                        // Hossam: Call of Compress New
//                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
//#if SC_ENABLE_PRINT
//                        cout << "Rec call  xCompressCUNew 1" << endl;
//#endif
//                        xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
//                        
//                    }
//                    else
//                    {
//#if SC_ENABLE_PRINT
//                        cout << "Rec call  xCompressCUNew 2 bits: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
//#endif
//                        
//                        //                        cout << "REport: " << pcSubBestPartCU ->getTotalBits() <<", " << pcSubTempPartCU->getTotalCost() << endl;
//                        //
//                        //                        cout << "REport: " << rpcBestCU ->getTotalBits() <<", " << rpcTempCU->getTotalCost() << endl;
//                        
//                        // Hossam: Call of Compress New
//                        xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
//                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
//                    }
//                    DEBUG_STRING_APPEND(sTempDebug, sChild)
//#else
//                    
//#if SC_ENABLE_PRINT
//                    // Hossam: Call of Compress New
//                    cout << "Rec call  xCompressCUNew 3" << endl;
//#endif
//                    
//                    xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
//                    //                    xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
//#endif
//                    
//                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
//                    xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
//                }
//                else if (bInSlice)
//                {
//                    pcSubBestPartCU->copyToPic( uhNextDepth );
//                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
//                }
//            }
//            
//            if( !bBoundary )
//            {
//                // Hossam: Removed the encode steps XXXX
//                m_pcEntropyCoder->resetBits();
//                m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );
//                
//                //                cout << "Entropy Encoder needs written bits and bins" << endl;
//                rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
//                rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
//                
//                //                cout << "Entropy Encoder needs written bits and bins Done" << endl;
//            }
//            rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
//            
//            if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
//            {
//                
//                //                cout << "About to loop on parts in mysterious loop " << endl;
//                Bool hasResidual = false;
//                for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
//                {
//                    if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) &&
//                       (     rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Y)
//                        || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cb) && (numberValidComponents > COMPONENT_Cb))
//                        || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cr) && (numberValidComponents > COMPONENT_Cr)) ) )
//                    {
//                        hasResidual = true;
//                        break;
//                    }
//                }
//                
//                UInt uiTargetPartIdx;
//                if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
//                {
//                    uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
//                }
//                else
//                {
//                    uiTargetPartIdx = 0;
//                }
//                if ( hasResidual )
//                {
//#if !RDO_WITHOUT_DQP_BITS
//                    // Hossam: Removed encode steps --> Reset Bits might be removed XXXXXX
//                    m_pcEntropyCoder->resetBits();
//                    m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
//                    
//                    //                    cout << "Remove maybe Entropy 3ak!!! 1" << endl;
//                    rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
//                    //                    cout << "Remove maybe Entropy 3ak!!! 2" << endl;
//                    rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
//                    
//                    //                    cout << "Entropy 3ak!!! 3" << endl;
//                    rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
//#endif
//                    
//                    Bool foundNonZeroCbf = false;
//                    rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
//                    assert( foundNonZeroCbf );
//                }
//                else
//                {
//                    rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
//                }
//            }
//            
//            //            cout << "Remove Entropy 3ak!!! 4" << endl;
//            m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
//            //            cout << "Remove Entropy 3ak!!! 5" << endl;
//            
//            Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
//            && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
//            Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
//            && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
//            if(isEndOfSlice||isEndOfSliceSegment)
//            {
//                if (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING)
//                    rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost() + (1.0 / m_pcRdCost->getLambda());
//                else
//                    rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
//            }
//            
//            // Hossam: Use the new version of the check best mode for future changes
//            
//            xCheckBestModeLastCall(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false)); // RD compare current larger prediction
//            //            xCheckBestModeNew( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
//            //            xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
//            // with sub partitioned prediction.
//        }
//    }
//    
//    DEBUG_STRING_APPEND(sDebug_, sDebug);
//    
//    // Hossam: Scene change
//    //    xSaveResiduals(rpcBestCU, rpcTempCU, uiDepth);
//    
//    rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.
//    
//    xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
//    if( bBoundary ||(bSliceEnd && bInsidePicture))
//    {
//        //        cout << "BRAVE HEART OF THE COMPRESS SLICEEEEEEEEE " << endl;
//        return;
//    }
//    
//    // Assert if Best prediction mode is NONE
//    // Selected mode's RD-cost must be not MAX_DOUBLE.
//    assert( rpcBestCU->getPartitionSize ( 0 ) != NUMBER_OF_PART_SIZES       );
//    assert( rpcBestCU->getPredictionMode( 0 ) != NUMBER_OF_PREDICTION_MODES );
//    assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE                 );
//} // end xCompressCUNew


static int mon = 0;
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCURunAtDepthOrg( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize, Int which_reference)
#else
Void TEncCu::xCompressCURunAtDepthOrg( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
    TComPic* pcPic = rpcBestCU->getPic();
    DEBUG_STRING_NEW(sDebug)
    
//    cout << "========================" << endl;
//    //        cout << " \n\nxCOmpressCUNew says hi from start " << uiDepth << endl;
//    cout << " \nxCompressCURunAtDepthOrg says hi from start " << (++mon) << ", dep: " << uiDepth  << endl;
//    cout << "========================" << endl;
    
#if SC_ENABLE_PRINT
    cout << "========================" << endl;
    //        cout << " \n\nxCOmpressCUNew says hi from start " << uiDepth << endl;
    cout << " \n\xCompressCURunAtDepthOrg says hi from start " << (++call) << ", dep: " << uiDepth  << endl;
    cout << "========================" << endl;
#endif
    
    
    // get Original YUV data from picture
    m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );
    
    // variable for Early CU determination
    Bool    bSubBranch = true;
    
    // variable for Cbf fast mode PU decision
    Bool    doNotBlockPu = true;
    Bool    earlyDetectionSkipMode = false;
    
    Bool bBoundary = false;
    UInt uiLPelX   = rpcBestCU->getCUPelX();
    UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
    UInt uiTPelY   = rpcBestCU->getCUPelY();
    UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;
    
    Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
    Int iMinQP;
    Int iMaxQP;
    Bool isAddLowestQP = false;
    
    const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();
    
    
    // Hossam: Now the min QP is equal to the max Qp -- no rate control
    if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
    }
    else
    {
        iMinQP = rpcTempCU->getQP(0);
        iMaxQP = rpcTempCU->getQP(0);
    }
    
    const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.
    
    
#if SC_ENABLE_PRINT
    cout << "xCompressCUNew transequent flag " << (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) << endl;
#endif
    
    if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
    {
        isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
        iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
        if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
        {
            iMaxQP = iMinQP;
            
        }
    }
    
    // If slice start or slice end is within this cu...
    TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
    Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
    Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
    
    
    // We need to split, so don't try these modes.
    
    if( (!bSliceEnd && !bSliceStart && bInsidePicture) && uiDepth == SC_AT_DEPTH)
    {
        for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
        {
            const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);
            
            if (bIsLosslessMode)
            {
                iQP = lowestQP;
            }
            
            m_ChromaQpAdjIdc = 0;
            if (pcSlice->getUseChromaQpAdj())
            {
                /* Pre-estimation of chroma QP based on input block activity may be performed
                 * here, using for example m_ppcOrigYuv[uiDepth] */
                /* To exercise the current code, the index used for adjustment is based on
                 * block position
                 */
                Int lgMinCuSize = pcSlice->getSPS()->getLog2MinCodingBlockSize();
                m_ChromaQpAdjIdc = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pcSlice->getPPS()->getChromaQpAdjTableSize() + 1);
            }
            
#if SC_ENABLE_PRINT
            cout << " xCompressCURunAtDepthOrg New (!bSliceEnd && !bSliceStart && bInsidePicture ) calls initEstData " << endl;
#endif
            
            // Hossam: Scene change ORG: He feels the original Qp = 22
//            if (rpcTempCU->getSlice()->getPOC() == 1) {
//                cout << "*******initEstData start Qp: " << iQP << endl;
//            }
            
            // Init temp CU
//            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            
              rpcTempCU->initEstDataSC( uiDepth, iQP, bIsLosslessMode );
            
            
            // do inter modes, 2Nx2N
            if (rpcBestCU->getSlice()->getSliceType() != I_SLICE) {
                
                //                if(m_pcEncCfg->getUseEarlySkipDetection())
                //                {
//                xCheckRDCostInterAtDepth( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
                // Hossam: Send the default value of bMrg here
                xCheckRDCostInterAtDepthOrg( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug), false, which_reference );
//                xCheckRDCostInterAtDepthOrg( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug), which_reference );
                 //-----Debug-----// failed the test
//                cout <<    rpcTempCU->getAddr() << ", " << rpcTempCU->getZorderIdxInCU() <<  ", After xCheckRDCost Reference: " << endl;
//                rpcTempCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//                
//                cout << ", After xCheckRDCost SC: " << endl;
//                rpcTempCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
                //-----Debug-----
                
                // Copy temp to best
                xCopyTempToBestSC(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
                

                
                // Removed this! Clears the MVs -- better copy the temp to best CU
//                rpcTempCU->initEstDataSC( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
                
                
//                //-----Debug-----// failed the test
//                cout <<    rpcTempCU->getAddr() << ", " << rpcTempCU->getZorderIdxInCU() <<  ", After initEstDataSC Reference: " << endl;
//                rpcTempCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//                
//                cout << ", After initEstDataSC SC: " << endl;
//                rpcTempCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//                //-----Debug-----
                
                //-----Debug-----// failed the test
//                cout <<    rpcBestCU->getAddr() << ", " << rpcBestCU->getZorderIdxInCU() <<  ", After initEstDataSC Reference Best: " << endl;
//                rpcBestCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//                
//                cout << ", After initEstDataSC SC Best: " << endl;
//                rpcBestCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
                //-----Debug-----


                
                //                }
                
            }// end if != I_Slice
            
            if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
            {
                iQP = iMinQP;
            }
            
        }// end for loop
        
        // No need for entropy encoder stuff
        
        
    }// end if(!SliceEnd...etc)
    else if(!(bSliceEnd && bInsidePicture))
    {
        bBoundary = true;
    }
    
    // ignore PCM,
    
    // QP stuff
    if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        //        cout << "g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
    }
    else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        //         cout << "g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
        iMinQP = iBaseQP;
        iMaxQP = iBaseQP;
    }
    else
    {
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic->getCU(rpcTempCU->getAddr()) << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU->getZorderIdxInCU() << endl;
#if SC_ENABLE_PRINT
        cout << "Flafeelo Should be equal!??? " << (pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr()) << endl;
#endif
        
        Int iStartQP;
        if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
        {
            
            //            cout << "Mini If part " << endl;
            iStartQP = rpcTempCU->getQP(0);
            //            cout << "Mini If part done" << endl;
        }
        else
        {
            //            cout << "Mini else part 1" << endl;
            UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
            
            //            cout << "Mini else part 2 " << (rpcTempCU->m_phQP==NULL) << ", " << uiCurSliceStartPartIdx << endl;
            //            cout << "Mini else part 2 " << (rpcTempCU->m_phQP[uiCurSliceStartPartIdx]) << endl;
            
            iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
            //            iStartQP = 22;
            //            cout << "Mini else part done " << endl;
        }
        iMinQP = iStartQP;
        iMaxQP = iStartQP;
        
        //        cout << "Flafeelo END cPic->getCU( rpcTempCU->getAddr() ) " << endl;
    }
    
    // ignore rate control, Transequent flag
    
    
// Hossam: Scene change 32x32
//    cout << "xCompressRunATORG W: " << static_cast<unsigned>(*rpcTempCU->getWidth())  << ", H: " << static_cast<unsigned>(*rpcTempCU->getWidth()) << endl;

    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
        
        // Hossam: This has to be changed to a limited SC depth
        // further split
        if( bSubBranch && uiDepth < SC_AT_DEPTH)
        {
            //            cout << "I'm about to split and go into the next depth " << endl;
            // Hossam: Increment Depth
            UChar       uhNextDepth         = uiDepth+1;
            TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
            TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
            DEBUG_STRING_NEW(sTempDebug)
            
#if SC_ENABLE_PRINT
            cout << "////////Next Depth Probe Marry me: \\\\\\\\" << Int(uhNextDepth) << endl;
#endif
            
            for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
            {
//                pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
//                pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );
                
                // Hossam: Changed SUB CU SC
                pcSubBestPartCU->initSubCUSC( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
                pcSubTempPartCU->initSubCUSC( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );

                
                Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
                if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
                {
                    
                    // Hossam: ignore entropy stuff
#if AMP_ENC_SPEEDUP
                    DEBUG_STRING_NEW(sChild)
                    if ( !rpcBestCU->isInter(0) )
                    {
                        // Hossam: Call of Compress New
                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
#if SC_ENABLE_PRINT
                        cout << "Rec call  xCompressCURunAtDepth 1" << endl;
#endif
                        
                        //                    xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
//                        xCompressCURunAtDepth( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );

                        xCompressCURunAtDepthOrg( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES, which_reference );

                        
                    }
                    else
                    {
#if SC_ENABLE_PRINT
                        cout << "Rec call  xCompressCURunAtDepth 2 bits: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
#endif
                        
                        //                        cout << "REport: " << pcSubBestPartCU ->getTotalBits() <<", " << pcSubTempPartCU->getTotalCost() << endl;
                        //
                        //                        cout << "REport: " << rpcBestCU ->getTotalBits() <<", " << rpcTempCU->getTotalCost() << endl;
                        
                        
                        // Hossam: Call of Compress New
                        //                    xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                        
                        xCompressCURunAtDepthOrg( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0), which_reference );

//                        xCompressCURunAtDepth( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                    }
                    DEBUG_STRING_APPEND(sTempDebug, sChild)
                    
#else
                    
#if SC_ENABLE_PRINT
                    // Hossam: Call of Compress New
                    cout << "Rec call  xCompressCUNew 3" << endl;
#endif
                    
                    //                xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
//                    xCompressCURunAtDepth( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
                      xCompressCURunAtDepthOrg( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
                    
                    //                    xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif
                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
                    xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
                    
                }// end if bInSlice && getCElPelX
                
                else if(bInSlice)
                {
                    // Hossam: Scene change: I guess that might not be needed for the current purpose!
//                    cout << "xCompressORG Will start copying!!! heree : " << pcSubBestPartCU->getCUMvField(Ref) << endl;
                    pcSubBestPartCU->copyToPic(uhNextDepth);
                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
                }
                
            }// end for loop on the part Id
            
           
            // Hossam: Move the temp to best XXXXX
            // I think this should be removed - won't make a difference, since i copy everytime
//          xCopyTempToBestSC( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
        }// end if subbranch
        
        
    }// end for min-max QP to subbranch
    
    
    // Take a copy from the best buffer into the buffer at 0
    
    //    xCheckBestModeLastCall(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false)); // RD compare current larger prediction
    
    
    // Propogate the best buffer to buffer # 0
    xPropogateBestResidualsAtDepth(rpcBestCU, rpcTempCU, uiDepth);
    
  
    //-----Debug-----
//                    cout <<    rpcBestCU->getAddr() << ", " << rpcBestCU->getZorderIdxInCU() <<  ", After xCompressRunAtDepth Reference: " << endl;
//                    rpcBestCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//    
//                    cout << ", After xCompressRunAtDepth SC: " << endl;
//                    rpcBestCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//-----Debug-----
    
    // Hossam: Scene change: Copy the best CU to the Picture
      rpcBestCU->copyToPicSC(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.
//    rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.
    
//    xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv

//    cout <<    rpcTempCU->getAddr() << ", " << rpcTempCU->getZorderIdxInCU() <<  ", After xCompressRunAtDepth Reference: " << endl;
//    rpcTempCU->getCUMvField(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//    
//    cout << ", After xCompressRunAtDepth SC: " << endl;
//    rpcTempCU->getCUMvFieldSC(REF_PIC_LIST_0)->printAllMv(PartSize(0), 0, 0);
//
    //-----Debug-----
    // dork
    //    if (uiDepth <= SC_MAX_DEPTH) {
    //
    //    }
    //    else if(uiDepth == 2)
    //    {
    //
    //    }
    //    else
    //        return;
    
    
}// end xCompressCURunAtDepthOrg

// Working xCompressCUNew with full depth
static int call = 2082;

#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCURunAtDepth( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize )
#else
Void TEncCu::xCompressCURunAtDepth( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
    TComPic* pcPic = rpcBestCU->getPic();
    DEBUG_STRING_NEW(sDebug)
    
#if SC_ENABLE_PRINT
    cout << "========================" << endl;
    //        cout << " \n\nxCOmpressCUNew says hi from start " << uiDepth << endl;
    cout << " \n\xCompressCURunAtDepth says hi from start " << (++call) << ", dep: " << uiDepth  << endl;
    cout << "========================" << endl;
#endif
    
    
    
    // get Original YUV data from picture
    m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );
    
    // variable for Early CU determination
    Bool    bSubBranch = true;
    
    // variable for Cbf fast mode PU decision
    Bool    doNotBlockPu = true;
    Bool    earlyDetectionSkipMode = false;
    
    Bool bBoundary = false;
    UInt uiLPelX   = rpcBestCU->getCUPelX();
    UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
    UInt uiTPelY   = rpcBestCU->getCUPelY();
    UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;
    
    Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
    Int iMinQP;
    Int iMaxQP;
    Bool isAddLowestQP = false;
    
    const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();
    
    
    // Hossam: Now the min QP is equal to the max Qp -- no rate control
    if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
    }
    else
    {
        iMinQP = rpcTempCU->getQP(0);
        iMaxQP = rpcTempCU->getQP(0);
    }
    
    const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.
    
    
#if SC_ENABLE_PRINT
    cout << "xCompressCUNew transequent flag " << (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) << endl;
#endif
    
    if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
    {
        isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
        iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
        if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
        {
            iMaxQP = iMinQP;
            
        }
    }
    
    // If slice start or slice end is within this cu...
    TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
    Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
    Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
    
    
    // We need to split, so don't try these modes.
    
    if( (!bSliceEnd && !bSliceStart && bInsidePicture) && uiDepth == SC_AT_DEPTH)
    {
        for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
        {
            const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);
            
            if (bIsLosslessMode)
            {
                iQP = lowestQP;
            }
            
            m_ChromaQpAdjIdc = 0;
            if (pcSlice->getUseChromaQpAdj())
            {
                /* Pre-estimation of chroma QP based on input block activity may be performed
                 * here, using for example m_ppcOrigYuv[uiDepth] */
                /* To exercise the current code, the index used for adjustment is based on
                 * block position
                 */
                Int lgMinCuSize = pcSlice->getSPS()->getLog2MinCodingBlockSize();
                m_ChromaQpAdjIdc = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pcSlice->getPPS()->getChromaQpAdjTableSize() + 1);
            }
            
#if SC_ENABLE_PRINT
            cout << " xCompressCU New (!bSliceEnd && !bSliceStart && bInsidePicture ) calls initEstData " << endl;
#endif
            // Init temp CU
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            
            
            // do inter modes, 2Nx2N
            if (rpcBestCU->getSlice()->getSliceType() != I_SLICE) {
                
                //                if(m_pcEncCfg->getUseEarlySkipDetection())
                //                {
                xCheckRDCostInterAtDepth( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
                //                }
                
            }// end if != I_Slice
            
            if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
            {
                iQP = iMinQP;
            }
            
        }// end for loop
        
        // No need for entropy encoder stuff
        
        
    }// end if(!SliceEnd...etc)
    else if(!(bSliceEnd && bInsidePicture))
    {
        bBoundary = true;
    }
    
    // ignore PCM,
    
    // QP stuff
    if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        //        cout << "g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
    }
    else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        //         cout << "g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
        iMinQP = iBaseQP;
        iMaxQP = iBaseQP;
    }
    else
    {
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic->getCU(rpcTempCU->getAddr()) << endl;
        //        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU->getZorderIdxInCU() << endl;
#if SC_ENABLE_PRINT
        cout << "Flafeelo Should be equal!??? " << (pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr()) << endl;
#endif
        
        Int iStartQP;
        if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
        {
            
            //            cout << "Mini If part " << endl;
            iStartQP = rpcTempCU->getQP(0);
            //            cout << "Mini If part done" << endl;
        }
        else
        {
            //            cout << "Mini else part 1" << endl;
            UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
            
            //            cout << "Mini else part 2 " << (rpcTempCU->m_phQP==NULL) << ", " << uiCurSliceStartPartIdx << endl;
            //            cout << "Mini else part 2 " << (rpcTempCU->m_phQP[uiCurSliceStartPartIdx]) << endl;
            
            iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
            //            iStartQP = 22;
            //            cout << "Mini else part done " << endl;
        }
        iMinQP = iStartQP;
        iMaxQP = iStartQP;
        
        //        cout << "Flafeelo END cPic->getCU( rpcTempCU->getAddr() ) " << endl;
    }
    
    // ignore rate control, Transequent flag
    
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
        
        // Hossam: This has to be changed to a limited SC depth
        // further split
        if( bSubBranch && uiDepth < SC_AT_DEPTH)
        {
            //            cout << "I'm about to split and go into the next depth " << endl;
            // Hossam: Increment Depth
            UChar       uhNextDepth         = uiDepth+1;
            TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
            TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
            DEBUG_STRING_NEW(sTempDebug)
            
#if SC_ENABLE_PRINT
            cout << "////////Next Depth Probe Marry me: \\\\\\\\" << Int(uhNextDepth) << endl;
#endif
            
            for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
            {
                pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
                pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );
                
                Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
                if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
                {
                    
                    // Hossam: ignore entropy stuff
#if AMP_ENC_SPEEDUP
                    DEBUG_STRING_NEW(sChild)
                    if ( !rpcBestCU->isInter(0) )
                    {
                        // Hossam: Call of Compress New
                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
#if SC_ENABLE_PRINT
                        cout << "Rec call  xCompressCURunAtDepth 1" << endl;
#endif
                        
                        //                    xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
                        xCompressCURunAtDepth( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
                        
                    }
                    else
                    {
#if SC_ENABLE_PRINT
                        cout << "Rec call  xCompressCURunAtDepth 2 bits: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
#endif
                        
                        //                        cout << "REport: " << pcSubBestPartCU ->getTotalBits() <<", " << pcSubTempPartCU->getTotalCost() << endl;
                        //
                        //                        cout << "REport: " << rpcBestCU ->getTotalBits() <<", " << rpcTempCU->getTotalCost() << endl;
                        
                        
                        // Hossam: Call of Compress New
                        //                    xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                        
                        xCompressCURunAtDepth( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                    }
                    DEBUG_STRING_APPEND(sTempDebug, sChild)
                    
#else
                    
#if SC_ENABLE_PRINT
                    // Hossam: Call of Compress New
                    cout << "Rec call  xCompressCUNew 3" << endl;
#endif
                    
                    //                xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
                    xCompressCURunAtDepth( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
                    //                    xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif
                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
                    xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
                    
                }// end if bInSlice && getCElPelX
                
                else if(bInSlice)
                {
                    pcSubBestPartCU->copyToPic(uhNextDepth);
                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
                }
                
            }// end for loop on the part Id
            
        }// end if subbranch
        
        
    }// end for min-max QP to subbranch
    
    
    // Take a copy from the best buffer into the buffer at 0
    
    //    xCheckBestModeLastCall(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false)); // RD compare current larger prediction
    
    
    // Propogate the best buffer to buffer # 0
    xPropogateBestResidualsAtDepth(rpcBestCU, rpcTempCU, uiDepth);
    
    // dork
    //    if (uiDepth <= SC_MAX_DEPTH) {
    //
    //    }
    //    else if(uiDepth == 2)
    //    {
    //        
    //    }
    //    else
    //        return;
    
    
}// end xCompressCURunAtDepth



#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCUNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize )
#else
Void TEncCu::xCompressCUNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
    TComPic* pcPic = rpcBestCU->getPic();
    DEBUG_STRING_NEW(sDebug)
    
#if SC_ENABLE_PRINT
        cout << "========================" << endl;
//        cout << " \n\nxCOmpressCUNew says hi from start " << uiDepth << endl;
            cout << " \n\nxCOmpressCUNew says hi from start " << (++call) << ", dep: " << uiDepth  << endl;
        cout << "========================" << endl;
#endif
    
    // get Original YUV data from picture
    m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );
    

    // variable for Early CU determination
    Bool    bSubBranch = true;
    
    // variable for Cbf fast mode PU decision
    Bool    doNotBlockPu = true;
    Bool    earlyDetectionSkipMode = false;
    
    Bool bBoundary = false;
    UInt uiLPelX   = rpcBestCU->getCUPelX();
    UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
    UInt uiTPelY   = rpcBestCU->getCUPelY();
    UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;
    
    // XXXX Hossam: I don't need use adaptive QP XXXXX No need -- Just get!
//    Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
    Int iBaseQP = rpcBestCU->getSlice()->getSliceQp();
    Int iMinQP;
    Int iMaxQP;
    Bool isAddLowestQP = false;
    
    const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();
    
    if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
    }
    else
    {
        iMinQP = rpcTempCU->getQP(0);
        iMaxQP = rpcTempCU->getQP(0);
    }
    
    // Hossam: I think I need to remove that!!!! XXXXXX
//    if ( m_pcEncCfg->getUseRateCtrl() )
//    {
//        iMinQP = m_pcRateCtrl->getRCQP();
//        iMaxQP = m_pcRateCtrl->getRCQP();
//    }
    
    // transquant-bypass (TQB) processing loop variable initialisation ---
    
    const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.
    
#if SC_ENABLE_PRINT
    cout << "xCompressCUNew transequent flag " << (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) << endl;
#endif
    
    // Hossam: Transequent flag is false XXXXX need to remove it!!!!
    if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
    {
        isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
        iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
        if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
        {
            iMaxQP = iMinQP;
        }
    }
    
    // If slice start or slice end is within this cu...
    TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
    Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
    Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
    
    // Hossam: He splits only on these conditions
    // We need to split, so don't try these modes.
    // If there's no slice start, slice end in this picture
    if(!bSliceEnd && !bSliceStart && bInsidePicture )
    {
        for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
        {
            
            //        cout << " xCOmpressCU says hi " << endl;
            const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);
            
            if (bIsLosslessMode)
            {
                iQP = lowestQP;
            }
            
            m_ChromaQpAdjIdc = 0;
            if (pcSlice->getUseChromaQpAdj())
            {
                /* Pre-estimation of chroma QP based on input block activity may be performed
                 * here, using for example m_ppcOrigYuv[uiDepth] */
                /* To exercise the current code, the index used for adjustment is based on
                 * block position
                 */
                Int lgMinCuSize = pcSlice->getSPS()->getLog2MinCodingBlockSize();
                m_ChromaQpAdjIdc = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pcSlice->getPPS()->getChromaQpAdjTableSize() + 1);
            }
            
            /** initialize prediction data with enabling sub-LCU-level delta QP
             *\param  uiDepth  depth of the current CU
             *\param  qp     qp for the current CU
             *- set CU width and CU height according to depth
             *- set qp value according to input qp
             *- set last-coded qp value according to input last-coded qp
             */
#if SC_ENABLE_PRINT
             cout << " xCompressCU New (!bSliceEnd && !bSliceStart && bInsidePicture ) calls initEstData " << endl;
#endif
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            
            // do inter modes, SKIP and 2Nx2N
            if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
            {
//                cout << "I'm Inter CU ppl :) " << endl;
                
//                if(call==2087)
//                {
//                    cout << "2087 Bits: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
//                }

#if SC_ENABLE_PRINT
                // It's false!
                cout << "xCompressCUNew m_pcEncCfg->getUseEarlySkipDetection() flag is if true remove!: " << m_pcEncCfg->getUseEarlySkipDetection() << endl; // false
#endif
                
                
                // 2Nx2N
                if(m_pcEncCfg->getUseEarlySkipDetection())
                {
                    // Check RD cost Inter
//                    xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );

#if SC_ENABLE_PRINT
                    // Hossam: check RD cost Inter new XXXX
                     cout << "\n xCompressCU New will call xCheckRDCostInterNew 1" << endl;
#endif
                    xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
                    
//                    cout << "Will Call initEstData 2Nx2N " << endl;
                    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
//                    cout << "ACK initEstData " << endl;
                }
                // SKIP
//                xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N

#if SC_ENABLE_PRINT
//                cout << "xCheckRDCostMerge2Nx2NNew  1" << endl;
                cout << "\nxCheckRDCostMerge2Nx2NNew  1 cost " << rpcTempCU->getTotalCost() << endl;
#endif
                
//                if(call==2087)
//                {
//                    cout << "2087 Bits 2: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
//                }

                
               xCheckRDCostMerge2Nx2NNew( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
                
//                cout << "Will Call initEstData " << endl;
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                cout << "Call initEstData All Done! " << endl;
//                cout << "initEstData  " << endl;
                
//                cout << "ACK initEstData " << endl;
//                cout << "xCompressCUNew m_pcEncCfg->getUseEarlySkipDetection() flag is if true remove!: " << m_pcEncCfg->getUseEarlySkipDetection() << endl;
                // Hossam: No need to check this case for now -- unsatisfied condition
                if(!m_pcEncCfg->getUseEarlySkipDetection())
                {
#if SC_ENABLE_PRINT
                     cout << "\n xCompressCU New will call xCheckRDCostInterNew  Inter Inside Not Early:  " << endl;
#endif
                // 2Nx2N, NxN
                    // Hossam: New version of the xCheckRDCostInter new
//                    xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );


                    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                    if(m_pcEncCfg->getUseCbfFastMode())
                    {
                        doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                    }
                }
            }// end if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
            
            //======================================================================
            
            if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
            {
                iQP = iMinQP;
            }
        }
        
//        cout << " earlyDetectionSkipMode: Should be false to enter: " << earlyDetectionSkipMode << endl;
        // Hossam: This case is not visited so far!
        if(!earlyDetectionSkipMode)
        {
            for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
            {
                const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP); // If lossless, then iQP is irrelevant for subsequent modules.
                
                if (bIsLosslessMode)
                {
                    iQP = lowestQP;
                }
                
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                
                // do inter modes, NxN, 2NxN, and Nx2N
                if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
                {
                    // 2Nx2N, NxN
                    if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
                    {
                        if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
                        {
//                            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
                            
                            
#if SC_ENABLE_PRINT
                            cout << " \nxCompressCU New will call xCheckRDCostInterNew 2" << endl;
#endif
                            // Hossam: New version of the xCheckRDCostInterNew XXX
                            xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
                            
                            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        }
                    }
                    
                    if(doNotBlockPu)
                    {
//                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );

#if SC_ENABLE_PRINT
                        cout << " \nxCompressCU New will call xCheckRDCostInterNew 3" << endl;
#endif
                        // Hossam: New version of the xCheckRDCostInter XXXX
                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
                        {
                            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                        }
                    }
                    if(doNotBlockPu)
                    {
#if SC_ENABLE_PRINT
                        // Hossam: Nev version of the xCheckRDCostInter XXXX
                        cout << " \nxCompressCU New will call xCheckRDCostInterNew 4" << endl;
#endif
//                        xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );
                        xCheckRDCostInterNew      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
                        {
                            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                        }
                    }
                    
                    //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
                    if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
                    {
#if AMP_ENC_SPEEDUP
                        Bool bTestAMP_Hor = false, bTestAMP_Ver = false;
                        
#if AMP_MRG
                        Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;
                        
                        deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
                        deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif
                        
                        //! Do horizontal AMP
                        if ( bTestAMP_Hor )
                        {
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 5" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                                }
                            }
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 6" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                                }
                            }
                        }
#if AMP_MRG
                        else if ( bTestMergeAMP_Hor )
                        {
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 7" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                                }
                            }
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 8" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                                }
                            }
                        }
#endif

#if SC_ENABLE_PRINT
//                        cout << "Vertical and horizontal flags: " << boolalpha << bTestAMP_Ver << ", " << bTestAMP_Hor <<  endl;
                        cout << "Vertical MERGE AMP FLAG : " << boolalpha << bTestMergeAMP_Ver << ", rpcTempCU " << rpcTempCU->getTotalCost()<<  endl;
#endif

                        //! Do horizontal AMP
                        if ( bTestAMP_Ver )
                        {
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 9" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                                }
                            }
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 10" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                            }
                        }
#if AMP_MRG
                        
                        else if ( bTestMergeAMP_Ver )
                        {
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 11" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                                {
                                    doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                                }
                            }
                            if(doNotBlockPu)
                            {
#if SC_ENABLE_PRINT
                                cout << " \nxCompressCU New will call xCheckRDCostInterNew 12" << endl;
#endif
                                // Hossam: New version of the xCheckRDCostInter XXXX
                                xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
//                                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                            }
                        }
#endif
                        
#else                   

#if SC_ENABLE_PRINT
                        cout << " \nxCompressCU New will call xCheckRDCostInterNew 13 A lot" << endl;
#endif
                        // Hossam: New version of the xCheckRDCostInter XXXX
//                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
//                        
//                        xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
//                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnU );
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_2NxnD );
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nLx2N );
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        
                        xCheckRDCostInterNew( rpcBestCU, rpcTempCU, SIZE_nRx2N );
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

                        
#endif
                    }
                }

#if SC_ENABLE_INTRA_MODE
                // do normal intra modes
                // speedup for inter frames
                Double intraCost = 0.0;
                
                if((rpcBestCU->getSlice()->getSliceType() == I_SLICE)                                     ||
                   (rpcBestCU->getCbf( 0, COMPONENT_Y  ) != 0)                                            ||
                   ((rpcBestCU->getCbf( 0, COMPONENT_Cb ) != 0) && (numberValidComponents > COMPONENT_Cb)) ||
                   ((rpcBestCU->getCbf( 0, COMPONENT_Cr ) != 0) && (numberValidComponents > COMPONENT_Cr))  ) // avoid very complex intra if it is unlikely
                {
                    cout << "Check INTRA 1 haleeeeeem" << endl;

                    xCheckRDCostIntraNew( rpcBestCU, rpcTempCU, intraCost, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );

//                    xCheckRDCostIntra( rpcBestCU, rpcTempCU, intraCost, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
                    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                    // Hossam: 6 - 0
                    if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
                    {
                        if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
                        {
                            cout << "Check INTRA 2 " << endl;
                            Double tmpIntraCost;
                            xCheckRDCostIntraNew( rpcBestCU, rpcTempCU, tmpIntraCost, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );

                            
//                            xCheckRDCostIntra( rpcBestCU, rpcTempCU, tmpIntraCost, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
                            intraCost = std::min(intraCost, tmpIntraCost);
                            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                        }
                    }
                }
#endif

                
                // Hossam: Probably not needed XXXXX
                // test PCM
                if(pcPic->getSlice(0)->getSPS()->getUsePCM()
                   && rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
                   && rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
                {
                    UInt uiRawBits = getTotalBits(rpcBestCU->getWidth(0), rpcBestCU->getHeight(0), rpcBestCU->getPic()->getChromaFormat(), g_bitDepth);
                    UInt uiBestBits = rpcBestCU->getTotalBits();
                    if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
                    {
                        cout << "xCheckIntraPCM has a load buffer from Cabac " << endl;
                        xCheckIntraPCM (rpcBestCU, rpcTempCU);
                        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                    }
                }
                
                if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
                {
                    iQP = iMinQP;
                }
            }
        }
        
        // Hossam:  XXXX I think this one might not be needed
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
        rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
        rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
        rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );
        
        
//      cout << "Middle of no where writtenBits: , <bits, cost>: " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", <" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">" << endl;

//        cout << "Middle of no where <bits, cost>: " << "<" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">" << endl;
        
        // Early CU determination
        if( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) )
        {
            bSubBranch = false;
        }
        else
        {
            bSubBranch = true;
        }
    }
    else if(!(bSliceEnd && bInsidePicture))
    {
        bBoundary = true;
    }
    
    // Hossam: XXXX I think I might not need that XXXXX
    // copy orginal YUV samples to PCM buffer
    
//    cout << "I never enter the xFillPCMBuffer case " << endl;
    if( rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
    {
//        cout << "xFillPCMBuffer" << endl;
        xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
    }
    
//    cout << "About to check g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() " << endl;

    if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
//        cout << "g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
        Int idQP = m_pcEncCfg->getMaxDeltaQP();
        iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
        iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
    }
    else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
    {
//         cout << "g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize()" << endl;
        iMinQP = iBaseQP;
        iMaxQP = iBaseQP;
    }
    else
    {
//        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << endl;
//        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU << endl;
//        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic << endl;
//        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << pcPic->getCU(rpcTempCU->getAddr()) << endl;
//        cout << "Flafeelo pcPic->getCU( rpcTempCU->getAddr() ) " << rpcTempCU->getZorderIdxInCU() << endl;
#if SC_ENABLE_PRINT
        cout << "Flafeelo Should be equal!??? " << (pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr()) << endl;
#endif
        
        Int iStartQP;
        if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
        {
            
//            cout << "Mini If part " << endl;
            iStartQP = rpcTempCU->getQP(0);
//            cout << "Mini If part done" << endl;
        }
        else
        {
//            cout << "Mini else part 1" << endl;
            UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
            
//            cout << "Mini else part 2 " << (rpcTempCU->m_phQP==NULL) << ", " << uiCurSliceStartPartIdx << endl;
//            cout << "Mini else part 2 " << (rpcTempCU->m_phQP[uiCurSliceStartPartIdx]) << endl;
            
            iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
//            iStartQP = 22;
//            cout << "Mini else part done " << endl;
        }
        iMinQP = iStartQP;
        iMaxQP = iStartQP;
        
//        cout << "Flafeelo END cPic->getCU( rpcTempCU->getAddr() ) " << endl;
    }
    
//    cout << "About to check m_pcEncCfg->getUseRateCtrl() " << endl;
    // Hossam: XXXX I think no need for that
    if ( m_pcEncCfg->getUseRateCtrl() )
    {
        iMinQP = m_pcRateCtrl->getRCQP();
        iMaxQP = m_pcRateCtrl->getRCQP();
    }
    
//    cout << "About to check m_pcEncCfg->getCUTransquantBypassFlagForceValue() " << endl;
    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
        iMaxQP = iMinQP; // If all TUs are forced into using transquant bypass, do not loop here.
    }
    
//    cout << "More splitting! " << endl;
    // Hossam: More modes -- Splitting
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
        const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.
        
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
        
        // Hossam: This has to be changed to a limited SC depth
        // further split
        if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
        {
//            cout << "I'm about to split and go into the next depth " << endl;
            // Hossam: Increment Depth
            UChar       uhNextDepth         = uiDepth+1;
            TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
            TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
            DEBUG_STRING_NEW(sTempDebug)
            
#if SC_ENABLE_PRINT
            cout << "////////Next Depth Probe Marry me: \\\\\\\\" << Int(uhNextDepth) << endl;
#endif
            for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
            {
                pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
                pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
                
                Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
                if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
                {
                    if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
                    {
//                        cout << "REMOVED m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load( part id = 0" << endl;
                        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
                    }
                    else
                    {
//                        cout << " REMOVED m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load( part id = 1" << endl;
                        
                        m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
                    }
                    
#if AMP_ENC_SPEEDUP
                    DEBUG_STRING_NEW(sChild)
                    if ( !rpcBestCU->isInter(0) )
                    {
                        // Hossam: Call of Compress New
                        //                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
#if SC_ENABLE_PRINT
                        cout << "Rec call  xCompressCUNew 1" << endl;
#endif
                        xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );

                    }
                    else
                    {
#if SC_ENABLE_PRINT
                        cout << "Rec call  xCompressCUNew 2 bits: " << m_pcEntropyCoder->getNumberOfWrittenBits()<< endl;
#endif
                        
//                        cout << "REport: " << pcSubBestPartCU ->getTotalBits() <<", " << pcSubTempPartCU->getTotalCost() << endl;
//                        
//                        cout << "REport: " << rpcBestCU ->getTotalBits() <<", " << rpcTempCU->getTotalCost() << endl;
                        
                       // Hossam: Call of Compress New
                        xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
//                        xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
                    }
                    DEBUG_STRING_APPEND(sTempDebug, sChild)
#else

#if SC_ENABLE_PRINT
                    // Hossam: Call of Compress New
                    cout << "Rec call  xCompressCUNew 3" << endl;
#endif
                    
                    xCompressCUNew( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
//                    xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif
                    
                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
                    xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
                }
                else if (bInSlice)
                {
                    pcSubBestPartCU->copyToPic( uhNextDepth );
                    rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
                }
            }
            
            if( !bBoundary )
            {
                // Hossam: Removed the encode steps XXXX
                m_pcEntropyCoder->resetBits();
                m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );
                
//                cout << "Entropy Encoder needs written bits and bins" << endl;
                rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
                rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
                
//                cout << "Entropy Encoder needs written bits and bins Done" << endl;
            }
            rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
            
            if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
            {
                
//                cout << "About to loop on parts in mysterious loop " << endl;
                Bool hasResidual = false;
                for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
                {
                    if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) &&
                       (     rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Y)
                        || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cb) && (numberValidComponents > COMPONENT_Cb))
                        || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cr) && (numberValidComponents > COMPONENT_Cr)) ) )
                    {
                        hasResidual = true;
                        break;
                    }
                }
                
                UInt uiTargetPartIdx;
                if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
                {
                    uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
                }
                else
                {
                    uiTargetPartIdx = 0;
                }
                if ( hasResidual )
                {
#if !RDO_WITHOUT_DQP_BITS
                    // Hossam: Removed encode steps --> Reset Bits might be removed XXXXXX
                    m_pcEntropyCoder->resetBits();
                    m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
                    
//                    cout << "Remove maybe Entropy 3ak!!! 1" << endl;
                    rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
//                    cout << "Remove maybe Entropy 3ak!!! 2" << endl;
                    rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
                    
//                    cout << "Entropy 3ak!!! 3" << endl;
                    rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
#endif
                    
                    Bool foundNonZeroCbf = false;
                    rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
                    assert( foundNonZeroCbf );
                }
                else
                {
                    rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
                }
            }
            
//            cout << "Remove Entropy 3ak!!! 4" << endl;
            m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
//            cout << "Remove Entropy 3ak!!! 5" << endl;
            
            Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
            && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
            Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
            && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
            if(isEndOfSlice||isEndOfSliceSegment)
            {
                if (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING)
                    rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost() + (1.0 / m_pcRdCost->getLambda());
                else
                    rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
            }

            // Hossam: Use the new version of the check best mode for future changes
            
            xCheckBestModeLastCall(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false)); // RD compare current larger prediction
//            xCheckBestModeNew( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
//            xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
            // with sub partitioned prediction.
        }
    }
    
    DEBUG_STRING_APPEND(sDebug_, sDebug);
    
    // Hossam: Scene change
//    xSaveResiduals(rpcBestCU, rpcTempCU, uiDepth);
    
    rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.
    
    xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
    if( bBoundary ||(bSliceEnd && bInsidePicture))
    {
//        cout << "BRAVE HEART OF THE COMPRESS SLICEEEEEEEEE " << endl;
        return;
    }
    
    // Assert if Best prediction mode is NONE
    // Selected mode's RD-cost must be not MAX_DOUBLE.
    assert( rpcBestCU->getPartitionSize ( 0 ) != NUMBER_OF_PART_SIZES       );
    assert( rpcBestCU->getPredictionMode( 0 ) != NUMBER_OF_PREDICTION_MODES );
    assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE                 );
}// end isCompressNew


// Hossam: XXXXX TO Be REMOVED
Void TEncCu::xSaveResiduals(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth)
{;}



// Hossam: Scene change extraction of CU information
Void TEncCu::xExtractCUInfo( TComDataCU* pcCU, UInt uiAbsPartIdx,   UInt uiDepth, UInt uiPartUnitIdx)
{
    
    TComPic* pcPic = pcCU->getPic();
    Bool bBoundary = false;
    
    
    // Hossam: Scene change
    // <<uiLPelX, uiTPelY>> defines the co-ordinates of the upper left corner  for a given partition of the CU
    // <<uiRPelX, uiBPelY>> defines the co-ordinates of the lower right corner for a given partition of the CU
    // uiAbsPartIdx is the one used to extract a certain CU
    
    UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
    UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
    
    
    UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
    
    
    UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
    
    // cout << "Function Start: uiDepth: " << uiDepth << ", uiAbsPartIdx " << uiAbsPartIdx << ", Z Scan To Raster: " << g_auiZscanToRaster[uiAbsPartIdx]  << endl;
    //    cout << "Z Scan To Raster: " << g_auiZscanToRaster[uiAbsPartIdx] << endl;
    
    
    TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
    
    // If slice start is within this cu...
    Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx &&
    pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcPic->getNumPartInCU() >> (uiDepth<<1) );
    
    // We need to split, so don't try these modes.
    if(!bSliceStart&&( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
        // CONFUSING INfo: Hossam
        //        cout << "SPLITT dep: " << (int) pcCU->getDepth(uiAbsPartIdx) << ", uiAbsPartIdx: " << uiAbsPartIdx << endl;
//        cout << "SPLITT dep Inside xLabel: "  << endl;
        
        
    }
    else
    {
        // it never comes here
//        cout << "NS dep: " << (int) pcCU->getDepth(uiAbsPartIdx) << ", uiAbsPartIdx: " << uiAbsPartIdx << endl;
        bBoundary = true;
    }
    
    
    // Abdullah: Move on to the Dump Partition onto the YUV file
//    pcCU->dumpPartitionYUV(uiDepth, uiAbsPartIdx, uiPartUnitIdx, cu_label_id);
    
    // Extract the partition information
    pcCU->xExtractPartitionInfo(uiDepth, uiAbsPartIdx, uiPartUnitIdx);
    

    
    
    // g_uiAddCUDepth = 0 and does not change
    if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
    {
        UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
        //        cout << "uiQNumParts " << uiQNumParts << endl;
        
        
        for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
        {
            //            UInt partAdd;
            //            Int width, height;
            //            rpcBestCU->getPartIndexAndSize(uiPartUnitIdx, partAdd, width, height);
            //            cout << "Function loop: uiPartUnitIdx: " << uiPartUnitIdx << ", " << ", partAdd " << partAdd << ", "  << width << ", " << height << endl;
            //            cout << "Function loop: uiPartUnitIdx: " <<  (int) rpcBestCU->getPartitionSize(0) << endl;
            
            //          cout << "Function loop: uiPartUnitIdx: " << uiPartUnitIdx << ", " << ", "  << width << ", " << height << endl;
            
            
            uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
            uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
            Bool bInSlice = pcCU->getSCUAddr()+uiAbsPartIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurEndCUAddr();
            
            if(bInSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
            {
                //                xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1 );
                //                xLabelCU(pcCU, rpcTempCU, uiAbsPartIdx, uiDepth+1);
                
                xExtractCUInfo(pcCU, uiAbsPartIdx, uiDepth+1,  uiPartUnitIdx);
            }
        }
        return;
    }
    else if( uiDepth == pcCU->getDepth( uiAbsPartIdx ) )
    {
        //        cout << "Function loop: uiPartUnitIdx: " << uiPartUnitIdx << ", " << ", "  << width << ", " << height << endl;
        
//        cout << "Try LinkedIn case  " << pcCU->getPartitionSize(0) << endl;
        
        return;
    }
    
    
    
}// end xExtractCUInfo

/** Compress a CU block recursively with enabling sub-LCU-level delta QP
 *\param   rpcBestCU
 *\param   rpcTempCU
 *\param   uiDepth
 *\returns Void
 *
 *- for loop of QP value to compress the current CU with all possible QP
*/

#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize )
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth )
#endif
{
  TComPic* pcPic = rpcBestCU->getPic();
  DEBUG_STRING_NEW(sDebug)

//    cout << " xCOmpressCU says hi from start " << endl;
    
  // get Original YUV data from picture
  m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU() );

    // variable for Early CU determination
  Bool    bSubBranch = true;

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool    earlyDetectionSkipMode = false;

  Bool bBoundary = false;
  UInt uiLPelX   = rpcBestCU->getCUPelX();
  UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;
  UInt uiTPelY   = rpcBestCU->getCUPelY();
  UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;

  Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;

  const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();

  if( (g_uiMaxCUWidth>>uiDepth) >= rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
  }
  else
  {
    iMinQP = rpcTempCU->getQP(0);
    iMaxQP = rpcTempCU->getQP(0);
  }

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  // transquant-bypass (TQB) processing loop variable initialisation ---

  const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

  if ( (rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag()) )
  {
    isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.
    iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs
    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
      iMaxQP = iMinQP;
    }
  }

  // If slice start or slice end is within this cu...
  TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurStartCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart();
  Bool bSliceEnd = (pcSlice->getSliceSegmentCurEndCUAddr()>rpcTempCU->getSCUAddr()&&pcSlice->getSliceSegmentCurEndCUAddr()<rpcTempCU->getSCUAddr()+rpcTempCU->getTotalNumPart());
  Bool bInsidePicture = ( uiRPelX < rpcBestCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < rpcBestCU->getSlice()->getSPS()->getPicHeightInLumaSamples() );
  // We need to split, so don't try these modes.
  if(!bSliceEnd && !bSliceStart && bInsidePicture )
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
    {
        
//        cout << " xCOmpressCU says hi " << endl;
      const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);

      if (bIsLosslessMode)
      {
        iQP = lowestQP;
      }

      m_ChromaQpAdjIdc = 0;
      if (pcSlice->getUseChromaQpAdj())
      {
        /* Pre-estimation of chroma QP based on input block activity may be performed
         * here, using for example m_ppcOrigYuv[uiDepth] */
        /* To exercise the current code, the index used for adjustment is based on
         * block position
         */
        Int lgMinCuSize = pcSlice->getSPS()->getLog2MinCodingBlockSize();
        m_ChromaQpAdjIdc = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pcSlice->getPPS()->getChromaQpAdjTableSize() + 1);
      }

      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

      // do inter modes, SKIP and 2Nx2N
      if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
      {
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())
        {
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        if(!m_pcEncCfg->getUseEarlySkipDetection())
        {
          // 2Nx2N, NxN
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if(m_pcEncCfg->getUseCbfFastMode())
          {
            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
          }
        }
      }

      if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
      {
        iQP = iMinQP;
      }
    }

    if(!earlyDetectionSkipMode)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
      {
        const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP); // If lossless, then iQP is irrelevant for subsequent modules.

        if (bIsLosslessMode)
        {
          iQP = lowestQP;
        }

        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        // do inter modes, NxN, 2NxN, and Nx2N
        if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN
          if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))
          {
            if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth && doNotBlockPu)
            {
              xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }

          if(doNotBlockPu)
          {
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }

          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if( pcPic->getSlice(0)->getSPS()->getAMPAcc(uiDepth) )
          {
#if AMP_ENC_SPEEDUP
            Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);
#else
            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }
#endif

#else
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#endif
          }
        }

        // do normal intra modes
        // speedup for inter frames
        Double intraCost = 0.0;

        if((rpcBestCU->getSlice()->getSliceType() == I_SLICE)                                     ||
           (rpcBestCU->getCbf( 0, COMPONENT_Y  ) != 0)                                            ||
          ((rpcBestCU->getCbf( 0, COMPONENT_Cb ) != 0) && (numberValidComponents > COMPONENT_Cb)) ||
          ((rpcBestCU->getCbf( 0, COMPONENT_Cr ) != 0) && (numberValidComponents > COMPONENT_Cr))  ) // avoid very complex intra if it is unlikely
        {
          xCheckRDCostIntra( rpcBestCU, rpcTempCU, intraCost, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if( uiDepth == g_uiMaxCUDepth - g_uiAddCUDepth )
          {
            if( rpcTempCU->getWidth(0) > ( 1 << rpcTempCU->getSlice()->getSPS()->getQuadtreeTULog2MinSize() ) )
            {
              Double tmpIntraCost;
              xCheckRDCostIntra( rpcBestCU, rpcTempCU, tmpIntraCost, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );
              intraCost = std::min(intraCost, tmpIntraCost);
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            }
          }
        }

        // test PCM
        if(pcPic->getSlice(0)->getSPS()->getUsePCM()
          && rpcTempCU->getWidth(0) <= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MaxSize())
          && rpcTempCU->getWidth(0) >= (1<<pcPic->getSlice(0)->getSPS()->getPCMLog2MinSize()) )
        {
          UInt uiRawBits = getTotalBits(rpcBestCU->getWidth(0), rpcBestCU->getHeight(0), rpcBestCU->getPic()->getChromaFormat(), g_bitDepth);
          UInt uiBestBits = rpcBestCU->getTotalBits();
          if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))
          {
            xCheckIntraPCM (rpcBestCU, rpcTempCU);
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          }
        }

        if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
        {
          iQP = iMinQP;
        }
      }
    }

    m_pcEntropyCoder->resetBits();
    m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );
    rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
    rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );

      
#if IS_CALC_INTERDEP_Rates_REF12 && !IS_CALC_INTERDEP_Rates_REF12_RES_ONLY
//#if IS_CALC_INTERDEP_Rates_REF12
//      cout << "YOU ARE POLLUTING THE xCOMPRESSCU" << endl;
      
    TComCUMvField*  pcCUMvField = rpcBestCU->getCUMvField( REF_PIC_LIST_0 );
//     Int which_reference = pcCUMvField->getRefIdx(uiAbsPartIdx);
    Int which_reference = pcCUMvField->getRefIdx(0);
      
     if(which_reference >= 0) {
      rpcBestCU->m_uiTotalBitsRefI[which_reference] += m_pcEntropyCoder->getNumberOfWrittenBits();
     }
      
//    cout << "----00---Middle of no where best writtenBits: , <bits, cost>: " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", <" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">, comp1: " << rpcBestCU->m_uiTotalBitsRefI[0] << endl;
//    cout << which_reference << ")----00---Middle of no where best <rpcBestCU->getTotalBits(), cost>: " << ", <" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">, comp1: " << rpcBestCU->m_uiTotalBitsRefI[0] << ", flagOne: " << (rpcBestCU->getTotalBits() == rpcBestCU->m_uiTotalBitsRefI[0]) <<  "\n"  <<  endl;
#endif
      
    // Early CU determination
    if( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->isSkipped(0) )
    {
      bSubBranch = false;
    }
    else
    {
      bSubBranch = true;
    }
  }
  else if(!(bSliceEnd && bInsidePicture))
  {
    bBoundary = true;
  }

  // copy orginal YUV samples to PCM buffer
  if( rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))
  {
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);
  }

  if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -rpcTempCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
  }
  else if( (g_uiMaxCUWidth>>uiDepth) > rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    Int iStartQP;
    if( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) == pcSlice->getSliceSegmentCurStartCUAddr())
    {
      iStartQP = rpcTempCU->getQP(0);
    }
    else
    {
      UInt uiCurSliceStartPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
      iStartQP = rpcTempCU->getQP(uiCurSliceStartPartIdx);
    }
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
  {
    iMaxQP = iMinQP; // If all TUs are forced into using transquant bypass, do not loop here.
  }

  for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)
  {
    const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.

    rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

    // further split
    if( bSubBranch && uiDepth < g_uiMaxCUDepth - g_uiAddCUDepth )
    {
      UChar       uhNextDepth         = uiDepth+1;
      TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
      DEBUG_STRING_NEW(sTempDebug)

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )
      {
        pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
        pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.

        Bool bInSlice = pcSubBestPartCU->getSCUAddr()+pcSubBestPartCU->getTotalNumPart()>pcSlice->getSliceSegmentCurStartCUAddr()&&pcSubBestPartCU->getSCUAddr()<pcSlice->getSliceSegmentCurEndCUAddr();
        if(bInSlice && ( pcSubBestPartCU->getCUPelX() < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
        {
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
          }

#if AMP_ENC_SPEEDUP
          DEBUG_STRING_NEW(sChild)
          if ( !rpcBestCU->isInter(0) )
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
          }
          else
          {

            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
          }
          DEBUG_STRING_APPEND(sTempDebug, sChild)
#else
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif

          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.
          xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );
        }
        else if (bInSlice)
        {
          pcSubBestPartCU->copyToPic( uhNextDepth );
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
        }
      }

      if( !bBoundary )
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );

        rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits
        rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          
         
#if IS_CALC_INTERDEP_Rates_REF12 && !IS_CALC_INTERDEP_Rates_REF12_RES_ONLY
//#if IS_CALC_INTERDEP_Rates_REF12
//          cout << "YOU ARE POLLUTING THE xCOMPRESSCU !bBoundary " << endl;
          TComCUMvField*  pcCUMvField = rpcTempCU->getCUMvField( REF_PIC_LIST_0 );
          //     Int which_reference = pcCUMvField->getRefIdx(uiAbsPartIdx);
          Int which_reference = pcCUMvField->getRefIdx(0);
          
          if(which_reference >= 0) {
           rpcTempCU->m_uiTotalBitsRefI[which_reference] += m_pcEntropyCoder->getNumberOfWrittenBits();
          }
          
//          rpcTempCU->showMeCU(uiDepth, 0);
//          cout << "which_reference=" << which_reference << "++++++Middle of !bBoundary no where TEMP comp1: " << rpcTempCU->m_uiTotalBitsRefI[0] << ", comp2: " <<  rpcTempCU->m_uiTotalBitsRefI[1] << ", rpcTempCU->getTotalBits(): " << rpcTempCU->getTotalBits() << ", m_pcEntropyCoder->getNumberOfWrittenBits(): " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", flagThree: " << (rpcTempCU->m_uiTotalBitsRefI[0] == rpcTempCU->getTotalBits()) << endl;
#endif

//    cout << "which_reference=" << rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx(0) << "++++++Middle of !bBoundary no where TEMP comp1: " << rpcTempCU->m_uiTotalBitsRefI[0] << ", comp2: " <<  rpcTempCU->m_uiTotalBitsRefI[1] << ", rpcTempCU->getTotalBits(): " << rpcTempCU->getTotalBits() << ", m_pcEntropyCoder->getNumberOfWrittenBits(): " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", flagThree: " << (rpcTempCU->m_uiTotalBitsRefI[0] == rpcTempCU->getTotalBits()) << endl;
          
      }// end if statement
      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

      if( (g_uiMaxCUWidth>>uiDepth) == rpcTempCU->getSlice()->getPPS()->getMinCuDQPSize() && rpcTempCU->getSlice()->getPPS()->getUseDQP())
      {
        Bool hasResidual = false;
        for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)
        {
          if( ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(uiBlkIdx+rpcTempCU->getZorderIdxInCU()) == rpcTempCU->getSlice()->getSliceSegmentCurStartCUAddr() ) &&
              (     rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Y)
                || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cb) && (numberValidComponents > COMPONENT_Cb))
                || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cr) && (numberValidComponents > COMPONENT_Cr)) ) )
          {
            hasResidual = true;
            break;
          }
        }

        UInt uiTargetPartIdx;
        if ( pcPic->getCU( rpcTempCU->getAddr() )->getSliceSegmentStartCU(rpcTempCU->getZorderIdxInCU()) != pcSlice->getSliceSegmentCurStartCUAddr() )
        {
          uiTargetPartIdx = pcSlice->getSliceSegmentCurStartCUAddr() % pcPic->getNumPartInCU() - rpcTempCU->getZorderIdxInCU();
        }
        else
        {
          uiTargetPartIdx = 0;
        }
        if ( hasResidual )
        {
#if !RDO_WITHOUT_DQP_BITS
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQP( rpcTempCU, uiTargetPartIdx, false );
          rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
          rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
            
#if IS_CALC_INTERDEP_Rates_REF12 && !IS_CALC_INTERDEP_Rates_REF12_RES_ONLY
//#if IS_CALC_INTERDEP_Rates_REF12

//            cout << "YOU ARE POLLUTING THE xCOMPRESSCU" << endl;
            TComCUMvField*  pcCUMvField = rpcTempCU->getCUMvField( REF_PIC_LIST_0 );
            //     Int which_reference = pcCUMvField->getRefIdx(uiAbsPartIdx);
            Int which_reference = pcCUMvField->getRefIdx(0);
            
            if(which_reference >= 0) {
                rpcTempCU->m_uiTotalBitsRefI[which_reference] += m_pcEntropyCoder->getNumberOfWrittenBits();
            }
            
            rpcTempCU->showMeCU(uiDepth, 0);
//            cout << "which_reference=" << which_reference <<  "-&&&&&&Middle of no where TEMP comp1: " << rpcTempCU->m_uiTotalBitsRefI[0] << endl;
            
#endif // end if calc rates12
            
#endif

          Bool foundNonZeroCbf = false;
          rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( uiTargetPartIdx ), rpcTempCU, 0, uiDepth, foundNonZeroCbf );
          assert( foundNonZeroCbf );
        }
        else
        {
          rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( uiTargetPartIdx ), 0, uiDepth ); // set QP to default QP
        }
      }

      m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

      Bool isEndOfSlice        = rpcBestCU->getSlice()->getSliceMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceArgument()<<3);
      Bool isEndOfSliceSegment = rpcBestCU->getSlice()->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
                                 && (rpcBestCU->getTotalBits()>rpcBestCU->getSlice()->getSliceSegmentArgument()<<3);
      if(isEndOfSlice||isEndOfSliceSegment)
      {
        if (m_pcEncCfg->getCostMode()==COST_MIXED_LOSSLESS_LOSSY_CODING)
          rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost() + (1.0 / m_pcRdCost->getLambda());
        else
          rpcBestCU->getTotalCost()=rpcTempCU->getTotalCost()+1;
      }

      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
                                                                                       // with sub partitioned prediction.
    }
  }

  DEBUG_STRING_APPEND(sDebug_, sDebug);

  rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.

  xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU(), uiDepth, uiDepth, rpcBestCU, uiLPelX, uiTPelY );   // Copy Yuv data to picture Yuv
    
   
//    bitsAccumulator = bitsAccumulator + rpcBestCU->getTotalBits();
//    cout << "\n-------FINAL-- " << endl;
//    rpcBestCU->showMeCU(uiDepth, 0);
//    cout << "......\n" << endl;
//    rpcBestCU->showMeCU(rpcBestCU->getDepth(0), 0);
//  cout << "FINAL Middle of no where writtenBits: , <bits, cost>: " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", <" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">" << ", bla: " << bitsAccumulator << ", rate[0]: " << rpcBestCU->m_uiTotalBitsRefI[0] << " flagTwo: " << (rpcBestCU->m_uiTotalBitsRefI[0] == rpcBestCU->getTotalBits()) << endl;
//    cout << "FINAL Middle of no where writtenBits: , <bits, cost>: " << m_pcEntropyCoder->getNumberOfWrittenBits() << ", <" << rpcBestCU->getTotalBits() << ", " << rpcBestCU->getTotalCost() << ">" << ", rate[0]: " << rpcBestCU->m_uiTotalBitsRefI[0] << " flagTwo: " << (rpcBestCU->m_uiTotalBitsRefI[0] == rpcBestCU->getTotalBits()) << endl;
    
    
  if( bBoundary ||(bSliceEnd && bInsidePicture))
  {
    return;
  }

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  assert( rpcBestCU->getPartitionSize ( 0 ) != NUMBER_OF_PART_SIZES       );
  assert( rpcBestCU->getPredictionMode( 0 ) != NUMBER_OF_PREDICTION_MODES );
  assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE                 );
}

/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TEncCu::finishCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

  //Calculate end address
  UInt uiCUAddr = pcCU->getSCUAddr()+uiAbsPartIdx;

  UInt uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
  UInt uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
  UInt uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
  UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
  while(uiPosX>=uiWidth||uiPosY>=uiHeight)
  {
    uiInternalAddress--;
    uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
    uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
  }
  uiInternalAddress++;
  if(uiInternalAddress==pcCU->getPic()->getNumPartInCU())
  {
    uiInternalAddress = 0;
    uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
  }
  UInt uiRealEndAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);

  // Encode slice finish
  Bool bTerminateSlice = false;
  if (uiCUAddr+(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)) == uiRealEndAddress)
  {
    bTerminateSlice = true;
  }
  UInt uiGranularityWidth = g_uiMaxCUWidth;
  uiPosX = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  uiPosY = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  Bool granularityBoundary=((uiPosX+pcCU->getWidth(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosX+pcCU->getWidth(uiAbsPartIdx)==uiWidth))
    &&((uiPosY+pcCU->getHeight(uiAbsPartIdx))%uiGranularityWidth==0||(uiPosY+pcCU->getHeight(uiAbsPartIdx)==uiHeight));

  if(granularityBoundary)
  {
    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    if (!bTerminateSlice)
    {
        m_pcEntropyCoder->encodeTerminatingBit( bTerminateSlice ? 1 : 0 );
    }
      
      bitsAccumulatorTerminateInfo += bTerminateSlice ? 0 : 1;
  }

  Int numberOfWrittenBits = 0;
  if (m_pcBitCounter)
  {
    numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();
  }

  // Calculate slice end IF this CU puts us over slice bit size.
  UInt iGranularitySize = pcCU->getPic()->getNumPartInCU();
  Int iGranularityEnd = ((pcCU->getSCUAddr()+uiAbsPartIdx)/iGranularitySize)*iGranularitySize;
  if(iGranularityEnd<=pcSlice->getSliceSegmentCurStartCUAddr())
  {
    iGranularityEnd+=max(iGranularitySize,(pcCU->getPic()->getNumPartInCU()>>(uiDepth<<1)));
  }
  // Set slice end parameter
  if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceBits()+numberOfWrittenBits>pcSlice->getSliceArgument()<<3)
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    pcSlice->setSliceCurEndCUAddr(iGranularityEnd);
    return;
  }
  // Set dependent slice end parameter
  if(pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES&&!pcSlice->getFinalized()&&pcSlice->getSliceSegmentBits()+numberOfWrittenBits > pcSlice->getSliceSegmentArgument()<<3)
  {
    pcSlice->setSliceSegmentCurEndCUAddr(iGranularityEnd);
    return;
  }
  if(granularityBoundary)
  {
    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);
      
    if (m_pcBitCounter)
    {
      m_pcEntropyCoder->resetBits();
    }
  }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP( TComDataCU* pcCU, UInt uiDepth )
{
  Int iBaseQp = pcCU->getSlice()->getSliceQp();
  Int iQpOffset = 0;
  if ( m_pcEncCfg->getUseAdaptiveQP() )
  {
    TEncPic* pcEPic = dynamic_cast<TEncPic*>( pcCU->getPic() );
    UInt uiAQDepth = min( uiDepth, pcEPic->getMaxAQDepth()-1 );
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer( uiAQDepth );
    UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
    UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
    UInt uiAQUStride = pcAQLayer->getAQPartStride();
    TEncQPAdaptationUnit* acAQU = pcAQLayer->getQPAdaptationUnit();

    Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange()/6.0);
    Double dAvgAct = pcAQLayer->getAvgActivity();
    Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
    Double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
    Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
    iQpOffset = Int(floor( dQpOffset + 0.49999 ));
  }

  return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQp+iQpOffset );
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
{
  TComPic* pcPic = pcCU->getPic();
    
    

  // <<uiLPelX, uiTPelY>> defines the co-ordinates of the upper left corner  for a given partition of the CU
  // <<uiRPelX, uiBPelY>> defines the co-ordinates of the lower right corner for a given partition of the CU
  Bool bBoundary = false;
  UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;

  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  // If slice start is within this cu...
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx &&
    pcSlice->getSliceSegmentCurStartCUAddr() < pcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcPic->getNumPartInCU() >> (uiDepth<<1) );
  
    
//    cout << "CU uiLPelX, uiTPelY: " << uiLPelX << ", " << uiTPelY << ", uiRPelX, uiBPelY: " << uiRPelX << ", " << uiBPelY << endl;
//    cout << "bits bits number of written bits: " << m_pcEntropyCoder->getNumberOfWrittenBits() << endl;
//    // cout << "bits bits number of written bits per CU: " << pcCU->getTotalBits() << endl;
    
  // We need to split, so don't try these modes.
  if(!bSliceStart&&( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
   
      UInt beforeSplitEntropyCoder = m_pcEntropyCoder->getNumberOfWrittenBits();
    m_pcEntropyCoder->encodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
      
      bitsAccumulatorSplitFlag += m_pcEntropyCoder->getNumberOfWrittenBits() - beforeSplitEntropyCoder;
//    cout << "Split bits now: " << m_pcEntropyCoder->getNumberOfWrittenBits() - beforeSplitEntropyCoder << endl;
//    cout << "Split bits so far: " << bitsAccumulatorSplitFlag << endl;
  }
  else
  {
    bBoundary = true;
  }
    
  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < (g_uiMaxCUDepth-g_uiAddCUDepth) ) ) || bBoundary )
  {
    UInt uiQNumParts = ( pcPic->getNumPartInCU() >> (uiDepth<<1) )>>2;
    if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
    {
      setdQPFlag(true);
    }

    if( (g_uiMaxCUWidth>>uiDepth) == pcCU->getSlice()->getPPS()->getMinCuChromaQpAdjSize() && pcCU->getSlice()->getUseChromaQpAdj())
    {
      setCodeChromaQpAdjFlag(true);
    }

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
      Bool bInSlice = pcCU->getSCUAddr()+uiAbsPartIdx+uiQNumParts>pcSlice->getSliceSegmentCurStartCUAddr()&&pcCU->getSCUAddr()+uiAbsPartIdx<pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiLPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1 );
      }
    }
    return;
  }

  if( (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() && pcCU->getSlice()->getPPS()->getUseDQP())
  {
    setdQPFlag(true);
  }

  if( (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuChromaQpAdjSize() && pcCU->getSlice()->getUseChromaQpAdj())
  {
    setCodeChromaQpAdjFlag(true);
  }

  if (pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    Int bits_prev_state5 = m_pcEntropyCoder->getNumberOfWrittenBits();
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );
    
      bitsAccumulatorCUTransquantBypassFlag += m_pcEntropyCoder -> getNumberOfWrittenBits() -  bits_prev_state5;
      
  }

  if( !pcCU->getSlice()->isIntra() )
  {
    Int bits_prev_state5 = m_pcEntropyCoder->getNumberOfWrittenBits();
    m_pcEntropyCoder->encodeSkipFlag( pcCU, uiAbsPartIdx );
      
    bitsAccumulatorSkipFlag += m_pcEntropyCoder -> getNumberOfWrittenBits() -  bits_prev_state5;
      
  }

  if( pcCU->isSkipped( uiAbsPartIdx ) )
  {
    Int bits_prev_state5 = m_pcEntropyCoder->getNumberOfWrittenBits();
    m_pcEntropyCoder->encodeMergeIndex( pcCU, uiAbsPartIdx );
    finishCU(pcCU,uiAbsPartIdx,uiDepth);
      
    bitsAccumulatorSkipFlag += m_pcEntropyCoder -> getNumberOfWrittenBits() -  bits_prev_state5;
    return;
  }
    
  Int bits_prev_state = m_pcEntropyCoder->getNumberOfWrittenBits();
  m_pcEntropyCoder->encodePredMode( pcCU, uiAbsPartIdx );
  bitsAccumulatorPredMode += m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state;
    
  Int bits_prev_state2 = m_pcEntropyCoder->getNumberOfWrittenBits();
  m_pcEntropyCoder->encodePartSize( pcCU, uiAbsPartIdx, uiDepth );
  bitsAccumulatorPartSize += m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state2;
  
    
   
    
  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )
  {
      
    Int bits_prev_state3 = m_pcEntropyCoder->getNumberOfWrittenBits();
    m_pcEntropyCoder->encodeIPCMInfo( pcCU, uiAbsPartIdx );
    bitsAccumulatorIPCMFlag += m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state3;

    if(pcCU->getIPCMFlag(uiAbsPartIdx))
    {
      // Encode slice finish
      finishCU(pcCU,uiAbsPartIdx,uiDepth);
      return;
    }
  }

  // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
  Int bits_prev_state4 = m_pcEntropyCoder->getNumberOfWrittenBits();
  m_pcEntropyCoder->encodePredInfo( pcCU, uiAbsPartIdx );
  bitsAccumulatorPredInfo += m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state4;

    // **
//  cout << "Prediction bits: " << (m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state) << endl;
    
  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  Bool codeChromaQpAdj = getCodeChromaQpAdjFlag();
  Int bits_prev_state5 = m_pcEntropyCoder->getNumberOfWrittenBits();
  m_pcEntropyCoder->encodeCoeff( pcCU, uiAbsPartIdx, uiDepth, bCodeDQP, codeChromaQpAdj );
  bitsAccumulatorCoeff += m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state5;

    // **
//  cout << "Residual bits: " << (m_pcEntropyCoder->getNumberOfWrittenBits() - bits_prev_state5) << endl;
    
  setCodeChromaQpAdjFlag( codeChromaQpAdj );
  setdQPFlag( bCodeDQP );


    // **
//  cout << "BITS BITS: " << m_pcEntropyCoder -> getNumberOfWrittenBits() << ", bitsSnapEntropyBeforePerCu " << bitsSnapEntropyBeforePerCu << ", bitsSnapEntropyAfterPerCu: " << bitsSnapEntropyAfterPerCu << endl;
   bitsSnapEntropyAfterPerCu = m_pcEntropyCoder -> getNumberOfWrittenBits() - bitsSnapEntropyBeforePerCu;
   bitsSnapEntropyBeforePerCu = m_pcEntropyCoder -> getNumberOfWrittenBits();

    
//    if( uiDepth == (Int) pcCU->getDepth(uiAbsPartIdx))
//    {
//        cout << "***** " << endl;
//        cout << "CU uiLPelX, uiTPelY: " << uiLPelX << ", " << uiTPelY << ", uiRPelX, uiBPelY: " << uiRPelX << ", " << uiBPelY << endl;
//        cout << "-----Bits per CU: " << bitsSnapEntropyAfterPerCu << endl;
//        cout << "***** " << endl;
//    }
    
    
  // --- write terminating bit ---
  finishCU(pcCU,uiAbsPartIdx,uiDepth);
    
}

Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg)
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] ;
    diff[k+1] = piOrg[1] ;
    diff[k+2] = piOrg[2] ;
    diff[k+3] = piOrg[3] ;
    diff[k+4] = piOrg[4] ;
    diff[k+5] = piOrg[5] ;
    diff[k+6] = piOrg[6] ;
    diff[k+7] = piOrg[7] ;

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad =(iSumHad+2)>>2;
  return(iSumHad);
}

Int  TEncCu::updateLCUDataISlice(TComDataCU* pcCU, Int LCUIdx, Int width, Int height)
{
  Int  xBl, yBl;
  const Int iBlkSize = 8;

  Pel* pOrgInit   = pcCU->getPic()->getPicYuvOrg()->getAddr(COMPONENT_Y, pcCU->getAddr(), 0);
  Int  iStrideOrig = pcCU->getPic()->getPicYuvOrg()->getStride(COMPONENT_Y);
  Pel  *pOrg;

  Int iSumHad = 0;
  for ( yBl=0; (yBl+iBlkSize)<=height; yBl+= iBlkSize)
  {
    for ( xBl=0; (xBl+iBlkSize)<=width; xBl+= iBlkSize)
    {
      pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2NNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU DEBUG_STRING_FN_DECLARE(sDebug), Bool *earlyDetectionSkipMode )
{
    
    
    assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );
    TComMvField  cMvFieldNeighbours[2 * MRG_MAX_NUM_CANDS]; // double length for mv of both lists
    UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
    Int numValidMergeCand = 0;
    const Bool bTransquantBypassFlag = rpcTempCU->getCUTransquantBypass(0);
    
    for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
    {
        uhInterDirNeighbours[ui] = 0;
    }
    UChar uhDepth = rpcTempCU->getDepth( 0 );
    rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
    rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );
    
    Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
    for( UInt ui = 0; ui < numValidMergeCand; ++ui )
    {
        mergeCandBuffer[ui] = 0;
    }
    
    Bool bestIsSkip = false;
    
    UInt iteration;
    if ( rpcTempCU->isLosslessCoded(0))
    {
        iteration = 1;
    }
    else
    {
        iteration = 2;
    }
    DEBUG_STRING_NEW(bestStr)
    
    for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )
    {
        for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
        {
            if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))
            {
                if( !(bestIsSkip && uiNoResidual == 0) )
                {
                    DEBUG_STRING_NEW(tmpStr)
                    // set MC parameters
                    rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
                    rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag, 0, uhDepth );
                    rpcTempCU->setChromaQpAdjSubParts( bTransquantBypassFlag ? 0 : m_ChromaQpAdjIdc, 0, uhDepth );
                    rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
                    rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
                    rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
                    rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
                    rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
                    rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
                    
                    // do MC
                    m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );
                    
                    // Hossam: Cost is not calculated here
                    //            cout << "Cost here maybe: " << rpcTempCU->getTotalCost() << endl;
                    //             cout << "Cost here maybe: " << rpcTempCU->getTotalCost() << endl;
                    // estimate residual and encode everything
                    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU,
                                                                 m_ppcOrigYuv    [uhDepth],
                                                                 m_ppcPredYuvTemp[uhDepth],
                                                                 m_ppcResiYuvTemp[uhDepth],
                                                                 m_ppcResiYuvBest[uhDepth],
                                                                 m_ppcRecoYuvTemp[uhDepth],
                                                                 (uiNoResidual != 0),
                                                                 scTempResidual[uhDepth]
                                                                 DEBUG_STRING_PASS_INTO(tmpStr) );
                    
//                    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU,
//                                                              m_ppcOrigYuv    [uhDepth],
//                                                              m_ppcPredYuvTemp[uhDepth],
//                                                              m_ppcResiYuvTemp[uhDepth],
//                                                              m_ppcResiYuvBest[uhDepth],
//                                                              m_ppcRecoYuvTemp[uhDepth],
//                                                              (uiNoResidual != 0) DEBUG_STRING_PASS_INTO(tmpStr) );

                    
//                    if (uhDepth==SC_MAX_DEPTH) {
//                        cout << "Insert the residual CU Merge" << endl;
//                        
//                        // Hossam: Scene change
//                        saveResidualSignal(rpcBestCU, uhDepth);

//                         m_scResidualFrame.pushBack(m_ppcResiYuvBest[uhDepth]);
//                        m_ppcResiYuvBest[uhDepth]->copyToPicYuv(m_scResidualFrame, rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU());
                        
//                        Pel* x = m_scResidualFrame->getBuf(COMPONENT_Y);
//                        
//                        cout << "YUV Y: 0 " << x[0] << endl;
                        
//                        m_ppcResiYuvBest[uhDepth]->copyToPicYuv(m_scResidualFrame, rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU());

//                    }

                    
#ifdef DEBUG_STRING
                    DebugInterPredResiReco(tmpStr, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif
                    
                    if ((uiNoResidual == 0) && (rpcTempCU->getQtRootCbf(0) == 0))
                    {
                        // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
                        mergeCandBuffer[uiMergeCand] = 1;
                    }
                    
                    rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
                    Int orgQP = rpcTempCU->getQP( 0 );
                    xCheckDQP( rpcTempCU );
                    xCheckBestModeNew(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(bestStr) DEBUG_STRING_PASS_INTO(tmpStr));
                    
                    rpcTempCU->initEstData( uhDepth, orgQP, bTransquantBypassFlag );
                    
                    if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
                    {
                        bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;
                    }
                }
            }
        }
        
        if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
        {
            if(rpcBestCU->getQtRootCbf( 0 ) == 0)
            {
                if( rpcBestCU->getMergeFlag( 0 ))
                {
                    *earlyDetectionSkipMode = true;
                }
                else if(m_pcEncCfg->getFastSearch() != SELECTIVE)
                {
                    Int absoulte_MV=0;
                    for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
                    {
                        if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
                        {
                            TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
                            Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
                            Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
                            absoulte_MV+=iHor+iVer;
                        }
                    }
                    
                    if(absoulte_MV == 0)
                    {
                        *earlyDetectionSkipMode = true;
                    }
                }
            }
        }
    }
    DEBUG_STRING_APPEND(sDebug, bestStr)

}


/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU DEBUG_STRING_FN_DECLARE(sDebug), Bool *earlyDetectionSkipMode )
{
  assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );
  TComMvField  cMvFieldNeighbours[2 * MRG_MAX_NUM_CANDS]; // double length for mv of both lists
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;
  const Bool bTransquantBypassFlag = rpcTempCU->getCUTransquantBypass(0);

  for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
  {
    uhInterDirNeighbours[ui] = 0;
  }
  UChar uhDepth = rpcTempCU->getDepth( 0 );
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
  rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );

  Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
  for( UInt ui = 0; ui < numValidMergeCand; ++ui )
  {
    mergeCandBuffer[ui] = 0;
  }

  Bool bestIsSkip = false;

  UInt iteration;
  if ( rpcTempCU->isLosslessCoded(0))
  {
    iteration = 1;
  }
  else
  {
    iteration = 2;
  }
  DEBUG_STRING_NEW(bestStr)

  for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )
  {
    for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )
    {
      if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))
      {
        if( !(bestIsSkip && uiNoResidual == 0) )
        {
          DEBUG_STRING_NEW(tmpStr)
          // set MC parameters
          rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag, 0, uhDepth );
          rpcTempCU->setChromaQpAdjSubParts( bTransquantBypassFlag ? 0 : m_ChromaQpAdjIdc, 0, uhDepth );
          rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to LCU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

          // do MC
          m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );
          // estimate residual and encode everything
          m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
                                                     m_ppcOrigYuv    [uhDepth],
                                                     m_ppcPredYuvTemp[uhDepth],
                                                     m_ppcResiYuvTemp[uhDepth],
                                                     m_ppcResiYuvBest[uhDepth],
                                                     m_ppcRecoYuvTemp[uhDepth],
                                                     (uiNoResidual != 0) DEBUG_STRING_PASS_INTO(tmpStr) );
            

        
#ifdef DEBUG_STRING
          DebugInterPredResiReco(tmpStr, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif

          if ((uiNoResidual == 0) && (rpcTempCU->getQtRootCbf(0) == 0))
          {
            // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
            mergeCandBuffer[uiMergeCand] = 1;
          }

          rpcTempCU->setSkipFlagSubParts( rpcTempCU->getQtRootCbf(0) == 0, 0, uhDepth );
          Int orgQP = rpcTempCU->getQP( 0 );
          xCheckDQP( rpcTempCU );
          xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(bestStr) DEBUG_STRING_PASS_INTO(tmpStr));

          rpcTempCU->initEstData( uhDepth, orgQP, bTransquantBypassFlag );

          if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
          {
            bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;
          }
        }
      }
    }

    if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      if(rpcBestCU->getQtRootCbf( 0 ) == 0)
      {
        if( rpcBestCU->getMergeFlag( 0 ))
        {
          *earlyDetectionSkipMode = true;
        }
        else if(m_pcEncCfg->getFastSearch() != SELECTIVE)
        {
          Int absoulte_MV=0;
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
              Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
              Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
              absoulte_MV+=iHor+iVer;
            }
          }

          if(absoulte_MV == 0)
          {
            *earlyDetectionSkipMode = true;
          }
        }
      }
    }
  }
  DEBUG_STRING_APPEND(sDebug, bestStr)
}


static Int y_arrow = 0;
static Int u_arrow = 0;
static Int v_arrow = 0;
// Hossam: Scene Change
// XXXX You dont' call the residaul in the checkmerge case take care!
Void TEncCu:: saveResidualSignal(TComDataCU* rpcBestCU, Char uhDepth)
{

    // Hossam: Taking the best Residual
//    Pel* Y = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Y);
//    Pel* U = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cb);
//    Pel* V = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cr);
//    
//    
//    Int width_y = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Y);
//    Int height_y = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Y);
//    
//    Int width_u = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Cb);
//    Int height_u = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Cb);
//    
//    Int width_v = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Cr);
//    Int height_v = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Cr);
//    
//    
//    Int n_y = width_y*height_y;
//    Int n_u = width_u*height_u;
//    Int n_v = width_v*height_v;
    
//    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_y << ", " << height_y << endl;
//    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_v << ", " << height_v << endl;

//    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_y<< ", " << height_y << endl;
//    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_v << ", " << height_v << endl;

    Pel* Y = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Y);
    Pel* U = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cb);
    Pel* V = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cr);
    
    
    Int width_y = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Y);
    Int height_y = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Y);
    
    Int width_u = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Cb);
    Int height_u = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Cb);
    
    Int width_v = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Cr);
    Int height_v = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Cr);
    
    
    Int n_y = width_y*height_y;
    Int n_u = width_u*height_u;
    Int n_v = width_v*height_v;

    
    
    // Save the Y residuals
//    for (int i = y_arrow; i<n_y; i++) {
//        allData_Y[i] = Y[i];
//    }
//    y_arrow = y_arrow + n_y;
//    
//    
//    //    y_arrow
//    // Save the U residuals
//    for (int i = u_arrow; i<n_u; i++) {
//        allData_U[i] = U[i];
//    }
//    u_arrow = u_arrow + n_u;
//    
//    // Save the V residuals
//    for (int i = v_arrow; i<n_v; i++) {
//        allData_V[i] = V[i];
//    }
//    v_arrow = v_arrow + n_v;
}

//// XXX it is semi working
//Void TEncCu:: saveResidualSignal(TComDataCU* rpcBestCU, Char uhDepth)
//{
//    
//
//    // Hossam: Taking the best Residual
//    Pel* Y = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Y);
//    Pel* U = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cb);
//    Pel* V = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cr);
//    
//    
//    Int width_y = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Y);
//    Int height_y = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Y);
//    
//    Int width_u = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Cb);
//    Int height_u = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Cb);
//    
//    Int width_v = m_ppcResiYuvBest[uhDepth]->getWidth(COMPONENT_Cr);
//    Int height_v = m_ppcResiYuvBest[uhDepth]->getHeight(COMPONENT_Cr);
//    
//   
//    Int n_y = width_y*height_y;
//    Int n_u = width_u*height_u;
//    Int n_v = width_v*height_v;
//    
//        cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_y << ", " << height_y << endl;
//        cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_v << ", " << height_v << endl;
//    
//    // Save the Y residuals
//    for (int i = y_arrow; i<n_y; i++) {
//        allData_Y[i] = Y[i];
//    }
//    y_arrow = y_arrow + n_y;
//    
//    
//    //    y_arrow
//    // Save the U residuals
//    for (int i = u_arrow; i<n_u; i++) {
//        allData_U[i] = U[i];
//    }
//    u_arrow = u_arrow + n_u;
//    
//    // Save the V residuals
//    for (int i = v_arrow; i<n_v; i++) {
//        allData_V[i] = V[i];
//    }
//    v_arrow = v_arrow + n_v;
//}

//Void TEncCu:: saveResidualSignal(TComDataCU* rpcBestCU, Char uhDepth)
//{
//    
//    // Hossam: Taking the best Residual
//    Pel* Y = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Y);
//    Pel* U = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cb);
//    Pel* V = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cr);
//    
//    
//    Int width_y = rpcBestCU->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Y);
//    Int height_y = rpcBestCU->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Y);
//    
//    Int width_u = rpcBestCU->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Cb);
//    Int height_u = rpcBestCU->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Cb);
//    
//    Int width_v = rpcBestCU->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Cr);
//    Int height_v = rpcBestCU->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Cr);
//    
//    Int n_y = width_y*height_y;
//    Int n_u = width_u*height_u;
//    Int n_v = width_v*height_v;
//
//    
////    cout << "VALLLLL of YYYYYYY: Sha3rawy " << start_Y[6] << endl;
////    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_y << ", " << height_y << endl;
////    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_u << ", " << height_u << endl;
////    cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_v << ", " << height_v << endl;
//    
//    // Save the Y residuals
//    for (int i = 0; i<n_y; i++) {
//        allData_Y[i] = Y[i];
//    }
//    
////    y_arrow
//    // Save the U residuals
//    for (int i = 0; i<n_u; i++) {
//        allData_U[i] = U[i];
//    }
//    
//    // Save the V residuals
//    for (int i = 0; i<n_v; i++) {
//        allData_V[i] = V[i];
//    }
//
////    for (int i = 0; i<n_y; i++) {
////        allData_Y[i] = Y[i];
////    }
////
////    // Save the U residuals
////    for (int i = 0; i<n_u; i++) {
////        allData_U[i] = U[i];
////    }
////
////    // Save the V residuals
////    for (int i = 0; i<n_v; i++) {
////        allData_V[i] = V[i];
////    }
//    
//}

// Hossam: Scene change ORG
#if AMP_MRG
Void TEncCu::xCheckRDCostInterAtDepthOrg( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG, Int which_reference)
#else
Void TEncCu::xCheckRDCostInterAtDepthOrg( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
    
    //    cout << " xCheckRDCostInterAtDepth " << endl;
    DEBUG_STRING_NEW(sTest)
    
    UChar uhDepth = rpcTempCU->getDepth( 0 );
    
    rpcTempCU->setDepthSubParts( uhDepth, 0 );
    
    rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );
    
    rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
    rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
    rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uhDepth );
    
// Hossam: First case is the one which is enabled
    // Hossam:
    // m_ppcOrg: pcYUVOrg
    // m_ppcPredYuvTemp: pcYuvPred (******IMP******)
    // m_ppcResiYuvTemp: rpcYuvResi
    // m_ppcResiYuvBest: rpcYuvResiBest
    // m_ppcRecoYuvTemp: rpcYuvRec
    // scTempResidual: scTempResidual
//    m_ppcPredYuvBest; ///< Best Prediction Yuv for each depth
//    m_ppcResiYuvBest; ///< Best Residual Yuv for each depth
//    m_ppcRecoYuvBest; ///< Best Reconstruction Yuv for each depth
//    m_ppcPredYuvTemp; ///< Temporary Prediction Yuv for each depth (******IMP******)
//    m_ppcResiYuvTemp; ///< Temporary Residual Yuv for each depth
//    m_ppcRecoYuvTemp; ///< Temporary Reconstruction Yuv for each depth
//    m_ppcOrigYuv;     ///< Original Yuv for each depth

    // Use this when you want to get the sigma squared based on 4 references
    if (which_reference < 0)
    {
//         cout << "(2) Use this when you want to get the sigma squared based on a specific reference: " << which_reference << endl;
        #if AMP_MRG
            rpcTempCU->setMergeAMP (true);
            m_pcPredSearch->predInterSearchOrg ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );
        #else
            m_pcPredSearch->predInterSearchOrg ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
        #endif
    }
    else
    {
        //// Use this when you want to get the sigma squared based on a specific reference
//        cout << "(1) Use this when you want to get the sigma squared based on a specific reference: " << which_reference << endl;
        #if AMP_MRG
            rpcTempCU->setMergeAMP (true);
        
            if(rpcTempCU->getPic()->getPOC() == 1)
                m_pcPredSearch->predInterSearchOrgSpecificRef ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], 0 DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );
            else
                m_pcPredSearch->predInterSearchOrgSpecificRef ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], which_reference DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );
        
        #else
        
            if(rpcTempCU->getPic()->getPOC() == 1)
                m_pcPredSearch->predInterSearchOrgSpecificRef ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], 0 );
            else
                m_pcPredSearch->predInterSearchOrgSpecificRef ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth], which_reference );
        #endif
        // Specific reference end
        
    }


    
#if AMP_MRG
    if ( !rpcTempCU->getMergeAMP() )
    {
        return;
    }
#endif
    
    //    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false DEBUG_STRING_PASS_INTO(sTest) );
    
    //    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false,
    //                                                 scTempResidual[uhDepth] DEBUG_STRING_PASS_INTO(sTest) );
    
    
    // Hossam:
    // m_ppcOrg: pcYUVOrg
    // m_ppcPredYuvTemp: pcYuvPred
    // m_ppcResiYuvTemp: rpcYuvResi
    // m_ppcResiYuvBest: rpcYuvResiBest
    // m_ppcRecoYuvTemp: rpcYuvRec
    // scTempResidual: scTempResidual
    // Hossam: Scene change: Subtracts the m_ppcOrig - m_ppcPred and place them in the m_ppcResiYUV (*** Does Subtraction only)
    m_pcPredSearch->encodeResAndCalcRdInterCUAtDepth( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false,
                                                     scTempResidual[uhDepth] DEBUG_STRING_PASS_INTO(sTest) );
    
    
    // Update the best buffer instead
    xUpdateBestResiduals(rpcBestCU, rpcTempCU, uhDepth);
    
    
#ifdef DEBUG_STRING
    DebugInterPredResiReco(sTest, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif
    
    //    xCheckDQP( rpcTempCU );
    //    xCheckBestModeNew(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
//    xCopyTempToBestSC(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
    
    
}




#if AMP_MRG
Void TEncCu::xCheckRDCostInterAtDepth( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInterAtDepth( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
    
    //    cout << " xCheckRDCostInterAtDepth " << endl;
    DEBUG_STRING_NEW(sTest)
    
    UChar uhDepth = rpcTempCU->getDepth( 0 );
    
    rpcTempCU->setDepthSubParts( uhDepth, 0 );
    
    rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );
    
    rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
    rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
    rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uhDepth );
    
#if AMP_MRG
    rpcTempCU->setMergeAMP (true);
    m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );
#else
    m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif
    
#if AMP_MRG
    if ( !rpcTempCU->getMergeAMP() )
    {
        return;
    }
#endif
    
    //    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false DEBUG_STRING_PASS_INTO(sTest) );
    
    //    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false,
    //                                                 scTempResidual[uhDepth] DEBUG_STRING_PASS_INTO(sTest) );
    
    
    m_pcPredSearch->encodeResAndCalcRdInterCUAtDepth( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false,
                                                     scTempResidual[uhDepth] DEBUG_STRING_PASS_INTO(sTest) );
    
    
    // Update the best buffer instead
    xUpdateBestResiduals(rpcBestCU, rpcTempCU, uhDepth);
    
    
#ifdef DEBUG_STRING
    DebugInterPredResiReco(sTest, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif
    
    //    xCheckDQP( rpcTempCU );
    //    xCheckBestModeNew(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
    
    
    
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInterNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInterNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
    
    //    cout << " xCheckRDCostINter " << endl;
    DEBUG_STRING_NEW(sTest)
    
    UChar uhDepth = rpcTempCU->getDepth( 0 );
    
    rpcTempCU->setDepthSubParts( uhDepth, 0 );
    
    rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );
    
    rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
    rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
    rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uhDepth );
    
#if AMP_MRG
    rpcTempCU->setMergeAMP (true);
    m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );
#else
    m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif
    
#if AMP_MRG
    if ( !rpcTempCU->getMergeAMP() )
    {
        return;
    }
#endif
    
//    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false DEBUG_STRING_PASS_INTO(sTest) );
    
    m_pcPredSearch->encodeResAndCalcRdInterCUNew( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false,
                                                 scTempResidual[uhDepth] DEBUG_STRING_PASS_INTO(sTest) );

    rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
    
//    if (uhDepth==SC_MAX_DEPTH) {
//        cout << "Insert the residual CU " << endl;
    
//        // Hossam: Scene change
//        saveResidualSignal(rpcBestCU, uhDepth);
        
        // Hossam: Taking the best XXXX
//        Pel* start_Y = m_ppcResiYuvTemp[uhDepth]->getAddr(COMPONENT_Y);
//        Pel* start_U = m_ppcResiYuvTemp[uhDepth]->getAddr(COMPONENT_Y);
//        Pel* start_V = m_ppcResiYuvTemp[uhDepth]->getAddr(COMPONENT_Y);
        
//        Pel* start_Y = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Y);
//        Pel* start_U = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cb);
//        Pel* start_V = m_ppcResiYuvBest[uhDepth]->getAddr(COMPONENT_Cr);
//     
//        
//        Int width_y = rpcBestCU->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Y);
//        Int height_y = rpcBestCU->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Y);
//        
//        Int width_u = rpcBestCU->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Cb);
//        Int height_u = rpcBestCU->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Cb);
//        
//        Int width_v = rpcBestCU->getPic()->getPicYuvOrg()->getWidth(COMPONENT_Cr);
//        Int height_v = rpcBestCU->getPic()->getPicYuvOrg()->getHeight(COMPONENT_Cr);
//        
//        Int n_y = width_y*height_y;
//        Int n_u = width_u*height_u;
//        Int n_v = width_v*height_v;
//        
//        cout << "VALLLLL of YYYYYYY: Sha3rawy " << start_Y[6] << endl;
//        cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_y << ", " << height_y << endl;
//        cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_u << ", " << height_u << endl;
//        cout << "VALLLLL of YYYYYYY: Sha3rawy " << width_v << ", " << height_v << endl;
        //                         m_scResidualFrame.pushBack(m_ppcResiYuvBest[uhDepth]);
//        m_ppcResiYuvBest[uhDepth]->copyToPicYuv(m_scResidualFrame, rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU());
        
//        m_ppcResiYuvBest[uhDepth]->copyToPicYuv(m_scResidualFrame, rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU());
        
//        Pel* x = m_scResidualFrame->getBuf(COMPONENT_Y);
        
        
//        cout << "YUV Y: 0 " << x[0] << endl;
        
//        TComPicYuv* tmp;
        
        // Hossam: XXXX SEG FAULT Possibility
//        if(tmp == NULL)
//        {
//         tmp = new TComPicYuv;
//        tmp->create(rpcBestCU->getPic()->get, <#const Int iPicHeight#>, <#const ChromaFormat chromaFormatIDC#>, <#const UInt uiMaxCUWidth#>, <#const UInt uiMaxCUHeight#>, <#const UInt uiMaxCUDepth#>)
//
//        }
        
            
        
//        cout << "SEG FAULT: " << endl;

//        m_ppcResiYuvBest[uhDepth]->copyToPicYuv(tmp, rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU());
       
        // Temp and not best
//      m_ppcResiYuvTemp[uhDepth]->copyToPicYuv(tmp, rpcBestCU->getAddr(), rpcBestCU->getZorderIdxInCU());

        
//        Pel*  tst = tmp->getBuf(COMPONENT_Y);
        
//        cout << "Test val 12 OUT: " << tst[12];
        
//        tmp->getBuf(COMPONENT_U)
//        tmp->getBuf(COMPONENT_V)


        
//    }// end if
    
#ifdef DEBUG_STRING
    DebugInterPredResiReco(sTest, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif
    
    xCheckDQP( rpcTempCU );
    xCheckBestModeNew(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG)
#else
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
    
//    cout << " xCheckRDCostINter " << endl;
  DEBUG_STRING_NEW(sTest)

  UChar uhDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setDepthSubParts( uhDepth, 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uhDepth );

  rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );
  rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );
  rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uhDepth );

#if AMP_MRG
  rpcTempCU->setMergeAMP (true);
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );
#else
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif

#if AMP_MRG
  if ( !rpcTempCU->getMergeAMP() )
  {
    return;
  }
#endif

  m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false DEBUG_STRING_PASS_INTO(sTest) );
  rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

#ifdef DEBUG_STRING
  DebugInterPredResiReco(sTest, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif

  xCheckDQP( rpcTempCU );
  xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
}


Void TEncCu::xCheckRDCostIntraNew( TComDataCU *&rpcBestCU,
                               TComDataCU *&rpcTempCU,
                               Double      &cost,
                               PartSize     eSize
                               DEBUG_STRING_FN_DECLARE(sDebug) )
{
    DEBUG_STRING_NEW(sTest)
    
    
    cout << "xCheckRDCostIntraNew says Willkommen! " << endl;
    
    UInt uiDepth = rpcTempCU->getDepth( 0 );
    
    rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );
    
    rpcTempCU->setPartSizeSubParts( eSize, 0, uiDepth );
    rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
    rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uiDepth );
    
    Bool bSeparateLumaChroma = true; // choose estimation mode
    
    Distortion uiPreCalcDistC = 0;
    if (rpcBestCU->getPic()->getChromaFormat()==CHROMA_400)
    {
        bSeparateLumaChroma=true;
    }
    
    Pel resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];
    
    if( !bSeparateLumaChroma )
    {
        // after this function, the direction will be PLANAR, DC, HOR or VER
        // however, if Luma ends up being one of those, the chroma dir must be later changed to DM_CHROMA.
        m_pcPredSearch->preestChromaPredMode( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth] );
    }
    m_pcPredSearch->estIntraPredQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], resiLuma, uiPreCalcDistC, bSeparateLumaChroma DEBUG_STRING_PASS_INTO(sTest) );
    
    m_ppcRecoYuvTemp[uiDepth]->copyToPicComponent(COMPONENT_Y, rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );
    
    if (rpcBestCU->getPic()->getChromaFormat()!=CHROMA_400)
    {
        m_pcPredSearch->estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], resiLuma, uiPreCalcDistC DEBUG_STRING_PASS_INTO(sTest) );
    }
    
    m_pcEntropyCoder->resetBits();
    
    if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
    {
        m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
    }
    
    m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
    m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,          true );
    m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth, true );
    m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0 );
    m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true );
    
    // Encode Coefficients to count the number of bits
    Bool bCodeDQP = getdQPFlag();
    Bool codeChromaQpAdjFlag = getCodeChromaQpAdjFlag();
    m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, bCodeDQP, codeChromaQpAdjFlag );
    setCodeChromaQpAdjFlag( codeChromaQpAdjFlag );
    setdQPFlag( bCodeDQP );
    
    m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);
    
    rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
    rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
    rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );
    
    xCheckDQP( rpcTempCU );
    
    cost = rpcTempCU->getTotalCost();
    
    xCheckBestModeNew(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
}

Void TEncCu::xCheckRDCostIntra( TComDataCU *&rpcBestCU,
                                TComDataCU *&rpcTempCU,
                                Double      &cost,
                                PartSize     eSize
                                DEBUG_STRING_FN_DECLARE(sDebug) )
{
  DEBUG_STRING_NEW(sTest)

    
  UInt uiDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );

  rpcTempCU->setPartSizeSubParts( eSize, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uiDepth );

  Bool bSeparateLumaChroma = true; // choose estimation mode

  Distortion uiPreCalcDistC = 0;
  if (rpcBestCU->getPic()->getChromaFormat()==CHROMA_400)
  {
    bSeparateLumaChroma=true;
  }

  Pel resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];

  if( !bSeparateLumaChroma )
  {
    // after this function, the direction will be PLANAR, DC, HOR or VER
    // however, if Luma ends up being one of those, the chroma dir must be later changed to DM_CHROMA.
    m_pcPredSearch->preestChromaPredMode( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth] );
  }
  m_pcPredSearch->estIntraPredQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], resiLuma, uiPreCalcDistC, bSeparateLumaChroma DEBUG_STRING_PASS_INTO(sTest) );

  m_ppcRecoYuvTemp[uiDepth]->copyToPicComponent(COMPONENT_Y, rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getAddr(), rpcTempCU->getZorderIdxInCU() );

  if (rpcBestCU->getPic()->getChromaFormat()!=CHROMA_400)
  {
    m_pcPredSearch->estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], resiLuma, uiPreCalcDistC DEBUG_STRING_PASS_INTO(sTest) );
  }

  m_pcEntropyCoder->resetBits();

  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }

  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0 );
  m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true );

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  Bool codeChromaQpAdjFlag = getCodeChromaQpAdjFlag();
  m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, bCodeDQP, codeChromaQpAdjFlag );
  setCodeChromaQpAdjFlag( codeChromaQpAdjFlag );
  setdQPFlag( bCodeDQP );

  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );

  cost = rpcTempCU->getTotalCost();

  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));
}


/** Check R-D costs for a CU with PCM mode.
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 *
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU )
{
  UInt uiDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );

  rpcTempCU->setIPCMFlag(0, true);
  rpcTempCU->setIPCMFlagSubParts (true, 0, rpcTempCU->getDepth(0));
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );
  rpcTempCU->setTrIdxSubParts ( 0, 0, uiDepth );
  rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_ChromaQpAdjIdc, 0, uiDepth );

  m_pcPredSearch->IPCMSearch( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth]);

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  m_pcEntropyCoder->resetBits();

  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );
  }

  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize ( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodeIPCMInfo ( rpcTempCU, 0, true );

  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );

  xCheckDQP( rpcTempCU );
  DEBUG_STRING_NEW(a)
  DEBUG_STRING_NEW(b)
  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(a) DEBUG_STRING_PASS_INTO(b));
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */
Void TEncCu::xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo) )
{
  if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
  {
    TComYuv* pcYuv;
    // Change Information data
    TComDataCU* pcCU = rpcBestCU;
    rpcBestCU = rpcTempCU;
    rpcTempCU = pcCU;

    // Change Prediction data
    pcYuv = m_ppcPredYuvBest[uiDepth];
    m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
    m_ppcPredYuvTemp[uiDepth] = pcYuv;

    // Change Reconstruction data
    pcYuv = m_ppcRecoYuvBest[uiDepth];
    m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
    m_ppcRecoYuvTemp[uiDepth] = pcYuv;

    pcYuv = NULL;
    pcCU  = NULL;

    // store temp best CI for next CU coding
    m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);

      
#ifdef DEBUG_STRING
    DEBUG_STRING_SWAP(sParent, sTest)
    const PredMode predMode=rpcBestCU->getPredictionMode(0);
    if ((DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode)) && bAddSizeInfo)
    {
      std::stringstream ss(stringstream::out);
      ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":"Inter   ") << partSizeToString[rpcBestCU->getPartitionSize(0)] << " CU at " << rpcBestCU->getCUPelX() << ", " << rpcBestCU->getCUPelY() << " width=" << UInt(rpcBestCU->getWidth(0)) << std::endl;
      sParent+=ss.str();
    }
#endif
  }
    
//    cout << "\nxCheckBestMode: TEMP " << rpcTempCU->getTotalBits() << ", " << rpcTempCU->m_uiTotalBitsRefI[0] << endl;
//    cout << "xCheckBestMode: Best " << rpcBestCU->getTotalBits() << ", " << rpcBestCU->m_uiTotalBitsRefI[0] << endl;
}

// Hossam: Scene change
Void TEncCu::setCompressCUFirstTime(Bool flag)
{
    isCompressCUFirstTime = flag;
}

// Hossam: Scene change
//Void TEncCu::xUpdateResiduals(Bool isUpdate)
//{
//    
//    if (isCompressCUFirstTime || isUpdate)
//    {
//        // update the flag
//        setCompressCUFirstTime(false);
//        
//        // Allocate the best depending on the temp
//        // If it's first time, make a copy
//        m_pcPredSearch->xUpdateResiduals();
//        
//    }
//    
//    // Either ways we need to destory the temp variable
//    m_pcPredSearch->xDestroyTempResiduals();
//    
//    return;
//}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \returns Void
 */


// Hossam: Scene change
Void TEncCu::xCheckBestModeLastCall( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo) )
{
    
    // Hossam DEBUG
    //    if(rpcBestCU ->getSlice()->getPOC() >= 1)
    //    {
    //            pFile = fopen ("out2.txt", "at");
    //
    //
    //       if(rpcBestCU->getTotalCost() <= MAX_INT)
    //           fprintf(pFile, "%f\t %f\t\n",
    //                (rpcTempCU->getTotalCost()) ,
    //                rpcBestCU->getTotalCost());
    //
    //            fclose(pFile);
    //    }
    
#if SC_ENABLE_PRINT
    cout << "\n//////////xCheckBestMode LAST CALLLLLL\\\\\\\\" << endl;
    cout << "uiDepth: " << uiDepth << ", "<<  "Cost of the temp CU: " << (rpcTempCU->getTotalCost()) << ", Cost of the best CU: " << rpcBestCU->getTotalCost() << endl;
    
    // Hossam: Call the update the residuals function
    //    xUpdateResiduals(rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost());
    
    cout << "Temp <Partition, Depth>: " << "<" << Int(rpcTempCU->getPartIdx())<< ", " << uiDepth << ">";
    cout << ", Best <Partition, Depth>: " << "<" << Int(rpcBestCU->getPartIdx())<< ", " << uiDepth << ">" << endl;
    
    cout << "Depth of Temp: " << Int(rpcTempCU->getDepth(0)) << ", Depth of Best: " << Int(rpcBestCU->getDepth(0)) << endl;
    // Hossam: It does not matter because we want to have the partitioning and it's square NxN
    
    cout << "Temp Absolute Index in the LCU: " << rpcTempCU->getZorderIdxInCU() << ", Best: " << rpcBestCU->getZorderIdxInCU() << endl;
    cout << "Part SCU Addr of temp: " << Int(rpcTempCU->getSCUAddr()) << ", Part SCU Addr of best: " << Int(rpcBestCU->getSCUAddr()) << endl;
    cout << "Part ID of temp: " << Int(rpcTempCU->getPartIdx()) << ", Part ID  of best: " << Int(rpcBestCU->getPartIdx()) << endl;
    cout << "Partition type of temp: " << Int(rpcTempCU->getPartitionSize(0)) << ", Partition type of best: " << Int(rpcBestCU->getPartitionSize(0)) << endl;
    cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
    cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
    cout << "Partition height of temp: " << Int(rpcTempCU->getHeight(0)) << ", Partition height of best: " << Int(rpcBestCU->getHeight(0)) << endl;
    //    cout << "Number of partitions for temp: " << Int(rpcTempCU->getNumPartitions()) << ", Number of partitions for best: " << Int(rpcBestCU->getNumPartitions()) << endl;
#endif
    
    if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
    {
        TComYuv* pcYuv;
        // Change Information data
        TComDataCU* pcCU = rpcBestCU;
        rpcBestCU = rpcTempCU;
        rpcTempCU = pcCU;
        
        // Change Prediction data
        pcYuv = m_ppcPredYuvBest[uiDepth];
        m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
        m_ppcPredYuvTemp[uiDepth] = pcYuv;
        
        // Change Reconstruction data
        pcYuv = m_ppcRecoYuvBest[uiDepth];
        m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
        m_ppcRecoYuvTemp[uiDepth] = pcYuv;
        
        pcYuv = NULL;
        pcCU  = NULL;
        
        // store temp best CI for next CU coding
        // Hossam: remove the m_pppRdeocter -- Never! :D
        m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);

#if SC_ENABLE_PRINT
        cout << "\t\t\tUPDATE BEST: ----- " << endl;
        cout << "Depth of Temp: " << Int(rpcTempCU->getDepth(0)) << ", Depth of Best: " << Int(rpcBestCU->getDepth(0)) << endl;
        
        // Hossam: It does not matter because we want to have the partitioning and it's square NxN
        
        cout << "Part SCU Addr of temp: " << Int(rpcTempCU->getSCUAddr()) << ", Part SCU Addr of best: " << Int(rpcBestCU->getSCUAddr()) << endl;
        
        cout << "Part ID of temp: " << Int(rpcTempCU->getPartIdx()) << ", Part ID  of best: " << Int(rpcBestCU->getPartIdx()) << endl;
        
        cout << "Partition type of temp: " << Int(rpcTempCU->getPartitionSize(0)) << ", Partition type of best: " << Int(rpcBestCU->getPartitionSize(0)) << endl;
        cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
        cout << "Partition height of temp: " << Int(rpcTempCU->getHeight(0)) << ", Partition height of best: " << Int(rpcBestCU->getHeight(0)) << endl;
        //        cout << "Number of partitions for temp: " << Int(rpcTempCU->getNumPartitions()) << ", Number of partitions for best: " << Int(rpcBestCU->getNumPartitions()) << endl;
#endif
        
        // Hossam: Scene change
//        xUpdateBestResiduals(rpcBestCU, rpcTempCU, uiDepth);
        xPropogateBestResiduals(rpcBestCU, rpcTempCU, uiDepth);
        
#ifdef DEBUG_STRING
        DEBUG_STRING_SWAP(sParent, sTest)
        const PredMode predMode=rpcBestCU->getPredictionMode(0);
        if ((DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode)) && bAddSizeInfo)
        {
            std::stringstream ss(stringstream::out);
            ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":"Inter   ") << partSizeToString[rpcBestCU->getPartitionSize(0)] << " CU at " << rpcBestCU->getCUPelX() << ", " << rpcBestCU->getCUPelY() << " width=" << UInt(rpcBestCU->getWidth(0)) << std::endl;
            sParent+=ss.str();
        }
#endif
    }
}



//static FILE* pFile = NULL;
Void TEncCu::xCheckBestModeNew( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo) )
{
    
    // Hossam DEBUG
//    if(rpcBestCU ->getSlice()->getPOC() >= 1)
//    {
//            pFile = fopen ("out2.txt", "at");
//        
//        
//       if(rpcBestCU->getTotalCost() <= MAX_INT)
//           fprintf(pFile, "%f\t %f\t\n",
//                (rpcTempCU->getTotalCost()) ,
//                rpcBestCU->getTotalCost());
//
//            fclose(pFile);
//    }

#if SC_ENABLE_PRINT
    cout << "\n//////////xCheckBestMode\\\\\\\\" << endl;
    cout << "uiDepth: " << uiDepth << ", "<<  "Cost of the temp CU: " << (rpcTempCU->getTotalCost()) << ", Cost of the best CU: " << rpcBestCU->getTotalCost() << endl;

    // Hossam: Call the update the residuals function
//    xUpdateResiduals(rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost());
    
    cout << "Temp <Partition, Depth>: " << "<" << Int(rpcTempCU->getPartIdx())<< ", " << uiDepth << ">";
    cout << ", Best <Partition, Depth>: " << "<" << Int(rpcBestCU->getPartIdx())<< ", " << uiDepth << ">" << endl;
    
    cout << "Depth of Temp: " << Int(rpcTempCU->getDepth(0)) << ", Depth of Best: " << Int(rpcBestCU->getDepth(0)) << endl;
    // Hossam: It does not matter because we want to have the partitioning and it's square NxN
    
    cout << "Temp Absolute Index in the LCU: " << rpcTempCU->getZorderIdxInCU() << ", Best: " << rpcBestCU->getZorderIdxInCU() << endl;
    cout << "Part SCU Addr of temp: " << Int(rpcTempCU->getSCUAddr()) << ", Part SCU Addr of best: " << Int(rpcBestCU->getSCUAddr()) << endl;
    cout << "Part ID of temp: " << Int(rpcTempCU->getPartIdx()) << ", Part ID  of best: " << Int(rpcBestCU->getPartIdx()) << endl;
    cout << "Partition type of temp: " << Int(rpcTempCU->getPartitionSize(0)) << ", Partition type of best: " << Int(rpcBestCU->getPartitionSize(0)) << endl;
    cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
    cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
    cout << "Partition height of temp: " << Int(rpcTempCU->getHeight(0)) << ", Partition height of best: " << Int(rpcBestCU->getHeight(0)) << endl;
//    cout << "Number of partitions for temp: " << Int(rpcTempCU->getNumPartitions()) << ", Number of partitions for best: " << Int(rpcBestCU->getNumPartitions()) << endl;
#endif
    
    if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
    {
        TComYuv* pcYuv;
        // Change Information data
        TComDataCU* pcCU = rpcBestCU;
        rpcBestCU = rpcTempCU;
        rpcTempCU = pcCU;
        
        // Change Prediction data
        pcYuv = m_ppcPredYuvBest[uiDepth];
        m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
        m_ppcPredYuvTemp[uiDepth] = pcYuv;
        
        // Change Reconstruction data
        pcYuv = m_ppcRecoYuvBest[uiDepth];
        m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
        m_ppcRecoYuvTemp[uiDepth] = pcYuv;
        
        pcYuv = NULL;
        pcCU  = NULL;
        
        // store temp best CI for next CU coding
        // Hossam: remove the m_pppRdeocter -- Never! :D
        m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);

#if SC_ENABLE_PRINT
        cout << "\t\t\tUPDATE BEST: ----- " << endl;
        cout << "Depth of Temp: " << Int(rpcTempCU->getDepth(0)) << ", Depth of Best: " << Int(rpcBestCU->getDepth(0)) << endl;
        
        // Hossam: It does not matter because we want to have the partitioning and it's square NxN
        
        cout << "Part SCU Addr of temp: " << Int(rpcTempCU->getSCUAddr()) << ", Part SCU Addr of best: " << Int(rpcBestCU->getSCUAddr()) << endl;

        cout << "Part ID of temp: " << Int(rpcTempCU->getPartIdx()) << ", Part ID  of best: " << Int(rpcBestCU->getPartIdx()) << endl;

        cout << "Partition type of temp: " << Int(rpcTempCU->getPartitionSize(0)) << ", Partition type of best: " << Int(rpcBestCU->getPartitionSize(0)) << endl;
        cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
        cout << "Partition height of temp: " << Int(rpcTempCU->getHeight(0)) << ", Partition height of best: " << Int(rpcBestCU->getHeight(0)) << endl;
//        cout << "Number of partitions for temp: " << Int(rpcTempCU->getNumPartitions()) << ", Number of partitions for best: " << Int(rpcBestCU->getNumPartitions()) << endl;

#endif
        
//            // Hossam: Scene change
            xUpdateBestResiduals(rpcBestCU, rpcTempCU, uiDepth);
        
#ifdef DEBUG_STRING
        DEBUG_STRING_SWAP(sParent, sTest)
        const PredMode predMode=rpcBestCU->getPredictionMode(0);
        if ((DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode)) && bAddSizeInfo)
        {
            std::stringstream ss(stringstream::out);
            ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":"Inter   ") << partSizeToString[rpcBestCU->getPartitionSize(0)] << " CU at " << rpcBestCU->getCUPelX() << ", " << rpcBestCU->getCUPelY() << " width=" << UInt(rpcBestCU->getWidth(0)) << std::endl;
            sParent+=ss.str();
        }
#endif
    }
}

Void TEncCu::xCopyTempToBestSC( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo) )
{
    
    
#if SC_ENABLE_PRINT
    cout << "\n//////////xCheckBestMode\\\\\\\\" << endl;
    cout << "uiDepth: " << uiDepth << ", "<<  "Cost of the temp CU: " << (rpcTempCU->getTotalCost()) << ", Cost of the best CU: " << rpcBestCU->getTotalCost() << endl;
    
    // Hossam: Call the update the residuals function
    //    xUpdateResiduals(rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost());
    
    cout << "Temp <Partition, Depth>: " << "<" << Int(rpcTempCU->getPartIdx())<< ", " << uiDepth << ">";
    cout << ", Best <Partition, Depth>: " << "<" << Int(rpcBestCU->getPartIdx())<< ", " << uiDepth << ">" << endl;
    
    cout << "Depth of Temp: " << Int(rpcTempCU->getDepth(0)) << ", Depth of Best: " << Int(rpcBestCU->getDepth(0)) << endl;
    // Hossam: It does not matter because we want to have the partitioning and it's square NxN
    
    cout << "Temp Absolute Index in the LCU: " << rpcTempCU->getZorderIdxInCU() << ", Best: " << rpcBestCU->getZorderIdxInCU() << endl;
    cout << "Part SCU Addr of temp: " << Int(rpcTempCU->getSCUAddr()) << ", Part SCU Addr of best: " << Int(rpcBestCU->getSCUAddr()) << endl;
    cout << "Part ID of temp: " << Int(rpcTempCU->getPartIdx()) << ", Part ID  of best: " << Int(rpcBestCU->getPartIdx()) << endl;
    cout << "Partition type of temp: " << Int(rpcTempCU->getPartitionSize(0)) << ", Partition type of best: " << Int(rpcBestCU->getPartitionSize(0)) << endl;
    cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
    cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
    cout << "Partition height of temp: " << Int(rpcTempCU->getHeight(0)) << ", Partition height of best: " << Int(rpcBestCU->getHeight(0)) << endl;
    //    cout << "Number of partitions for temp: " << Int(rpcTempCU->getNumPartitions()) << ", Number of partitions for best: " << Int(rpcBestCU->getNumPartitions()) << endl;
#endif
    
//    if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )
//    {
        TComYuv* pcYuv;
        // Change Information data
        TComDataCU* pcCU = rpcBestCU;
        rpcBestCU = rpcTempCU;
        rpcTempCU = pcCU;
        
        // Change Prediction data
        pcYuv = m_ppcPredYuvBest[uiDepth];
        m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
        m_ppcPredYuvTemp[uiDepth] = pcYuv;
        
        // Change Reconstruction data
//        pcYuv = m_ppcRecoYuvBest[uiDepth];
//        m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
//        m_ppcRecoYuvTemp[uiDepth] = pcYuv;
    
        pcYuv = NULL;
        pcCU  = NULL;
        
        // store temp best CI for next CU coding
        // Hossam: remove the m_pppRdeocter -- Never! :D
        m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
        
#if SC_ENABLE_PRINT
        cout << "\t\t\tUPDATE BEST: ----- " << endl;
        cout << "Depth of Temp: " << Int(rpcTempCU->getDepth(0)) << ", Depth of Best: " << Int(rpcBestCU->getDepth(0)) << endl;
        
        // Hossam: It does not matter because we want to have the partitioning and it's square NxN
        
        cout << "Part SCU Addr of temp: " << Int(rpcTempCU->getSCUAddr()) << ", Part SCU Addr of best: " << Int(rpcBestCU->getSCUAddr()) << endl;
        
        cout << "Part ID of temp: " << Int(rpcTempCU->getPartIdx()) << ", Part ID  of best: " << Int(rpcBestCU->getPartIdx()) << endl;
        
        cout << "Partition type of temp: " << Int(rpcTempCU->getPartitionSize(0)) << ", Partition type of best: " << Int(rpcBestCU->getPartitionSize(0)) << endl;
        cout << "Partition width of temp: " << Int(rpcTempCU->getWidth(0)) << ", Partition width of best: " << Int(rpcBestCU->getWidth(0)) << endl;
        cout << "Partition height of temp: " << Int(rpcTempCU->getHeight(0)) << ", Partition height of best: " << Int(rpcBestCU->getHeight(0)) << endl;
        //        cout << "Number of partitions for temp: " << Int(rpcTempCU->getNumPartitions()) << ", Number of partitions for best: " << Int(rpcBestCU->getNumPartitions()) << endl;
        
#endif
        
        //            // Hossam: Scene change
    // Hossam: Not needed since it's done either ways after the encodeCalcInterDepth here in this class - Done anyways
//        xUpdateBestResiduals(rpcBestCU, rpcTempCU, uiDepth);
    
#ifdef DEBUG_STRING
        DEBUG_STRING_SWAP(sParent, sTest)
        const PredMode predMode=rpcBestCU->getPredictionMode(0);
        if ((DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode)) && bAddSizeInfo)
        {
            std::stringstream ss(stringstream::out);
            ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":"Inter   ") << partSizeToString[rpcBestCU->getPartitionSize(0)] << " CU at " << rpcBestCU->getCUPelX() << ", " << rpcBestCU->getCUPelY() << " width=" << UInt(rpcBestCU->getWidth(0)) << std::endl;
            sParent+=ss.str();
        }
#endif
//    }

}// end xCopyBestToTmp



static Int up = 0;
Void TEncCu::xUpdateBestResiduals(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth)
{
 #if SC_ENABLE_PRINT
    cout << "\n\nUPDATE BEST residuals with id: "  << (up++) << ", dep: "<< uiDepth << endl;
#endif
    
    // Probably, You will need to update the depth of Best with what inside the depth in Temp
    // upadate ui Depth
    
//    scTempResidual[uiDepth]->printComponent(ComponentID(0), 0);
//    scBestResidual[uiDepth]->printComponent(ComponentID(0), 0);
    
//    cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, scBestResidual[uiDepth]) << endl;
    
//    TComYuv* pcYuv;

    // Change Residual data
//    pcYuv = scBestResidual[uiDepth];
//    scBestResidual[uiDepth] = scTempResidual[uiDepth];
//    scTempResidual[uiDepth] = pcYuv;
//    
//    
//    pcYuv = NULL;
//
    

 // This is when id is 2094... Block #3 in Block #2 16x16 in Block#1 32x32


#if SC_DEBUG_COPY_SPECIAL
    if(up == 25)
    {
        cout << "TEMP: " << endl;
        //            scTempResidual[uiDepth] ->print(rpcTempCU->getZorderIdxInCU());
        //            cout << "BEST Before: " << endl;
        //            scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scTempResidual[uiDepth]->mockPrintComponent(COMPONENT_Y, rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        
    }
    
    if(up == 69)
    {
        cout << "TEMP: " << endl;
        //                scTempResidual[uiDepth] ->print(rpcTempCU->getZorderIdxInCU());
        //                scTempResidual[uiDepth] -> mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
        
        scTempResidual[uiDepth] -> mockPrintComponent(COMPONENT_Y, rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
        
        cout << "BEST Before: " << endl;
        //                scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        //                scBestResidual[uiDepth]->mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
        scBestResidual[uiDepth]->mockPrintComponent(COMPONENT_Y,rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
    }
#endif
    
    // Last thing
    //    scTempResidual[uiDepth]->copyToPartYuv(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU());
    
    // Hossam: XXXXX I'm afraid that I need to save and send the previous width and height from the last trial
    
    
    scTempResidual[uiDepth]->copyToPartYuvSpecial(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
    
    
    
    
#if SC_DEBUG_COPY_SPECIAL
    
    if(up == 25)
    {
        cout << "BEST After: " << endl;
        //            scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth]->mockPrintComponent(COMPONENT_Y, rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        exit (EXIT_FAILURE);
        
    }
    if(up == 69)
    {
        cout << "BEST After: " << endl;
        //             scBestResidual[uiDepth]-> mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
        //            scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth]->mockPrintComponent(COMPONENT_Y, rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0), true);
        
        exit (EXIT_FAILURE);
        
    }
#endif
    // Print the best residuals
//    scBestResidual[uiDepth]->print(0);
//    scBestResidual[uiDepth]->printComponent(ComponentID(0), 0);
    
//  cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, scBestResidual[uiDepth]) << endl;
//      cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, pcYuv) << endl;
    
//    if(uiDepth == 1)
//    {
//        cout << "BEST After: " << endl;
//        scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
//        exit (EXIT_FAILURE);
//    }
    
}


static int prop = 0;
Void TEncCu::xPropogateBestResiduals(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth)
{
#if SC_ENABLE_PRINT
    cout << "\nPROPOGATE BACKWARDS BEST residuals id: " << prop++ << ", dep: " << uiDepth << endl;
#endif
//    cout << "Same betweeen the temp and the best in the same level: " << scTempResidual[uiDepth]->isSame(0, scBestResidual[uiDepth]) << endl;
    
    
//    scTempResidual[uiDepth]->copyToPartYuv(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU());
//
    
    // Last one
//    scBestResidual[uiDepth+1]->copyToPartYuv(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU());

#if SC_DEBUG_COPY_SPECIAL
    if(prop == 69)
    {
        cout << "Best of Dep+1: " << endl;
        //                scTempResidual[uiDepth] ->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth+1] -> mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        cout << "BEST of Dep before: " << endl;
        //                scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth]->mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        
    }
#endif
    
    
    scBestResidual[uiDepth+1]->copyToPartYuvSpecial(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
    
    
#if SC_DEBUG_COPY_SPECIAL
    if(prop == 69)
    {
        cout << "Best of Dep+1: " << endl;
        //                scTempResidual[uiDepth] ->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth+1] -> mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        cout << "BEST of Dep after: " << endl;
        //                scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth]->mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        exit (EXIT_FAILURE);
        
    }
#endif
    
//    cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, scBestResidual[uiDepth]) << endl;
    //      cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, pcYuv) << endl;
}


Void TEncCu::xPropogateBestResidualsAtDepth(TComDataCU *&rpcBestCU, TComDataCU *&rpcTempCU, UInt uiDepth)
{
#if SC_ENABLE_PRINT
    cout << "\nPROPOGATE BACKWARDS BEST AT DEPTH residuals id: " << prop++ << ", dep: " << uiDepth << endl;
#endif
    //    cout << "Same betweeen the temp and the best in the same level: " << scTempResidual[uiDepth]->isSame(0, scBestResidual[uiDepth]) << endl;
    
    
    //    scTempResidual[uiDepth]->copyToPartYuv(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU());
    //
    
    // Last one
    //    scBestResidual[uiDepth+1]->copyToPartYuv(scBestResidual[uiDepth], rpcTempCU->getZorderIdxInCU());
    
#if SC_DEBUG_COPY_SPECIAL
    if(prop == 69)
    {
        cout << "Best of Dep+1: " << endl;
        //                scTempResidual[uiDepth] ->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth+1] -> mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        cout << "BEST of Dep before: " << endl;
        //                scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth]->mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        
    }
#endif
    
    
    UInt toDepth = SC_BEST_BUFFER_IND;
    scBestResidual[SC_AT_DEPTH]->copyToPartYuvSpecial(scBestResidual[toDepth], rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
    
    
#if SC_DEBUG_COPY_SPECIAL
    if(prop == 69)
    {
        cout << "Best of Dep+1: " << endl;
        //                scTempResidual[uiDepth] ->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth+1] -> mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        cout << "BEST of Dep after: " << endl;
        //                scBestResidual[uiDepth]->print(rpcTempCU->getZorderIdxInCU());
        scBestResidual[uiDepth]->mockPrint(rpcTempCU->getZorderIdxInCU(), rpcTempCU->getWidth(0), rpcTempCU->getHeight(0));
        exit (EXIT_FAILURE);
        
    }
#endif
    
    //    cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, scBestResidual[uiDepth]) << endl;
    //      cout << "Same Flag: " << scTempResidual[uiDepth]->isSame(0, pcYuv) << endl;
}



Void TEncCu::xCheckDQP( TComDataCU* pcCU )
{
  UInt uiDepth = pcCU->getDepth( 0 );

  if( pcCU->getSlice()->getPPS()->getUseDQP() && (g_uiMaxCUWidth>>uiDepth) >= pcCU->getSlice()->getPPS()->getMinCuDQPSize() )
  {
    if ( pcCU->getQtRootCbf( 0) )
    {
#if !RDO_WITHOUT_DQP_BITS
        cout << "xCheckDPQ: Yes I'm failing here" << endl;
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQP( pcCU, 0, false );
      pcCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits
      pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      pcCU->getTotalCost() = m_pcRdCost->calcRdCost( pcCU->getTotalBits(), pcCU->getTotalDistortion() );
                cout << "xCheckDPQ: Now I'm NOT failing here" << endl;
#endif
    }
    else
    {
      pcCU->setQPSubParts( pcCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
    }
  }
}

Void TEncCu::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];
  }
}
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth, TComDataCU* pcCU, UInt uiLPelX, UInt uiTPelY )
{
  UInt uiRPelX   = uiLPelX + (g_uiMaxCUWidth>>uiDepth)  - 1;
  UInt uiBPelY   = uiTPelY + (g_uiMaxCUHeight>>uiDepth) - 1;
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());
  Bool bSliceStart = pcSlice->getSliceSegmentCurStartCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx &&
    pcSlice->getSliceSegmentCurStartCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  Bool bSliceEnd   = pcSlice->getSliceSegmentCurEndCUAddr() > rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx &&
    pcSlice->getSliceSegmentCurEndCUAddr() < rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) );
  if(!bSliceEnd && !bSliceStart && ( uiRPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiBPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
  {
    UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
    UInt uiSrcBlkWidth = rpcPic->getNumPartInWidth() >> (uiSrcDepth);
    UInt uiBlkWidth    = rpcPic->getNumPartInWidth() >> (uiDepth);
    UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
    UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;
    m_ppcRecoYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);

    m_ppcPredYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvPred (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);
  }
  else
  {
    UInt uiQNumParts = ( pcCU->getPic()->getNumPartInCU() >> (uiDepth<<1) )>>2;

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )
    {
      UInt uiSubCULPelX   = uiLPelX + ( g_uiMaxCUWidth >>(uiDepth+1) )*( uiPartUnitIdx &  1 );
      UInt uiSubCUTPelY   = uiTPelY + ( g_uiMaxCUHeight>>(uiDepth+1) )*( uiPartUnitIdx >> 1 );

      Bool bInSlice = rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx+uiQNumParts > pcSlice->getSliceSegmentCurStartCUAddr() &&
        rpcPic->getPicSym()->getInverseCUOrderMap(pcCU->getAddr())*pcCU->getPic()->getNumPartInCU()+uiAbsPartIdx < pcSlice->getSliceSegmentCurEndCUAddr();
      if(bInSlice&&( uiSubCULPelX < pcSlice->getSPS()->getPicWidthInLumaSamples() ) && ( uiSubCUTPelY < pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xCopyYuv2Pic( rpcPic, uiCUAddr, uiAbsPartIdx, uiDepth+1, uiSrcDepth, pcCU, uiSubCULPelX, uiSubCUTPelY );   // Copy Yuv data to picture Yuv
      }
    }
  }
}

Void TEncCu::xCopyYuv2Tmp( UInt uiPartUnitIdx, UInt uiNextDepth )
{
  UInt uiCurrDepth = uiNextDepth - 1;
  m_ppcRecoYuvBest[uiNextDepth]->copyToPartYuv( m_ppcRecoYuvTemp[uiCurrDepth], uiPartUnitIdx );
  m_ppcPredYuvBest[uiNextDepth]->copyToPartYuv( m_ppcPredYuvBest[uiCurrDepth], uiPartUnitIdx);
}

/** Function for filling the PCM buffer of a CU using its original sample array
 * \param pcCU pointer to current CU
 * \param pcOrgYuv pointer to original sample array
 * \returns Void
 */
Void TEncCu::xFillPCMBuffer     ( TComDataCU*& pCU, TComYuv* pOrgYuv )
{
  const ChromaFormat format = pCU->getPic()->getChromaFormat();
  const UInt numberValidComponents = getNumberValidComponents(format);
  for (UInt componentIndex = 0; componentIndex < numberValidComponents; componentIndex++)
  {
    const ComponentID component = ComponentID(componentIndex);

    const UInt width  = pCU->getWidth(0)  >> getComponentScaleX(component, format);
    const UInt height = pCU->getHeight(0) >> getComponentScaleY(component, format);

    Pel *source      = pOrgYuv->getAddr(component, 0, width);
    Pel *destination = pCU->getPCMSample(component);

    const UInt sourceStride = pOrgYuv->getStride(component);

    for (Int line = 0; line < height; line++)
    {
      for (Int column = 0; column < width; column++)
      {
        destination[column] = source[column];
      }

      source      += sourceStride;
      destination += width;
    }
  }
}

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff* rpcCoeff, TCoeff* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples )
{
  for( Int n = 0; n < NumCoeffInCU; n++ )
  {
    TCoeff u = abs( rpcCoeff[ n ] );
    TCoeff absc = rpcArlCoeff[ n ];

    if( u != 0 )
    {
      if( u < LEVEL_RANGE )
      {
        cSum[ u ] += ( Double )absc;
        numSamples[ u ]++;
      }
      else
      {
        cSum[ LEVEL_RANGE ] += ( Double )absc - ( Double )( u << ARL_C_PRECISION );
        numSamples[ LEVEL_RANGE ]++;
      }
    }
  }

  return 0;
}

/** Collect ARL statistics from one LCU
 * \param pcCU
 */
Void TEncCu::xLcuCollectARLStats(TComDataCU* rpcCU )
{
  Double cSum[ LEVEL_RANGE + 1 ];     //: the sum of DCT coefficients corresponding to datatype and quantization output
  UInt numSamples[ LEVEL_RANGE + 1 ]; //: the number of coefficients corresponding to datatype and quantization output

  TCoeff* pCoeffY = rpcCU->getCoeff(COMPONENT_Y);
  TCoeff* pArlCoeffY = rpcCU->getArlCoeff(COMPONENT_Y);

  UInt uiMinCUWidth = g_uiMaxCUWidth >> g_uiMaxCUDepth;
  UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;

  memset( cSum, 0, sizeof( Double )*(LEVEL_RANGE+1) );
  memset( numSamples, 0, sizeof( UInt )*(LEVEL_RANGE+1) );

  // Collect stats to cSum[][] and numSamples[][]
  for(Int i = 0; i < rpcCU->getTotalNumPart(); i ++ )
  {
    UInt uiTrIdx = rpcCU->getTransformIdx(i);

    if(rpcCU->isInter(i) && rpcCU->getCbf( i, COMPONENT_Y, uiTrIdx ) )
    {
      xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
    }//Note that only InterY is processed. QP rounding is based on InterY data only.

    pCoeffY  += uiMinNumCoeffInCU;
    pArlCoeffY  += uiMinNumCoeffInCU;
  }

  for(Int u=1; u<LEVEL_RANGE;u++)
  {
    m_pcTrQuant->getSliceSumC()[u] += cSum[ u ] ;
    m_pcTrQuant->getSliceNSamples()[u] += numSamples[ u ] ;
  }
  m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[ LEVEL_RANGE ] ;
  m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[ LEVEL_RANGE ] ;
}
#endif
//! \}
