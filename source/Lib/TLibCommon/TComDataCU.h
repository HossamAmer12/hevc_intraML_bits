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

/** \file     TComDataCU.h
    \brief    CU data structure (header)
    \todo     not all entities are documented
*/

#ifndef __TCOMDATACU__
#define __TCOMDATACU__

#include <assert.h>

// Include files
#include "CommonDef.h"
#include "TComMotionInfo.h"
#include "TComSlice.h"
#include "TComRdCost.h"
#include "TComPattern.h"

#include <algorithm>
#include <vector>

//! \ingroup TLibCommon
//! \{

class TComTU; // forward declaration

static const UInt NUM_MOST_PROBABLE_MODES=3; // NOTE: RExt - new definition

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU data structure class
class TComDataCU
{
private:

    // Hossam: Save the partIdx
    UInt sc_partIdx;
    
  // -------------------------------------------------------------------------------------------------------------------
  // class pointers
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*      m_pcPic;              ///< picture class pointer
  TComSlice*    m_pcSlice;            ///< slice header pointer

  // -------------------------------------------------------------------------------------------------------------------
  // CU description
  // -------------------------------------------------------------------------------------------------------------------

  UInt          m_uiCUAddr;           ///< CU address in a slice
  UInt          m_uiAbsIdxInLCU;      ///< absolute address in a CU. It's Z scan order
  UInt          m_uiCUPelX;           ///< CU position in a pixel (X)
  UInt          m_uiCUPelY;           ///< CU position in a pixel (Y)
  UInt          m_uiNumPartition;     ///< total number of minimum partitions in a CU
  UChar*        m_puhWidth;           ///< array of widths
  UChar*        m_puhHeight;          ///< array of heights
  UChar*        m_puhDepth;           ///< array of depths
  Int           m_unitSize;           ///< size of a "minimum partition"

  // -------------------------------------------------------------------------------------------------------------------
  // CU data
  // -------------------------------------------------------------------------------------------------------------------

  Bool*          m_skipFlag;           ///< array of skip flags
  Char*          m_pePartSize;         ///< array of partition sizes
  Char*          m_pePredMode;         ///< array of prediction modes
  Char*          m_crossComponentPredictionAlpha[MAX_NUM_COMPONENT]; ///< array of cross-component prediction alpha values
  Bool*          m_CUTransquantBypass;   ///< array of cu_transquant_bypass flags
//  Char*          m_phQP;               ///< array of QP values
  UChar*         m_ChromaQpAdj;        ///< array of chroma QP adjustments (indexed)
  UInt           m_codedChromaQpAdj;
  UChar*         m_puhTrIdx;           ///< array of transform indices
  UChar*         m_puhTransformSkip[MAX_NUM_COMPONENT];///< array of transform skipping flags
  UChar*         m_puhCbf[MAX_NUM_COMPONENT];          ///< array of coded block flags (CBF)
  TComCUMvField  m_acCUMvField[NUM_REF_PIC_LIST_01];    ///< array of motion vectors.
  TCoeff*        m_pcTrCoeff[MAX_NUM_COMPONENT];       ///< array of transform coefficient buffers (0->Y, 1->Cb, 2->Cr)
    
    // Hossam: Scene change SC
    TComCUMvField  m_acCUMvFieldSC[NUM_REF_PIC_LIST_01];    ///< array of SC motion vectors.

    
#if ADAPTIVE_QP_SELECTION
  TCoeff*        m_pcArlCoeff[MAX_NUM_COMPONENT];  // ARL coefficient buffer (0->Y, 1->Cb, 2->Cr)
  static TCoeff* m_pcGlbArlCoeff[MAX_NUM_COMPONENT]; // global ARL buffer
  Bool           m_ArlCoeffIsAliasedAllocation;  ///< ARL coefficient buffer is an alias of the global buffer and must not be free()'d
#endif

  Pel*           m_pcIPCMSample[MAX_NUM_COMPONENT];    ///< PCM sample buffer (0->Y, 1->Cb, 2->Cr)

  // -------------------------------------------------------------------------------------------------------------------
  // neighbour access variables
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   m_pcCUAboveLeft;      ///< pointer of above-left CU
  TComDataCU*   m_pcCUAboveRight;     ///< pointer of above-right CU
  TComDataCU*   m_pcCUAbove;          ///< pointer of above CU
  TComDataCU*   m_pcCULeft;           ///< pointer of left CU
  TComDataCU*   m_apcCUColocated[NUM_REF_PIC_LIST_01];  ///< pointer of temporally colocated CU's for both directions
  TComMvField   m_cMvFieldA;          ///< motion vector of position A
  TComMvField   m_cMvFieldB;          ///< motion vector of position B
  TComMvField   m_cMvFieldC;          ///< motion vector of position C
  TComMv        m_cMvPred;            ///< motion vector predictor

  // -------------------------------------------------------------------------------------------------------------------
  // coding tool information
  // -------------------------------------------------------------------------------------------------------------------

  Bool*         m_pbMergeFlag;        ///< array of merge flags
  UChar*        m_puhMergeIndex;      ///< array of merge candidate indices
#if AMP_MRG
  Bool          m_bIsMergeAMP;
#endif
  UChar*        m_puhIntraDir[MAX_NUM_CHANNEL_TYPE]; // 0-> Luma, 1-> Chroma
  UChar*        m_puhInterDir;        ///< array of inter directions
  Char*         m_apiMVPIdx[NUM_REF_PIC_LIST_01];       ///< array of motion vector predictor candidates
  Char*         m_apiMVPNum[NUM_REF_PIC_LIST_01];       ///< array of number of possible motion vectors predictors
  Bool*         m_pbIPCMFlag;         ///< array of intra_pcm flags
    
    // Hossam: Scene change
    // Arrays for the SCD only
    UChar*        m_puhInterDirSC;        ///< array of inter directions
    Char*         m_apiMVPIdxSC[NUM_REF_PIC_LIST_01];       ///< array of motion vector predictor candidates
    Char*         m_apiMVPNumSC[NUM_REF_PIC_LIST_01];       ///< array of number of possible motion vectors predictors

  // -------------------------------------------------------------------------------------------------------------------
  // misc. variables
  // -------------------------------------------------------------------------------------------------------------------

  Bool          m_bDecSubCu;          ///< indicates decoder-mode
  Double        m_dTotalCost;         ///< sum of partition RD costs
  Distortion    m_uiTotalDistortion;  ///< sum of partition distortion
  UInt          m_uiTotalBits;        ///< sum of partition bits
  UInt          m_uiTotalBins;        ///< sum of partition bins
  UInt*         m_sliceStartCU;       ///< Start CU address of current slice
  UInt*         m_sliceSegmentStartCU;///< Start CU address of current slice
  Char          m_codedQP;
  UChar*        m_explicitRdpcmMode[MAX_NUM_COMPONENT]; ///< Stores the explicit RDPCM mode for all TUs belonging to this CU

public:
    Char*          m_phQP;               ///< array of QP values
    
    // Hossam: Scene change
    UInt intra_modes=0;
    UInt intra_modes_small;
    UInt cu_total_parts = 0;
    
    
    // four bit counts for each CU
    UInt          m_uiTotalBitsRefI [4];        ///< sum of partition bits for each RefI
    
    // four references utilization
    Double reference_utitilization_counts[4];
    
    // Reset the reference Utilization every CTU
    Void resetReferenceUtitlization()
    {
        for(Int i = 0; i < 4; ++i)
        {
            reference_utitilization_counts[i] = 0;
        }
    }
    
    // Reset the reference Utilization every CTU
    Void resetBitRefICounts()
    {
//        cout << "XXXX Reset the BitRefI Counts -- I need to be removed XXX " << endl;
        for(Int i = 0; i < 4; ++i)
        {
            m_uiTotalBitsRefI[i] = 0;
        }
    }
    
protected:

  /// add possible motion vector predictor candidates
  Bool          xAddMVPCand           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );
  Bool          xAddMVPCandOrder      ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );

 // Hossam: Scene change
  Bool          xAddMVPCandOrg           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );
  Bool          xAddMVPCandOrderOrg      ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );

// Hossam: Scene change
    Bool          xAddMVPCandSC           ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );
    Bool          xAddMVPCandOrderSC      ( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir );
    
    
  Void          deriveRightBottomIdx        ( UInt uiPartIdx, UInt& ruiPartIdxRB );
  Bool          xGetColMVP( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx );
    
// Hossam: Scene change
Bool          xGetColMVPOrg( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx );
    
// Hossam: Scene change
Bool          xGetColMVPSC( RefPicList eRefPicList, Int uiCUAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx );

  /// compute required bits to encode MVD (used in AMVP)
  UInt          xGetMvdBits           ( TComMv cMvd );
  UInt          xGetComponentBits     ( Int iVal );

  /// compute scaling factor from POC difference
  Int           xGetDistScaleFactor   ( Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC );

  Void xDeriveCenterIdx( UInt uiPartIdx, UInt& ruiPartIdxCenter );

public:
  TComDataCU();
  virtual ~TComDataCU();

  
    // Hossam: Scene change
    UInt getPartIdx()
    {
        return sc_partIdx;
    }
    
  // -------------------------------------------------------------------------------------------------------------------
  // create / destroy / initialize / copy
  // -------------------------------------------------------------------------------------------------------------------

  Void          create                ( ChromaFormat chromaFormatIDC, UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize
#if ADAPTIVE_QP_SELECTION
    , Bool bGlobalRMARLBuffer = false
#endif
    );
  Void          destroy               ();

    //Hossam: SC
    Void          initCUNew                ( TComPic* pcPic, UInt uiCUAddr );
    
   //Hossam: SC
   Void          initCUNewOrg                ( TComPic* pcPic, UInt uiCUAddr );
    
    
  Void          initCU                ( TComPic* pcPic, UInt uiCUAddr );
  Void          initEstData           ( const UInt uiDepth, const Int qp, const Bool bTransquantBypass );
  Void          initSubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp );
  Void          setOutsideCUPart      ( UInt uiAbsPartIdx, UInt uiDepth );

// Hossam: Scene change
    Void          initEstDataSC           ( const UInt uiDepth, const Int qp, const Bool bTransquantBypass );
    Void          initSubCUSC             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp );

    
  Void          copySubCU             ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );
  Void          copyInterPredInfoFrom ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList );
  Void          copyPartFrom          ( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth );

  Void          copyToPic             ( UChar uiDepth );
  Void          copyToPic             ( UChar uiDepth, UInt uiPartIdx, UInt uiPartDepth );

// Hossam: Scene change
 Void          copyToPicSC             ( UChar uiDepth );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU description
  // -------------------------------------------------------------------------------------------------------------------

  TComPic*        getPic              ()                        { return m_pcPic;           }
  const TComPic*  getPic              ()   const                { return m_pcPic;           }
  TComSlice*       getSlice           ()                        { return m_pcSlice;         }
  const TComSlice* getSlice           ()   const                { return m_pcSlice;         }
  UInt&         getAddr               ()                        { return m_uiCUAddr;        }
  UInt&         getZorderIdxInCU      ()                        { return m_uiAbsIdxInLCU; }
  UInt          getSCUAddr            ();
  UInt          getCUPelX             ()                        { return m_uiCUPelX;        }
  UInt          getCUPelY             ()                        { return m_uiCUPelY;        }

  UChar*        getDepth              ()                        { return m_puhDepth;        }
  UChar         getDepth              ( UInt uiIdx ) const      { return m_puhDepth[uiIdx]; }
  Void          setDepth              ( UInt uiIdx, UChar  uh ) { m_puhDepth[uiIdx] = uh;   }

  Void          setDepthSubParts      ( UInt uiDepth, UInt uiAbsPartIdx );

    
    
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for CU data
  // -------------------------------------------------------------------------------------------------------------------

  Char*         getPartitionSize      ()                        { return m_pePartSize;        }
  PartSize      getPartitionSize      ( UInt uiIdx )            { return static_cast<PartSize>( m_pePartSize[uiIdx] ); }
  Void          setPartitionSize      ( UInt uiIdx, PartSize uh){ m_pePartSize[uiIdx] = uh;   }
  Void          setPartSizeSubParts   ( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth );
  Void          setCUTransquantBypassSubParts( Bool flag, UInt uiAbsPartIdx, UInt uiDepth );

  Bool*         getSkipFlag            ()                        { return m_skipFlag;          }
  Bool          getSkipFlag            (UInt idx)                { return m_skipFlag[idx];     }
  Void          setSkipFlag           ( UInt idx, Bool skip)     { m_skipFlag[idx] = skip;   }
  Void          setSkipFlagSubParts   ( Bool skip, UInt absPartIdx, UInt depth );

  Char*         getPredictionMode     ()                        { return m_pePredMode;        }
  PredMode      getPredictionMode     ( UInt uiIdx )            { return static_cast<PredMode>( m_pePredMode[uiIdx] ); }
  Void          setPredictionMode     ( UInt uiIdx, PredMode uh){ m_pePredMode[uiIdx] = uh;   }
  Void          setPredModeSubParts   ( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth );

  Char*         getCrossComponentPredictionAlpha( ComponentID compID )             { return m_crossComponentPredictionAlpha[compID];         }
  Char          getCrossComponentPredictionAlpha( UInt uiIdx, ComponentID compID ) { return m_crossComponentPredictionAlpha[compID][uiIdx];  }

  Bool*         getCUTransquantBypass ()                        { return m_CUTransquantBypass;        }
  Bool          getCUTransquantBypass( UInt uiIdx )             { return m_CUTransquantBypass[uiIdx]; }

  UChar*        getWidth              ()                        { return m_puhWidth;          }
  UChar         getWidth              ( UInt uiIdx )            { return m_puhWidth[uiIdx];   }
  Void          setWidth              ( UInt uiIdx, UChar  uh ) { m_puhWidth[uiIdx] = uh;     }

  UChar*        getHeight             ()                        { return m_puhHeight;         }
  UChar         getHeight             ( UInt uiIdx )            { return m_puhHeight[uiIdx];  }
  Void          setHeight             ( UInt uiIdx, UChar  uh ) { m_puhHeight[uiIdx] = uh;    }

  Void          setSizeSubParts       ( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth );

  Char*         getQP                 ()                        { return m_phQP;              }
  Char          getQP                 ( UInt uiIdx ) const      { return m_phQP[uiIdx];       }
  Void          setQP                 ( UInt uiIdx, Char value ){ m_phQP[uiIdx] =  value;     }
  Void          setQPSubParts         ( Int qp,   UInt uiAbsPartIdx, UInt uiDepth );
  Int           getLastValidPartIdx   ( Int iAbsPartIdx );
  Char          getLastCodedQP        ( UInt uiAbsPartIdx );
  Void          setQPSubCUs           ( Int qp, TComDataCU* pcCU, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf );
  Void          setCodedQP            ( Char qp )               { m_codedQP = qp;             }
  Char          getCodedQP            ()                        { return m_codedQP;           }

  UChar*        getChromaQpAdj        ()                        { return m_ChromaQpAdj;       }
  UChar         getChromaQpAdj        (Int idx)           const { return m_ChromaQpAdj[idx];  }
  Void          setChromaQpAdj        (Int idx, UChar val)      { m_ChromaQpAdj[idx] = val;   }
  Void          setChromaQpAdjSubParts( UChar val, Int absPartIdx, Int depth );
  Void          setCodedChromaQpAdj   ( Char qp )               { m_codedChromaQpAdj = qp;    }
  Char          getCodedChromaQpAdj   ()                        { return m_codedChromaQpAdj;  }

  Bool          isLosslessCoded       ( UInt absPartIdx );

  UChar*        getTransformIdx       ()                        { return m_puhTrIdx;          }
  UChar         getTransformIdx       ( UInt uiIdx )            { return m_puhTrIdx[uiIdx];   }
  Void          setTrIdxSubParts      ( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getTransformSkip      ( ComponentID compID )    { return m_puhTransformSkip[compID];}
  UChar         getTransformSkip      ( UInt uiIdx, ComponentID compID)    { return m_puhTransformSkip[compID][uiIdx];}
  Void          setTransformSkipSubParts  ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth);
  Void          setTransformSkipSubParts  ( const UInt useTransformSkip[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth );

  UChar*        getExplicitRdpcmMode      ( ComponentID component ) { return m_explicitRdpcmMode[component]; }
  UChar         getExplicitRdpcmMode      ( ComponentID component, UInt partIdx ) {return m_explicitRdpcmMode[component][partIdx]; }
  Void          setExplicitRdpcmModePartRange ( UInt rdpcmMode, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );

  Bool          isRDPCMEnabled         ( UInt uiAbsPartIdx )  { return getSlice()->getSPS()->getUseResidualDPCM(isIntra(uiAbsPartIdx) ? RDPCM_SIGNAL_IMPLICIT : RDPCM_SIGNAL_EXPLICIT); }

  Void          setCrossComponentPredictionAlphaPartRange    ( Char alphaValue, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );
  Void          setTransformSkipPartRange                    ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes );

  UInt          getQuadtreeTULog2MinSizeInCU( UInt uiIdx );

  TComCUMvField* getCUMvField         ( RefPicList e )          { return  &m_acCUMvField[e];  }
    
// Hossam: Scene change
    TComCUMvField* getCUMvFieldSC         ( RefPicList e )          { return  &m_acCUMvFieldSC[e];  }

  TCoeff*&      getCoeff              (ComponentID component)   { return m_pcTrCoeff[component]; }

#if ADAPTIVE_QP_SELECTION
  TCoeff*&      getArlCoeff           ( ComponentID component ) { return m_pcArlCoeff[component]; }
#endif
  Pel*&         getPCMSample          ( ComponentID component ) { return m_pcIPCMSample[component]; }

  UChar         getCbf    ( UInt uiIdx, ComponentID eType )                  { return m_puhCbf[eType][uiIdx];  }
  UChar*        getCbf    ( ComponentID eType )                              { return m_puhCbf[eType];         }
  UChar         getCbf    ( UInt uiIdx, ComponentID eType, UInt uiTrDepth )  { return ( ( getCbf( uiIdx, eType ) >> uiTrDepth ) & 0x1 ); }
  Void          setCbf    ( UInt uiIdx, ComponentID eType, UChar uh )        { m_puhCbf[eType][uiIdx] = uh;    }
  Void          clearCbf  ( UInt uiIdx, ComponentID eType, UInt uiNumParts );
  UChar         getQtRootCbf          ( UInt uiIdx );

  Void          setCbfSubParts        ( const UInt uiCbf[MAX_NUM_COMPONENT],  UInt uiAbsPartIdx, UInt uiDepth           );
  Void          setCbfSubParts        ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth                    );
  Void          setCbfSubParts        ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth    );

  Void          setCbfPartRange       ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes      );
  Void          bitwiseOrCbfPartRange ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes      );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for coding tool information
  // -------------------------------------------------------------------------------------------------------------------

  Bool*         getMergeFlag          ()                        { return m_pbMergeFlag;               }
  Bool          getMergeFlag          ( UInt uiIdx )            { return m_pbMergeFlag[uiIdx];        }
  Void          setMergeFlag          ( UInt uiIdx, Bool b )    { m_pbMergeFlag[uiIdx] = b;           }
  Void          setMergeFlagSubParts  ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  UChar*        getMergeIndex         ()                        { return m_puhMergeIndex;                         }
  UChar         getMergeIndex         ( UInt uiIdx )            { return m_puhMergeIndex[uiIdx];                  }
  Void          setMergeIndex         ( UInt uiIdx, UInt uiMergeIndex ) { m_puhMergeIndex[uiIdx] = uiMergeIndex;  }
  Void          setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  template <typename T>
  Void          setSubPart            ( T bParameter, T* pbBaseLCU, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx );

#if AMP_MRG
  Void          setMergeAMP( Bool b )      { m_bIsMergeAMP = b; }
  Bool          getMergeAMP( )             { return m_bIsMergeAMP; }
#endif

  UChar*        getIntraDir         ( const ChannelType channelType )                   const { return m_puhIntraDir[channelType];         }
  UChar         getIntraDir         ( const ChannelType channelType, const UInt uiIdx ) const { return m_puhIntraDir[channelType][uiIdx];  }

  Void          setIntraDirSubParts ( const ChannelType channelType,
                                      const UInt uiDir,
                                      const UInt uiAbsPartIdx,
                                      const UInt uiDepth );

  UChar*        getInterDir           ()                        { return m_puhInterDir;               }
  UChar         getInterDir           ( UInt uiIdx )            { return m_puhInterDir[uiIdx];        }
  Void          setInterDir           ( UInt uiIdx, UChar  uh ) { m_puhInterDir[uiIdx] = uh;          }
  Void          setInterDirSubParts   ( UInt uiDir,  UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Bool*         getIPCMFlag           ()                        { return m_pbIPCMFlag;               }
  Bool          getIPCMFlag           (UInt uiIdx )             { return m_pbIPCMFlag[uiIdx];        }
  Void          setIPCMFlag           (UInt uiIdx, Bool b )     { m_pbIPCMFlag[uiIdx] = b;           }
  Void          setIPCMFlagSubParts   (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth);

    // Hossam: Scene change
    UChar*        getInterDirSC           ()                        { return m_puhInterDirSC;               }
    UChar         getInterDirSC           ( UInt uiIdx )            { return m_puhInterDirSC[uiIdx];        }
    Void          setInterDirSC           ( UInt uiIdx, UChar  uh ) { m_puhInterDirSC[uiIdx] = uh;          }
    Void          setInterDirSubPartsSC   ( UInt uiDir,  UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
    
    
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for accessing partition information
  // -------------------------------------------------------------------------------------------------------------------

  Void          getPartIndexAndSize   ( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight );
  UChar         getNumPartitions      ( const UInt uiAbsPartIdx = 0 );
  Bool          isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth);

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for motion vector
  // -------------------------------------------------------------------------------------------------------------------

  Void          getMvField            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField );

    
// Hossam: Scene change
    Void          getMvFieldSC            ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField );
    

  // Hossam: Scene Change
   Void          fillMvpCandOrg           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo );
    
// Hossam: Scene change
  Void          fillMvpCandSC           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo );
    
    
  Void          fillMvpCand           ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo );
  Bool          isDiffMER             ( Int xN, Int yN, Int xP, Int yP);
  Void          getPartPosition       ( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH);

  Void          setMVPIdx             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPIdx)  { m_apiMVPIdx[eRefPicList][uiIdx] = iMVPIdx;  }
  Int           getMVPIdx             ( RefPicList eRefPicList, UInt uiIdx)               { return m_apiMVPIdx[eRefPicList][uiIdx];     }
  Char*         getMVPIdx             ( RefPicList eRefPicList )                          { return m_apiMVPIdx[eRefPicList];            }

  Void          setMVPNum             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPNum ) { m_apiMVPNum[eRefPicList][uiIdx] = iMVPNum;  }
  Int           getMVPNum             ( RefPicList eRefPicList, UInt uiIdx )              { return m_apiMVPNum[eRefPicList][uiIdx];     }
  Char*         getMVPNum             ( RefPicList eRefPicList )                          { return m_apiMVPNum[eRefPicList];            }

  Void          setMVPIdxSubParts     ( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
  Void          setMVPNumSubParts     ( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

  // Hossam: Scene change ORG
    Void          setMVPIdxSC             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPIdx)  { m_apiMVPIdxSC[eRefPicList][uiIdx] = iMVPIdx;  }
    Int           getMVPIdxSC             ( RefPicList eRefPicList, UInt uiIdx)               { return m_apiMVPIdxSC[eRefPicList][uiIdx];     }
    Char*         getMVPIdxSC             ( RefPicList eRefPicList )                          { return m_apiMVPIdxSC[eRefPicList];            }
    
    Void          setMVPNumSC             ( RefPicList eRefPicList, UInt uiIdx, Int iMVPNum ) { m_apiMVPNumSC[eRefPicList][uiIdx] = iMVPNum;  }
    Int           getMVPNumSC             ( RefPicList eRefPicList, UInt uiIdx )              { return m_apiMVPNumSC[eRefPicList][uiIdx];     }
    Char*         getMVPNumSC             ( RefPicList eRefPicList )                          { return m_apiMVPNumSC[eRefPicList];            }
    
    Void          setMVPIdxSubPartsSC     ( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );
    Void          setMVPNumSubPartsSC     ( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth );

    
    // -------------------------------------------------------------------------------------------------------------------
    // Debug
    // -------------------------------------------------------------------------------------------------------------------
    Void mockPrintMvs(TComDataCU* pcCU, Int type, RefPicList eRefPicList);

    
    
  Void          clipMv                ( TComMv&     rcMv     );
    
    // Hossam: This could be a problem bugggg!!!!!
  Void          getMvPredLeft         ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldA.getMv(); }
  Void          getMvPredAbove        ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldB.getMv(); }
  Void          getMvPredAboveRight   ( TComMv&     rcMvPred )   { rcMvPred = m_cMvFieldC.getMv(); }

  Void          compressMV            ();

  // -------------------------------------------------------------------------------------------------------------------
  // utility functions for neighbouring information
  // -------------------------------------------------------------------------------------------------------------------

  TComDataCU*   getCULeft                   () { return m_pcCULeft;       }
  TComDataCU*   getCUAbove                  () { return m_pcCUAbove;      }
  TComDataCU*   getCUAboveLeft              () { return m_pcCUAboveLeft;  }
  TComDataCU*   getCUAboveRight             () { return m_pcCUAboveRight; }
  TComDataCU*   getCUColocated              ( RefPicList eRefPicList ) { return m_apcCUColocated[eRefPicList]; }


  TComDataCU*   getPULeft                   ( UInt&  uiLPartUnitIdx,
                                              UInt uiCurrPartUnitIdx,
                                              Bool bEnforceSliceRestriction=true,
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAbove                  ( UInt&  uiAPartUnitIdx,
                                              UInt uiCurrPartUnitIdx,
                                              Bool bEnforceSliceRestriction=true,
                                              Bool planarAtLCUBoundary = false,
                                              Bool bEnforceTileRestriction=true );
  TComDataCU*   getPUAboveLeft              ( UInt&  uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getPUAboveRight             ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getPUBelowLeft              ( UInt&  uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction=true );

  TComDataCU*   getQpMinCuLeft              ( UInt&  uiLPartUnitIdx , UInt uiCurrAbsIdxInLCU );
  TComDataCU*   getQpMinCuAbove             ( UInt&  uiAPartUnitIdx , UInt uiCurrAbsIdxInLCU );
  Char          getRefQP                    ( UInt   uiCurrAbsIdxInLCU                       );

  TComDataCU*   getPUAboveRightAdi          ( UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );
  TComDataCU*   getPUBelowLeftAdi           ( UInt&  uiBLPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset = 1, Bool bEnforceSliceRestriction=true );

  Void          deriveLeftRightTopIdx       ( UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdx         ( UInt uiPartIdx, UInt& ruiPartIdxLB );

  Void          deriveLeftRightTopIdxAdi    ( UInt& ruiPartIdxLT, UInt& ruiPartIdxRT, UInt uiPartOffset, UInt uiPartDepth );
  Void          deriveLeftBottomIdxAdi      ( UInt& ruiPartIdxLB, UInt  uiPartOffset, UInt uiPartDepth );

  Bool          hasEqualMotion              ( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx );
  Void          getInterMergeCandidates       ( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand, Int mrgCandIdx = -1 );

  Void          deriveLeftRightTopIdxGeneral  ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT );
  Void          deriveLeftBottomIdxGeneral    ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for modes
  // -------------------------------------------------------------------------------------------------------------------

  Bool          isIntra            ( UInt uiPartIdx )  const { return m_pePredMode[ uiPartIdx ] == MODE_INTRA;                                              }
  Bool          isInter            ( UInt uiPartIdx )  const { return m_pePredMode[ uiPartIdx ] == MODE_INTER;                                              }
  Bool          isSkipped          ( UInt uiPartIdx );                                                     ///< SKIP (no residual)
  Bool          isBipredRestriction( UInt puIdx );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for symbol prediction (most probable / mode conversion)
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getIntraSizeIdx                 ( UInt uiAbsPartIdx                                       );

  Void          getAllowedChromaDir             ( UInt uiAbsPartIdx, UInt* uiModeList );
  Int           getIntraDirPredictor            ( UInt uiAbsPartIdx, Int uiIntraDirPred[NUM_MOST_PROBABLE_MODES], const ComponentID compID, Int* piMode = NULL );

  // -------------------------------------------------------------------------------------------------------------------
  // member functions for SBAC context
  // -------------------------------------------------------------------------------------------------------------------

  UInt          getCtxSplitFlag                 ( UInt   uiAbsPartIdx, UInt uiDepth                   );
  UInt          getCtxQtCbf                     ( TComTU &rTu, const ChannelType chType );

  UInt          getCtxSkipFlag                  ( UInt   uiAbsPartIdx                                 );
  UInt          getCtxInterDir                  ( UInt   uiAbsPartIdx                                 );

  UInt          getSliceStartCU         ( UInt pos )                  { return m_sliceStartCU[pos-m_uiAbsIdxInLCU];        }
  UInt          getSliceSegmentStartCU  ( UInt pos )                  { return m_sliceSegmentStartCU[pos-m_uiAbsIdxInLCU]; }
  UInt&         getTotalBins            ()                            { return m_uiTotalBins;                              }
  // -------------------------------------------------------------------------------------------------------------------
  // member functions for RD cost storage
  // -------------------------------------------------------------------------------------------------------------------

  Double&       getTotalCost()                  { return m_dTotalCost;        }
  Distortion&   getTotalDistortion()            { return m_uiTotalDistortion; }
  UInt&         getTotalBits()                  { return m_uiTotalBits;       }
  UInt&         getTotalNumPart()               { return m_uiNumPartition;    }

  UInt          getCoefScanIdx(const UInt uiAbsPartIdx, const UInt uiWidth, const UInt uiHeight, const ComponentID compID) const ;

// Hossam: Scene change
public:
    // Extract Info from a certain partition CU
    Void xExtractPartitionInfo(UInt uiDepth, UInt uiAbsPartIdx, UInt uiPartUnitIdx, Bool bAdd = false);
    
    // Dumps the intra information
    Void dumpIntraInfo(UInt uiDepth, UInt uiAbsPartIdx, UInt uiPartUnitIdx, Bool bAdd = false);
    
    Void getIntraModesCount(UInt uiDepth, UInt uiAbsPartIdx, UInt uiPartUnitIdx, Bool bAdd = false);
    
    // Get the total number of parts
    Void getTotalPartsCount(UInt uiDepth, UInt uiAbsPartIdx, UInt uiPartUnitIdx, Bool bAdd = false);
    
    // Gets the reference utitlization for the nearest reference frame
    Void getNearestRefUtilization(UInt uiDepth, UInt uiAbsPartIdx, UInt uiPartUnitIdx, Bool bAdd = false);

    
    // Gets the total bits for the CU based on each ref utitlization for the nearest reference frame
    Void getTotalBitsRefI(UInt uiDepth, UInt uiAbsPartIdx, UInt uiPartUnitIdx = -1, Bool bAdd = false);

    
    // Prints some information about the CU
    Void showMeCU(UInt uiDepth, UInt uiAbsPartIdx, Int uiPartUnitIdx = -1, Bool bAdd = false);

    

};


namespace RasterAddress
{
  /** Check whether 2 addresses point to the same column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA % numUnitsPerRow == addrB % numUnitsPerRow
    return (( addrA ^ addrB ) &  ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether 2 addresses point to the same row
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRow( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    // addrA / numUnitsPerRow == addrB / numUnitsPerRow
    return (( addrA ^ addrB ) &~ ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether 2 addresses point to the same row or column
   * \param addrA          First address in raster scan order
   * \param addrB          Second address in raters scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isEqualRowOrCol( Int addrA, Int addrB, Int numUnitsPerRow )
  {
    return isEqualCol( addrA, addrB, numUnitsPerRow ) | isEqualRow( addrA, addrB, numUnitsPerRow );
  }

  /** Check whether one address points to the first column
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroCol( Int addr, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow == 0
    return ( addr & ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether one address points to the first row
   * \param addr           Address in raster scan order
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool isZeroRow( Int addr, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow == 0
    return ( addr &~ ( numUnitsPerRow - 1 ) ) == 0;
  }

  /** Check whether one address points to a column whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given column index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanCol( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr % numUnitsPerRow < val
    return ( addr & ( numUnitsPerRow - 1 ) ) < val;
  }

  /** Check whether one address points to a row whose index is smaller than a given value
   * \param addr           Address in raster scan order
   * \param val            Given row index value
   * \param numUnitsPerRow Number of units in a row
   * \return Result of test
   */
  static inline Bool lessThanRow( Int addr, Int val, Int numUnitsPerRow )
  {
    // addr / numUnitsPerRow < val
    return addr < val * numUnitsPerRow;
  }
}

//! \}

#endif
