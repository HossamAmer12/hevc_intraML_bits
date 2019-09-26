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

/** \file     TEncTop.h
 \brief    encoder class (header)
 */

#ifndef __TENCTOP__
#define __TENCTOP__

// Include files
#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"

#include "TLibVideoIO/TVideoIOYuv.h"

#include "TEncCfg.h"
#include "TEncGOP.h"
#include "TEncSlice.h"
#include "TEncEntropy.h"
#include "TEncCavlc.h"
#include "TEncSbac.h"
#include "TEncSearch.h"
#include "TEncSampleAdaptiveOffset.h"
#include "TEncPreanalyzer.h"
#include "TEncRateCtrl.h"

// Scene change engine
#include "TEncSceneChange.h"


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class TEncTop : public TEncCfg
{
private:
    // picture
    Int                     m_iPOCLast;                     ///< time index (POC)
    Int                     m_iNumPicRcvd;                  ///< number of received pictures
    UInt                    m_uiNumAllPicCoded;             ///< number of coded pictures
    TComList<TComPic*>      m_cListPic;                     ///< dynamic list of pictures
    
    
    
    // Hossam: Look ahead list
    TComList<TComPic*>      m_cListPicLookAhead;                     ///< dynamic list of pictures

    
    // Hossam: look ahead
    Int m_iPOCLastLast;
    Int m_iNumPicRvdLast;
    
    // Hossam: Scene Change
    TComList<TComPic*>      m_cListOrgPic;                     ///< dynamic list of original pictures

    
    // encoder search
    TEncSearch              m_cSearch;                      ///< encoder search class
    //TEncEntropy*            m_pcEntropyCoder;                     ///< entropy encoder
    TEncCavlc*              m_pcCavlcCoder;                       ///< CAVLC encoder
    // coding tool
    TComTrQuant             m_cTrQuant;                     ///< transform & quantization class
    TComLoopFilter          m_cLoopFilter;                  ///< deblocking filter class
    TEncSampleAdaptiveOffset m_cEncSAO;                     ///< sample adaptive offset class
    TEncEntropy             m_cEntropyCoder;                ///< entropy encoder
    TEncCavlc               m_cCavlcCoder;                  ///< CAVLC encoder
    TEncSbac                m_cSbacCoder;                   ///< SBAC encoder
    TEncBinCABAC            m_cBinCoderCABAC;               ///< bin coder CABAC
    TEncSbac*               m_pcSbacCoders;                 ///< SBAC encoders (to encode substreams )
    TEncBinCABAC*           m_pcBinCoderCABACs;             ///< bin coders CABAC (one per substream)
    
    
    // processing unit
    TEncGOP                 m_cGOPEncoder;                  ///< GOP encoder
    TEncSlice               m_cSliceEncoder;                ///< slice encoder
    TEncCu                  m_cCuEncoder;                   ///< CU encoder
    
    // Scene change processing unit
    TEncSceneChange       m_cSCEncoder;                   ///< SC encoder
    
    
    // SPS
    TComSPS                 m_cSPS;                         ///< SPS
    TComPPS                 m_cPPS;                         ///< PPS
    // RD cost computation
    TComBitCounter          m_cBitCounter;                  ///< bit counter for RD optimization
    TComRdCost              m_cRdCost;                      ///< RD cost computation class
    TEncSbac***             m_pppcRDSbacCoder;              ///< temporal storage for RD computation
    TEncSbac                m_cRDGoOnSbacCoder;             ///< going on SBAC model for RD stage
#if FAST_BIT_EST
    TEncBinCABACCounter***  m_pppcBinCoderCABAC;            ///< temporal CABAC state storage for RD computation
    TEncBinCABACCounter     m_cRDGoOnBinCoderCABAC;         ///< going on bin coder CABAC for RD stage
#else
    TEncBinCABAC***         m_pppcBinCoderCABAC;            ///< temporal CABAC state storage for RD computation
    TEncBinCABAC            m_cRDGoOnBinCoderCABAC;         ///< going on bin coder CABAC for RD stage
#endif
    Int                     m_iNumSubstreams;                ///< # of top-level elements allocated.
    TComBitCounter*         m_pcBitCounters;                 ///< bit counters for RD optimization per substream
    TComRdCost*             m_pcRdCosts;                     ///< RD cost computation class per substream
    TEncSbac****            m_ppppcRDSbacCoders;             ///< temporal storage for RD computation per substream
    TEncSbac*               m_pcRDGoOnSbacCoders;            ///< going on SBAC model for RD stage per substream
    TEncBinCABAC****        m_ppppcBinCodersCABAC;           ///< temporal CABAC state storage for RD computation per substream
    TEncBinCABAC*           m_pcRDGoOnBinCodersCABAC;        ///< going on bin coder CABAC for RD stage per substream
    
    // quality control
    TEncPreanalyzer         m_cPreanalyzer;                 ///< image characteristics analyzer for TM5-step3-like adaptive QP
    
    TComScalingList         m_scalingList;                 ///< quantization matrix information
    TEncRateCtrl            m_cRateCtrl;                    ///< Rate control class
    
protected:
    Void  xGetNewPicBuffer  ( TComPic*& rpcPic );           ///< get picture buffer which will be processed

    // Hossam: Look ahead buffer
    Void  xGetNewPicBufferLookAhead  ( TComPic*& rpcPic );           ///< get picture buffer which will be processed

    // Hossam: Look Ahead actual
    Void  xGetNewPicBufferLookActual  ( TComPic*& rpcPic );           ///< get picture buffer which will be processed

    
    
    // Hossam: Scene Change
    Void  xGetNewPicBufferOrg  ( TComPic*& rpcPic );           ///< get ORG picture buffer which will be processed
    
    Void  xInitSPS          ();                             ///< initialize SPS from encoder options
    Void  xInitPPS          ();                             ///< initialize PPS from encoder options
    
    Void  xInitPPSforTiles  ();
    Void  xInitRPS          (Bool isFieldCoding);           ///< initialize PPS from encoder options
    
    
    
    
public:
    
    // Hossam: to enable semi-multipass and look ahead
    Void setLastNumPicRcvd();
    Void updateNumePicRcvd();
    Void setLastPOCLast();
    Void updatePOCLast();
    
    Void showMeRcLists();
    
    Void xRestorePicBufferInTop();
    
    Void revertPOCLast() { m_iPOCLast = m_iPOCLastLast; };
    Void revertNumPicRcvd() {m_iNumPicRcvd = m_iNumPicRvdLast;};
    Void updateOverallPicsRecived() {m_uiNumAllPicCoded += m_iNumPicRcvd;};
    
    Void IncrementOverallPicsRecivedByOne() {m_uiNumAllPicCoded += 1;};
    
    Void IncrementPOCLastByOne() {m_iPOCLast = (m_iPOCLast + 1) % m_iGOPSize;};

    Void incrementPicsRecivedByOne() {m_iNumPicRcvd += 1;};

    
    
    Void reportCounterStatus () {
        cout << "((((((((( TEncTop REPORT : POCLast, RcvdPics, allCoded " << m_iPOCLast << "\t"
        << ", " << m_iNumPicRcvd << ", " << m_uiNumAllPicCoded << endl;
        
        cout << "((((((((( TEncTop REPORT LASTLAST : POCLast, RcvdPics, allCoded " << m_iPOCLastLast << "\t"
        << ", " << m_iNumPicRvdLast << ", " << m_uiNumAllPicCoded << endl;
        
    }
    
    
    // Hossam
    Void  xInitRPSNew          (Bool isFieldCoding, Int state);           ///< initialize PPS from SC
    //    Void  xInitRPSNew          (Bool isFieldCoding);           ///< initialize PPS from SC options
    
    // Hossam: GOP_STR_TYPE
    //    UInt m_gop_str_type;
    //    std::string m_inputFileName;
    TEncTop();
    virtual ~TEncTop();
    
    Void      create          ();
    Void      destroy         ();
    Void      init            (Bool isFieldCoding);
    Void      deletePicBuffer ();
    
    Void      deleteOrgPicBuffer ();
    
    // Hossam: Look ahead list
    Void      deletePicBufferLookAhead ();
    
    
    
    Void      createWPPCoders(Int iNumSubstreams);
    
    // -------------------------------------------------------------------------------------------------------------------
    // member access functions
    // -------------------------------------------------------------------------------------------------------------------
    
    TComList<TComPic*>*     getListPic            () { return  &m_cListPic;             }

   // Hossam: Scene Change: Return by address because it's fast - Copies the address to the caller
   TComList<TComPic*>*     getListOrgPic          () { return  &m_cListOrgPic;        }

    TEncSearch*             getPredSearch         () { return  &m_cSearch;              }
    
    TComTrQuant*            getTrQuant            () { return  &m_cTrQuant;             }
    TComLoopFilter*         getLoopFilter         () { return  &m_cLoopFilter;          }
    TEncSampleAdaptiveOffset* getSAO              () { return  &m_cEncSAO;              }
    TEncGOP*                getGOPEncoder         () { return  &m_cGOPEncoder;          }
    TEncSlice*              getSliceEncoder       () { return  &m_cSliceEncoder;        }
    TEncCu*                 getCuEncoder          () { return  &m_cCuEncoder;           }
    TEncEntropy*            getEntropyCoder       () { return  &m_cEntropyCoder;        }
    TEncCavlc*              getCavlcCoder         () { return  &m_cCavlcCoder;          }
    TEncSbac*               getSbacCoder          () { return  &m_cSbacCoder;           }
    TEncBinCABAC*           getBinCABAC           () { return  &m_cBinCoderCABAC;       }
    TEncSbac*               getSbacCoders     () { return  m_pcSbacCoders;      }
    TEncBinCABAC*           getBinCABACs          () { return  m_pcBinCoderCABACs;      }
    
    TComBitCounter*         getBitCounter         () { return  &m_cBitCounter;          }
    TComRdCost*             getRdCost             () { return  &m_cRdCost;              }
    TEncSbac***             getRDSbacCoder        () { return  m_pppcRDSbacCoder;       }
    TEncSbac*               getRDGoOnSbacCoder    () { return  &m_cRDGoOnSbacCoder;     }
    TComBitCounter*         getBitCounters        () { return  m_pcBitCounters;         }
    TComRdCost*             getRdCosts            () { return  m_pcRdCosts;             }
    TEncSbac****            getRDSbacCoders       () { return  m_ppppcRDSbacCoders;     }
    TEncSbac*               getRDGoOnSbacCoders   () { return  m_pcRDGoOnSbacCoders;   }
    TEncRateCtrl*           getRateCtrl           () { return &m_cRateCtrl;             }
    TComSPS*                getSPS                () { return  &m_cSPS;                 }
    TComPPS*                getPPS                () { return  &m_cPPS;                 }
    Void selectReferencePictureSet(TComSlice* slice, Int POCCurr, Int GOPid );
    Int getReferencePictureSetIdxForSOP(TComSlice* slice, Int POCCurr, Int GOPid );
    TComScalingList*        getScalingList        () { return  &m_scalingList;         }
    
    // Hossam
    TEncSceneChange*            getSceneChangeCoder       () { return  &m_cSCEncoder;     }
//    Void selectReferencePictureSetNew(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSC );
    
    Void selectReferencePictureSetNew(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSC, Bool isSmooth );

    
    Int getCaseNumber(Int POCCurr);
    Int getSelectIdx (Int CaseNumber, Bool isSmooth);
    
//    Void selectReferencePictureSetNewAttempt(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSC );
     Void selectReferencePictureSetNewAttempt(TComSlice* slice, Int POCCurr, Int GOPid, Int state, Int lastSC, Bool isSmooth );
    
    Void resetSelectAttemptProcess(Bool isSmooth, Int POC);
    // -------------------------------------------------------------------------------------------------------------------
    // encoder function
    // -------------------------------------------------------------------------------------------------------------------

    // Hossam: Scene Chnage: several number of pictures until end-of-sequence
    Void encodeNew( Bool bEos,
                TComPicYuv* pcPicYuvOrg,
                TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
                TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg,
                std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );
    
    
    // Hossam: Scene Chnage: several number of pictures until end-of-sequence
    Void encodeNewInter( Bool bEos,
                   TComPicYuv* pcPicYuvOrg,
                   TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
                   TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg,
                   std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );

    
    // Hossam: encode P frames
    Void encodeNewP04( Bool bEos,
                   TComPicYuv* pcPicYuvOrg,
                   TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
                   TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg,
                   std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );
    
    Void encodeNewP(Bool isMockEncoding, Bool isAddPSNR, Bool bEos,
                    TComPicYuv* pcPicYuvOrg,
                    TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
                    TComList<TComPicYuv*>& rcListPicYuvRecOut, TComList<TComPicYuv*>& rcListPicYuvOrg,
                    std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );
    
    /// encode several number of pictures until end-of-sequence
    Void encode( Bool bEos,
                TComPicYuv* pcPicYuvOrg,
                TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
                TComList<TComPicYuv*>& rcListPicYuvRecOut,
                std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded );
    
    /// encode several number of pictures until end-of-sequence
    Void encode( Bool bEos, TComPicYuv* pcPicYuvOrg,
                TComPicYuv* pcPicYuvTrueOrg, const InputColourSpaceConversion snrCSC, // used for SNR calculations. Picture in original colour space.
                TComList<TComPicYuv*>& rcListPicYuvRecOut,
                std::list<AccessUnit>& accessUnitsOut, Int& iNumEncoded, Bool isTff);
    
    Void printSummary(Bool isField) { m_cGOPEncoder.printOutSummary (m_uiNumAllPicCoded, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE); }
    
    // Dumps reference utilization
    Void dumpReferenceUtilization()
    {
#if IS_CALC_INTERDEP_REF_UTILIZATION_Acc
        m_cGOPEncoder.dumpReferenceUtilizationAccumulated();
#else
        m_cGOPEncoder.dumpReferenceUtilization();
#endif
    }
    
};

//! \}

#endif // __TENCTOP__

