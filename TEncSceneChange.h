//
//  TEncSceneChange.h
//  HM
//
//  Created by Hossam Amer on 2015-03-10.
//
//

#ifndef __HM__TEncSceneChange__
#define __HM__TEncSceneChange__

#include <stdio.h>

#endif /* defined(__HM__TEncSceneChange__) */

#ifndef __TENCSC__
#define __TENCSC__

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

#define noSCs   15

//! \ingroup TLibEncoder
//! \{


#if OUT_OUTLIER
extern ofstream OutlierYuvFile;
#endif

#if GEN_RESI_FRAME
extern ofstream residualYuvFile;
#endif

// To create an access channel
class TEncTop;


// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TEncSceneChange
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
#if ALLOW_RECOVERY_POINT_AS_RAP
    Int                     m_iLastRecoveryPicPOC;
#endif
    
    //  Access channel -- List of Pics
    TEncTop*                m_pcEncTop;
    TEncCfg*                m_pcCfg;
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
    
    // clean decoding refresh
    Bool                    m_bRefreshPending;
    Int                     m_pocCRA;
    std::vector<Int>        m_storedStartCUAddrForEncodingSlice;
    std::vector<Int>        m_storedStartCUAddrForEncodingSliceSegment;
    
    // SC last scene change
    Int                 m_iLastSC;
    
    Int                 m_iSCState;
    
    // Yc for the last SC
    Double m_iLastYcSC;
    
    // Current Yc for the current frame
    Double m_iCurrentYc;
    
    // History of Ycs
    std::vector<double> ycArray;
    
    
    // Sum of Ycs so far
    double sumYc;
    
    // Number of frames so far --> POC
    
    
    // current_variance for the fast variance
    Double prev_variance;
    

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
    
    // Sigma squared original
    Double curr_sigmaSquared;
    
    // SC change temp list
public:
    UInt m_scList[noSCs]; // Using the directive above
    //        static const UInt m_scList[noSCs]; // Using the directive above
    
    //    static const UInt scList[] = {-1, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130};
    
public:
    
    TEncSceneChange();
    virtual ~TEncSceneChange();
    
    Void      create          ();
    Void      destroy         ();
    Void      init ( TEncTop* pcTEncTop );
    Void      deletePicBuffer ();
    
//    Bool      isSceneChange(UInt POC);
    
        Bool      isSceneChangePOC(UInt POC);
    
    // Scene change detection algorithm to be
    Bool      isSceneChange(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic, TComPic* pcPic);

    
    // Scene change with interdep
    Bool      isSceneChangeInter(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic, TComPic* pcPic);
    
    // Scene change detection algorithm from original
    Bool      isSceneChangeOrg(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic, TComPic* pcPic);

    
    // Scene change detection algorithm: Ding
    Bool      isSceneChangeDing(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic, TComPic* pcPic);
    
    // Scene change detection algorithm: Yao
    Bool      isSceneChangeYao(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic, TComPic* pcPic);
    
    
    // Scene change detection algorithm: Sastre
    Bool      isSceneChangeSastre(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic, TComPic* pcPic);
    
    
    Void      deleteSCList();
    
    Void      setLastSC(Int sc) {m_iLastSC = sc;}
    
    Int       getLastSC(){return m_iLastSC;}
    
    Void      setSCstate(Int sc) {m_iSCState = sc;}
    
    Int       getSCState(){return m_iSCState;}
    
    
    // Gets the current sigma squared original
    Double getSigmaSquaredOriginal(){ return curr_sigmaSquared;}
    
    // Student-t distribution
    Bool    isSceneChangeStudentT   (Double current_yc, UInt current_poc);
    
    
    // Student-t distribution Energy
    Bool    isSceneChangeStudentTEnergy   (Double energy, UInt current_poc);
    
    
    // is the input sequence a YouTube sequence?
    Bool isYouTubeSequence();

    
    //    Void    getOutliers   (TComPic* rpcPic);
    
    Bool    getOutliers   (TComPic* rpcPic);
    
    Bool    getSATD   (TComPic* rpcPic);
    Distortion xCalcHADs4x4( Pel *diffOrg, Int iStrideOrg, Int iStep );
    
    Void    getResiduals   (TComPic* rpcPic);
    
    
    // Get the energy for the residual frame
    Bool    getEnergy   (TComPic* rpcPic);

    
    // Get the energy for the residual frame
    Double    calculateEnergy(Pel *pData, UInt uiStrideSrc, const UInt uiFrameWidth, const UInt uiFrameHeight);
    
    
    Pel getMax(Pel* p, UInt uiWidth, UInt uiHeight);
    
    Pel getMin(Pel* p, UInt uiWidth, UInt uiHeight);
    
    Double countOutliers(TComPicYuv* data, Double Yc);
    
    Bool isSceneChangeStd(Double current_yc, UInt current_poc);
    
    Bool isSceneChangeStdReset(Double current_yc, UInt current_poc);

    
    Bool isSmooth();
    
    Double getCurrentMean();
    
    Double getCurrentStd(Double current_mean);
    
    
    Double getCurrentStdFast(Double current_mean);

    Double getSigmaSquaredOrig(UInt POC, TComSlice* pcSlice, TComList<TComPic*>& rcListPic,  TComPic* pcPic, Int which_reference = -1);
    
//    Void xWriteResiduals(TComPicYuv* data, const ComponentID ch);
    
    
#if GEN_OUTLIER
    Void    getOutlierWithDCT   (TComPic* rpcPic);
#endif
    
};// END CLASS DEFINITION TEncSceneChange

//! \}

#endif // __TENCSC__

