//
//  TEncOffsetCalc.h
//  HM
//
//  Created by Hossam Amer on 2016-01-24.
//
//

#ifndef __HM__TEncOffsetCalc__
#define __HM__TEncOffsetCalc__

#include <stdio.h>

#include <list>

#include "TLibCommon/TComList.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComLoopFilter.h"
#include "TLibCommon/AccessUnit.h"
#include "TEncSampleAdaptiveOffset.h"
//#include "TEncSlice.h"


#include "TEncAnalyze.h"
#include "TEncRateCtrl.h"
#include <vector>

// #include "TEncSceneChange.h"

// To create the access channel
class TEncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TEncOffsetCalc
{
private:
    
    //  Access channel -- List of Pics
//    TEncTop*                m_pcEncTop;
//    TEncCfg*                m_pcCfg;
//    TEncSlice*              m_pcSliceEncoder;
//    TComList<TComPic*>*     m_pcListPic;
    
    // SC last scene change
    Int                 m_iLastSC;
    
    // SC state
    Int                 m_iSCState;
    
    // Yc for the last SC
    Double m_iLastYcSC;
    
    // Current Yc for the current frame
    Double m_iCurrentYc;
    
    // isSmooth flag
    Bool m_isSmooth;
    
    // isSceneChange flag
    Bool m_isSceneChange;
    
    // Currnet POC
    Int m_uiCurrentPOC;
    
    // Current Case Number
    Int m_uiCurrentCaseNumber;
    
    // Current QP
    Int m_uiCurrentQP;
    
    // Current qpFactIdx
    Int m_uiCurrentQPFactIdx;
    Int m_uiCurrentDepthIdx;
    
    

public:
    
    TEncOffsetCalc();
    virtual ~TEncOffsetCalc();
    
    Void      create          ();
    Void      destroy         ();
//    Void      init ( TEncTop* pcTEncTop );
    Void      init ();
    Void      deletePicBuffer ();
    
    
    // Getters and Setters
    Void      setLastSC(Int sc) {m_iLastSC = sc;}
    Int       getLastSC(){return m_iLastSC;}
    
    Void      setSCstate(Int sc) {m_iSCState = sc;}
    Int       getSCState(){return m_iSCState;}
    
    Void      setCurrPOC(Int sc) {m_uiCurrentPOC = sc;}
    Int       getCurrPOC(){return m_uiCurrentPOC;}
    
    Void      setCurrCase(Int sc) {m_uiCurrentCaseNumber = sc;}
    Int       getCurrCase(){return m_uiCurrentCaseNumber;}
    
    Void      setLastYcSC(Double sc) {m_iLastYcSC = sc;}
    Double    getLastYcSC(){return m_iLastYcSC;}

    Void      setCurrentYc(Double sc) {m_iCurrentYc = sc;}
    Double    getCurrentYc(){return m_iCurrentYc;}
    
    Void      setSmooth(Bool sc) {m_isSmooth = sc;}
    Bool      getSmooth(){return m_isSmooth;}
    
    Void      setSCflag(Bool sc) {m_isSceneChange = sc;}
    Bool      getSCflag(){return m_isSceneChange;}
    
    
    Int      getQpFactIdx(){return m_uiCurrentQPFactIdx;}
    Int      getDepthIdx(){return m_uiCurrentDepthIdx;}
    
    
    
    // Main method 
//    Int getOffset(TComSlice* pcSlice, Bool isSmooth, Bool isSceneChange, Int scState, Int lastSc, Double currentYc);
    Int getOffset(TComSlice* pcSlice, Bool isSmooth, Bool isSceneChange, Int scState, Int lastSc, Int currentQP);
    
    
    // SC
    Int getOffsetSC();
    Int getQpFactSCIdxHelper();
    Int getDepthSCIdxHelper();
    
    // Concept I
    Int getQpFactConceptIdxHelper();
    Int getQpFactConceptIdxHelper1();
    Int getQpFactConceptIdxHelper2();
    Int getQpFactConceptIdxHelper3();
    Int getQpFactConceptIdxHelper4();
    
    // Concept I
    Int getDepthConceptIdxHelper();
    Int getDepthConceptIdxHelper1();
    Int getDepthConceptIdxHelper2();
    Int getDepthConceptIdxHelper3();
    Int getDepthConceptIdxHelper4();

    // Concept I
    Int getOffsetConceptI();
    Int getOffsetConceptI1();
    Int getOffsetConceptI2();
    Int getOffsetConceptI3();
    Int getOffsetConceptI4();
    

    Void getCaseNumber();
    
    
    
    
};// END CLASS DEFINITION TEncOffsetCalc


#endif /* defined(__HM__TEncOffsetCalc__) */

