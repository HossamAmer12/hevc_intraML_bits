//
//  TEncOffsetCalc.cpp
//  HM
//
//  Created by Hossam Amer on 2016-01-24.
//
//

#include "TEncOffsetCalc.h"
#include <list>
#include <algorithm>
#include <functional>

#include "TEncTop.h"
#include "TEncGOP.h"
#include "TEncAnalyze.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/NAL.h"
#include "NALwrite.h"
#include <time.h>
#include <math.h>

using namespace std;


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// =================================================================================

TEncOffsetCalc::TEncOffsetCalc()
{
    // SC last scene change
    m_iLastSC = -1;
    
    // SC state
    m_iSCState = -1;
    
    // Yc for the last SC
    m_iLastYcSC = 0;
    
    // Current Yc for the current frame
    m_iCurrentYc = 0;
    
    // isSmooth flag
    m_isSmooth = false;
    
    // isSceneChange flag
    m_isSceneChange = false;
    
    // Currnet POC
    m_uiCurrentPOC = 0;
    
    // Current Case Number
    m_uiCurrentCaseNumber = 0;
    
    
    // Init for the stuff here
//    m_pcCfg               = NULL;
//    m_pcSliceEncoder      = NULL;
//    m_pcListPic           = NULL;
    
    
}

TEncOffsetCalc::~TEncOffsetCalc()
{
}

/** Create list to contain pointers to LCU start addresses of slice.
 */
Void  TEncOffsetCalc::create()
{
    // SC last scene change
    m_iLastSC = -1;
    
    // SC state
    m_iSCState = -1;
    
    // Yc for the last SC
    m_iLastYcSC = 0;
    
    // Current Yc for the current frame
    m_iCurrentYc = 0;
    
    // currentQP
    m_uiCurrentQP = 0;
}

Void  TEncOffsetCalc::destroy()
{
}

Void TEncOffsetCalc::init ()
{
    // SC last scene change
    m_iLastSC = -1;
    
    // SC state
    m_iSCState = -1;
    
    // Yc for the last SC
    m_iLastYcSC = 0;
    
    // Current Yc for the current frame
    m_iCurrentYc = 0;
    
    // isSmooth flag
    m_isSmooth = false;
    
    // isSceneChange flag
    m_isSceneChange = false;
    
    // Currnet POC
    m_uiCurrentPOC = 0;
    
    // Current Case Number
    m_uiCurrentCaseNumber = 0;
    
    // currentQP
    m_uiCurrentQP = 0;
    
    //QPfact IDx
    m_uiCurrentQPFactIdx = 0;
    
    // Current Depth IDx
    m_uiCurrentDepthIdx = 0;
    
}

// static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};
Int TEncOffsetCalc::getQpFactSCIdxHelper()
{
    Int idx = 0;
    
    switch (m_uiCurrentCaseNumber) {
        case 1:
            idx = 1;
            break;
        case 2:
            idx = 1;
            break;
        case 3:
            idx = 1;
            break;
        case 4:
            idx = 3;
            break;
            
        default:
            cout << "ERROR getQpFactSCIdxHelper OFFSET CALC " << endl;
            break;
    }
    
    m_uiCurrentQPFactIdx = idx;
    return idx;

}

// static int depths[] ={2, 1, 2, 0};
Int TEncOffsetCalc::getDepthSCIdxHelper()
{
    Int idx = 0;
    
    switch (m_uiCurrentCaseNumber) {
        case 1:
            idx = 1;
            break;
        case 2:
            idx = 1;
            break;
        case 3:
            idx = 1;
            break;
        case 4:
            idx = 3;
            break;
            
        default:
            cout << "ERROR getQpFactSCIdxHelper OFFSET CALC " << endl;
            break;
    }
    
    // Save it!
    m_uiCurrentDepthIdx = idx;
    return idx;
    
}

// static int depths[] ={2, 1, 2, 0};
Int TEncOffsetCalc::getDepthConceptIdxHelper()
{
    Int idx = 0;
    
    switch (m_uiCurrentCaseNumber) {
        case 1:
            idx = m_isSmooth? 1:0;
            break;
        case 2:
            idx = m_isSmooth? 1:0;
            break;
        case 3:
            idx = m_isSmooth? 1:0;
            break;
        case 4:
            idx = m_isSmooth? 0:0;
            break;
            
        default:
            
            cout << "ERROR getQpFactConceptIdxHelper OFFSET CALC " << endl;
            break;
    }
    
    // Save it!
    m_uiCurrentDepthIdx = idx;
    return idx;
}

// static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};
Int TEncOffsetCalc::getQpFactConceptIdxHelper()
{
    Int idx = 0;
    
    switch (m_uiCurrentCaseNumber) {
        case 1:
            idx = getQpFactConceptIdxHelper1();
            break;
        case 2:
            idx = getQpFactConceptIdxHelper2();
            break;
        case 3:
            idx = getQpFactConceptIdxHelper3();
            break;
        case 4:
            idx = getQpFactConceptIdxHelper4();
            break;
            
        default:
            
            cout << "ERROR getQpFactConceptIdxHelper OFFSET CALC " << endl;
            break;
    }
    
    // Save it
    m_uiCurrentQPFactIdx = idx;
    return idx;
}

Int TEncOffsetCalc::getQpFactConceptIdxHelper1()
{
    Int idx = 0;
    
    if (m_isSmooth) {
        idx = 1;
    }
    else
    {
        idx = 0;
    }
    
    return idx;
}


Int TEncOffsetCalc::getQpFactConceptIdxHelper2()
{
    Int idx = 0;
    
    if (m_isSmooth) {
        idx = 1;
    }
    else
    {
        idx = 0;
    }
    
    return idx;
}

Int TEncOffsetCalc::getQpFactConceptIdxHelper3()
{
    Int idx = 0;
    
    if (m_isSmooth) {
        idx = 1;
    }
    else
    {
        idx = 0;
    }
    
    return idx;
}

Int TEncOffsetCalc::getQpFactConceptIdxHelper4()
{
    Int idx = 0;
    
    if (m_isSmooth) {
        idx = 0;
    }
    else
    {
        idx = 0;
    }
    
    return idx;
}

//Int TEncOffsetCalc::getOffset(TComSlice *pcSlice, Bool isSmooth, Bool isSceneChange, Int scState, Int lastSc, Double currentYc)
Int TEncOffsetCalc::getOffset(TComSlice *pcSlice, Bool isSmooth, Bool isSceneChange, Int scState, Int lastSc, Int currentQP)
{
    
    Int offset = 0;
    
    // Set the Calculator variables
    m_iLastSC = lastSc;
    m_iSCState = scState;
    m_iLastYcSC = -1200; // Init dummy for now
    m_iCurrentYc = -1200; // Init dummy for now
    m_isSmooth = isSmooth;
    m_isSceneChange = isSceneChange;
    m_uiCurrentPOC = pcSlice->getPOC();
    m_uiCurrentQP = currentQP;
    
    if(m_isSceneChange)
    {
        getCaseNumber();
        offset = getOffsetSC();
        getQpFactSCIdxHelper();
        getDepthSCIdxHelper();

    }
    else if(m_uiCurrentPOC == m_iLastSC + 1)
    {
        offset = getOffsetConceptI();
        getQpFactConceptIdxHelper();
        getDepthConceptIdxHelper();
    }
    else
    {
        // Will be left for the TEncSlice!
    }
    

    
    return offset;
    
}

Int TEncOffsetCalc::getOffsetSC()
{
    Int offset = +1;
    return offset;
}


Int TEncOffsetCalc::getOffsetConceptI()
{
    Int offset = 0;
    
    switch (m_uiCurrentCaseNumber) {
        case 1:
            offset = getOffsetConceptI1();
            break;
        case 2:
            offset = getOffsetConceptI2();
            break;
        case 3:
            offset = getOffsetConceptI3();
            break;
        case 4:
            offset = getOffsetConceptI4();
            break;
    
        default:
            
            cout << "ERROR getOffsetConceptI OFFSET CALC " << endl;
            exit(0);
            break;
    }
    
    return offset;
}

Int TEncOffsetCalc::getOffsetConceptI1()
{
    Int offset = 0;
    
    if (m_isSmooth) {
        
        switch (m_uiCurrentQP) {
            case 22:
                offset = -2;
                break;
            case 27:
                offset = -2;
                break;
            case 32:
            case 37:
                offset = -4;
                break;
            case 42:
                offset = -6;
                break;
            default:
                offset = 0;
                cout << "ERROR getOffsetConceptI1" << endl;
                break;
        }
    }
    
    else
    {
        offset = +3;
    }
    
    return offset;
}

Int TEncOffsetCalc::getOffsetConceptI2()
{
    Int offset = 0;
    
    if (m_isSmooth) {
        
        switch (m_uiCurrentQP) {
            case 22:
                offset = -2;
                break;
            case 27:
                offset = -2;
                break;
            case 32:
            case 37:
                offset = -4;
                break;
            case 42:
                offset = -6;
                break;
            default:
                offset = 0;
                cout << "ERROR getOffsetConceptI2" << endl;
                break;
        }
    }
    
    else
    {
        offset = +3;
    }
    
    return offset;
}

Int TEncOffsetCalc::getOffsetConceptI3()
{
    Int offset = 0;
    
    if (m_isSmooth) {
        
        switch (m_uiCurrentQP) {
            case 22:
                offset = -2;
                break;
            case 27:
                offset = -2;
                break;
            case 32:
            case 37:
                offset = -4;
                break;
            case 42:
                offset = -6;
                break;
            default:
                offset = 0;
                cout << "ERROR getOffsetConceptI3" << endl;
                break;
        }
    }
    else
    {
        offset = +3;
    }
    
    return offset;
}

Int TEncOffsetCalc::getOffsetConceptI4()
{
    Int offset = 0;
    
    if (m_isSmooth) {
        offset = +3;
    }
    else
    {
        offset = +3;
    }
    
    return offset;
}

Void TEncOffsetCalc::getCaseNumber()
{
    Int pocHelper = m_uiCurrentPOC; // Will be called at SC
    
    if(pocHelper % 4 == 0)
    {
        m_uiCurrentCaseNumber = 4;
    }
    else if(pocHelper % 4 == 1)
    {
        m_uiCurrentCaseNumber = 1;
    }
    else if(pocHelper % 4 == 2)
    {
        m_uiCurrentCaseNumber = 2;
    }
    else if(pocHelper % 4 == 3)
    {
        m_uiCurrentCaseNumber = 3;
    }
    else
    {
        m_uiCurrentCaseNumber = -1;
        cout << "ERROR ERROR ERROR In get Case Number() Offset calculator " << endl;
        exit(-1);
    }
}


