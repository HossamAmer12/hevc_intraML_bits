//// Hossam be y2ool:
//// Hossam: I produced this method to send the buffer -- it's now used here 18/05/2016
////  rcListPic: Original frames I guess
////  rcListPicYuvRecOut: Reconstructed frames Out, list of reconstruction YUV files coming from the very top encoder main, sent by reference
////  rcListPicYuvOrg: My list
//Void TEncGOP::compressGOPNew(TComList<TComPic *> &rcListPicYuvOrg, Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*> &rcListPic, TComList<TComPicYuv *> &rcListPicYuvRecOut, std::list<AccessUnit> &accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE)
////Void TEncGOP:: compressGOPNew ( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic, TComList<TComPicYuv*>& rcListPicYuvRec,
////                                     std::list<AccessUnit>& accessUnitsInGOP, Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE, TComList<TComPicYuv*>& rcListPicYuvOrg )
//{
//    
//    cout << "Yang: TEncGOP: compressGOP: NEW Yalla beena :)" << "\n" << endl;
//    //    getchar();
//    
//    TComPic*        pcPic;
//    TComPicYuv*     pcPicYuvRecOut;
//    TComSlice*      pcSlice;
//    TComOutputBitstream  *pcBitstreamRedirect;
//    pcBitstreamRedirect = new TComOutputBitstream;
//    AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted
//    UInt                  uiOneBitstreamPerSliceLength = 0;
//    TEncSbac* pcSbacCoders = NULL;
//    TComOutputBitstream* pcSubstreamsOut = NULL;
//    
//    // Hossam: Initializing the GOP with the necessary static size is done in this method
//    // Hossam: Very narrow functionality
//    // GOP size static
//    // isField is always sent as true --- Check the TEncTop file
//    xInitGOP( iPOCLast, iNumPicRcvd, rcListPic, rcListPicYuvRecOut, isField );
//    
//    // SC previous & current state;
//    // Hossam: It's a nice idea, but in this case
//    // There won't be communication with the Entropy encoder, no Bitstream write
//    // Either I save the steps (not feasible coz I will do different ME), or I skip some steps
//    // as init...etc
//    //    Int sc_prev_state, sc_current_state;
//    
//    m_iNumPicCoded = 0;
//    SEIPictureTiming pictureTimingSEI;
//    Bool writeSOP = m_pcCfg->getSOPDescriptionSEIEnabled();
//    
//    // Initialize Scalable Nesting SEI with single layer values
//    SEIScalableNesting scalableNestingSEI;
//    scalableNestingSEI.m_bitStreamSubsetFlag           = 1;      // If the nested SEI messages are picture buffereing SEI mesages, picure timing SEI messages or sub-picture timing SEI messages, bitstream_subset_flag shall be equal to 1
//    scalableNestingSEI.m_nestingOpFlag                 = 0;
//    scalableNestingSEI.m_nestingNumOpsMinus1           = 0;      //nesting_num_ops_minus1
//    scalableNestingSEI.m_allLayersFlag                 = 0;
//    scalableNestingSEI.m_nestingNoOpMaxTemporalIdPlus1 = 6 + 1;  //nesting_no_op_max_temporal_id_plus1
//    scalableNestingSEI.m_nestingNumLayersMinus1        = 1 - 1;  //nesting_num_layers_minus1
//    scalableNestingSEI.m_nestingLayerId[0]             = 0;
//    scalableNestingSEI.m_callerOwnsSEIs                = true;
//    
//    Int picSptDpbOutputDuDelay = 0;
//    UInt *accumBitsDU = NULL;
//    UInt *accumNalsDU = NULL;
//    SEIDecodingUnitInfo decodingUnitInfoSEI;
//    
//#if EFFICIENT_FIELD_IRAP
//    Int IRAPGOPid = -1;
//    Bool IRAPtoReorder = false;
//    Bool swapIRAPForward = false;
//    if(isField)
//    {
//        Int pocCurr;
//        for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
//        {
//            //        cout << "Yang: TEncGOP: compressGOP: GopId--A " << (iGOPid) << " is inProgress" << "\n" << endl;
//            //        getchar();
//            
//            // determine actual POC
//            if(iPOCLast == 0) //case first frame or first top field
//            {
//                pocCurr=0;
//            }
//            else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
//            {
//                pocCurr = 1;
//            }
//            else
//            {
//                pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - isField;
//            }
//            
//            // Dady: Gets the NAL unit type based on the slice type before my tweaking
//            // check if POC corresponds to IRAP
//            NalUnitType tmpUnitType = getNalUnitType(pocCurr, m_iLastIDR, isField);
//            if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
//            {
//                if(pocCurr%2 == 0 && iGOPid < m_iGopSize-1 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid+1).m_POC-1)
//                { // if top field and following picture in enc order is associated bottom field
//                    IRAPGOPid = iGOPid;
//                    IRAPtoReorder = true;
//                    swapIRAPForward = true;
//                    break;
//                }
//                if(pocCurr%2 != 0 && iGOPid > 0 && m_pcCfg->getGOPEntry(iGOPid).m_POC == m_pcCfg->getGOPEntry(iGOPid-1).m_POC+1)
//                {
//                    // if picture is an IRAP remember to process it first
//                    IRAPGOPid = iGOPid;
//                    IRAPtoReorder = true;
//                    swapIRAPForward = false;
//                    break;
//                }
//            }
//        }
//    }
//#endif
//    
//    
//    // Have an attempt with a copy of rcList
//    //    TComList<TComPic*> rcListPicAttempt;
//    
//    // loop on GOPids
//    for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
//    {
//        
//        //      cout << "Yang: TEncGOP: compressGOP: GopId--B " << (iGOPid) << " is inProgress" << "\n" << endl;
//        //      getchar();
//        
//        
//#if EFFICIENT_FIELD_IRAP
//        if(IRAPtoReorder)
//        {
//            if(swapIRAPForward)
//            {
//                if(iGOPid == IRAPGOPid)
//                {
//                    iGOPid = IRAPGOPid +1;
//                }
//                else if(iGOPid == IRAPGOPid +1)
//                {
//                    iGOPid = IRAPGOPid;
//                }
//            }
//            else
//            {
//                if(iGOPid == IRAPGOPid -1)
//                {
//                    iGOPid = IRAPGOPid;
//                }
//                else if(iGOPid == IRAPGOPid)
//                {
//                    iGOPid = IRAPGOPid -1;
//                }
//            }
//        }
//#endif
//        
//        UInt uiColDir = 1;
//        //-- For time output for each slice
//        clock_t iBeforeTime = clock();
//        
//        //select uiColDir
//        Int iCloseLeft=1, iCloseRight=-1;
//        for(Int i = 0; i<m_pcCfg->getGOPEntry(iGOPid).m_numRefPics; i++)
//        {
//            Int iRef = m_pcCfg->getGOPEntry(iGOPid).m_referencePics[i];
//            if(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))
//            {
//                iCloseRight=iRef;
//            }
//            else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1))
//            {
//                iCloseLeft=iRef;
//            }
//        }
//        if(iCloseRight>-1)
//        {
//            iCloseRight=iCloseRight+m_pcCfg->getGOPEntry(iGOPid).m_POC-1;
//        }
//        if(iCloseLeft<1)
//        {
//            iCloseLeft=iCloseLeft+m_pcCfg->getGOPEntry(iGOPid).m_POC-1;
//            while(iCloseLeft<0)
//            {
//                iCloseLeft+=m_iGopSize;
//            }
//        }
//        Int iLeftQP=0, iRightQP=0;
//        for(Int i=0; i<m_iGopSize; i++)
//        {
//            if(m_pcCfg->getGOPEntry(i).m_POC==(iCloseLeft%m_iGopSize)+1)
//            {
//                iLeftQP= m_pcCfg->getGOPEntry(i).m_QPOffset;
//            }
//            if (m_pcCfg->getGOPEntry(i).m_POC==(iCloseRight%m_iGopSize)+1)
//            {
//                iRightQP=m_pcCfg->getGOPEntry(i).m_QPOffset;
//            }
//        }
//        if(iCloseRight>-1&&iRightQP<iLeftQP)
//        {
//            uiColDir=0;
//        }
//        
//        /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
//        Int iTimeOffset;
//        Int pocCurr;
//        
//        if(iPOCLast == 0) //case first frame or first top field
//        {
//            pocCurr=0;
//            iTimeOffset = 1;
//        }
//        else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
//        {
//            pocCurr = 1;
//            iTimeOffset = 1;
//        }
//        else
//        {
//            pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1:0);
//            iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
//        }
//        
//        if(pocCurr>=m_pcCfg->getFramesToBeEncoded())
//        {
//#if EFFICIENT_FIELD_IRAP
//            if(IRAPtoReorder)
//            {
//                if(swapIRAPForward)
//                {
//                    if(iGOPid == IRAPGOPid)
//                    {
//                        iGOPid = IRAPGOPid +1;
//                        IRAPtoReorder = false;
//                    }
//                    else if(iGOPid == IRAPGOPid +1)
//                    {
//                        iGOPid --;
//                    }
//                }
//                else
//                {
//                    if(iGOPid == IRAPGOPid)
//                    {
//                        iGOPid = IRAPGOPid -1;
//                    }
//                    else if(iGOPid == IRAPGOPid -1)
//                    {
//                        iGOPid = IRAPGOPid;
//                        IRAPtoReorder = false;
//                    }
//                }
//            }
//#endif
//            continue;
//        }
//        
//        
//        
//        
//        if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
//        {
//            m_iLastIDR = pocCurr;
//        }
//        
//        
//        
//        //==========================================================================================
//        // start a new access unit: create an entry in the list of output access units
//        accessUnitsInGOP.push_back(AccessUnit());
//        AccessUnit& accessUnit = accessUnitsInGOP.back();
//        
//        // Hossam: Get the current pic from the list
//        // Hossam: XXXX SKIP BUFFER THIS STEP
//        xGetBuffer( rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isField );
//        
//        //        TComPic*        pcPicOrg;
//        //        xGetBufferNew(rcListPicYuvOrg, iNumPicRcvd, iTimeOffset, pcPicOrg, pocCurr, isField);
//        
//        //  Slice data initialization
//        pcPic->clearSliceBuffer();
//        assert(pcPic->getNumAllocatedSlice() == 1);
//        m_pcSliceEncoder->setSliceIdx(0);
//        pcPic->setCurrSliceIdx(0);
//        
//        
//        
//        //================================Hossam==========================================================
//        // Start Attempt
//        
//        //        samah
//        // Hossam: It should be put in the initSlice due to the assignment of the B slice in the beginining of the method
//        // Initialize the Slice Encoder
//        //        m_pcSliceEncoder->initEncSliceNew ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField, isSceneChange, m_pcEncTop->getSceneChangeCoder()->getLastSC() );
//        
//        // Hossam: Scene Change ORG
//        //        m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField );
//        m_pcSliceEncoder->initEncSliceNewAttempt ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField, isSceneChange, m_pcEncTop->getSceneChangeCoder()->getLastSC(), m_pcEncTop->getSceneChangeCoder()->getSCState(), isSmooth);
//        
//        //Set Frame/Field coding
//        pcSlice->getPic()->setField(isField);
//        
//        pcSlice->setLastIDR(m_iLastIDR);
//        pcSlice->setSliceIdx(0);
//        //set default slice level flag to the same as SPS level flag
//        pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );
//        pcSlice->setScalingList ( m_pcEncTop->getScalingList()  );
//        if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_OFF)
//        {
//            m_pcEncTop->getTrQuant()->setFlatScalingList(pcSlice->getSPS()->getChromaFormatIdc());
//            m_pcEncTop->getTrQuant()->setUseScalingList(false);
//            m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
//            m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
//        }
//        else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_DEFAULT)
//        {
//            pcSlice->setDefaultScalingList ();
//            m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
//            m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
//            m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
//            m_pcEncTop->getTrQuant()->setUseScalingList(true);
//        }
//        else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_FILE_READ)
//        {
//            if(pcSlice->getScalingList()->xParseScalingList(m_pcCfg->getScalingListFile()))
//            {
//                pcSlice->setDefaultScalingList ();
//            }
//            pcSlice->getScalingList()->checkDcOfMatrix();
//            m_pcEncTop->getSPS()->setScalingListPresentFlag(pcSlice->checkDefaultScalingList());
//            m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
//            m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
//            m_pcEncTop->getTrQuant()->setUseScalingList(true);
//        }
//        else
//        {
//            printf("error : ScalingList == %d no support\n",m_pcEncTop->getUseScalingListId());
//            assert(0);
//        }
//        
//        // Hossam: Setting the slice type -- Code Tweaking!!
//        //=================================================================
//        
//        /*
//         Int sc = pcSlice->getPOC();
//         if(sc == 2)
//         {
//         cout << "\nYang: TEncGOP: compressGOP: Forcing an I frame " << sc << "\n" << endl;
//         //    getchar();
//         pcSlice->setSliceType(I_SLICE);
//         //        getchar();
//         }
//         else
//         */
//        // Hossam: Setting the slice type -- Code Tweaking ENDS!!
//        //=================================================================
//        
//        // Hossam: Slice Type is set to P_SLICE here in compress GOP
//        if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
//        {
//            pcSlice->setSliceType(P_SLICE);
//        }
//        if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
//        {
//            pcSlice->setSliceType(I_SLICE);
//        }
//        
//        // Set the nal unit type
//        pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
//        if(pcSlice->getTemporalLayerNonReferenceFlag())
//        {
//            if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
//                !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
//                // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
//            {
//                cout << "nalUnitType:  NAL_UNIT_CODED_SLICE_TRAIL_N " << endl;
//                pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
//            }
//            if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
//            {
//                cout << "nalUnitType:  NAL_UNIT_CODED_SLICE_RADL_N " << endl;
//                pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
//            }
//            if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
//            {
//                cout << "nalUnitType:  NAL_UNIT_CODED_SLICE_RADL_N " << endl;
//                pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
//            }
//        }
//        
//#if EFFICIENT_FIELD_IRAP
//#if FIX1172
//        if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
//        {
//            m_associatedIRAPType = pcSlice->getNalUnitType();
//            m_associatedIRAPPOC = pocCurr;
//        }
//        pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
//        pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
//#endif
//#endif
//        
//        // Hossam: Scene change
//        if(m_pcEncTop->getSceneChangeCoder()->getLastSC() > 0 && pocCurr == m_pcEncTop->getSceneChangeCoder()->getLastSC() + 1)
//        {
//            //            cout << "compressGOP: STATEEEEEE FOURRRRRRRRRRRRR ATT " << m_pcEncTop->getSceneChangeCoder()->getLastSC()  << " " << pocCurr << endl;
//            
//            // Set the last SC happened!
//            // Should be defined in states
//            m_pcEncTop -> getSceneChangeCoder() -> setSCstate(4);
//            
//            // reset the select attempt process
//            m_pcEncTop->resetSelectAttemptProcess(isSmooth, pocCurr);
//        }
//        
//        // Do decoding refresh marking if any
//        pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic);
//        
//        // Attempt with the true references in either before an SC exist or not!
//        m_pcEncTop->selectReferencePictureSetNewAttempt(pcSlice, pocCurr, iGOPid, m_pcEncTop->getSceneChangeCoder()->getSCState(), m_pcEncTop->getSceneChangeCoder()->getLastSC(), isSmooth);
//        
//        //        m_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
//        
//        //        cout << "I'm done with select " << boolalpha << (pcSlice->getRPS()==NULL) << endl;
//        pcSlice->getRPS()->setNumberOfLongtermPictures(0);
//        //        cout << "I'm done with after select " << endl;
//        
//        
//#if !EFFICIENT_FIELD_IRAP
//#if FIX1172
//        if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
//            || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
//        {
//            m_associatedIRAPType = pcSlice->getNalUnitType();
//            m_associatedIRAPPOC = pocCurr;
//        }
//        pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
//        pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
//#endif
//#endif
//        
//        Bool printCheckErrors = true; // Initially false
//#if ALLOW_RECOVERY_POINT_AS_RAP
//        if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), printCheckErrors, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP())
//#if EFFICIENT_FIELD_IRAP
//            || (isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
//#endif
//            )
//        {
//            pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3);
//        }
//#else
//        if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), printCheckErrors) != 0) || (pcSlice->isIRAP()))
//        {
//            pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP());
//        }
//#endif
//        
//        pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());
//        
//        // Hossam: True for all
//        //      cout << "SEG FAULT BEFORE "  << ", " << boolalpha << (pcSlice->getRefPic(REF_PIC_LIST_0, 0)==NULL) << endl;
//        
//        if(pcSlice->getTLayer() > 0
//           &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
//                 || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
//                 || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
//                 || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
//           )
//        {
//            if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
//            {
//                if(pcSlice->getTemporalLayerNonReferenceFlag())
//                {
//                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
//                }
//                else
//                {
//                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
//                }
//            }
//            else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
//            {
//                Bool isSTSA=true;
//                for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
//                {
//                    Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
//                    if(lTid==pcSlice->getTLayer())
//                    {
//                        TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
//                        for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
//                        {
//                            if(nRPS->getUsed(jj))
//                            {
//                                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
//                                Int kk=0;
//                                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
//                                {
//                                    if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
//                                        break;
//                                }
//                                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
//                                if(tTid >= pcSlice->getTLayer())
//                                {
//                                    isSTSA=false;
//                                    break;
//                                }
//                            }
//                        }
//                    }
//                }
//                if(isSTSA==true)
//                {
//                    if(pcSlice->getTemporalLayerNonReferenceFlag())
//                    {
//                        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
//                    }
//                    else
//                    {
//                        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
//                    }
//                }
//            }
//        }
//        arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
//        TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
//        refPicListModification->setRefPicListModificationFlagL0(0);
//        refPicListModification->setRefPicListModificationFlagL1(0);
//        pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
//        pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
//        
//        
//        // Hossam: XXXXX I think setList1toList can be removed!
//        //  Set reference list
//        //        pcSlice->setRefPicList ( rcListPic );
//        pcSlice->setRefPicList ( rcListPic );
//        pcSlice->setRefPOCList();
//        pcSlice->setList1IdxToList0Idx();
//        
//        
//        
//        //=================================================================
//        //========================ATTEMPT===================================
//        //=================================================================
//        // End Attempt
//        
//        
//        //=================================================================
//        //========================SC=======================================
//        //=================================================================
//        
//        
//        //        Bool isSceneChange = pcSlice->getPOC() == 11;// || pcSlice->getPOC() == 11;
//        
//        //         Bool isSceneChange = pcSlice->getPOC() == 11;// || pcSlice->getPOC() == 11;
//        
//        //        isSceneChange = pcSlice->getPOC() == 11 || pcSlice->getPOC() == 17 || pcSlice->getPOC() == 25;// || pcSlice->getPOC() == 11;
//        
//        //        isSceneChange = m_pcEncTop->getSceneChangeCoder()->isSceneChange(pcSlice->getPOC(), pcSlice, rcListPic, pcPic);
//        
//        
//        //#if SC_FROM_RECONS
//        //        isSceneChange = m_pcEncTop->getSceneChangeCoder()->isSceneChangePOC(pcSlice->getPOC());
//        //#else
//        isSceneChange = m_pcEncTop->getSceneChangeCoder()->isSceneChange(pcSlice->getPOC(), pcSlice, rcListPic, pcPic);
//        //#endif
//        isSmooth      = m_pcEncTop->getSceneChangeCoder()->isSmooth();// Hossam: this can be only used in the state + 1, leave it for now!
//        //
//        //        // HARD CODE
//        //        if (pocCurr == 59) {
//        //            isSmooth = true;
//        //        }
//        //
//        // NO SC, or before an SC --> Hossam XXXXX needs to get removed!
//        //        if (!isSceneChange && m_pcEncTop->getSceneChangeCoder()->getLastSC() <= 0) {
//        //
//        //        }
//        
//        if(isSceneChange)
//        {
//            
//            //            cout << "SCEENNEEEE CHANGE BIRDDDDDDD MANNNNNN " << pocCurr << endl;
//            // Reset the SC state -- Set it to zero- SC occured
//            m_pcEncTop -> getSceneChangeCoder() -> setSCstate(0);
//            // Set the last SC happened!
//            m_pcEncTop -> getSceneChangeCoder() -> setLastSC(pocCurr);
//            
//            // Hossam: XXX I don't think this makes much difference
//            //            m_iLastRecoveryPicPOC = pcSlice->getPOC();
//            //
//            //            // Reset the select flags in order to avoid the mismatch in the Select process later on
//            //            m_pcEncTop->resetSelectAttemptProcess();
//            
//        }
//        
//        else if(m_pcEncTop->getSceneChangeCoder()->getLastSC() > 0)
//        {
//            Int lastSc = m_pcEncTop->getSceneChangeCoder()->getLastSC();
//            //XXX In case Gopsize = 4;
//            //            if(pocCurr == lastSc + m_iGopSize + 1)
//            
//            // If I'm in state 4, act as a conceptual I frame and go back to state 0
//            //            if (m_pcEncTop->getSceneChangeCoder()->getSCState() == 4) {
//            //                m_pcEncTop -> getSceneChangeCoder() -> setSCstate(0);
//            //            }
//            
//            // Hossam: Scene Change
//            if(pocCurr == m_pcEncTop->getSceneChangeCoder()->getLastSC() + 1)
//            {
//                cout << "compressGOP: STATEEEEEE FOURRRRRRRRRRRRR " << m_pcEncTop->getSceneChangeCoder()->getLastSC()  << " " << pocCurr << endl;
//                
//                // Set the last SC happened!
//                // Should be defined in states
//                m_pcEncTop -> getSceneChangeCoder() -> setSCstate(4);
//                
//                m_iLastRecoveryPicPOC = pcSlice->getPOC();
//                
//                referenceSlice = isSmooth? 1:0;
//                
//                // Reset the select flags in order to avoid the mismatch in the Select process later on
//                //                m_pcEncTop->resetSelectAttemptProcess();
//                
//            }
//            else if(pocCurr == lastSc + m_iGopSize + 1 + referenceSlice)
//            {
//                cout << "compressGOP: STATEEEEEE ONEEEEE " << lastSc  << " " << pocCurr << endl;
//                
//                // Set the last SC happened!
//                m_pcEncTop -> getSceneChangeCoder() -> setSCstate(1);
//                // Adjust the references!
//                //                m_pcEncTop->xInitRPSNew(isField, m_pcEncTop->getSceneChangeCoder()->getSCState());
//                
//                //                pcSlice->setRPS(pcSlice->getSPS()->getRPSList() -> getReferencePictureSet(0));
//                
//            }
//            else if(pocCurr == lastSc + 2*m_iGopSize + 1 + referenceSlice)
//            {
//                // Set the last SC happened!
//                m_pcEncTop -> getSceneChangeCoder() -> setSCstate(2);
//                // Adjust the references!
//                //                m_pcEncTop->xInitRPSNew(isField, m_pcEncTop->getSceneChangeCoder()->getSCState());
//            }
//            else if(pocCurr == lastSc + 3*m_iGopSize + 1 + referenceSlice)
//            {
//                // Set the last SC happened!
//                m_pcEncTop -> getSceneChangeCoder() -> setSCstate(3);
//                
//            }
//            
//        }
//        
//        //        if(isSceneChange || m_pcEncTop->getSceneChangeCoder()->getLastSC() > 0)
//        if(m_pcEncTop->getSceneChangeCoder()->getLastSC() > 0)
//        {
//            cout << "Start Executing the SC modification " << endl;
//            
//            // Hossam: Scene Change - QP distribution
//            m_pcSliceEncoder->initEncSliceNew(pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField, isSceneChange, m_pcEncTop->getSceneChangeCoder()->getLastSC());
//            
//            //                        m_pcSliceEncoder->initEncSliceNew ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField, m_pcEncTop->getSceneChangeCoder()->isSceneChange(pocCurr), m_pcEncTop->getSceneChangeCoder()->getLastSC() );
//            //
//            // Send the isSceneChange --> REmovesd for smooth for now
//            //            m_pcSliceEncoder->initEncSliceNew ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField, isSceneChange, m_pcEncTop->getSceneChangeCoder()->getLastSC(),
//            //                                               m_pcEncTop->getSceneChangeCoder()->getSCState(), isSmooth);
//            
//            // Hossam: Reset the Scene change flag
//            isSceneChange = false;
//            
//            
//            //Set Frame/Field coding
//            pcSlice->getPic()->setField(isField);
//            
//            pcSlice->setLastIDR(m_iLastIDR);
//            pcSlice->setSliceIdx(0);
//            //set default slice level flag to the same as SPS level flag
//            pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );
//            pcSlice->setScalingList ( m_pcEncTop->getScalingList()  );
//            if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_OFF)
//            {
//                m_pcEncTop->getTrQuant()->setFlatScalingList(pcSlice->getSPS()->getChromaFormatIdc());
//                m_pcEncTop->getTrQuant()->setUseScalingList(false);
//                m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
//                m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
//            }
//            else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_DEFAULT)
//            {
//                pcSlice->setDefaultScalingList ();
//                m_pcEncTop->getSPS()->setScalingListPresentFlag(false);
//                m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
//                m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
//                m_pcEncTop->getTrQuant()->setUseScalingList(true);
//            }
//            else if(m_pcEncTop->getUseScalingListId() == SCALING_LIST_FILE_READ)
//            {
//                if(pcSlice->getScalingList()->xParseScalingList(m_pcCfg->getScalingListFile()))
//                {
//                    pcSlice->setDefaultScalingList ();
//                }
//                pcSlice->getScalingList()->checkDcOfMatrix();
//                m_pcEncTop->getSPS()->setScalingListPresentFlag(pcSlice->checkDefaultScalingList());
//                m_pcEncTop->getPPS()->setScalingListPresentFlag(false);
//                m_pcEncTop->getTrQuant()->setScalingList(pcSlice->getScalingList(), pcSlice->getSPS()->getChromaFormatIdc());
//                m_pcEncTop->getTrQuant()->setUseScalingList(true);
//            }
//            else
//            {
//                printf("error : ScalingList == %d no support\n",m_pcEncTop->getUseScalingListId());
//                assert(0);
//            }
//            
//            
//            // Hossam: Setting the slice type -- Code Tweaking ENDS!!
//            //=================================================================
//            
//            if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
//            {
//                pcSlice->setSliceType(P_SLICE);
//            }
//            if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
//            {
//                pcSlice->setSliceType(I_SLICE);
//            }
//            
//            // Set the nal unit type
//            pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
//            
//            if(pcSlice->getTemporalLayerNonReferenceFlag())
//            {
//                if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
//                    !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
//                    // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
//                {
//                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
//                }
//                if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
//                {
//                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
//                }
//                if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
//                {
//                    pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
//                }
//            }
//            
//#if EFFICIENT_FIELD_IRAP
//#if FIX1172
//            if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
//            {
//                m_associatedIRAPType = pcSlice->getNalUnitType();
//                m_associatedIRAPPOC = pocCurr;
//            }
//            pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
//            pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
//#endif
//#endif
//            
//            // Do decoding refresh marking if any
//            pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic);
//            
//            // Hossam: Scene change
//            // Select New
//            // Send the SC state to choose the the GOP entry
//            
//            //            m_pcEncTop->selectReferencePictureSetNew(pcSlice, pocCurr, iGOPid, m_pcEncTop->getSceneChangeCoder()->getSCState(), m_pcEncTop->getSceneChangeCoder()->getLastSC());
//            m_pcEncTop->selectReferencePictureSetNew(pcSlice, pocCurr, iGOPid, m_pcEncTop->getSceneChangeCoder()->getSCState(), m_pcEncTop->getSceneChangeCoder()->getLastSC(), isSmooth);
//            //      m_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);
//            
//            //            // Hossam: Set the new state accordingly
//            //            m_pcEncTop->getSceneChangeCoder()->setSCstate(state);
//            
//            
//            // Hossam: I don't need the SC state XXXXX
//            // After selection and before check thing Modify the RC List
//            // Modify the list accordingly PENDING XXXXXXX
//            // I wanted to check whether there is a SC, No need ! -> The condition guarantees
//            //           if(m_pcEncTop)
//            pcSlice->modifyRCList(rcListPic, pcSlice->getRPS(), m_pcEncTop->getSceneChangeCoder()->getSCState());
//            
//            
//            
//            pcSlice->getRPS()->setNumberOfLongtermPictures(0);
//            
//            
//            // Hossam: Scene change
//            if(m_pcEncTop->getSceneChangeCoder()->getLastSC() > 0 && pocCurr == m_pcEncTop->getSceneChangeCoder()->getLastSC() + 1)
//            {
//                //            cout << "compressGOP: STATEEEEEE FOURRRRRRRRRRRRR ATT " << m_pcEncTop->getSceneChangeCoder()->getLastSC()  << " " << pocCurr << endl;
//                // reset the select attempt process
//                m_pcEncTop->resetSelectAttemptProcess(isSmooth, pocCurr);
//            }
//            
//            // Hossam: Scene change: Reset the Smooth flag
//            isSmooth = false;
//            
//#if !EFFICIENT_FIELD_IRAP
//#if FIX1172
//            if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
//                || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
//            {
//                m_associatedIRAPType = pcSlice->getNalUnitType();
//                m_associatedIRAPPOC = pocCurr;
//            }
//            pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
//            pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
//#endif
//#endif
//            
//            
//#if ALLOW_RECOVERY_POINT_AS_RAP
//            if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), printCheckErrors, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP())
//#if EFFICIENT_FIELD_IRAP
//                || (isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
//#endif
//                )
//            {
//                /// Hossam: this one is most probably called
//                // Hossam: XXXXX Removed Create Explicit from Reference in SC coding path
//                pcSlice->createExplicitReferencePictureSetFromReferenceNew(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3);
//            }
//#else
//            // Hossam: XXXXX Removed Create Explicit from Reference in SC coding path
//            //                if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), printCheckErrors) != 0) || (pcSlice->isIRAP()))
//            if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), printCheckErrors) != 0) || (pcSlice->isIRAP()))
//                
//            {
//                pcSlice->createExplicitReferencePictureSetFromReferenceNew(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP());
//            }
//#endif
//            
//            pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());
//            
//            
//            if(pcSlice->getTLayer() > 0
//               &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
//                     || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
//                     || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
//                     || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
//               )
//            {
//                if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
//                {
//                    if(pcSlice->getTemporalLayerNonReferenceFlag())
//                    {
//                        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
//                    }
//                    else
//                    {
//                        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
//                    }
//                }
//                else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
//                {
//                    Bool isSTSA=true;
//                    for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
//                    {
//                        Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
//                        if(lTid==pcSlice->getTLayer())
//                        {
//                            TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
//                            for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
//                            {
//                                if(nRPS->getUsed(jj))
//                                {
//                                    Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
//                                    Int kk=0;
//                                    for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
//                                    {
//                                        if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
//                                            break;
//                                    }
//                                    Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
//                                    if(tTid >= pcSlice->getTLayer())
//                                    {
//                                        isSTSA=false;
//                                        break;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                    if(isSTSA==true)
//                    {
//                        if(pcSlice->getTemporalLayerNonReferenceFlag())
//                        {
//                            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
//                        }
//                        else
//                        {
//                            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
//                        }
//                    }
//                }
//            }
//            arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
//            // Hossam: XXXXX no use of this list!!!
//            TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
//            refPicListModification->setRefPicListModificationFlagL0(0);
//            refPicListModification->setRefPicListModificationFlagL1(0);
//            pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
//            pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
//            
//        }// end if isSceneChange
//        else// SC not working
//        {
//            cout << "Modify QP will happen now!!! " << endl;
//            cout << "Modify QP will Double Initialise now!!! " << endl;
//            
//            
//            //            // Send the isSceneChange
//            //            m_pcSliceEncoder->initEncSliceNew ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField, isSceneChange, m_pcEncTop->getSceneChangeCoder()->getLastSC(),
//            //                                               m_pcEncTop->getSceneChangeCoder()->getSCState(), isSmooth);
//            
//            
//            //            m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField );
//            m_pcSliceEncoder->modifyQP ( pcPic, iPOCLast, pocCurr, iNumPicRcvd, iGOPid, pcSlice, m_pcEncTop->getSPS(), m_pcEncTop->getPPS(), isField,       isSceneChange, m_pcEncTop->getSceneChangeCoder()->getLastSC(),
//                                        m_pcEncTop->getSceneChangeCoder()->getSCState(), isSmooth);
//        }// end else Scene change
//        
//        
//        //================================Hossam==========================================================
//        
//        
//#if ADAPTIVE_QP_SELECTION
//        pcSlice->setTrQuant( m_pcEncTop->getTrQuant() );
//#endif
//        
//        //  Set reference list
//        pcSlice->setRefPicList ( rcListPic );
//        
//        //  Slice info. refinement
//        if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
//        {
//            pcSlice->setSliceType ( P_SLICE );
//        }
//        
//        if (pcSlice->getSliceType() == B_SLICE)
//        {
//            pcSlice->setColFromL0Flag(1-uiColDir);
//            Bool bLowDelay = true;
//            Int  iCurrPOC  = pcSlice->getPOC();
//            Int iRefIdx = 0;
//            
//            for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
//            {
//                if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
//                {
//                    bLowDelay = false;
//                }
//            }
//            for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
//            {
//                if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
//                {
//                    bLowDelay = false;
//                }
//            }
//            
//            pcSlice->setCheckLDC(bLowDelay);
//        }
//        else
//        {
//            pcSlice->setCheckLDC(true);
//        }
//        
//        uiColDir = 1-uiColDir;
//        
//        //-------------------------------------------------------------
//        
//        pcSlice->setRefPOCList();
//        
//        pcSlice->setList1IdxToList0Idx();
//        
//        if (m_pcEncTop->getTMVPModeId() == 2)
//        {
//            if (iGOPid == 0) // first picture in SOP (i.e. forward B)
//            {
//                pcSlice->setEnableTMVPFlag(0);
//            }
//            else
//            {
//                // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
//                pcSlice->setEnableTMVPFlag(1);
//            }
//            pcSlice->getSPS()->setTMVPFlagsPresent(1);
//        }
//        else if (m_pcEncTop->getTMVPModeId() == 1)
//        {
//            pcSlice->getSPS()->setTMVPFlagsPresent(1);
//            pcSlice->setEnableTMVPFlag(1);
//        }
//        else
//        {
//            pcSlice->getSPS()->setTMVPFlagsPresent(0);
//            pcSlice->setEnableTMVPFlag(0);
//        }
//        /////////////////////////////////////////////////////////////////////////////////////////////////// Compress a slice
//        //  Slice compression
//        if (m_pcCfg->getUseASR())
//        {
//            m_pcSliceEncoder->setSearchRange(pcSlice);
//        }
//        
//        Bool bGPBcheck=false;
//        if ( pcSlice->getSliceType() == B_SLICE)
//        {
//            if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
//            {
//                bGPBcheck=true;
//                Int i;
//                for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
//                {
//                    if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
//                    {
//                        bGPBcheck=false;
//                        break;
//                    }
//                }
//            }
//        }
//        if(bGPBcheck)
//        {
//            pcSlice->setMvdL1ZeroFlag(true);
//        }
//        else
//        {
//            pcSlice->setMvdL1ZeroFlag(false);
//        }
//        pcPic->getSlice(pcSlice->getSliceIdx())->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());
//        
//        Double lambda            = 0.0;
//        Int actualHeadBits       = 0;
//        Int actualTotalBits      = 0;
//        Int estimatedBits        = 0;
//        Int tmpBitsBeforeWriting = 0;
//        if ( m_pcCfg->getUseRateCtrl() )
//        {
//            Int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
//            if ( pcPic->getSlice(0)->getSliceType() == I_SLICE )
//            {
//                frameLevel = 0;
//            }
//            m_pcRateCtrl->initRCPic( frameLevel );
//            estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();
//            
//            Int sliceQP = m_pcCfg->getInitialQP();
//            if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
//            {
//                Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
//                Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)NumberBFrames );
//                Double dQPFactor     = 0.57*dLambda_scale;
//                Int    SHIFT_QP      = 12;
//                Int    bitdepth_luma_qp_scale = 0;
//                Double qp_temp = (Double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
//                lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
//            }
//            else if ( frameLevel == 0 )   // intra case, but use the model
//            {
//                m_pcSliceEncoder->calCostSliceI(pcPic);
//                
//                if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
//                {
//                    Int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
//                    bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );
//                    if ( bits < 200 )
//                    {
//                        bits = 200;
//                    }
//                    m_pcRateCtrl->getRCPic()->setTargetBits( bits );
//                }
//                
//                list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
//                m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
//                lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
//                sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
//            }
//            else    // normal case
//            {
//                list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
//                lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
//                sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
//            }
//            
//            sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
//            m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );
//            
//            m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
//        }
//        
//        UInt uiNumSlices = 1;
//        
//        UInt uiInternalAddress = pcPic->getNumPartInCU()-4;
//        UInt uiExternalAddress = pcPic->getPicSym()->getNumberOfCUsInFrame()-1;
//        UInt uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
//        UInt uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
//        UInt uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
//        UInt uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
//        while(uiPosX>=uiWidth||uiPosY>=uiHeight)
//        {
//            uiInternalAddress--;
//            uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
//            uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
//        }
//        uiInternalAddress++;
//        if(uiInternalAddress==pcPic->getNumPartInCU())
//        {
//            uiInternalAddress = 0;
//            uiExternalAddress++;
//        }
//        UInt uiRealEndAddress = uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress;
//        
//        Int  p, j;
//        UInt uiEncCUAddr;
//        
//        pcPic->getPicSym()->initTiles(pcSlice->getPPS());
//        
//        // Allocate some coders, now we know how many tiles there are.
//        const Int iNumSubstreams = pcSlice->getPPS()->getNumSubstreams();
//        
//        //generate the Coding Order Map and Inverse Coding Order Map
//        for(p=0, uiEncCUAddr=0; p<pcPic->getPicSym()->getNumberOfCUsInFrame(); p++, uiEncCUAddr = pcPic->getPicSym()->xCalculateNxtCUAddr(uiEncCUAddr))
//        {
//            pcPic->getPicSym()->setCUOrderMap(p, uiEncCUAddr);
//            pcPic->getPicSym()->setInverseCUOrderMap(uiEncCUAddr, p);
//        }
//        pcPic->getPicSym()->setCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
//        pcPic->getPicSym()->setInverseCUOrderMap(pcPic->getPicSym()->getNumberOfCUsInFrame(), pcPic->getPicSym()->getNumberOfCUsInFrame());
//        
//        // Allocate some coders, now we know how many tiles there are.
//        m_pcEncTop->createWPPCoders(iNumSubstreams);
//        pcSbacCoders = m_pcEncTop->getSbacCoders();
//        pcSubstreamsOut = new TComOutputBitstream[iNumSubstreams];
//        
//        UInt startCUAddrSliceIdx = 0; // used to index "m_uiStoredStartCUAddrForEncodingSlice" containing locations of slice boundaries
//        UInt startCUAddrSlice    = 0; // used to keep track of current slice's starting CU addr.
//        pcSlice->setSliceCurStartCUAddr( startCUAddrSlice ); // Setting "start CU addr" for current slice
//        m_storedStartCUAddrForEncodingSlice.clear();
//        
//        UInt startCUAddrSliceSegmentIdx = 0; // used to index "m_uiStoredStartCUAddrForEntropyEncodingSlice" containing locations of slice boundaries
//        UInt startCUAddrSliceSegment    = 0; // used to keep track of current Dependent slice's starting CU addr.
//        pcSlice->setSliceSegmentCurStartCUAddr( startCUAddrSliceSegment ); // Setting "start CU addr" for current Dependent slice
//        
//        m_storedStartCUAddrForEncodingSliceSegment.clear();
//        UInt nextCUAddr = 0;
//        m_storedStartCUAddrForEncodingSlice.push_back (nextCUAddr);
//        startCUAddrSliceIdx++;
//        m_storedStartCUAddrForEncodingSliceSegment.push_back(nextCUAddr);
//        startCUAddrSliceSegmentIdx++;
//        
//        while(nextCUAddr<uiRealEndAddress) // determine slice boundaries
//        {
//            pcSlice->setNextSlice       ( false );
//            pcSlice->setNextSliceSegment( false );
//            assert(pcPic->getNumAllocatedSlice() == startCUAddrSliceIdx);
//            m_pcSliceEncoder->precompressSlice( pcPic );
//            
//            // Hossam: Compress Slice
//            m_pcSliceEncoder->compressSlice   ( pcPic );
//            //            cout << "Ack Done with Compress Slice " << endl;
//            
//            Bool bNoBinBitConstraintViolated = (!pcSlice->isNextSlice() && !pcSlice->isNextSliceSegment());
//            if (pcSlice->isNextSlice() || (bNoBinBitConstraintViolated && m_pcCfg->getSliceMode()==FIXED_NUMBER_OF_LCU))
//            {
//                startCUAddrSlice = pcSlice->getSliceCurEndCUAddr();
//                // Reconstruction slice
//                m_storedStartCUAddrForEncodingSlice.push_back(startCUAddrSlice);
//                startCUAddrSliceIdx++;
//                // Dependent slice
//                if (startCUAddrSliceSegmentIdx>0 && m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx-1] != startCUAddrSlice)
//                {
//                    m_storedStartCUAddrForEncodingSliceSegment.push_back(startCUAddrSlice);
//                    startCUAddrSliceSegmentIdx++;
//                }
//                
//                if (startCUAddrSlice < uiRealEndAddress)
//                {
//                    pcPic->allocateNewSlice();
//                    pcPic->setCurrSliceIdx                  ( startCUAddrSliceIdx-1 );
//                    m_pcSliceEncoder->setSliceIdx           ( startCUAddrSliceIdx-1 );
//                    pcSlice = pcPic->getSlice               ( startCUAddrSliceIdx-1 );
//                    pcSlice->copySliceInfo                  ( pcPic->getSlice(0)      );
//                    pcSlice->setSliceIdx                    ( startCUAddrSliceIdx-1 );
//                    pcSlice->setSliceCurStartCUAddr         ( startCUAddrSlice      );
//                    pcSlice->setSliceSegmentCurStartCUAddr  ( startCUAddrSlice      );
//                    pcSlice->setSliceBits(0);
//                    uiNumSlices ++;
//                }
//            }
//            else if (pcSlice->isNextSliceSegment() || (bNoBinBitConstraintViolated && m_pcCfg->getSliceSegmentMode()==FIXED_NUMBER_OF_LCU))
//            {
//                startCUAddrSliceSegment                                                     = pcSlice->getSliceSegmentCurEndCUAddr();
//                m_storedStartCUAddrForEncodingSliceSegment.push_back(startCUAddrSliceSegment);
//                startCUAddrSliceSegmentIdx++;
//                pcSlice->setSliceSegmentCurStartCUAddr( startCUAddrSliceSegment );
//            }
//            else
//            {
//                startCUAddrSlice                                                            = pcSlice->getSliceCurEndCUAddr();
//                startCUAddrSliceSegment                                                     = pcSlice->getSliceSegmentCurEndCUAddr();
//            }
//            
//            nextCUAddr = (startCUAddrSlice > startCUAddrSliceSegment) ? startCUAddrSlice : startCUAddrSliceSegment;
//        }
//        m_storedStartCUAddrForEncodingSlice.push_back( pcSlice->getSliceCurEndCUAddr());
//        startCUAddrSliceIdx++;
//        m_storedStartCUAddrForEncodingSliceSegment.push_back(pcSlice->getSliceCurEndCUAddr());
//        startCUAddrSliceSegmentIdx++;
//        
//        pcSlice = pcPic->getSlice(0);
//        
//        // SAO parameter estimation using non-deblocked pixels for LCU bottom and right boundary areas
//        if( pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoLcuBoundary() )
//        {
//            m_pcSAO->getPreDBFStatistics(pcPic);
//        }
//        
//        //-- Loop filter
//        Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();
//        m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
//        if ( m_pcCfg->getDeblockingFilterMetric() )
//        {
//            dblMetric(pcPic, uiNumSlices);
//        }
//        m_pcLoopFilter->loopFilterPic( pcPic );
//        
//        /////////////////////////////////////////////////////////////////////////////////////////////////// File writing
//        // Set entropy coder
//        m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
//        
//        /* write various header sets. */
//        //    if ( m_bSeqFirst )
//        // Hossam SCENE CHANGE SPS
//        //      Bool isSceneChange = pcSlice->getPOC() == 5 || pcSlice->getPOC() == 9
//        //                        || pcSlice->getPOC() == 13 || pcSlice->getPOC() == 17;
//        
//        
//        // Hossam: SC should be IRAP  XXXXXX
//        //        Bool isSceneChange = pcSlice->getPOC() == 5 || pcSlice->getPOC() == 10
//        //        || pcSlice->getPOC() == 14 || pcSlice->getPOC() == 18;
//        //
//        //        Int lastSc = m_pcEncTop->getSceneChangeCoder()->getLastSC();
//        //
//        //        Bool isSceneChange = pcSlice->getPOC()  == lastSc + 0*m_iGopSize + 0||
//        //        pcSlice->getPOC() == lastSc + 1*m_iGopSize + 1 ||
//        //        pcSlice->getPOC() == lastSc + 2*m_iGopSize + 1 ||
//        //        pcSlice->getPOC() == lastSc + 3*m_iGopSize + 1;
//        //
//        
//        //        if ( m_bSeqFirst || isSceneChange )
//        if ( m_bSeqFirst )
//        {
//            //            cout << "Menna El Kashef: Change SPS for the SC and First frame!" << endl;
//            
//            OutputNALUnit nalu(NAL_UNIT_VPS);
//            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//            m_pcEntropyCoder->encodeVPS(m_pcEncTop->getVPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            accessUnit.push_back(new NALUnitEBSP(nalu));
//            actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
//            
//            nalu = NALUnit(NAL_UNIT_SPS);
//            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//            if (m_bSeqFirst)
//            {
//                pcSlice->getSPS()->setNumLongTermRefPicSPS(m_numLongTermRefPicSPS);
//                assert (m_numLongTermRefPicSPS <= MAX_NUM_LONG_TERM_REF_PICS);
//                for (Int k = 0; k < m_numLongTermRefPicSPS; k++)
//                {
//                    pcSlice->getSPS()->setLtRefPicPocLsbSps(k, m_ltRefPicPocLsbSps[k]);
//                    pcSlice->getSPS()->setUsedByCurrPicLtSPSFlag(k, m_ltRefPicUsedByCurrPicFlag[k]);
//                }
//            }
//            if( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() )
//            {
//                UInt maxCU = m_pcCfg->getSliceArgument() >> ( pcSlice->getSPS()->getMaxCUDepth() << 1);
//                UInt numDU = ( m_pcCfg->getSliceMode() == 1 ) ? ( pcPic->getNumCUsInFrame() / maxCU ) : ( 0 );
//                if( pcPic->getNumCUsInFrame() % maxCU != 0 || numDU == 0 )
//                {
//                    numDU ++;
//                }
//                pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->setNumDU( numDU );
//                pcSlice->getSPS()->setHrdParameters( m_pcCfg->getFrameRate(), numDU, m_pcCfg->getTargetBitrate(), ( m_pcCfg->getIntraPeriod() > 0 ) );
//            }
//            if( m_pcCfg->getBufferingPeriodSEIEnabled() || m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() )
//            {
//                pcSlice->getSPS()->getVuiParameters()->setHrdParametersPresentFlag( true );
//            }
//            m_pcEntropyCoder->encodeSPS(pcSlice->getSPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            accessUnit.push_back(new NALUnitEBSP(nalu));
//            actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
//            
//            nalu = NALUnit(NAL_UNIT_PPS);
//            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//            m_pcEntropyCoder->encodePPS(pcSlice->getPPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            accessUnit.push_back(new NALUnitEBSP(nalu));
//            actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
//            
//            xCreateLeadingSEIMessages(accessUnit, pcSlice->getSPS());
//            
//            
//            m_bSeqFirst = false;
//        }
//        
//        if (writeSOP) // write SOP description SEI (if enabled) at the beginning of GOP
//        {
//            Int SOPcurrPOC = pocCurr;
//            
//            OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
//            m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//            
//            SEISOPDescription SOPDescriptionSEI;
//            SOPDescriptionSEI.m_sopSeqParameterSetId = pcSlice->getSPS()->getSPSId();
//            
//            UInt i = 0;
//            UInt prevEntryId = iGOPid;
//            for (j = iGOPid; j < m_iGopSize; j++)
//            {
//                Int deltaPOC = m_pcCfg->getGOPEntry(j).m_POC - m_pcCfg->getGOPEntry(prevEntryId).m_POC;
//                if ((SOPcurrPOC + deltaPOC) < m_pcCfg->getFramesToBeEncoded())
//                {
//                    SOPcurrPOC += deltaPOC;
//                    SOPDescriptionSEI.m_sopDescVclNaluType[i] = getNalUnitType(SOPcurrPOC, m_iLastIDR, isField);
//                    SOPDescriptionSEI.m_sopDescTemporalId[i] = m_pcCfg->getGOPEntry(j).m_temporalId;
//                    SOPDescriptionSEI.m_sopDescStRpsIdx[i] = m_pcEncTop->getReferencePictureSetIdxForSOP(pcSlice, SOPcurrPOC, j);
//                    SOPDescriptionSEI.m_sopDescPocDelta[i] = deltaPOC;
//                    
//                    prevEntryId = j;
//                    i++;
//                }
//            }
//            
//            SOPDescriptionSEI.m_numPicsInSopMinus1 = i - 1;
//            
//            m_seiWriter.writeSEImessage( nalu.m_Bitstream, SOPDescriptionSEI, pcSlice->getSPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            accessUnit.push_back(new NALUnitEBSP(nalu));
//            
//            writeSOP = false;
//        }
//        
//        if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
//           ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
//           ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
//            || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
//        {
//            if( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() )
//            {
//                UInt numDU = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNumDU();
//                pictureTimingSEI.m_numDecodingUnitsMinus1     = ( numDU - 1 );
//                pictureTimingSEI.m_duCommonCpbRemovalDelayFlag = false;
//                
//                if( pictureTimingSEI.m_numNalusInDuMinus1 == NULL )
//                {
//                    pictureTimingSEI.m_numNalusInDuMinus1       = new UInt[ numDU ];
//                }
//                if( pictureTimingSEI.m_duCpbRemovalDelayMinus1  == NULL )
//                {
//                    pictureTimingSEI.m_duCpbRemovalDelayMinus1  = new UInt[ numDU ];
//                }
//                if( accumBitsDU == NULL )
//                {
//                    accumBitsDU                                  = new UInt[ numDU ];
//                }
//                if( accumNalsDU == NULL )
//                {
//                    accumNalsDU                                  = new UInt[ numDU ];
//                }
//            }
//            pictureTimingSEI.m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, m_totalCoded - m_lastBPSEI), static_cast<Int>(pow(2, static_cast<Double>(pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
//            pictureTimingSEI.m_picDpbOutputDelay = pcSlice->getSPS()->getNumReorderPics(pcSlice->getSPS()->getMaxTLayers()-1) + pcSlice->getPOC() - m_totalCoded;
//#if EFFICIENT_FIELD_IRAP
//            if(IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
//            {
//                // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
//                pictureTimingSEI.m_picDpbOutputDelay ++;
//            }
//#endif
//            Int factor = pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2;
//            pictureTimingSEI.m_picDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
//            if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
//            {
//                picSptDpbOutputDuDelay = factor * pictureTimingSEI.m_picDpbOutputDelay;
//            }
//        }
//        
//        if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) &&
//           ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
//           ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
//            || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
//        {
//            OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
//            m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//            
//            SEIBufferingPeriod sei_buffering_period;
//            
//            UInt uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
//            sei_buffering_period.m_initialCpbRemovalDelay      [0][0]     = uiInitialCpbRemovalDelay;
//            sei_buffering_period.m_initialCpbRemovalDelayOffset[0][0]     = uiInitialCpbRemovalDelay;
//            sei_buffering_period.m_initialCpbRemovalDelay      [0][1]     = uiInitialCpbRemovalDelay;
//            sei_buffering_period.m_initialCpbRemovalDelayOffset[0][1]     = uiInitialCpbRemovalDelay;
//            
//            Double dTmp = (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getNumUnitsInTick() / (Double)pcSlice->getSPS()->getVuiParameters()->getTimingInfo()->getTimeScale();
//            
//            UInt uiTmp = (UInt)( dTmp * 90000.0 );
//            uiInitialCpbRemovalDelay -= uiTmp;
//            uiInitialCpbRemovalDelay -= uiTmp / ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getTickDivisorMinus2() + 2 );
//            sei_buffering_period.m_initialAltCpbRemovalDelay      [0][0]  = uiInitialCpbRemovalDelay;
//            sei_buffering_period.m_initialAltCpbRemovalDelayOffset[0][0]  = uiInitialCpbRemovalDelay;
//            sei_buffering_period.m_initialAltCpbRemovalDelay      [0][1]  = uiInitialCpbRemovalDelay;
//            sei_buffering_period.m_initialAltCpbRemovalDelayOffset[0][1]  = uiInitialCpbRemovalDelay;
//            
//            sei_buffering_period.m_rapCpbParamsPresentFlag              = 0;
//            //for the concatenation, it can be set to one during splicing.
//            sei_buffering_period.m_concatenationFlag = 0;
//            //since the temporal layer HRD is not ready, we assumed it is fixed
//            sei_buffering_period.m_auCpbRemovalDelayDelta = 1;
//            
//            sei_buffering_period.m_cpbDelayOffset = 0;
//            sei_buffering_period.m_dpbDelayOffset = 0;
//            
//            m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_buffering_period, pcSlice->getSPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            
//            {
//                UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
//                UInt offsetPosition = m_activeParameterSetSEIPresentInAU;   // Insert BP SEI after APS SEI
//                AccessUnit::iterator it;
//                for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
//                {
//                    it++;
//                }
//                accessUnit.insert(it, new NALUnitEBSP(nalu));
//                m_bufferingPeriodSEIPresentInAU = true;
//            }
//            
//            if (m_pcCfg->getScalableNestingSEIEnabled())
//            {
//                OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI);
//                m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//                m_pcEntropyCoder->setBitstream(&naluTmp.m_Bitstream);
//                scalableNestingSEI.m_nestedSEIs.clear();
//                scalableNestingSEI.m_nestedSEIs.push_back(&sei_buffering_period);
//                m_seiWriter.writeSEImessage( naluTmp.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
//                writeRBSPTrailingBits(naluTmp.m_Bitstream);
//                UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
//                UInt offsetPosition = m_activeParameterSetSEIPresentInAU + m_bufferingPeriodSEIPresentInAU + m_pictureTimingSEIPresentInAU;   // Insert BP SEI after non-nested APS, BP and PT SEIs
//                AccessUnit::iterator it;
//                for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
//                {
//                    it++;
//                }
//                accessUnit.insert(it, new NALUnitEBSP(naluTmp));
//                m_nestedBufferingPeriodSEIPresentInAU = true;
//            }
//            
//            m_lastBPSEI = m_totalCoded;
//            m_cpbRemovalDelay = 0;
//        }
//        m_cpbRemovalDelay ++;
//        
//        if(pcSlice->getSPS()->getVuiParametersPresentFlag() && m_pcCfg->getChromaSamplingFilterHintEnabled() && ( pcSlice->getSliceType() == I_SLICE ))
//        {
//            SEIChromaSamplingFilterHint *seiChromaSamplingFilterHint = xCreateSEIChromaSamplingFilterHint(m_pcCfg->getChromaLocInfoPresentFlag(), m_pcCfg->getChromaSamplingHorFilterIdc(), m_pcCfg->getChromaSamplingVerFilterIdc());
//            
//            OutputNALUnit naluTmp(NAL_UNIT_PREFIX_SEI);
//            m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//            m_pcEntropyCoder->setBitstream(&naluTmp.m_Bitstream);
//            m_seiWriter.writeSEImessage(naluTmp.m_Bitstream, *seiChromaSamplingFilterHint, pcSlice->getSPS());
//            writeRBSPTrailingBits(naluTmp.m_Bitstream);
//            accessUnit.push_back(new NALUnitEBSP(naluTmp));
//            delete seiChromaSamplingFilterHint;
//        }
//        
//        if( ( m_pcEncTop->getRecoveryPointSEIEnabled() ) && ( pcSlice->getSliceType() == I_SLICE ) )
//        {
//            if( m_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !pcSlice->getRapPicFlag() )
//            {
//                // Gradual decoding refresh SEI
//                OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
//                m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//                m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//                
//                SEIGradualDecodingRefreshInfo seiGradualDecodingRefreshInfo;
//                seiGradualDecodingRefreshInfo.m_gdrForegroundFlag = true; // Indicating all "foreground"
//                
//                m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiGradualDecodingRefreshInfo, pcSlice->getSPS() );
//                writeRBSPTrailingBits(nalu.m_Bitstream);
//                accessUnit.push_back(new NALUnitEBSP(nalu));
//            }
//            // Recovery point SEI
//            OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
//            m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//            
//            SEIRecoveryPoint sei_recovery_point;
//            sei_recovery_point.m_recoveryPocCnt    = 0;
//            sei_recovery_point.m_exactMatchingFlag = ( pcSlice->getPOC() == 0 ) ? (true) : (false);
//            sei_recovery_point.m_brokenLinkFlag    = false;
//#if ALLOW_RECOVERY_POINT_AS_RAP
//            if(m_pcCfg->getDecodingRefreshType() == 3)
//            {
//                m_iLastRecoveryPicPOC = pocCurr;
//            }
//#endif
//            
//            m_seiWriter.writeSEImessage( nalu.m_Bitstream, sei_recovery_point, pcSlice->getSPS() );
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            accessUnit.push_back(new NALUnitEBSP(nalu));
//        }
//        
//        if( m_pcEncTop->getNoDisplaySEITLayer() )
//        {
//            if( pcSlice->getTLayer() >= m_pcEncTop->getNoDisplaySEITLayer() )
//            {
//                // No display SEI
//                OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
//                m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//                m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//                
//                SEINoDisplay seiNoDisplay;
//                seiNoDisplay.m_noDisplay = true;
//                
//                m_seiWriter.writeSEImessage( nalu.m_Bitstream, seiNoDisplay, pcSlice->getSPS() );
//                writeRBSPTrailingBits(nalu.m_Bitstream);
//                accessUnit.push_back(new NALUnitEBSP(nalu));
//            }
//        }
//        
//        /* use the main bitstream buffer for storing the marshalled picture */
//        m_pcEntropyCoder->setBitstream(NULL);
//        
//        startCUAddrSliceIdx = 0;
//        startCUAddrSlice    = 0;
//        
//        startCUAddrSliceSegmentIdx = 0;
//        startCUAddrSliceSegment    = 0;
//        nextCUAddr                 = 0;
//        pcSlice = pcPic->getSlice(startCUAddrSliceIdx);
//        
//        Int processingState = (pcSlice->getSPS()->getUseSAO())?(EXECUTE_INLOOPFILTER):(ENCODE_SLICE);
//        Bool skippedSlice=false;
//        while (nextCUAddr < uiRealEndAddress) // Iterate over all slices
//        {
//            switch(processingState) // NOTE: RExt - the indentation in this switch statement needs updating.
//            {
//                case ENCODE_SLICE:
//                {
//                    pcSlice->setNextSlice       ( false );
//                    pcSlice->setNextSliceSegment( false );
//                    if (nextCUAddr == m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx])
//                    {
//                        pcSlice = pcPic->getSlice(startCUAddrSliceIdx);
//                        if(startCUAddrSliceIdx > 0 && pcSlice->getSliceType()!= I_SLICE)
//                        {
//                            pcSlice->checkColRefIdx(startCUAddrSliceIdx, pcPic);
//                        }
//                        pcPic->setCurrSliceIdx(startCUAddrSliceIdx);
//                        m_pcSliceEncoder->setSliceIdx(startCUAddrSliceIdx);
//                        assert(startCUAddrSliceIdx == pcSlice->getSliceIdx());
//                        // Reconstruction slice
//                        pcSlice->setSliceCurStartCUAddr( nextCUAddr );  // to be used in encodeSlice() + context restriction
//                        pcSlice->setSliceCurEndCUAddr  ( m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx+1 ] );
//                        // Dependent slice
//                        pcSlice->setSliceSegmentCurStartCUAddr( nextCUAddr );  // to be used in encodeSlice() + context restriction
//                        pcSlice->setSliceSegmentCurEndCUAddr  ( m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx+1 ] );
//                        
//                        pcSlice->setNextSlice       ( true );
//                        
//                        startCUAddrSliceIdx++;
//                        startCUAddrSliceSegmentIdx++;
//                    }
//                    else if (nextCUAddr == m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx])
//                    {
//                        // Dependent slice
//                        pcSlice->setSliceSegmentCurStartCUAddr( nextCUAddr );  // to be used in encodeSlice() + context restriction
//                        pcSlice->setSliceSegmentCurEndCUAddr  ( m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx+1 ] );
//                        
//                        pcSlice->setNextSliceSegment( true );
//                        
//                        startCUAddrSliceSegmentIdx++;
//                    }
//                    
//                    pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
//                    pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());
//                    UInt uiDummyStartCUAddr;
//                    UInt uiDummyBoundingCUAddr;
//                    m_pcSliceEncoder->xDetermineStartAndBoundingCUAddr(uiDummyStartCUAddr,uiDummyBoundingCUAddr,pcPic,true);
//                    
//                    uiInternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) % pcPic->getNumPartInCU();
//                    uiExternalAddress = pcPic->getPicSym()->getPicSCUAddr(pcSlice->getSliceSegmentCurEndCUAddr()-1) / pcPic->getNumPartInCU();
//                    uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
//                    uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
//                    uiWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
//                    uiHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
//                    while(uiPosX>=uiWidth||uiPosY>=uiHeight)
//                    {
//                        uiInternalAddress--;
//                        uiPosX = ( uiExternalAddress % pcPic->getFrameWidthInCU() ) * g_uiMaxCUWidth+ g_auiRasterToPelX[ g_auiZscanToRaster[uiInternalAddress] ];
//                        uiPosY = ( uiExternalAddress / pcPic->getFrameWidthInCU() ) * g_uiMaxCUHeight+ g_auiRasterToPelY[ g_auiZscanToRaster[uiInternalAddress] ];
//                    }
//                    uiInternalAddress++;
//                    if(uiInternalAddress==pcPic->getNumPartInCU())
//                    {
//                        uiInternalAddress = 0;
//                        uiExternalAddress = pcPic->getPicSym()->getCUOrderMap(pcPic->getPicSym()->getInverseCUOrderMap(uiExternalAddress)+1);
//                    }
//                    UInt endAddress = pcPic->getPicSym()->getPicSCUEncOrder(uiExternalAddress*pcPic->getNumPartInCU()+uiInternalAddress);
//                    if(endAddress<=pcSlice->getSliceSegmentCurStartCUAddr())
//                    {
//                        UInt boundingAddrSlice, boundingAddrSliceSegment;
//                        boundingAddrSlice          = m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx];
//                        boundingAddrSliceSegment = m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx];
//                        nextCUAddr               = min(boundingAddrSlice, boundingAddrSliceSegment);
//                        if(pcSlice->isNextSlice())
//                        {
//                            skippedSlice=true;
//                        }
//                        continue;
//                    }
//                    if(skippedSlice)
//                    {
//                        pcSlice->setNextSlice       ( true );
//                        pcSlice->setNextSliceSegment( false );
//                    }
//                    skippedSlice=false;
//                    pcSlice->allocSubstreamSizes( iNumSubstreams );
//                    for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
//                    {
//                        pcSubstreamsOut[ui].clear();
//                    }
//                    
//                    m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
//                    m_pcEntropyCoder->resetEntropy      ();
//                    /* start slice NALunit */
//                    OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
//                    Bool sliceSegment = (!pcSlice->isNextSlice());
//                    if (!sliceSegment)
//                    {
//                        uiOneBitstreamPerSliceLength = 0; // start of a new slice
//                    }
//                    m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//                    
//#if SETTING_NO_OUT_PIC_PRIOR
//                    pcSlice->setNoRaslOutputFlag(false);
//                    if (pcSlice->isIRAP())
//                    {
//                        if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
//                        {
//                            pcSlice->setNoRaslOutputFlag(true);
//                        }
//                        //the inference for NoOutputPriorPicsFlag
//                        // KJS: This cannot happen at the encoder
//                        if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
//                        {
//                            if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
//                            {
//                                pcSlice->setNoOutputPriorPicsFlag(true);
//                            }
//                        }
//                    }
//#endif
//                    
//                    tmpBitsBeforeWriting = m_pcEntropyCoder->getNumberOfWrittenBits();
//                    m_pcEntropyCoder->encodeSliceHeader(pcSlice);
//                    actualHeadBits += ( m_pcEntropyCoder->getNumberOfWrittenBits() - tmpBitsBeforeWriting );
//                    
//                    // is it needed?
//                    {
//                        if (!sliceSegment)
//                        {
//                            pcBitstreamRedirect->writeAlignOne();
//                        }
//                        else
//                        {
//                            // We've not completed our slice header info yet, do the alignment later.
//                        }
//                        m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
//                        m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
//                        m_pcEntropyCoder->resetEntropy    ();
//                        for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ )
//                        {
//                            m_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
//                            m_pcEntropyCoder->resetEntropy    ();
//                        }
//                    }
//                    
//                    if(pcSlice->isNextSlice())
//                    {
//                        // set entropy coder for writing
//                        m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
//                        {
//                            for ( UInt ui = 0 ; ui < iNumSubstreams ; ui++ )
//                            {
//                                m_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[ui], pcSlice );
//                                m_pcEntropyCoder->resetEntropy    ();
//                            }
//                            pcSbacCoders[0].load(m_pcSbacCoder);
//                            m_pcEntropyCoder->setEntropyCoder ( &pcSbacCoders[0], pcSlice );  //ALF is written in substream #0 with CABAC coder #0 (see ALF param encoding below)
//                        }
//                        m_pcEntropyCoder->resetEntropy    ();
//                        // File writing
//                        if (!sliceSegment)
//                        {
//                            m_pcEntropyCoder->setBitstream(pcBitstreamRedirect);
//                        }
//                        else
//                        {
//                            m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//                        }
//                        // for now, override the TILES_DECODER setting in order to write substreams.
//                        m_pcEntropyCoder->setBitstream    ( &pcSubstreamsOut[0] );
//                        
//                    }
//                    pcSlice->setFinalized(true);
//                    
//                    m_pcSbacCoder->load( &pcSbacCoders[0] );
//                    
//                    pcSlice->setTileOffstForMultES( uiOneBitstreamPerSliceLength );
//                    
//#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
//                    g_bFinalEncode = true;
//#endif
//                    
//                    pcSlice->setTileLocationCount ( 0 );
//                    m_pcSliceEncoder->encodeSlice(pcPic, pcSubstreamsOut);
//                    
//#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
//                    g_bFinalEncode = false;
//#endif
//                    
//                    {
//                        // Construct the final bitstream by flushing and concatenating substreams.
//                        // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
//                        UInt* puiSubstreamSizes = pcSlice->getSubstreamSizes();
//                        UInt uiTotalCodedSize = 0; // for padding calcs.
//                        UInt uiNumSubstreamsPerTile = iNumSubstreams; // Only used if wavefronts not enabled.
//                        if (iNumSubstreams > 1)
//                        {
//                            uiNumSubstreamsPerTile /= pcPic->getPicSym()->getNumTiles(); // Only used if wavefronts not enabled.
//                        }
//                        for ( UInt ui = 0 ; ui < iNumSubstreams; ui++ )
//                        {
//                            // Flush all substreams -- this includes empty ones.
//                            // Terminating bit and flush.
//                            m_pcEntropyCoder->setEntropyCoder   ( &pcSbacCoders[ui], pcSlice );
//                            m_pcEntropyCoder->setBitstream      (  &pcSubstreamsOut[ui] );
//                            m_pcEntropyCoder->encodeTerminatingBit( 1 );
//                            m_pcEntropyCoder->encodeSliceFinish();
//                            
//                            pcSubstreamsOut[ui].writeByteAlignment();   // Byte-alignment in slice_data() at end of sub-stream
//                            // Byte alignment is necessary between tiles when tiles are independent.
//                            uiTotalCodedSize += pcSubstreamsOut[ui].getNumberOfWrittenBits();
//                            
//                            Bool bNextSubstreamInNewTile = ((ui+1) < iNumSubstreams)&& ((ui+1)%uiNumSubstreamsPerTile == 0);
//                            if (bNextSubstreamInNewTile &&  !pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() )
//                            {
//                                pcSlice->setTileLocation(ui/uiNumSubstreamsPerTile, pcSlice->getTileOffstForMultES()+(uiTotalCodedSize>>3));
//                            }
//                            if (ui+1 < iNumSubstreams)
//                            {
//                                puiSubstreamSizes[ui] = pcSubstreamsOut[ui].getNumberOfWrittenBits() + (pcSubstreamsOut[ui].countStartCodeEmulations()<<3);
//                            }
//                        }
//                        
//                        // Complete the slice header info.
//                        m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder, pcSlice );
//                        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
//                        m_pcEntropyCoder->encodeTilesWPPEntryPoint( pcSlice );
//                        
//                        // Substreams...
//                        TComOutputBitstream *pcOut = pcBitstreamRedirect;
//                        Int numZeroSubstreamsAtStartOfSlice = 0;
//                        Int numSubstreamsToCode = pcSlice->getPPS()->getNumSubstreams();
//                        if (pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag())
//                        {
//                            Int  maxNumParts                      = pcPic->getNumPartInCU();
//                            numZeroSubstreamsAtStartOfSlice  = pcPic->getSubstreamForLCUAddr(pcSlice->getSliceSegmentCurStartCUAddr()/maxNumParts, false, pcSlice);
//                            // 1st line present for WPP.
//                            numSubstreamsToCode  = pcSlice->getNumEntryPointOffsets()+1;
//                        }
//                        for ( UInt ui = 0 ; ui < numSubstreamsToCode; ui++ )
//                        {
//                            pcOut->addSubstream(&pcSubstreamsOut[ui+numZeroSubstreamsAtStartOfSlice]);
//                        }
//                    }
//                    
//                    UInt boundingAddrSlice, boundingAddrSliceSegment;
//                    boundingAddrSlice        = m_storedStartCUAddrForEncodingSlice[startCUAddrSliceIdx];
//                    boundingAddrSliceSegment = m_storedStartCUAddrForEncodingSliceSegment[startCUAddrSliceSegmentIdx];
//                    nextCUAddr               = min(boundingAddrSlice, boundingAddrSliceSegment);
//                    // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
//                    // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
//                    Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
//                    xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
//                    accessUnit.push_back(new NALUnitEBSP(nalu));
//                    actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;
//                    bNALUAlignedWrittenToList = true;
//                    uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits(); // length of bitstream after byte-alignment
//                    
//                    if (!bNALUAlignedWrittenToList)
//                    {
//                        {
//                            nalu.m_Bitstream.writeAlignZero();
//                        }
//                        accessUnit.push_back(new NALUnitEBSP(nalu));
//                        uiOneBitstreamPerSliceLength += nalu.m_Bitstream.getNumberOfWrittenBits() + 24; // length of bitstream after byte-alignment + 3 byte startcode 0x000001
//                    }
//                    
//                    if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
//                       ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
//                       ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
//                        || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) &&
//                       ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() ) )
//                    {
//                        UInt numNalus = 0;
//                        UInt numRBSPBytes = 0;
//                        for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
//                        {
//                            UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());
//                            if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
//                            {
//                                numRBSPBytes += numRBSPBytes_nal;
//                                numNalus ++;
//                            }
//                        }
//                        accumBitsDU[ pcSlice->getSliceIdx() ] = ( numRBSPBytes << 3 );
//                        accumNalsDU[ pcSlice->getSliceIdx() ] = numNalus;   // SEI not counted for bit count; hence shouldn't be counted for # of NALUs - only for consistency
//                    }
//                    processingState = ENCODE_SLICE;
//                }
//                    break;
//                    
//                case EXECUTE_INLOOPFILTER:
//                {
//                    // set entropy coder for RD
//                    m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder, pcSlice );
//                    if ( pcSlice->getSPS()->getUseSAO() )
//                    {
//                        m_pcEntropyCoder->resetEntropy();
//                        m_pcEntropyCoder->setBitstream( m_pcBitCounter );
//                        Bool sliceEnabled[MAX_NUM_COMPONENT];
//                        m_pcSAO->initRDOCabacCoder(m_pcEncTop->getRDGoOnSbacCoder(), pcSlice);
//                        m_pcSAO->SAOProcess(pcPic
//                                            , sliceEnabled
//                                            , pcPic->getSlice(0)->getLambdas()
//#if SAO_ENCODE_ALLOW_USE_PREDEBLOCK
//                                            , m_pcCfg->getSaoLcuBoundary()
//#endif
//                                            );
//                        m_pcSAO->PCMLFDisableProcess(pcPic);
//                        
//                        //assign SAO slice header
//                        for(Int s=0; s< uiNumSlices; s++)
//                        {
//                            pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
//                            assert(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]);
//                            pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
//                        }
//                    }
//                    
//                    processingState = ENCODE_SLICE;
//                }
//                    break;
//                    
//                default:
//                {
//                    printf("Not a supported encoding state\n");
//                    assert(0);
//                    exit(-1);
//                }
//            }
//        } // end iteration over slices
//        
//        pcPic->compressMotion();
//        
//        //-- For time output for each slice
//        Double dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;
//        
//        std::string digestStr;
//        if (m_pcCfg->getDecodedPictureHashSEIEnabled())
//        {
//            /* calculate MD5sum for entire reconstructed picture */
//            SEIDecodedPictureHash sei_recon_picture_digest;
//            if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
//            {
//                sei_recon_picture_digest.method = SEIDecodedPictureHash::MD5;
//                UInt numChar=calcMD5(*pcPic->getPicYuvRec(), sei_recon_picture_digest.m_digest);
//                digestStr = digestToString(sei_recon_picture_digest.m_digest, numChar);
//            }
//            else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
//            {
//                sei_recon_picture_digest.method = SEIDecodedPictureHash::CRC;
//                UInt numChar=calcCRC(*pcPic->getPicYuvRec(), sei_recon_picture_digest.m_digest);
//                digestStr = digestToString(sei_recon_picture_digest.m_digest, numChar);
//            }
//            else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
//            {
//                sei_recon_picture_digest.method = SEIDecodedPictureHash::CHECKSUM;
//                UInt numChar=calcChecksum(*pcPic->getPicYuvRec(), sei_recon_picture_digest.m_digest);
//                digestStr = digestToString(sei_recon_picture_digest.m_digest, numChar);
//            }
//            OutputNALUnit nalu(NAL_UNIT_SUFFIX_SEI, pcSlice->getTLayer());
//            
//            /* write the SEI messages */
//            m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//            m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_recon_picture_digest, pcSlice->getSPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            
//            accessUnit.insert(accessUnit.end(), new NALUnitEBSP(nalu));
//        }
//        if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
//        {
//            SEITemporalLevel0Index sei_temporal_level0_index;
//            if (pcSlice->getRapPicFlag())
//            {
//                m_tl0Idx = 0;
//                m_rapIdx = (m_rapIdx + 1) & 0xFF;
//            }
//            else
//            {
//                m_tl0Idx = (m_tl0Idx + (pcSlice->getTLayer() ? 0 : 1)) & 0xFF;
//            }
//            sei_temporal_level0_index.tl0Idx = m_tl0Idx;
//            sei_temporal_level0_index.rapIdx = m_rapIdx;
//            
//            OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);
//            
//            /* write the SEI messages */
//            m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//            m_seiWriter.writeSEImessage(nalu.m_Bitstream, sei_temporal_level0_index, pcSlice->getSPS());
//            writeRBSPTrailingBits(nalu.m_Bitstream);
//            
//            /* insert the SEI message NALUnit before any Slice NALUnits */
//            AccessUnit::iterator it = find_if(accessUnit.begin(), accessUnit.end(), mem_fun(&NALUnit::isSlice));
//            accessUnit.insert(it, new NALUnitEBSP(nalu));
//        }
//        
//        xCalculateAddPSNR( pcPic, pcPic->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );
//        //           xCalculateAddPSNR( pcPic, pcPic->getPicYuvOrg(), accessUnit, dEncTime, snr_conversion, printFrameMSE );
//        
//        //In case of field coding, compute the interlaced PSNR for both fields
//        if (isField && ((!pcPic->isTopField() && isTff) || (pcPic->isTopField() && !isTff)) && (pcPic->getPOC()%m_iGopSize != 1))
//        {
//            //get complementary top field
//            
//            TComList<TComPic*>::iterator   iterPic = rcListPic.begin();
//            while ((*iterPic)->getPOC() != pcPic->getPOC()-1)
//            {
//                iterPic ++;
//            }
//            TComPic* pcPicFirstField = *(iterPic);
//            xCalculateInterlacedAddPSNR(pcPicFirstField, pcPic, pcPicFirstField->getPicYuvRec(), pcPic->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );
//        }
//        else if (isField && pcPic->getPOC()!= 0 && (pcPic->getPOC()%m_iGopSize == 0))
//        {
//            //get complementary bottom field
//            
//            TComList<TComPic*>::iterator   iterPic = rcListPic.begin();
//            while ((*iterPic)->getPOC() != pcPic->getPOC()+1)
//            {
//                iterPic ++;
//            }
//            TComPic* pcPicFirstField = *(iterPic);
//            xCalculateInterlacedAddPSNR(pcPic, pcPicFirstField, pcPic->getPicYuvRec(), pcPicFirstField->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );
//        }
//        
//        if (!digestStr.empty())
//        {
//            if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)
//            {
//                printf(" [MD5:%s]", digestStr.c_str());
//            }
//            else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)
//            {
//                printf(" [CRC:%s]", digestStr.c_str());
//            }
//            else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)
//            {
//                printf(" [Checksum:%s]", digestStr.c_str());
//            }
//        }
//        
//        if ( m_pcCfg->getUseRateCtrl() )
//        {
//            Double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
//            Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
//            if ( avgLambda < 0.0 )
//            {
//                avgLambda = lambda;
//            }
//            
//            m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
//            m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );
//            
//            m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
//            if ( pcSlice->getSliceType() != I_SLICE )
//            {
//                m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
//            }
//            else    // for intra picture, the estimated bits are used to update the current status in the GOP
//            {
//                m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
//            }
//        }
//        
//        if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
//           ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
//           ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
//            || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
//        {
//            TComVUI *vui = pcSlice->getSPS()->getVuiParameters();
//            TComHRD *hrd = vui->getHrdParameters();
//            
//            if( hrd->getSubPicCpbParamsPresentFlag() )
//            {
//                Int i;
//                UInt64 ui64Tmp;
//                UInt uiPrev = 0;
//                UInt numDU = ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 );
//                UInt *pCRD = &pictureTimingSEI.m_duCpbRemovalDelayMinus1[0];
//                UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;
//                
//                for( i = 0; i < numDU; i ++ )
//                {
//                    pictureTimingSEI.m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( accumNalsDU[ i ] - 1 ) : ( accumNalsDU[ i ] - accumNalsDU[ i - 1] - 1 );
//                }
//                
//                if( numDU == 1 )
//                {
//                    pCRD[ 0 ] = 0; /* don't care */
//                }
//                else
//                {
//                    pCRD[ numDU - 1 ] = 0;/* by definition */
//                    UInt tmp = 0;
//                    UInt accum = 0;
//                    
//                    for( i = ( numDU - 2 ); i >= 0; i -- )
//                    {
//                        ui64Tmp = ( ( ( accumBitsDU[ numDU - 1 ]  - accumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
//                        if( (UInt)ui64Tmp > maxDiff )
//                        {
//                            tmp ++;
//                        }
//                    }
//                    uiPrev = 0;
//                    
//                    UInt flag = 0;
//                    for( i = ( numDU - 2 ); i >= 0; i -- )
//                    {
//                        flag = 0;
//                        ui64Tmp = ( ( ( accumBitsDU[ numDU - 1 ]  - accumBitsDU[ i ] ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
//                        
//                        if( (UInt)ui64Tmp > maxDiff )
//                        {
//                            if(uiPrev >= maxDiff - tmp)
//                            {
//                                ui64Tmp = uiPrev + 1;
//                                flag = 1;
//                            }
//                            else                            ui64Tmp = maxDiff - tmp + 1;
//                        }
//                        pCRD[ i ] = (UInt)ui64Tmp - uiPrev - 1;
//                        if( (Int)pCRD[ i ] < 0 )
//                        {
//                            pCRD[ i ] = 0;
//                        }
//                        else if (tmp > 0 && flag == 1)
//                        {
//                            tmp --;
//                        }
//                        accum += pCRD[ i ] + 1;
//                        uiPrev = accum;
//                    }
//                }
//            }
//            
//            if( m_pcCfg->getPictureTimingSEIEnabled() )
//            {
//                {
//                    OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
//                    m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//                    pictureTimingSEI.m_picStruct = (isField && pcSlice->getPic()->isTopField())? 1 : isField? 2 : 0;
//                    m_seiWriter.writeSEImessage(nalu.m_Bitstream, pictureTimingSEI, pcSlice->getSPS());
//                    writeRBSPTrailingBits(nalu.m_Bitstream);
//                    UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
//                    UInt offsetPosition = m_activeParameterSetSEIPresentInAU
//                    + m_bufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
//                    AccessUnit::iterator it;
//                    for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
//                    {
//                        it++;
//                    }
//                    accessUnit.insert(it, new NALUnitEBSP(nalu));
//                    m_pictureTimingSEIPresentInAU = true;
//                }
//                
//                if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
//                {
//                    OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
//                    m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//                    scalableNestingSEI.m_nestedSEIs.clear();
//                    scalableNestingSEI.m_nestedSEIs.push_back(&pictureTimingSEI);
//                    m_seiWriter.writeSEImessage(nalu.m_Bitstream, scalableNestingSEI, pcSlice->getSPS());
//                    writeRBSPTrailingBits(nalu.m_Bitstream);
//                    UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
//                    UInt offsetPosition = m_activeParameterSetSEIPresentInAU
//                    + m_bufferingPeriodSEIPresentInAU + m_pictureTimingSEIPresentInAU + m_nestedBufferingPeriodSEIPresentInAU;    // Insert PT SEI after APS and BP SEI
//                    AccessUnit::iterator it;
//                    for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
//                    {
//                        it++;
//                    }
//                    accessUnit.insert(it, new NALUnitEBSP(nalu));
//                    m_nestedPictureTimingSEIPresentInAU = true;
//                }
//            }
//            
//            if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
//            {
//                m_pcEntropyCoder->setEntropyCoder(m_pcCavlcCoder, pcSlice);
//                for( Int i = 0; i < ( pictureTimingSEI.m_numDecodingUnitsMinus1 + 1 ); i ++ )
//                {
//                    OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI, pcSlice->getTLayer());
//                    
//                    SEIDecodingUnitInfo tempSEI;
//                    tempSEI.m_decodingUnitIdx = i;
//                    tempSEI.m_duSptCpbRemovalDelay = pictureTimingSEI.m_duCpbRemovalDelayMinus1[i] + 1;
//                    tempSEI.m_dpbOutputDuDelayPresentFlag = false;
//                    tempSEI.m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;
//                    
//                    AccessUnit::iterator it;
//                    // Insert the first one in the right location, before the first slice
//                    if(i == 0)
//                    {
//                        // Insert before the first slice.
//                        m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
//                        writeRBSPTrailingBits(nalu.m_Bitstream);
//                        
//                        UInt seiPositionInAu = xGetFirstSeiLocation(accessUnit);
//                        UInt offsetPosition = m_activeParameterSetSEIPresentInAU
//                        + m_bufferingPeriodSEIPresentInAU
//                        + m_pictureTimingSEIPresentInAU;  // Insert DU info SEI after APS, BP and PT SEI
//                        for(j = 0, it = accessUnit.begin(); j < seiPositionInAu + offsetPosition; j++)
//                        {
//                            it++;
//                        }
//                        accessUnit.insert(it, new NALUnitEBSP(nalu));
//                    }
//                    else
//                    {
//                        Int ctr;
//                        // For the second decoding unit onwards we know how many NALUs are present
//                        for (ctr = 0, it = accessUnit.begin(); it != accessUnit.end(); it++)
//                        {
//                            if(ctr == accumNalsDU[ i - 1 ])
//                            {
//                                // Insert before the first slice.
//                                m_seiWriter.writeSEImessage(nalu.m_Bitstream, tempSEI, pcSlice->getSPS());
//                                writeRBSPTrailingBits(nalu.m_Bitstream);
//                                
//                                accessUnit.insert(it, new NALUnitEBSP(nalu));
//                                break;
//                            }
//                            if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
//                            {
//                                ctr++;
//                            }
//                        }
//                    }
//                }
//            }
//        }
//        
//        xResetNonNestedSEIPresentFlags();
//        xResetNestedSEIPresentFlags();
//        
//        // Hossam: Scene Change: Copy the rec of the pcPic to the out of pcPic
//        pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);
//        
//        pcPic->setReconMark   ( true );
//        m_bFirst = false;
//        m_iNumPicCoded++;
//        m_totalCoded ++;
//        /* logging: insert a newline at end of picture period */
//        printf("\n");
//        fflush(stdout);
//        
//        delete[] pcSubstreamsOut;
//        
//#if EFFICIENT_FIELD_IRAP
//        if(IRAPtoReorder)
//        {
//            if(swapIRAPForward)
//            {
//                if(iGOPid == IRAPGOPid)
//                {
//                    iGOPid = IRAPGOPid +1;
//                    IRAPtoReorder = false;
//                }
//                else if(iGOPid == IRAPGOPid +1)
//                {
//                    iGOPid --;
//                }
//            }
//            else
//            {
//                if(iGOPid == IRAPGOPid)
//                {
//                    iGOPid = IRAPGOPid -1;
//                }
//                else if(iGOPid == IRAPGOPid -1)
//                {
//                    iGOPid = IRAPGOPid;
//                    IRAPtoReorder = false;
//                }
//            }
//        }
//#endif
//    }
//    
//    delete pcBitstreamRedirect;
//    
//    if( accumBitsDU != NULL) delete accumBitsDU;
//    if( accumNalsDU != NULL) delete accumNalsDU;
//    
//    assert ( (m_iNumPicCoded == iNumPicRcvd) );
//}
