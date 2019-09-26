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

/** \file     TAppEncTop.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>

#include "TAppEncTop.h"
#include "TLibEncoder/AnnexBwrite.h"

using namespace std;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

TAppEncTop::TAppEncTop()
{
  m_iFrameRcvd = 0;
  m_totalBytes = 0;
  m_essentialBytes = 0;
}

TAppEncTop::~TAppEncTop()
{
}

Void TAppEncTop::xInitLibCfg()
{
  TComVPS vps;

  vps.setMaxTLayers                                               ( m_maxTempLayer );
  if (m_maxTempLayer == 1)
  {
    vps.setTemporalNestingFlag(true);
  }
  vps.setMaxLayers                                                ( 1 );
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    vps.setNumReorderPics                                         ( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering                                     ( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS(&vps);

  m_cTEncTop.setProfile                                           ( m_profile);
  m_cTEncTop.setLevel                                             ( m_levelTier, m_level);
  m_cTEncTop.setProgressiveSourceFlag                             ( m_progressiveSourceFlag);
  m_cTEncTop.setInterlacedSourceFlag                              ( m_interlacedSourceFlag);
  m_cTEncTop.setNonPackedConstraintFlag                           ( m_nonPackedConstraintFlag);
  m_cTEncTop.setFrameOnlyConstraintFlag                           ( m_frameOnlyConstraintFlag);
  m_cTEncTop.setBitDepthConstraintValue                           ( m_bitDepthConstraint );
  m_cTEncTop.setChromaFormatConstraintValue                       ( m_chromaFormatConstraint );
  m_cTEncTop.setIntraConstraintFlag                               ( m_intraConstraintFlag );
  m_cTEncTop.setLowerBitRateConstraintFlag                        ( m_lowerBitRateConstraintFlag );

  m_cTEncTop.setPrintMSEBasedSequencePSNR                         ( m_printMSEBasedSequencePSNR);
  m_cTEncTop.setPrintFrameMSE                                     ( m_printFrameMSE);
  m_cTEncTop.setPrintSequenceMSE                                  ( m_printSequenceMSE);

  m_cTEncTop.setFrameRate                                         ( m_iFrameRate );
  m_cTEncTop.setFrameSkip                                         ( m_FrameSkip );
  m_cTEncTop.setSourceWidth                                       ( m_iSourceWidth );
  m_cTEncTop.setSourceHeight                                      ( m_iSourceHeight );
  m_cTEncTop.setConformanceWindow                                 ( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
  m_cTEncTop.setFramesToBeEncoded                                 ( m_framesToBeEncoded );

  // Hossam: Coding Structure settings from the configuration file
  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod                                       ( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType                               ( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize                                           ( m_iGOPSize );
  m_cTEncTop.setGopList                                           ( m_GOPList );
  m_cTEncTop.setExtraRPSs                                         ( m_extraRPSs );

    
  for(Int i = 0; i < MAX_TLAYER; i++)
  {
    m_cTEncTop.setNumReorderPics                                  ( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering                              ( m_maxDecPicBuffering[i], i );
  }
  for( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop )
  {
    m_cTEncTop.setLambdaModifier                                  ( uiLoop, m_adLambdaModifier[ uiLoop ] );
  }
  m_cTEncTop.setQP                                                ( m_iQP );

  m_cTEncTop.setPad                                               ( m_aiPad );

  m_cTEncTop.setMaxTempLayer                                      ( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );

  //===== Slice ========

  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable                                 ( m_bLoopFilterDisable       );
  m_cTEncTop.setLoopFilterOffsetInPPS                             ( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset                              ( m_loopFilterBetaOffsetDiv2  );
  m_cTEncTop.setLoopFilterTcOffset                                ( m_loopFilterTcOffsetDiv2    );
  m_cTEncTop.setDeblockingFilterControlPresent                    ( m_DeblockingFilterControlPresent);
  m_cTEncTop.setDeblockingFilterMetric                            ( m_DeblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setFastSearch                                        ( m_iFastSearch  );
  m_cTEncTop.setSearchRange                                       ( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange                                 ( m_bipredSearchRange );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP                                        ( m_iMaxDeltaQP  );
  m_cTEncTop.setMaxCuDQPDepth                                     ( m_iMaxCuDQPDepth  );
  m_cTEncTop.setMaxCUChromaQpAdjustmentDepth                      ( m_maxCUChromaQpAdjustmentDepth );
  m_cTEncTop.setChromaCbQpOffset                                  ( m_cbQpOffset     );
  m_cTEncTop.setChromaCrQpOffset                                  ( m_crQpOffset  );

  m_cTEncTop.setChromaFormatIdc                                   ( m_chromaFormatIDC  );

#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect                                  ( m_bUseAdaptQpSelect   );
#endif

  m_cTEncTop.setUseAdaptiveQP                                     ( m_bUseAdaptiveQP  );
  m_cTEncTop.setQPAdaptationRange                                 ( m_iQPAdaptationRange );
  m_cTEncTop.setUseExtendedPrecision                              ( m_useExtendedPrecision );
  m_cTEncTop.setUseHighPrecisionPredictionWeighting               ( m_useHighPrecisionPredictionWeighting );
  //====== Tool list ========
  m_cTEncTop.setDeltaQpRD                                         ( m_uiDeltaQpRD  );
  m_cTEncTop.setUseASR                                            ( m_bUseASR      );
  m_cTEncTop.setUseHADME                                          ( m_bUseHADME    );
  m_cTEncTop.setdQPs                                              ( m_aidQP        );
  m_cTEncTop.setUseRDOQ                                           ( m_useRDOQ     );
  m_cTEncTop.setUseRDOQTS                                         ( m_useRDOQTS   );
  m_cTEncTop.setRDpenalty                                         ( m_rdPenalty );
  m_cTEncTop.setQuadtreeTULog2MaxSize                             ( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize                             ( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter                           ( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra                           ( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setUseFastEnc                                        ( m_bUseFastEnc  );
  m_cTEncTop.setUseEarlyCU                                        ( m_bUseEarlyCU  );
  m_cTEncTop.setUseFastDecisionForMerge                           ( m_useFastDecisionForMerge  );
  m_cTEncTop.setUseCbfFastMode                                    ( m_bUseCbfFastMode  );
  m_cTEncTop.setUseEarlySkipDetection                             ( m_useEarlySkipDetection );
  m_cTEncTop.setUseCrossComponentPrediction                       ( m_useCrossComponentPrediction );
  m_cTEncTop.setUseReconBasedCrossCPredictionEstimate             ( m_reconBasedCrossCPredictionEstimate );
  m_cTEncTop.setSaoOffsetBitShift                                 ( CHANNEL_TYPE_LUMA  , m_saoOffsetBitShift[CHANNEL_TYPE_LUMA]   );
  m_cTEncTop.setSaoOffsetBitShift                                 ( CHANNEL_TYPE_CHROMA, m_saoOffsetBitShift[CHANNEL_TYPE_CHROMA] );
  m_cTEncTop.setUseTransformSkip                                  ( m_useTransformSkip      );
  m_cTEncTop.setUseTransformSkipFast                              ( m_useTransformSkipFast  );
  m_cTEncTop.setUseResidualRotation                               ( m_useResidualRotation   );
  m_cTEncTop.setUseSingleSignificanceMapContext                   ( m_useSingleSignificanceMapContext   );
  m_cTEncTop.setUseGolombRiceParameterAdaptation                  ( m_useGolombRiceParameterAdaptation );
  m_cTEncTop.setAlignCABACBeforeBypass                            ( m_alignCABACBeforeBypass );
  m_cTEncTop.setTransformSkipLog2MaxSize                          ( m_transformSkipLog2MaxSize  );
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_cTEncTop.setUseResidualDPCM                                 ( RDPCMSignallingMode(signallingModeIndex), m_useResidualDPCM[signallingModeIndex]);
  }
  m_cTEncTop.setUseConstrainedIntraPred                           ( m_bUseConstrainedIntraPred );
  m_cTEncTop.setPCMLog2MinSize                                    ( m_uiPCMLog2MinSize);
  m_cTEncTop.setUsePCM                                            ( m_usePCM );
  m_cTEncTop.setPCMLog2MaxSize                                    ( m_pcmLog2MaxSize);
  m_cTEncTop.setMaxNumMergeCand                                   ( m_maxNumMergeCand );


  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP                                             ( m_useWeightedPred      );
  m_cTEncTop.setWPBiPred                                          ( m_useWeightedBiPred   );
  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2                      ( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode                                         ( m_sliceMode                );
  m_cTEncTop.setSliceArgument                                     ( m_sliceArgument            );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode                                  ( m_sliceSegmentMode         );
  m_cTEncTop.setSliceSegmentArgument                              ( m_sliceSegmentArgument     );
  Int iNumPartInCU = 1<<(m_uiMaxCUDepth<<1);
  if(m_sliceSegmentMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceSegmentArgument                            ( m_sliceSegmentArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_LCU)
  {
    m_cTEncTop.setSliceArgument                                   ( m_sliceArgument * iNumPartInCU );
  }
  if(m_sliceMode==FIXED_NUMBER_OF_TILES)
  {
    m_cTEncTop.setSliceArgument                                   ( m_sliceArgument );
  }

  if(m_sliceMode == 0 )
  {
    m_bLFCrossSliceBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossSliceBoundaryFlag                          ( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO                                            ( m_bUseSAO );
  m_cTEncTop.setMaxNumOffsetsPerPic                               ( m_maxNumOffsetsPerPic);

  m_cTEncTop.setSaoLcuBoundary                                    ( m_saoLcuBoundary);
  m_cTEncTop.setPCMInputBitDepthFlag                              ( m_bPCMInputBitDepthFlag);
  m_cTEncTop.setPCMFilterDisableFlag                              ( m_bPCMFilterDisableFlag);

  m_cTEncTop.setDisableIntraReferenceSmoothing                    (!m_enableIntraReferenceSmoothing );
  m_cTEncTop.setDecodedPictureHashSEIEnabled                      ( m_decodedPictureHashSEIEnabled );
  m_cTEncTop.setRecoveryPointSEIEnabled                           ( m_recoveryPointSEIEnabled );
  m_cTEncTop.setBufferingPeriodSEIEnabled                         ( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled                           ( m_pictureTimingSEIEnabled );
  m_cTEncTop.setToneMappingInfoSEIEnabled                         ( m_toneMappingInfoSEIEnabled );
  m_cTEncTop.setTMISEIToneMapId                                   ( m_toneMapId );
  m_cTEncTop.setTMISEIToneMapCancelFlag                           ( m_toneMapCancelFlag );
  m_cTEncTop.setTMISEIToneMapPersistenceFlag                      ( m_toneMapPersistenceFlag );
  m_cTEncTop.setTMISEICodedDataBitDepth                           ( m_toneMapCodedDataBitDepth );
  m_cTEncTop.setTMISEITargetBitDepth                              ( m_toneMapTargetBitDepth );
  m_cTEncTop.setTMISEIModelID                                     ( m_toneMapModelId );
  m_cTEncTop.setTMISEIMinValue                                    ( m_toneMapMinValue );
  m_cTEncTop.setTMISEIMaxValue                                    ( m_toneMapMaxValue );
  m_cTEncTop.setTMISEISigmoidMidpoint                             ( m_sigmoidMidpoint );
  m_cTEncTop.setTMISEISigmoidWidth                                ( m_sigmoidWidth );
  m_cTEncTop.setTMISEIStartOfCodedInterva                         ( m_startOfCodedInterval );
  m_cTEncTop.setTMISEINumPivots                                   ( m_numPivots );
  m_cTEncTop.setTMISEICodedPivotValue                             ( m_codedPivotValue );
  m_cTEncTop.setTMISEITargetPivotValue                            ( m_targetPivotValue );
  m_cTEncTop.setTMISEICameraIsoSpeedIdc                           ( m_cameraIsoSpeedIdc );
  m_cTEncTop.setTMISEICameraIsoSpeedValue                         ( m_cameraIsoSpeedValue );
  m_cTEncTop.setTMISEIExposureIndexIdc                            ( m_exposureIndexIdc );
  m_cTEncTop.setTMISEIExposureIndexValue                          ( m_exposureIndexValue );
  m_cTEncTop.setTMISEIExposureCompensationValueSignFlag           ( m_exposureCompensationValueSignFlag );
  m_cTEncTop.setTMISEIExposureCompensationValueNumerator          ( m_exposureCompensationValueNumerator );
  m_cTEncTop.setTMISEIExposureCompensationValueDenomIdc           ( m_exposureCompensationValueDenomIdc );
  m_cTEncTop.setTMISEIRefScreenLuminanceWhite                     ( m_refScreenLuminanceWhite );
  m_cTEncTop.setTMISEIExtendedRangeWhiteLevel                     ( m_extendedRangeWhiteLevel );
  m_cTEncTop.setTMISEINominalBlackLevelLumaCodeValue              ( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTop.setTMISEINominalWhiteLevelLumaCodeValue              ( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTop.setTMISEIExtendedWhiteLevelLumaCodeValue             ( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTop.setChromaSamplingFilterHintEnabled                   ( m_chromaSamplingFilterSEIenabled );
  m_cTEncTop.setChromaSamplingHorFilterIdc                        ( m_chromaSamplingHorFilterIdc );
  m_cTEncTop.setChromaSamplingVerFilterIdc                        ( m_chromaSamplingVerFilterIdc );
  m_cTEncTop.setFramePackingArrangementSEIEnabled                 ( m_framePackingSEIEnabled );
  m_cTEncTop.setFramePackingArrangementSEIType                    ( m_framePackingSEIType );
  m_cTEncTop.setFramePackingArrangementSEIId                      ( m_framePackingSEIId );
  m_cTEncTop.setFramePackingArrangementSEIQuincunx                ( m_framePackingSEIQuincunx );
  m_cTEncTop.setFramePackingArrangementSEIInterpretation          ( m_framePackingSEIInterpretation );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIEnabled    ( m_segmentedRectFramePackingSEIEnabled );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEICancel     ( m_segmentedRectFramePackingSEICancel );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIType       ( m_segmentedRectFramePackingSEIType );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIPersistence( m_segmentedRectFramePackingSEIPersistence );
  m_cTEncTop.setDisplayOrientationSEIAngle                        ( m_displayOrientationSEIAngle );
  m_cTEncTop.setTemporalLevel0IndexSEIEnabled                     ( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTop.setGradualDecodingRefreshInfoEnabled                 ( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTop.setNoDisplaySEITLayer                                ( m_noDisplaySEITLayer );
  m_cTEncTop.setDecodingUnitInfoSEIEnabled                        ( m_decodingUnitInfoSEIEnabled );
  m_cTEncTop.setSOPDescriptionSEIEnabled                          ( m_SOPDescriptionSEIEnabled );
  m_cTEncTop.setScalableNestingSEIEnabled                         ( m_scalableNestingSEIEnabled );
  m_cTEncTop.setTMCTSSEIEnabled                                   ( m_tmctsSEIEnabled );
  m_cTEncTop.setTimeCodeSEIEnabled                                ( m_timeCodeSEIEnabled );
  m_cTEncTop.setNumberOfTimeSets                                  ( m_timeCodeSEINumTs );
  for(Int i = 0; i < m_timeCodeSEINumTs; i++) { m_cTEncTop.setTimeSet(m_timeSetArray[i], i); }
  m_cTEncTop.setKneeSEIEnabled                                    ( m_kneeSEIEnabled );
  m_cTEncTop.setKneeSEIId                                         ( m_kneeSEIId );
  m_cTEncTop.setKneeSEICancelFlag                                 ( m_kneeSEICancelFlag );
  m_cTEncTop.setKneeSEIPersistenceFlag                            ( m_kneeSEIPersistenceFlag );
  m_cTEncTop.setKneeSEIMappingFlag                                ( m_kneeSEIMappingFlag );
  m_cTEncTop.setKneeSEIInputDrange                                ( m_kneeSEIInputDrange );
  m_cTEncTop.setKneeSEIInputDispLuminance                         ( m_kneeSEIInputDispLuminance );
  m_cTEncTop.setKneeSEIOutputDrange                               ( m_kneeSEIOutputDrange );
  m_cTEncTop.setKneeSEIOutputDispLuminance                        ( m_kneeSEIOutputDispLuminance );
  m_cTEncTop.setKneeSEINumKneePointsMinus1                        ( m_kneeSEINumKneePointsMinus1 );
  m_cTEncTop.setKneeSEIInputKneePoint                             ( m_kneeSEIInputKneePoint );
  m_cTEncTop.setKneeSEIOutputKneePoint                            ( m_kneeSEIOutputKneePoint );
  m_cTEncTop.setMasteringDisplaySEI                               ( m_masteringDisplay );

  m_cTEncTop.setTileUniformSpacingFlag                            ( m_tileUniformSpacingFlag );
  m_cTEncTop.setNumColumnsMinus1                                  ( m_numTileColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1                                     ( m_numTileRowsMinus1 );
  if(!m_tileUniformSpacingFlag)
  {
    m_cTEncTop.setColumnWidth                                     ( m_tileColumnWidth );
    m_cTEncTop.setRowHeight                                       ( m_tileRowHeight );
  }
  m_cTEncTop.xCheckGSParameters();
  Int uiTilesCount = (m_numTileRowsMinus1+1) * (m_numTileColumnsMinus1+1);
  if(uiTilesCount == 1)
  {
    m_bLFCrossTileBoundaryFlag = true;
  }
  m_cTEncTop.setLFCrossTileBoundaryFlag                           ( m_bLFCrossTileBoundaryFlag );
  m_cTEncTop.setWaveFrontSynchro                                  ( m_iWaveFrontSynchro );
  m_cTEncTop.setWaveFrontSubstreams                               ( m_iWaveFrontSubstreams );
  m_cTEncTop.setTMVPModeId                                        ( m_TMVPModeId );
  m_cTEncTop.setUseScalingListId                                  ( m_useScalingListId  );
  m_cTEncTop.setScalingListFile                                   ( m_scalingListFile   );
  m_cTEncTop.setSignHideFlag                                      ( m_signHideFlag);
  m_cTEncTop.setUseRateCtrl                                       ( m_RCEnableRateControl );
  m_cTEncTop.setTargetBitrate                                     ( m_RCTargetBitrate );
  m_cTEncTop.setKeepHierBit                                       ( m_RCKeepHierarchicalBit );
  m_cTEncTop.setLCULevelRC                                        ( m_RCLCULevelRC );
  m_cTEncTop.setUseLCUSeparateModel                               ( m_RCUseLCUSeparateModel );
  m_cTEncTop.setInitialQP                                         ( m_RCInitialQP );
  m_cTEncTop.setForceIntraQP                                      ( m_RCForceIntraQP );
  m_cTEncTop.setTransquantBypassEnableFlag                        ( m_TransquantBypassEnableFlag );
  m_cTEncTop.setCUTransquantBypassFlagForceValue                  ( m_CUTransquantBypassFlagForce );
  m_cTEncTop.setCostMode                                          ( m_costMode );
  m_cTEncTop.setUseRecalculateQPAccordingToLambda                 ( m_recalculateQPAccordingToLambda );
  m_cTEncTop.setUseStrongIntraSmoothing                           ( m_useStrongIntraSmoothing );
  m_cTEncTop.setActiveParameterSetsSEIEnabled                     ( m_activeParameterSetsSEIEnabled );
  m_cTEncTop.setVuiParametersPresentFlag                          ( m_vuiParametersPresentFlag );
  m_cTEncTop.setAspectRatioInfoPresentFlag                        ( m_aspectRatioInfoPresentFlag);
  m_cTEncTop.setAspectRatioIdc                                    ( m_aspectRatioIdc );
  m_cTEncTop.setSarWidth                                          ( m_sarWidth );
  m_cTEncTop.setSarHeight                                         ( m_sarHeight );
  m_cTEncTop.setOverscanInfoPresentFlag                           ( m_overscanInfoPresentFlag );
  m_cTEncTop.setOverscanAppropriateFlag                           ( m_overscanAppropriateFlag );
  m_cTEncTop.setVideoSignalTypePresentFlag                        ( m_videoSignalTypePresentFlag );
  m_cTEncTop.setVideoFormat                                       ( m_videoFormat );
  m_cTEncTop.setVideoFullRangeFlag                                ( m_videoFullRangeFlag );
  m_cTEncTop.setColourDescriptionPresentFlag                      ( m_colourDescriptionPresentFlag );
  m_cTEncTop.setColourPrimaries                                   ( m_colourPrimaries );
  m_cTEncTop.setTransferCharacteristics                           ( m_transferCharacteristics );
  m_cTEncTop.setMatrixCoefficients                                ( m_matrixCoefficients );
  m_cTEncTop.setChromaLocInfoPresentFlag                          ( m_chromaLocInfoPresentFlag );
  m_cTEncTop.setChromaSampleLocTypeTopField                       ( m_chromaSampleLocTypeTopField );
  m_cTEncTop.setChromaSampleLocTypeBottomField                    ( m_chromaSampleLocTypeBottomField );
  m_cTEncTop.setNeutralChromaIndicationFlag                       ( m_neutralChromaIndicationFlag );
  m_cTEncTop.setDefaultDisplayWindow                              ( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset, m_defDispWinBottomOffset );
  m_cTEncTop.setFrameFieldInfoPresentFlag                         ( m_frameFieldInfoPresentFlag );
  m_cTEncTop.setPocProportionalToTimingFlag                       ( m_pocProportionalToTimingFlag );
  m_cTEncTop.setNumTicksPocDiffOneMinus1                          ( m_numTicksPocDiffOneMinus1    );
  m_cTEncTop.setBitstreamRestrictionFlag                          ( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag                           ( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag                ( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMinSpatialSegmentationIdc                         ( m_minSpatialSegmentationIdc );
  m_cTEncTop.setMaxBytesPerPicDenom                               ( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom                              ( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal                         ( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical                           ( m_log2MaxMvLengthVertical );
}

Void TAppEncTop::xCreateLib()
{
    
  // Video I/O
  m_cTVideoIOYuvInputFile.open( m_pchInputFile,     false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth );  // read  mode
  m_cTVideoIOYuvInputFile.skipFrames(m_FrameSkip, m_iSourceWidth - m_aiPad[0], m_iSourceHeight - m_aiPad[1], m_InputChromaFormatIDC);

  if (m_pchReconFile)
  {
    m_cTVideoIOYuvReconFile.open(m_pchReconFile, true, m_outputBitDepth, m_outputBitDepth, m_internalBitDepth);  // write mode
  }

  // Neo Decoder
  m_cTEncTop.create();
}

Void TAppEncTop::xDestroyLib()
{
  // Video I/O
  m_cTVideoIOYuvInputFile.close();
  m_cTVideoIOYuvReconFile.close();

  // Neo Decoder
  m_cTEncTop.destroy();
}

Void TAppEncTop::xInitLib(Bool isFieldCoding)
{
  m_cTEncTop.init(isFieldCoding);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in TEncTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
Void TAppEncTop::encode()
{
    
    // Hossam: GOP_STR_TYPE
    
//    m_cTEncTop.m_gop_str_type = m_gop_str_type;
//    m_cTEncTop.m_inputFileName = m_inputFileName;
    

    
//    cout << "Yang: TAppEncTop: encode: Will initialize the buffers for encoding fun" << "\n" << endl;
//    getchar();
   
  //Dady: Open a bitstream file to be able to write the result
  fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
  if (!bitstreamFile)
  {
    fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
    exit(EXIT_FAILURE);
  }

  // Hossam: The first expression is used to allocate memory to contain one single element of type type
 // This would be a cinema with one seat, and pointer would be the ticket to that seat
  TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
  TComPicYuv*       pcPicYuvRec = NULL;
  
  // Hossam: Scene change
    TComPicYuv*     pcPicYuvOrgB = NULL;
    
  // Dady: Here it sets every thing from the library i.e the configuration file
  // initialize internal class & member variables
  xInitLibCfg();
  xCreateLib();
  xInitLib(m_isField);

  printChromaFormat();
    
  // TODO: I should extract the Frames I want from the pcPicYUVOrg here... Method should be implemented

  // main encoder loop
  Int   iNumEncoded = 0;
  Bool  bEos = false;

  const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process

  TComPicYuv cPicYuvTrueOrg;

  //Dady: The difference between the two cases is that one of them is creating with the original source dimensions, the other with the normal source dimensions
  // m_iSourceHeight                  :                source height in pixel (when interlaced = field height)
  // m_iSourceHeightOrg;              :                original source height in pixel (when interlaced = frame height)
  // pcPicYUVTrueOrg                  :                buffer where he stores the YUV components

    
  // allocate original YUV buffer for a single frame -- Creation
  if( m_isField )// If it's true then, it is a YUV -- Field
  {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  cPicYuvTrueOrg.create(m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth);
  }
  else
  {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
  cPicYuvTrueOrg.create(m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth);
  }

    
//    int i = 1;
//    cout << "Yang: TAppEncTop: encode: Encoding fun started" << "\n" << endl;
//    getchar();
 
 //Hossam:  it reads frame by frame
  while ( !bEos )
  {
    // get buffers
    xGetBuffer(pcPicYuvRec);
      
   // Hossam: Scene change Get Buffers by passing the original data
   // Passing addresses by reference
   // Now by value
      // Hossam: Change it to match exactly the reconstructed
//    xGetBufferOrg(pcPicYuvOrg);
      xGetBufferOrg(pcPicYuvOrgB);
      
#if SC_ENABLE_PRINT_TWO
    cout << "Monna: xGetBufferOrg: Size of the original buffer: " << m_cListPicYuvOrg.size() << endl;
#endif

     // Dady: m_InputChromaFormatIDC: Which subsampling I am using 4:0:0? 4:4:4? It is there for high profiles
    // read input YUV file
      // Hossam: pcPicYuvOrg is created and will be filled here
    m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, &cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );

    // increase number of received frames
    m_iFrameRcvd++;

    bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );

    Bool flush = 0;
    // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
    if (m_cTVideoIOYuvInputFile.isEof())
    {
      flush = true;
      bEos = true;
      m_iFrameRcvd--;
      m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
    }
   
      
//      cout << "Before Encode " << endl;
//      m_cTEncTop.reportCounterStatus();
      
//     cout << "Yang: TAppEncTop: encode: Frame "  << " will start " << m_isField <<  " " << m_isTopFieldFirst << "\n" << endl;
      // Both are always false! --- I think they relate to the config file
//      getchar();
      
    // call encoding function for one frame (Hossam: 2nd call only)
    if ( m_isField ) m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
//    else             m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
    
      // Hossam: m_cListPicYuvRec: List of reconstruction YUV pictures
      // Hossam: m_cListPicYuvOrg: List of original YUV pictures
      // Hossam: Scene change
    else m_cTEncTop.encodeNew( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
    
      // write bistream to file if necessary
    if ( iNumEncoded > 0 )
    {
      xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
      outputAccessUnits.clear();
    }
      
//      cout << "After Encode " << endl;
//      m_cTEncTop.reportCounterStatus();
      
//      cout << "Yang: TAppEncTop: encode: DONE Frame " << (i-1) << "\n" << endl;
//      getchar();
      
  }//end while -- frames ended

  m_cTEncTop.printSummary(m_isField);

  // delete original YUV buffer
  pcPicYuvOrg->destroy();
  delete pcPicYuvOrg;
  pcPicYuvOrg = NULL;

  // delete used buffers in encoder class
  m_cTEncTop.deletePicBuffer();
  cPicYuvTrueOrg.destroy();

    // Hossam: Scene change
    m_cTEncTop.deleteOrgPicBuffer();
    
  // delete buffers & classes
  xDeleteBuffer();
  xDestroyLib();
  
   // Hossam: Scene Change

    // XXXXXXXXXXXXXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXX
//   xDeleteBufferOrg();
    
#if IS_ENABLE_SC_TIME
    // Open a file and write the total SC time
    string fileName = "";
    std::ostringstream oss;
    
#if IS_STUDENT_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_yc_fps.txt";
#elif IS_STUDENT_Energy_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_energy_fps.txt";
#elif IS_DING_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_ding_fps.txt";
#elif IS_SASTRE_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_sastre_fps.txt";
#elif IS_YAO_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_yao_fps.txt";    
#endif
    
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    FILE*  my_pFile = fopen (pYUVFileName, "at");
    
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD
    string text = "";
    std::ostringstream ossText;
    
    ossText << g_input_FileName << "\t" << (1.0 * m_iFrameRcvd / g_totalSCTime) << "\t" << g_totalSCTime << "\n";
    text = ossText.str();
    fprintf(my_pFile, "%s", text.c_str());

//    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#else
    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#endif
    fclose(my_pFile);
#endif

  // Hossam: Print Rate Summary
  printRateSummary();

  return;
}



Void TAppEncTop::encodeInter()
{
    
    // Hossam: GOP_STR_TYPE
    
    //    m_cTEncTop.m_gop_str_type = m_gop_str_type;
    //    m_cTEncTop.m_inputFileName = m_inputFileName;
    
    
    
    //    cout << "Yang: TAppEncTop: encode: Will initialize the buffers for encoding fun" << "\n" << endl;
    //    getchar();
    
    //Dady: Open a bitstream file to be able to write the result
    fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
    if (!bitstreamFile)
    {
        fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
        exit(EXIT_FAILURE);
    }
    
    // Hossam: The first expression is used to allocate memory to contain one single element of type type
    // This would be a cinema with one seat, and pointer would be the ticket to that seat
    TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
    TComPicYuv*       pcPicYuvRec = NULL;
    
    // Hossam: Scene change
    TComPicYuv*     pcPicYuvOrgB = NULL;
    
    // Dady: Here it sets every thing from the library i.e the configuration file
    // initialize internal class & member variables
    xInitLibCfg();
    xCreateLib();
    xInitLib(m_isField);
    
    printChromaFormat();
    
    // TODO: I should extract the Frames I want from the pcPicYUVOrg here... Method should be implemented
    
    // main encoder loop
    Int   iNumEncoded = 0;
    Bool  bEos = false;
    
    const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
    const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
    
    list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
    
    TComPicYuv cPicYuvTrueOrg;
    
    //Dady: The difference between the two cases is that one of them is creating with the original source dimensions, the other with the normal source dimensions
    // m_iSourceHeight                  :                source height in pixel (when interlaced = field height)
    // m_iSourceHeightOrg;              :                original source height in pixel (when interlaced = frame height)
    // pcPicYUVTrueOrg                  :                buffer where he stores the YUV components
    
    
    // allocate original YUV buffer for a single frame -- Creation
    if( m_isField )// If it's true then, it is a YUV -- Field
    {
        pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        cPicYuvTrueOrg.create(m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth);
    }
    else
    {
        pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        cPicYuvTrueOrg.create(m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth);
    }
    
    
    //    int i = 1;
    //    cout << "Yang: TAppEncTop: encode: Encoding fun started" << "\n" << endl;
    //    getchar();
    
    //Hossam:  it reads frame by frame
    while ( !bEos )
    {
        // get buffers
        xGetBuffer(pcPicYuvRec);
        
        // Hossam: Scene change Get Buffers by passing the original data
        // Passing addresses by reference
        // Now by value
        // Hossam: Change it to match exactly the reconstructed
        //    xGetBufferOrg(pcPicYuvOrg);
        xGetBufferOrg(pcPicYuvOrgB);
        
#if SC_ENABLE_PRINT_TWO
        cout << "Monna: xGetBufferOrg: Size of the original buffer: " << m_cListPicYuvOrg.size() << endl;
#endif
        
        // Dady: m_InputChromaFormatIDC: Which subsampling I am using 4:0:0? 4:4:4? It is there for high profiles
        // read input YUV file
        // Hossam: pcPicYuvOrg is created and will be filled here
        m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, &cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );
        
        // increase number of received frames
        m_iFrameRcvd++;
        
        bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
        
        Bool flush = 0;
        // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
        if (m_cTVideoIOYuvInputFile.isEof())
        {
            flush = true;
            bEos = true;
            m_iFrameRcvd--;
            m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
        }
        
        
        //      cout << "Before Encode " << endl;
        //      m_cTEncTop.reportCounterStatus();
        
        //     cout << "Yang: TAppEncTop: encode: Frame "  << " will start " << m_isField <<  " " << m_isTopFieldFirst << "\n" << endl;
        // Both are always false! --- I think they relate to the config file
        //      getchar();
        
        // call encoding function for one frame (Hossam: 2nd call only)
        if ( m_isField ) m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
        //    else             m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
        
        // Hossam: m_cListPicYuvRec: List of reconstruction YUV pictures
        // Hossam: m_cListPicYuvOrg: List of original YUV pictures
        // Hossam: Scene change
        else m_cTEncTop.encodeNewInter( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
        
        // write bistream to file if necessary
        if ( iNumEncoded > 0 )
        {
            xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
            outputAccessUnits.clear();
        }
        
        //      cout << "After Encode " << endl;
        //      m_cTEncTop.reportCounterStatus();
        
        //      cout << "Yang: TAppEncTop: encode: DONE Frame " << (i-1) << "\n" << endl;
        //      getchar();
        
    }//end while -- frames ended
    
// Hossam: Warning: You should disable that one if you are not using it
#if IS_CALC_INTERDEP_REF_UTILIZATION
    m_cTEncTop.dumpReferenceUtilization();
#endif
    
    m_cTEncTop.printSummary(m_isField);
    
    // delete original YUV buffer
    pcPicYuvOrg->destroy();
    delete pcPicYuvOrg;
    pcPicYuvOrg = NULL;
    
    // delete used buffers in encoder class
    m_cTEncTop.deletePicBuffer();
    cPicYuvTrueOrg.destroy();
    
    // Hossam: Scene change
    m_cTEncTop.deleteOrgPicBuffer();
    
    // delete buffers & classes
    xDeleteBuffer();
    xDestroyLib();
    // Hossam: Scene Change
    
    // XXXXXXXXXXXXXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXX
    //   xDeleteBufferOrg();
    
#if IS_ENABLE_SC_TIME
    // Open a file and write the total SC time
    string fileName = "";
    std::ostringstream oss;
    
#if IS_STUDENT_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_yc_fps.txt";
#elif IS_STUDENT_Energy_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_energy_fps.txt";
#elif IS_DING_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_ding_fps.txt";
#elif IS_SASTRE_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_sastre_fps.txt";
#elif IS_YAO_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_yao_fps.txt";
#endif
        //    const char &dirChar = static_cast<const char>(*ossExist.str().c_str());
    //    bool isDir = isDirectoryExist(dirChar);
    std::ostringstream ossExist;
    ossExist << "Gen//Seq-Time//";
    const char *dirChar = ossExist.str().c_str();
    
    // Check if Gen/Seq-Time does exist -> if not, make it exist
    if(!isDirectoryExist(*dirChar))
    {
        cout << "[Warning] Cannot access directory name: " << ossExist.str() << ", but will create it for you ;)" << endl;
        std::ostringstream cmd_concat;
        cmd_concat << "mkdir -p " << ossExist.str();
        string mkdir_cmd = cmd_concat.str();
        system(mkdir_cmd.c_str());
    }
        fileName = oss.str();
        Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
        FILE*  my_pFile = fopen (pYUVFileName, "at");
    
    #if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD
        string text = "";
        std::ostringstream ossText;
    
    
        ossText << g_input_FileName << "\t" << (1.0 * m_iFrameRcvd / g_totalSCTime) << "\n";
        text = ossText.str();
        fprintf(my_pFile, "%s", text.c_str());
    
        //    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
    #else
        fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
    #endif
        fclose(my_pFile);
#endif

    
    // Hossam: Print Rate Summary
    printRateSummary();
    return;
}


// Checks the directory exist or not
Bool TAppEncTop::isDirectoryExist(const char &directoryName)
{
    struct stat info;
    const char * ptr_for_stat = &directoryName;
    return stat(ptr_for_stat, &info) == 0;
}


/**
 - create internal class
 - initialize internal variable
 - until the end of input YUV file, call encoding function in TEncTop class
 - delete allocated buffers
 - destroy internal class
 .
 */
// Hossam: encodeNewP with P frames buffering
Void TAppEncTop::encodeNewP04()
{
    // Dady: Open a bitstream file to be able to write the result
    fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
    if (!bitstreamFile)
    {
        fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
        exit(EXIT_FAILURE);
    }
    
    // Hossam: The first expression is used to allocate memory to contain one single element of type type
    // This would be a cinema with one seat, and pointer would be the ticket to that seat
//    TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
    TComPicYuv*       pcPicYuvOrg = NULL;
    TComPicYuv*       pcPicYuvRec = NULL;
    
    // Dady: Here it sets every thing from the library i.e the configuration file
    // initialize internal class & member variables
    xInitLibCfg();
    xCreateLib();
    xInitLib(m_isField);
    
    printChromaFormat();
    
    // TODO: I should extract the Frames I want from the pcPicYUVOrg here... Method should be implemented
    
    // main encoder loop
    Int   iNumEncoded = 0;
    Bool  bEos = false;
    
    const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
    const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
    
    list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
    
    // Hossam: allocate on the stack for a short-term use
    TComPicYuv* cPicYuvTrueOrg = NULL;
    
    // Dady: The difference between the two cases is that one of them is creating with the original source dimensions, the other with the normal source dimensions
    // m_iSourceHeight                  :                source height in pixel (when interlaced = field height)
    // m_iSourceHeightOrg;              :                original source height in pixel (when interlaced = frame height)
    // pcPicYUVTrueOrg                  :                buffer where he stores the YUV components
    
    // Hossam: index for debugging reading frames
    Int read_idx = 0;
    
    
    //Hossam:  it reads frame by frame
    while ( !bEos )
    {
        // get buffers
        xGetBuffer(pcPicYuvRec);

        cout << "Read Frame " << read_idx++ << endl;
        
        // Hossam: set the flush to zero here
        Bool flush = 0;
        
        // Iterations length
        UInt iter_length = 1 + g_pFramesSize;
        
        for (Int cnt = 0; cnt < iter_length; cnt++)
        {
            // Store the pictures of pcPicYuvOrg
            xGetBufferOrgP(pcPicYuvOrg);
            xGetBufferOrgTrueP(cPicYuvTrueOrg);
            
            // Read frame
            m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );
            // increase number of received frames
            m_iFrameRcvd++;
        } //end for

        
        
        // Hossam: Field encoding is disabled!
        if ( m_isField ){
            m_cTEncTop.encode( bEos, flush ? 0 : m_cListPicYuvOrg.popFront(),
                              flush ? 0 : m_cListPicYuvOrgTrue.popFront(), snrCSC,
                              m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
        } // end if
        else
        {
            cout << "ENCODE GHADA MENNA MAYAR (2)" << endl;
            
            // Fetch 0
            pcPicYuvOrg = m_cListPicYuvOrg.popFront();
            cPicYuvTrueOrg = m_cListPicYuvOrgTrue.popFront();
            
            // Encode 0
            m_cTEncTop.encodeNewP04( bEos, flush ? 0 : pcPicYuvOrg,
                                  flush ? 0 : cPicYuvTrueOrg, snrCSC,
                                  m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );

            // Encode the rest
            for (Int cnt = 1; cnt < iter_length; cnt++)
            {
             
              // Encode 1, 2, 3, 4
              // xGetBuffer for Recons
              xGetBuffer(pcPicYuvRec);
         
              cout << cnt << ") About to pop: Size org 1 and 2: " << m_cListPicYuvOrg.size() << ", " << m_cListPicYuvOrgTrue.size() << endl;
                // Encode the rest
               m_cTEncTop.encodeNewP04( bEos, flush ? 0 : m_cListPicYuvOrg.popFront(),
                                      flush ? 0 : m_cListPicYuvOrgTrue.popFront(), snrCSC,
                                      m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
            } // end for loop
        
            // Check the end of stream and flushing after encoding
            bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
            
            
            // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
            if (m_cTVideoIOYuvInputFile.isEof())
            {
                flush = true;
                bEos = true;
                m_iFrameRcvd--;
                m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
            }
            

        } // end else
        
        
        // write bistream to file if necessary
        if ( iNumEncoded > 0)
        {
            cout << "xWriteOutput " << read_idx << ", " << iNumEncoded << endl;
            xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
            outputAccessUnits.clear();
        }
        
    }//end while -- frames ended
    
    m_cTEncTop.printSummary(m_isField);
    
    // delete original YUV buffers
    pcPicYuvOrg->destroy();
    
    delete pcPicYuvOrg;
    
    pcPicYuvOrg = NULL;
    
    // delete used buffers in encoder class
    
    m_cTEncTop.deletePicBuffer();
    
    cPicYuvTrueOrg->destroy();
//    cPicYuvTrueOrg.destroy();
    
    
    // Hossam: Scene change
    m_cTEncTop.deleteOrgPicBuffer();
    
    // delete buffers & classes
    xDeleteBuffer();
    xDestroyLib();
    
    
    // Hossam: Scene Change
    
    // XXXXXXXXXXXXXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXX
    // Hossam: Simply those methods do not do anything, since buffers
    // are empty at the end
    xDeleteBufferOrg();
    xDeleteBufferOrgTrue();

    
#if IS_ENABLE_SC_TIME
    // Open a file and write the total SC time
    string fileName = "";
    std::ostringstream oss;
    
#if IS_STUDENT_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_yc_fps.txt";
#elif IS_STUDENT_Energy_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_energy_fps.txt";
#elif IS_DING_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_ding_fps.txt";
#elif IS_SASTRE_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_sastre_fps.txt";
#elif IS_YAO_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_yao_fps.txt";
#endif
    
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    FILE*  my_pFile = fopen (pYUVFileName, "at");
    
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD
    string text = "";
    std::ostringstream ossText;
    
    ossText << g_input_FileName << "\t" << (1.0 * m_iFrameRcvd / g_totalSCTime) << "\n";
    text = ossText.str();
    fprintf(my_pFile, "%s", text.c_str());
    
    //    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#else
    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#endif
    fclose(my_pFile);
#endif
    
    // Hossam: Print Rate Summary
    printRateSummary();
    
    return;
}


////// encode with look ahead
// Hossam: encodeNewP with P frames buffering
Void TAppEncTop::encodeNewP05()
{
    // Dady: Open a bitstream file to be able to write the result
    fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
    if (!bitstreamFile)
    {
        fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
        exit(EXIT_FAILURE);
    }
    
    // Hossam: The first expression is used to allocate memory to contain one single element of type type
    // This would be a cinema with one seat, and pointer would be the ticket to that seat
    //    TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
    TComPicYuv*       pcPicYuvOrg = NULL;
    TComPicYuv*       pcPicYuvRec = NULL;
    
    // Dady: Here it sets every thing from the library i.e the configuration file
    // initialize internal class & member variables
    xInitLibCfg();
    xCreateLib();
    xInitLib(m_isField);
    
    printChromaFormat();
    
    // TODO: I should extract the Frames I want from the pcPicYUVOrg here... Method should be implemented
    
    // main encoder loop
    Int   iNumEncoded = 0;
    Bool  bEos = false;
    
    const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
    const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
    
    list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
    
    // Hossam: for look Ahead
    list<AccessUnit> outputAccessUnitsLookAhead; ///< list of access units to write out.  is populated by the encoding process

    
    // Hossam: allocate on the stack for a short-term use
    TComPicYuv* cPicYuvTrueOrg = NULL;
    
    // Dady: The difference between the two cases is that one of them is creating with the original source dimensions, the other with the normal source dimensions
    // m_iSourceHeight                  :                source height in pixel (when interlaced = field height)
    // m_iSourceHeightOrg;              :                original source height in pixel (when interlaced = frame height)
    // pcPicYUVTrueOrg                  :                buffer where he stores the YUV components
    
    // Hossam: index for debugging reading frames
    Int read_idx = 0;
    
    
    //Hossam:  it reads frame by frame
    while ( !bEos )
    {
//        cout << "Read Frame " << read_idx++ << endl;
        
        // Hossam: set the flush to zero here
        Bool flush = 0;
        
        // Iterations length
        UInt iter_length = 1 + g_pFramesSize;
        
        // Do the same process for each frame in the window
        for(UInt within_p = 0; within_p < iter_length; within_p++)
        {
        
            cout << "Start within_p " << within_p << endl;
            if(m_iFrameRcvd + iter_length <= m_framesToBeEncoded)
            {
                for (Int cnt = 0; cnt < iter_length; cnt++)
                {
                    // Store the pictures of pcPicYuvOrg
                    xGetBufferOrgP(pcPicYuvOrg);
                    xGetBufferOrgTrueP(cPicYuvTrueOrg);
                    
                    // Read frame
                    m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC);
                    // increase number of received frames
                    m_iFrameRcvd++;
                    
                    cout << "Read Frame " << cnt << ", org buffer size: " << m_cListPicYuvOrg.size() << endl;
                } //end for
            }
            else
            {
                ;
                // XXXX
            }
            
            // get buffers
//            xGetBuffer(pcPicYuvRec);
            xGetBufferP(pcPicYuvRec);

            
            // Hossam: Field encoding is disabled!
            if ( m_isField ){
                m_cTEncTop.encode( bEos, flush ? 0 : m_cListPicYuvOrg.popFront(),
                                  flush ? 0 : m_cListPicYuvOrgTrue.popFront(), snrCSC,
                                  m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
            } // end if
            else
            {
//                cout << "ENCODE GHADA MENNA MAYAR (2)" << endl;
                
                // Fetch 0
                pcPicYuvOrg = m_cListPicYuvOrg.popFront();
                cPicYuvTrueOrg = m_cListPicYuvOrgTrue.popFront();
                
                Bool isAddPSNR = true;
                Bool isMockEncoding = true;
                
                // Store the last state
                m_cTEncTop.setLastPOCLast();
                m_cTEncTop.setLastNumPicRcvd();
                
//                cout << "000 Before Size of the reconsturcted list " << m_cListPicYuvRec.size() << endl;

                
                cout << "before encode 0 OutputAccessUnits " << outputAccessUnits.size() << endl;
                
//                cout << "**** Before Encode 0 " << endl;
                // Encode 0
//                m_cTEncTop.encodeNewP(isMockEncoding, isAddPSNR, bEos, flush ? 0 : pcPicYuvOrg,
//                                      flush ? 0 : cPicYuvTrueOrg, snrCSC,
//                                      m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
                
//                m_cTEncTop.reportCounterStatus();
                
//                cout << "\n\n\n******* Enter Encode 0 " << endl;
//                m_cTEncTop.reportCounterStatus();
                
                
                m_cTEncTop.encodeNewP(isMockEncoding, isAddPSNR, bEos, flush ? 0 : pcPicYuvOrg,
                                      flush ? 0 : cPicYuvTrueOrg, snrCSC,
                                      m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnitsLookAhead, iNumEncoded );
                
                
                cout << "\n\n\n**** After Encode 0 " << endl;
                m_cTEncTop.reportCounterStatus();
                
//                cout << "after encode 0 OutputAccessUnits " << outputAccessUnits.size() << endl;
                // Encode the rest and calculate mu_i, j
                for (Int cnt = 1; cnt < iter_length - within_p; cnt++)
                {
//                    cout << "111 Before Size of the reconsturcted list " << m_cListPicYuvRec.size() << endl;
                    
                    isAddPSNR = false;
//                    cout << "Look Ahead from  " << cnt << ", to " <<  iter_length - within_p << endl;
                    // Encode 1, 2, 3, 4
                    // xGetBuffer for Recons
//                    xGetBuffer(pcPicYuvRec);
                    xGetBufferP(pcPicYuvRec);
                    
                    
                    // Encode the rest (look Ahead)
//                    m_cTEncTop.encodeNewP( isMockEncoding, isAddPSNR, bEos, flush ? 0 : m_cListPicYuvOrg.front(),
//                                          flush ? 0 : m_cListPicYuvOrgTrue.front(), snrCSC,
//                                          m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
                    
                    
                    cout << "\n\n\n******* Enter Encode  " << cnt << endl;
                    m_cTEncTop.reportCounterStatus();
                    
                    m_cTEncTop.encodeNewP( isMockEncoding, isAddPSNR, bEos, flush ? 0 : m_cListPicYuvOrg.front(),
                                          flush ? 0 : m_cListPicYuvOrgTrue.front(), snrCSC,
                                          m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnitsLookAhead, iNumEncoded );
                    
                    
                    cout << "\n\n\n******* After Encode  " << cnt << endl;
                    m_cTEncTop.reportCounterStatus();

                    
                    // increment the received pictures
//                    m_cTEncTop.incrementPicsRecivedByOne();
                    
   
//                    cout << "OutputAccessUnits " << outputAccessUnits.size() << endl;
                    // Rotate
                    m_cListPicYuvOrg.pushBack(m_cListPicYuvOrg.popFront());
                    
                    // Rotate
                    m_cListPicYuvOrgTrue.pushBack(m_cListPicYuvOrgTrue.popFront());
                } // end for loop
                
                
                // Encode 0 again for real
                cout << "\n \n ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
                cout << "XXXXXXXXXXX Encode 0 again for real " << endl;
                cout << "\n \n ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
                m_cTEncTop.revertPOCLast();
                m_cTEncTop.revertNumPicRcvd();
                
                /// XXXXX
                cout << "MaybeError update overall pics "<< endl;
//                m_cTEncTop.updateOverallPicsRecived();
                isMockEncoding = false;
                
                // Restore the picture buffer in top
//                m_cTEncTop.xRestorePicBufferInTop();
                
//                cout << "Before Size of the reconsturcted list " << m_cListPicYuvRec.size() << endl;
                // Encode 1, 2, 3, 4
                // xGetBuffer for Recons
                // Pop all the future (XXXXXXXXXXX bug here)
                
                Int popLength = iter_length - within_p - 1;
                cout << "EEEEEE Popping the future for OUT will cause the error in the future " << endl;
                cout << "Pop length: " << popLength << ", " << m_cListPicYuvRec.size() << endl;
                for(Int cnt = 0; cnt < popLength; cnt++)
                {
                    m_cListPicYuvRec.popBack();
                }


//                cout << "Size of the reconsturcted list " << m_cListPicYuvRec.size() << endl;
                m_cTEncTop.reportCounterStatus();
                xGetBufferP(pcPicYuvRec);
                m_cTEncTop.encodeNewP(isMockEncoding, isAddPSNR, bEos, flush ? 0 : pcPicYuvOrg,
                                      flush ? 0 : cPicYuvTrueOrg, snrCSC,
                                      m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
                
                
                
                cout << "After Actual Encoding " << endl;
                m_cTEncTop.reportCounterStatus();
                
                
                cout << "After actual Encoding Size of reconstructed OUTPUT buffer " << m_cListPicYuvRec.size() << endl;
                
                cout << "After Actual Encoding: OutputAccessUnits " << outputAccessUnits.size() << endl;

                
                cout << "Restore the recons buffer and shape it as the original buffer FOR next iteration " << endl;
                // Restore the picture buffer in top
                m_cTEncTop.xRestorePicBufferInTop();
                
                cout << "Increment the total number of pictures encoded so far " << endl;
                m_cTEncTop.IncrementOverallPicsRecivedByOne();
                
                cout << "[MaybeError %4] Increment POCLast " << endl;
//                m_cTEncTop.IncrementPOCLastByOne();
//                m_cTEncTop.setLastPOCLast();
//                m_cTEncTop.setLastNumPicRcvd();

                m_cTEncTop.reportCounterStatus();
                
                
                
                // Check the end of stream and flushing after encoding
                //            bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) );
                
                // Hossam: Add is Empty in the EoS
                bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) && m_cListPicYuvOrg.empty());
                
                
                // Hossam: queued pictures without an empty buffer
                // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
                if (m_cTVideoIOYuvInputFile.isEof() && m_cListPicYuvOrg.empty())
                    //            if (m_cTVideoIOYuvInputFile.isEof())
                {
                    flush = true;
                    bEos = true;
                    m_iFrameRcvd--;
                    m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
                }
                
                
            } // end else
            
            // Hossam: Inside the loop
            // write bistream to file if necessary
            static Bool first_frame = true;
            if ( iNumEncoded > 0 || first_frame)
            {
                cout << "xWriteOutput actual_encode_idx, iNumEncoded: " << read_idx << ", " << iNumEncoded << endl;
//                xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
                xWriteOutput(bitstreamFile, 1, outputAccessUnits);

                outputAccessUnits.clear();
                
                first_frame = false;
            }
            
            cout << "Clear up the output access units for lookAhead " << endl;
            outputAccessUnitsLookAhead.clear();

        
            // Restore the state and update it (xXXXXX)
            
//            cout << "Copy back the last state to Original POC Last " << endl;
//            m_cTEncTop.updatePOCLast();
//            m_cTEncTop.updateNumePicRcvd();
//
            cout << "Just before moving on to the next state " << endl;
            m_cTEncTop.reportCounterStatus();
            
            
            cout << "Done Frame " << within_p << ", org buffer size: " << m_cListPicYuvOrg.size() << endl;
            cout << "\n \n \n \n =================== " << within_p << " \n \n \n " << endl;
            
//            if(within_p == 2)
//            {
//                exit(0);
//            }
            
            
        } // Within p loop
        
        // write bistream to file if necessary
//        if ( iNumEncoded > 0)
//        {
//            cout << "xWriteOutput read_idx, iNumEncoded: " << read_idx << ", " << iNumEncoded << endl;
//            xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
//            outputAccessUnits.clear();
//        }
        
    }//end while -- frames ended
    
    m_cTEncTop.printSummary(m_isField);
    
    // delete original YUV buffers
    pcPicYuvOrg->destroy();
    
    delete pcPicYuvOrg;
    
    pcPicYuvOrg = NULL;
    
    // delete used buffers in encoder class
    
    m_cTEncTop.deletePicBuffer();
    
    cPicYuvTrueOrg->destroy();
    //    cPicYuvTrueOrg.destroy();
    
    
    // Hossam: Scene change
    m_cTEncTop.deleteOrgPicBuffer();
    
    cout << "If you are using Mock buffers in top, you need to clear them " << endl;
    
    // delete buffers & classes
    xDeleteBuffer();
    xDestroyLib();
    
    
    // Hossam: Scene Change
    
    // XXXXXXXXXXXXXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXX
    // Hossam: Simply those methods do not do anything, since buffers
    // are empty at the end
    xDeleteBufferOrg();
    xDeleteBufferOrgTrue();
    
    
#if IS_ENABLE_SC_TIME
    // Open a file and write the total SC time
    string fileName = "";
    std::ostringstream oss;
    
#if IS_STUDENT_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_yc_fps.txt";
#elif IS_STUDENT_Energy_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_energy_fps.txt";
#elif IS_DING_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_ding_fps.txt";
#elif IS_SASTRE_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_sastre_fps.txt";
#elif IS_YAO_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_yao_fps.txt";
#endif
    
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    FILE*  my_pFile = fopen (pYUVFileName, "at");
    
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD
    string text = "";
    std::ostringstream ossText;
    
    ossText << g_input_FileName << "\t" << (1.0 * m_iFrameRcvd / g_totalSCTime) << "\n";
    text = ossText.str();
    fprintf(my_pFile, "%s", text.c_str());
    
    //    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#else
    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#endif
    fclose(my_pFile);
#endif
    
    // Hossam: Print Rate Summary
    printRateSummary();
    
    return;
}

////// encode with look ahead
// Hossam: encodeNewP with P frames buffering
Void TAppEncTop::encodeNewP()
{
    // Dady: Open a bitstream file to be able to write the result
    fstream bitstreamFile(m_pchBitstreamFile, fstream::binary | fstream::out);
    if (!bitstreamFile)
    {
        fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", m_pchBitstreamFile);
        exit(EXIT_FAILURE);
    }
    
    // Hossam: The first expression is used to allocate memory to contain one single element of type type
    // This would be a cinema with one seat, and pointer would be the ticket to that seat
    //    TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
    TComPicYuv*       pcPicYuvOrg = NULL;
    TComPicYuv*       pcPicYuvRec = NULL;
    
    // Dady: Here it sets every thing from the library i.e the configuration file
    // initialize internal class & member variables
    xInitLibCfg();
    xCreateLib();
    xInitLib(m_isField);
    
    printChromaFormat();
    
    // TODO: I should extract the Frames I want from the pcPicYUVOrg here... Method should be implemented
    
    // main encoder loop
    Int   iNumEncoded = 0;
    Bool  bEos = false;
    
    const InputColourSpaceConversion ipCSC  =  m_inputColourSpaceConvert;
    const InputColourSpaceConversion snrCSC = (!m_snrInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
    
    list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
    
    // Hossam: for look Ahead
    list<AccessUnit> outputAccessUnitsLookAhead; ///< list of access units to write out.  is populated by the encoding process
    
    
    // Hossam: allocate on the stack for a short-term use
    TComPicYuv* cPicYuvTrueOrg = NULL;
    
    // Dady: The difference between the two cases is that one of them is creating with the original source dimensions, the other with the normal source dimensions
    // m_iSourceHeight                  :                source height in pixel (when interlaced = field height)
    // m_iSourceHeightOrg;              :                original source height in pixel (when interlaced = frame height)
    // pcPicYUVTrueOrg                  :                buffer where he stores the YUV components
    
    // Hossam: index for debugging reading frames
//    Int read_idx = 0;
    
    // counter for the frames
//    UInt within_p = 0;
    
    
    //Hossam:  it reads frame by frame
    while ( !bEos )
    {
//        cout << "Read Frame " << read_idx++ << endl;
        
        // Hossam: set the flush to zero here
        Bool flush = 0;
        
        // Iterations length
        //        UInt iter_length = 1 + g_pFramesSize;
        
        // Read 0
        xGetBufferOrgP(pcPicYuvOrg);
        xGetBufferOrgTrueP(cPicYuvTrueOrg);
        
        // Read frame
        m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );
        // increase number of received frames
        m_iFrameRcvd++;
        
        
//        cout << "Read Frame " << 0 << ", org buffer size: " << m_cListPicYuvOrg.size() << endl;
 
        while(!m_cListPicYuvOrg.empty())
        {
            
            Bool isEndLookAheadRead = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded));

            // Determine the readLength
            UInt readLength = 0;
            if(m_cListPicYuvOrg.empty())
            {
                readLength = g_pFramesSize;
            }
            else
            {
                // Hossam: read the rest of your p
                UInt future_size = m_cListPicYuvOrg.size() - 1;
                readLength = g_pFramesSize - future_size;
            }
            
            // Read what you can read
            for (Int cnt = 0; cnt < readLength && !isEndLookAheadRead; cnt++)
            {
                // Store the pictures of pcPicYuvOrg
                xGetBufferOrgP(pcPicYuvOrg);
                xGetBufferOrgTrueP(cPicYuvTrueOrg);
                
                // Read frame
                m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );
                // increase number of received frames
                m_iFrameRcvd++;
                
                isEndLookAheadRead = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded));
                
            } //end for
            
            
            // Hossam: Field encoding is disabled!
            if ( m_isField ){
                xGetBufferP(pcPicYuvRec);
                m_cTEncTop.encode( bEos, flush ? 0 : m_cListPicYuvOrg.popFront(),
                                  flush ? 0 : m_cListPicYuvOrgTrue.popFront(), snrCSC,
                                  m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
            } // end if
            else
            {
                //                cout << "ENCODE GHADA MENNA MAYAR (2)" << endl;
                
                
                // Fetch 0
                pcPicYuvOrg = m_cListPicYuvOrg.popFront();
                cPicYuvTrueOrg = m_cListPicYuvOrgTrue.popFront();

                
                UInt lookAheadLength = m_cListPicYuvOrg.size();
//                cout << "LookAhead future " << lookAheadLength << " frames." << endl;

                
                // Flags for encoding
                Bool isAddPSNR = true;
                Bool isMockEncoding = true;
                
                // Store the last state
                m_cTEncTop.setLastPOCLast();
                m_cTEncTop.setLastNumPicRcvd();
                
            
                // if there is nothing to lookAhead, do not do mock
                if(lookAheadLength > 0)
                {
//                    cout << "before encode 0 OutputAccessUnits " << outputAccessUnits.size() << endl;
                    
                    m_cTEncTop.encodeNewP(isMockEncoding, isAddPSNR, bEos, flush ? 0 : pcPicYuvOrg,
                                          flush ? 0 : cPicYuvTrueOrg, snrCSC,
                                          m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnitsLookAhead, iNumEncoded );
                    
                    
//                    if(within_p == 10 || within_p == 11)
//                    {
//                        cout <<  within_p << ") Show me RC Lists from AppTop After First Mock " << endl;
//                        m_cTEncTop.showMeRcLists();
//                    }
                    
//                    cout << "\n\n\n**** After Encode 0 " << endl;
//                    m_cTEncTop.reportCounterStatus();
                    
                    
//                    if(within_p==1) {
//                        string residualYuvFileName = "OrgBuffer.yuv";
//                        const char *pFileName = residualYuvFileName.c_str();
//                        TComPicYuv* test = pcPicYuvOrg;
//                        test->dump (pFileName, true);
//                    }
                    
                    for (Int cnt = 0; cnt < lookAheadLength; cnt++)
                    {
                        //                    cout << "111 Before Size of the reconsturcted list " << m_cListPicYuvRec.size() << endl;
                        
                        isAddPSNR = false;

//                        if(within_p==1) {
//                                    string residualYuvFileName = "OrgBuffer.yuv";
//                                    const char *pFileName = residualYuvFileName.c_str();
//                                    TComPicYuv* test = m_cListPicYuvOrg.front();
//                                    test->dump (pFileName, true);
//                        }
                        
//                        m_cTEncTop.reportCounterStatus();
//                        cout << "\n\n\n******* Enter Encode LookAhead " << cnt << endl;
                        
                        m_cTEncTop.encodeNewP( isMockEncoding, isAddPSNR, bEos, flush ? 0 : m_cListPicYuvOrg.front(),
                                              flush ? 0 : m_cListPicYuvOrgTrue.front(), snrCSC,
                                              m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnitsLookAhead, iNumEncoded );
                        
                        
//                        if(within_p == 10 || within_p == 11)
//                        {
//                            cout <<  within_p << ") Show me RC Lists from AppTop After Encode LookAhead " << cnt << endl;
//                            m_cTEncTop.showMeRcLists();
//                        }
                        
//                        cout << "\n\n\n******* After Encode  " << cnt << endl;
//                        m_cTEncTop.reportCounterStatus();
                        
                        
                        // increment the received pictures
                        //                    m_cTEncTop.incrementPicsRecivedByOne();
                        
                        
                        //                    cout << "OutputAccessUnits " << outputAccessUnits.size() << endl;
                        // Rotate
                        m_cListPicYuvOrg.pushBack(m_cListPicYuvOrg.popFront());
                        
                        // Rotate
                        m_cListPicYuvOrgTrue.pushBack(m_cListPicYuvOrgTrue.popFront());
                    } // end for loop
                
                }// end if (lookAheadLength > 0)
                
//                if(within_p == 1)
//                    exit(0);
                
//                cout <<  within_p << ") Show me RC Lists from AppTop BEFORE Encode Actual " << endl;
//                m_cTEncTop.showMeRcLists();

                
                // Encode 0 again for real
//                cout << "\n \n ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
//                cout << "XXXXXXXXXXX Encode ACTUAL again for real " << endl;
//                cout << "\n \n ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
                m_cTEncTop.revertPOCLast();
                m_cTEncTop.revertNumPicRcvd();
                
                /// Set Mock encoding
                isMockEncoding = false;
                

                xGetBufferP(pcPicYuvRec);
                m_cTEncTop.encodeNewP(isMockEncoding, isAddPSNR, bEos, flush ? 0 : pcPicYuvOrg,
                                      flush ? 0 : cPicYuvTrueOrg, snrCSC,
                                      m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
                
                
                
                
                
//                cout << "Restore the recons buffer and shape it as the original buffer FOR next iteration " << endl;
                // Restore the picture buffer in top
                m_cTEncTop.xRestorePicBufferInTop();
                
                
//                cout << "Increment the total number of pictures encoded so far " << endl;
                m_cTEncTop.IncrementOverallPicsRecivedByOne();
                
                
                // Hossam: Add is Empty in the EoS
                bEos = (m_isField && (m_iFrameRcvd == (m_framesToBeEncoded >> 1) )) || ( !m_isField && (m_iFrameRcvd == m_framesToBeEncoded) && m_cListPicYuvOrg.empty());
                
                
                // Hossam: queued pictures without an empty buffer
                // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
                if (m_cTVideoIOYuvInputFile.isEof() && m_cListPicYuvOrg.empty())
                    //            if (m_cTVideoIOYuvInputFile.isEof())
                {
                    flush = true;
                    bEos = true;
                    m_iFrameRcvd--;
                    m_cTEncTop.setFramesToBeEncoded(m_iFrameRcvd);
                }
                
                
            } // end else
            
            // Hossam: Inside the loop
            // write bistream to file if necessary
            static Bool first_frame = true;
            if ( iNumEncoded > 0 || first_frame)
            {
//                cout << "xWriteOutput actual_encode_idx, iNumEncoded: " << read_idx << ", " << iNumEncoded << endl;
                //                xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
                xWriteOutput(bitstreamFile, 1, outputAccessUnits);
                
                outputAccessUnits.clear();
                first_frame = false;
            }
            
//            cout << "Clear up the output access units for lookAhead " << endl;
            outputAccessUnitsLookAhead.clear();
            
            
            
            
//            cout << "Just before moving on to the next state " << endl;
//            m_cTEncTop.reportCounterStatus();
            
            
        } // within p loop while(!m_cListPicYuvOrg.empty())
        

    }//end while -- frames ended
    
    m_cTEncTop.printSummary(m_isField);
    
    // delete original YUV buffers
    pcPicYuvOrg->destroy();
    
    delete pcPicYuvOrg;
    
    pcPicYuvOrg = NULL;
    
    // delete used buffers in encoder class
    
    m_cTEncTop.deletePicBuffer();
    
    cPicYuvTrueOrg->destroy();
    //    cPicYuvTrueOrg.destroy();
    
    
    // Hossam: Scene change
    m_cTEncTop.deleteOrgPicBuffer();
    
//    cout << "If you are using Mock buffers in top, you need to clear them " << endl;
    
    // delete buffers & classes
    xDeleteBuffer();
    xDestroyLib();
    
    
    // XXXXXXXXXXXXXXXXXXXX POSSIBLE BUG BUFFER XXXXXXXXXXXXXXXXXXXX
    // Hossam: Simply those methods do not do anything, since buffers
    // are empty at the end
    xDeleteBufferOrg();
    xDeleteBufferOrgTrue();
    
    
#if IS_ENABLE_SC_TIME
    // Open a file and write the total SC time
    string fileName = "";
    std::ostringstream oss;
    
#if IS_STUDENT_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_yc_fps.txt";
#elif IS_STUDENT_Energy_SCD
    oss << "Gen//Seq-Time//" << "Time" << "_energy_fps.txt";
#elif IS_DING_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_ding_fps.txt";
#elif IS_SASTRE_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_sastre_fps.txt";
#elif IS_YAO_SCD
    oss << "Gen//Seq-Time//" << g_input_FileName << "_yao_fps.txt";
#endif
    
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    FILE*  my_pFile = fopen (pYUVFileName, "at");
    
#if IS_STUDENT_SCD || IS_STUDENT_Energy_SCD
    string text = "";
    std::ostringstream ossText;
    
    ossText << g_input_FileName << "\t" << (1.0 * m_iFrameRcvd / g_totalSCTime) << "\n";
    text = ossText.str();
    fprintf(my_pFile, "%s", text.c_str());
    
    //    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#else
    fprintf(my_pFile, "%-12.3f\n", m_iFrameRcvd / g_totalSCTime);
#endif
    fclose(my_pFile);
#endif
    
    // Hossam: Print Rate Summary
    printRateSummary();
    
    return;
}



// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - application has picture buffer list with size of GOP
 - picture buffer list acts as ring buffer
 - end of the list has the latest picture
 .
 */
Void TAppEncTop::xGetBuffer( TComPicYuv*& rpcPicYuvRec)
{
  assert( m_iGOPSize > 0 );

  // org. buffer
  if ( m_cListPicYuvRec.size() >= (UInt)m_iGOPSize ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
  {
    rpcPicYuvRec = m_cListPicYuvRec.popFront();
  }
  else
  {
    rpcPicYuvRec = new TComPicYuv;

    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );

  }
  m_cListPicYuvRec.pushBack( rpcPicYuvRec );
}


// xGetBuffer for LookAhead
Void TAppEncTop::xGetBufferP( TComPicYuv*& rpcPicYuvRec)
{
    assert( m_iGOPSize > 0 );
    
    // org. buffer
//    if ( m_cListPicYuvRec.size() >= g_pFramesSize + 1 ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
    if ( m_cListPicYuvRec.size() >= (UInt)m_iGOPSize ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.

//    if ( m_cListPicYuvRec.size() >= 1 + g_pFramesSize ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
    {
        rpcPicYuvRec = m_cListPicYuvRec.popFront();
        
//        cout << "Pop recons OUT ameer " << m_cListPicYuvRec.size() << endl;
    }
    else
    {
        rpcPicYuvRec = new TComPicYuv;
        
        rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        
//        cout << "Create new pic recons OUT ameer " << m_cListPicYuvRec.size() << endl;

        
    }
    
    m_cListPicYuvRec.pushBack( rpcPicYuvRec );
//    cout << "Push recons OUT ameer " << m_cListPicYuvRec.size() << endl;

}


/**
 - Hossam: Scene Change
 - application has picture buffer list with size of GOP
 - picture buffer list acts as ring buffer
 - end of the list has the latest picture
 .
 */
// rpcPicYuvRec is now a reference to a pointer, so any changes made to rpcPicYuvRec will change the argument as well!
//Void TAppEncTop::xGetBufferOrg( TComPicYuv*& rpcPicYuvRec)
// tempPtr will receive a copy of ptr
//Void TAppEncTop::xGetBufferOrg( TComPicYuv* rpcPicYuvRec)
Void TAppEncTop::xGetBufferOrg( TComPicYuv*& rpcPicYuvRec)
{
    assert( m_iGOPSize > 0 );
    
    // org. buffer
    if ( m_cListPicYuvOrg.size() >= (UInt)m_iGOPSize ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
    {
        // 
//        cout << "xGetBufferOrg: Monna: Pop front " << endl; // ringer buffer = 4, be yelaf-fhom
        rpcPicYuvRec = m_cListPicYuvOrg.popFront();
        
    }
    else
    {
        rpcPicYuvRec = new TComPicYuv;
        
        rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        
    }
    m_cListPicYuvOrg.pushBack( rpcPicYuvRec );
}


Void TAppEncTop::xGetBufferOrgP( TComPicYuv*& rpcPicYuvOrg)
{
    assert( m_iGOPSize > 0 );
    
    // If I am greater than the size of the P, I have to pop the Front
    // org. buffer
    if ( m_cListPicYuvOrg.size() >= g_pFramesSize + 1 ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
    {
        //
//        cout << "xGetBufferOrg: Monna: Pop front P-size " << g_pFramesSize << endl; // ringer buffer = 4, be yelaf-fhom
//        cout << "pop front before size " << m_cListPicYuvOrg.size() << endl;
        rpcPicYuvOrg = m_cListPicYuvOrg.popFront();
        
//        cout << "pop front after size " << m_cListPicYuvOrg.size() << endl;
    }
    else{
        
        rpcPicYuvOrg = new TComPicYuv;
        // allocate original YUV buffer for a single frame -- Creation
        if( m_isField )// If it's true then, it is a YUV -- Field
        {
           rpcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        }
        else
        {
            rpcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        }

    }
    
    m_cListPicYuvOrg.pushBack( rpcPicYuvOrg );
}



Void TAppEncTop::xGetBufferOrgTrueP (TComPicYuv*& rpcPicYuvOrg)
{
    assert( m_iGOPSize > 0 );
    
    // If I am greater than the size of the P, I have to pop the Front
    // org. buffer
    if ( m_cListPicYuvOrgTrue.size() >= g_pFramesSize + 1 ) // buffer will be 1 element longer when using field coding, to maintain first field whilst processing second.
    {
        //
        cout << "xGetBufferOrgTrue: Monna: Pop front P-size " << g_pFramesSize << endl; // ringer buffer = 4, be yelaf-fhom
        rpcPicYuvOrg = m_cListPicYuvOrgTrue.popFront();
    }
    else{
        
        
        rpcPicYuvOrg = new TComPicYuv;
        // allocate original YUV buffer for a single frame -- Creation
        if( m_isField )// If it's true then, it is a YUV -- Field
        {
            rpcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        }
        else
        {
            rpcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxCUDepth );
        }
        
    }
    
    m_cListPicYuvOrgTrue.pushBack( rpcPicYuvOrg );
}


Void TAppEncTop::xDeleteBuffer( )
{
  TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvRec.begin();

  Int iSize = Int( m_cListPicYuvRec.size() );

  for ( Int i = 0; i < iSize; i++ )
  {
    TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
    pcPicYuvRec->destroy();
    delete pcPicYuvRec; pcPicYuvRec = NULL;
  }

}

// Hossam: Scene Change
Void TAppEncTop::xDeleteBufferOrg( )
{
    TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvOrg.begin();
    
    Int iSize = Int( m_cListPicYuvOrg.size() );
    
    for ( Int i = 0; i < iSize; i++ )
    {
        TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
        pcPicYuvRec->destroy();
        delete pcPicYuvRec; pcPicYuvRec = NULL;
    }
    
}

// Hossam: Scene Change delete the true buffer
Void TAppEncTop::xDeleteBufferOrgTrue( )
{
    TComList<TComPicYuv*>::iterator iterPicYuvRec  = m_cListPicYuvOrgTrue.begin();
    
    Int iSize = Int( m_cListPicYuvOrgTrue.size() );
    
    for ( Int i = 0; i < iSize; i++ )
    {
        TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
        pcPicYuvRec->destroy();
        delete pcPicYuvRec; pcPicYuvRec = NULL;
    }
    
}

/** \param iNumEncoded  number of encoded frames
 */
Void TAppEncTop::xWriteOutput(std::ostream& bitstreamFile, Int iNumEncoded, const std::list<AccessUnit>& accessUnits)
{
  const InputColourSpaceConversion ipCSC = (!m_outputInternalColourSpace) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  if (m_isField)
  {
    //Reinterlace fields
    Int i;
    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded/2; i++ )
    {
      TComPicYuv*  pcPicYuvRecTop  = *(iterPicYuvRec++);
      TComPicYuv*  pcPicYuvRecBottom  = *(iterPicYuvRec++);

      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRecTop, pcPicYuvRecBottom, ipCSC, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom, NUM_CHROMA_FORMAT, m_isTopFieldFirst );
      }

      const AccessUnit& auTop = *(iterBitstream++);
      const vector<UInt>& statsTop = writeAnnexB(bitstreamFile, auTop);
      rateStatsAccum(auTop, statsTop);

      const AccessUnit& auBottom = *(iterBitstream++);
      const vector<UInt>& statsBottom = writeAnnexB(bitstreamFile, auBottom);
      rateStatsAccum(auBottom, statsBottom);
    }
  }
  else
  {
    Int i;

    TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();

    for ( i = 0; i < iNumEncoded; i++ )
    {
      --iterPicYuvRec;
    }

    for ( i = 0; i < iNumEncoded; i++ )
    {
      TComPicYuv*  pcPicYuvRec  = *(iterPicYuvRec++);
      if (m_pchReconFile)
      {
        m_cTVideoIOYuvReconFile.write( pcPicYuvRec, ipCSC, m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
      }

      const AccessUnit& au = *(iterBitstream++);
      const vector<UInt>& stats = writeAnnexB(bitstreamFile, au);
      rateStatsAccum(au, stats);
    }
  }
}

/**
 *
 */
Void TAppEncTop::rateStatsAccum(const AccessUnit& au, const std::vector<UInt>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<UInt>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL_R:
    case NAL_UNIT_CODED_SLICE_TRAIL_N:
    case NAL_UNIT_CODED_SLICE_TSA_R:
    case NAL_UNIT_CODED_SLICE_TSA_N:
    case NAL_UNIT_CODED_SLICE_STSA_R:
    case NAL_UNIT_CODED_SLICE_STSA_N:
    case NAL_UNIT_CODED_SLICE_BLA_W_LP:
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
    case NAL_UNIT_CODED_SLICE_BLA_N_LP:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_RADL_N:
    case NAL_UNIT_CODED_SLICE_RADL_R:
    case NAL_UNIT_CODED_SLICE_RASL_N:
    case NAL_UNIT_CODED_SLICE_RASL_R:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

Void TAppEncTop::printRateSummary()
{
  Double time = (Double) m_iFrameRcvd / m_iFrameRate;
  printf("Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
#if VERBOSE_RATE
  printf("Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time);
#endif
}

Void TAppEncTop::printChromaFormat()
{
  std::cout << std::setw(43) << "Input ChromaFormatIDC = ";
  switch (m_InputChromaFormatIDC)
  {
  case CHROMA_400:  std::cout << "  4:0:0"; break;
  case CHROMA_420:  std::cout << "  4:2:0"; break;
  case CHROMA_422:  std::cout << "  4:2:2"; break;
  case CHROMA_444:  std::cout << "  4:4:4"; break;
  default:
    std::cerr << "Invalid";
    exit(1);
  }
  std::cout << std::endl;

  std::cout << std::setw(43) << "Output (internal) ChromaFormatIDC = ";
  switch (m_cTEncTop.getChromaFormatIdc())
  {
  case CHROMA_400:  std::cout << "  4:0:0"; break;
  case CHROMA_420:  std::cout << "  4:2:0"; break;
  case CHROMA_422:  std::cout << "  4:2:2"; break;
  case CHROMA_444:  std::cout << "  4:4:4"; break;
  default:
    std::cerr << "Invalid";
    exit(1);
  }
  std::cout << "\n" << std::endl;
}

//! \}
