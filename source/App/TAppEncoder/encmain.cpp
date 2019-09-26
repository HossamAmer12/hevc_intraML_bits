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

/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"



//! \ingroup TAppEncoder
//! \{

#if OUT_OUTLIER
ofstream OutlierYuvFile;
#endif

#if GEN_RESI_FRAME
ofstream residualYuvFile;
#endif


#include "../Lib/TLibCommon/Debug.h"

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char* argv[])
{
  TAppEncTop  cTAppEncTop;

    // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "HM software: Encoder Version [%s] (including RExt)", NV_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n\n" );

  // create application encoder class
  cTAppEncTop.create();
    
//   cout << "Yang: encmain: main: cTAppEncoderTop is created" << "\n" << endl;
//    getchar();

   /*
    for (int i = 0; i < 10; i++) {
        printf("\n 7as7as with the yang 6666667777712121\n");
    }
     */
    
    
  // parse configuration
  try
  {
    if(!cTAppEncTop.parseCfg( argc, argv ))
    {
      cTAppEncTop.destroy();
#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      EnvVar::printEnvVar();
#endif
      return 1;
    }
      
      // Hossam: GOP_STR_TYPE
     // cout << " COME ON LAST RESORT A 111 :D " << cTAppEncTop.m_inputFileName << " , asjkdnaklsdnlkasn " << cTAppEncTop.m_pchBitstreamFile;
      // one right and one wrong
  }
  catch (df::program_options_lite::ParseFailure &e)
  {
    std::cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << std::endl;
    return 1;
  }

#if RExt__PRINT_MACRO_VALUES
  printRExtMacroSettings();
#endif

#if RExt__ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
  EnvVar::printEnvVarInUse();
#endif


#if OUT_OUTLIER
    string FileYuvName = "Outlier.yuv";
    OutlierYuvFile.open(FileYuvName.c_str());;
#endif

    
#if GEN_RESI_FRAME
    string residualYuvFileName = "Residual.yuv";
    residualYuvFile.open(residualYuvFileName.c_str());
#endif
    
  // starting time
  Double dResult;
  clock_t lBefore = clock();

#if USE_ENCODE_P
//    cTAppEncTop.encode();

//    cTAppEncTop.encodeNewP05();

//  cTAppEncTop.encodeNewP();
    
    cTAppEncTop.encodeInter();
    
#else
   // call encoding function
  cTAppEncTop.encode();
#endif

  // ending time
  dResult = (Double)(clock()-lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);


#if IS_ENABLE_RECORD_TIME
    // Open a file and write the total time
    string fileNameTime = "";
    std::ostringstream ossTime;
    
    ossTime << "Gen//Seq-Time//" << "Time_all.txt";
    fileNameTime = ossTime.str();
    Char* pYUVFileNameTime = fileNameTime.empty()? NULL: strdup(fileNameTime.c_str());
    FILE*  my_pFileTime = fopen (pYUVFileNameTime, "at");
    
    string text = "";
    std::ostringstream ossText;
    ossText << g_input_FileName << "_" << g_qpInit << "\t " << dResult << "\n";
    text = ossText.str();
    fprintf(my_pFileTime, "%s", text.c_str());
    fclose(my_pFileTime);
    
#endif
    
  // destroy application encoder class
  cTAppEncTop.destroy();


#if OUT_OUTLIER
    OutlierYuvFile.close();
#endif
    
#if GEN_RESI_FRAME
    residualYuvFile.close();
#endif
    
#if GEN_YAO_SCs
    string fileName = "";
    std::ostringstream oss;
//    oss << "Gen//Seq-SCs//" << g_input_FileName << "_" << "_yao" << ".txt";
    
    oss << "Gen//Seq-SCs//" << g_input_FileName << "_" << "_yao"  << g_skipInterval << ".txt";

    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    
    FILE* yao_pFile = fopen (pYUVFileName, "at");
    fprintf(yao_pFile, "\n----------\n");
    fclose(yao_pFile);
#endif

#if GEN_DING_SCs
    string fileName = "";
    std::ostringstream oss;
//    oss << "Gen//Seq-SCs//" << g_input_FileName << "_22"  << "_ding"  << ".txt";// Everytime it's 22
//    oss << "Gen//Seq-SCs//" << g_input_FileName << "_22"  << "_ding" << g_skipInterval << ".txt";// Everytime it's 22

    
    oss << "Gen//Seq-SCs//" << g_input_FileName << "_ding" << g_skipInterval << ".txt";// Everytime it's 22

    //    oss << "Gen//Seq-SCs//" << g_input_FileName << "_"  << rpcPic->getSlice(0)->getSliceQp() << "_ding" << g_skipInterval << ".txt";

    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    
    FILE* ding_pFile = fopen (pYUVFileName, "at");
    fprintf(ding_pFile, "\n-------------\n");
    fclose(ding_pFile);
#endif
    
#if GEN_SASTRE_SCs
    string fileName = "";
    std::ostringstream oss;
//    oss << "Gen//Seq-SCs//" << g_input_FileName << "_"  << "_sastre" << ".txt";
    oss << "Gen//Seq-SCs//" << g_input_FileName << "_" << "_sastre" << g_skipInterval << ".txt";
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    
    FILE* sastre_pFile = fopen (pYUVFileName, "at");
    fprintf(sastre_pFile, "\n-------------\n");
    fclose(sastre_pFile);
#endif
    
    
#if GEN_MY_SCs
    string fileName = "";
    std::ostringstream oss;
    // Unified:
//    oss << "Gen//Seq-SCs//" << g_input_FileName << "_22" << ".txt";
    // Not Unified:
    oss << "Gen//Seq-SCs//" << g_input_FileName  << "_" << g_qpInit << ".txt";
    fileName = oss.str();
    Char* pYUVFileName = fileName.empty()? NULL: strdup(fileName.c_str());
    
    FILE* my_pFile = fopen (pYUVFileName, "at");
    fprintf(my_pFile, "\n-------------\n");
    fclose(my_pFile);
    
#endif
    
#if GEN_QPFactor_Files
    string fileName_qp = "";
    std::ostringstream oss_qp;
    oss_qp << "Gen//Seq-TXT//" << g_input_FileName << "_22" << "_sum" << ".txt";
    fileName_qp = oss_qp.str();
    Char* pYUVFileName_qp = fileName_qp.empty()? NULL: strdup(fileName_qp.c_str());
    
    FILE* my_pFile_qp = fopen (pYUVFileName_qp, "at");
    if (g_qpInit == g_qpUnderTest) {
        fprintf(my_pFile_qp, "\n-------------\n");
    }
    fclose(my_pFile_qp);
    

    
    oss_qp.clear(); oss_qp.str("");
    oss_qp << "Gen//Seq-TXT//" << g_input_FileName << "_sum" << ".txt";
    fileName_qp = oss_qp.str();
    pYUVFileName_qp = fileName_qp.empty()? NULL: strdup(fileName_qp.c_str());
    
    my_pFile_qp = fopen (pYUVFileName_qp, "at");
    
    if (g_qpInit == 37) {
        fprintf(my_pFile_qp, "\n------END %d-------\n", g_qpRef);
        fprintf(my_pFile_qp, "\n------END %f-------\n", g_qpFact);
    }
    else{
        fprintf(my_pFile_qp, "\n");
    }
    fclose(my_pFile_qp);
    

#if GEN_QPOffset_Files
    // QP offset files
    oss_qp.clear(); oss_qp.str("");
    oss_qp << "Gen//Seq-TXT//" << g_input_FileName << "_offset" << ".txt";
    fileName_qp = oss_qp.str();
    pYUVFileName_qp = fileName_qp.empty()? NULL: strdup(fileName_qp.c_str());
    my_pFile_qp = fopen (pYUVFileName_qp, "at");
    fprintf(my_pFile_qp, "\n-------------\n");
    fclose(my_pFile_qp);
#endif

    
#endif
    
    
  return 0;
}

//! \}
