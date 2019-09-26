////////// Adjust/Adapt Lambda according to Epsilon here!////////////////////////////////////
// read the file
cout << "\nReading the Epsilon file" << endl;
string fileName2 = "";
std::ostringstream oss2;
//    oss2 << "Gen//Seq-TXT//_eps_" << g_input_FileName << "_MSE" << g_qpInit  << ".txt";
oss2 << "Gen//Seq-TXT//_eps_" << "BasketballPass" << "_MSE" << g_qpInit  << ".txt";
fileName2 = oss2.str();
std::ifstream infile(fileName2);

// Read the MSE file for weighted lambda
oss2.clear(); oss2.str("");
oss2 << "Gen//Seq-TXT//" << g_input_FileName << "_MSE" << g_qpInit  << ".txt";
string fileName3 = oss2.str();
std::ifstream msefile(fileName3);

cout << "Read " << fileName2 << " & " << fileName3 << endl;

Double epsilon = 0;
std::string input;
Double sum_top = 0;
Double sum_bottom = 0;
Double D_i = 0;

// Skip N lines
//    for(UInt eps_i = 0; eps_i < rpcSlice->getPOC(); eps_i++)
//    {
//        std::getline(infile, input);
//    }
//
//    std::getline(infile, input);
//    // Read epsilon of i
//    epsilon = atof(input.c_str());

// Weighted
// Skipe N lines
for(UInt eps_i = 0; eps_i <= rpcSlice->getPOC(); eps_i++)
{
    std::getline(infile, input);
    // Read epsilon of i
    epsilon = atof(input.c_str());
    
    // Read Distortion of i
    if(rpcSlice->getPOC() != 0)
    {
        std::getline(msefile, input);
        D_i = atof(input.c_str());
        sum_top += D_i;
        sum_bottom += D_i * epsilon;
    }
    
}
if(rpcSlice->getPOC() != 0)
{
    epsilon = sum_top/sum_bottom;
}

// Weighted

// Produce the new lambda
cout << "dLambda before: " << dLambda << ", " << " dLambda after: " << (dLambda / epsilon) << endl;
dLambda = dLambda / epsilon;



==================================================================
****with weighted based on the previous similar distortions... tiny improvement.. bad at low bitrate


// read the file
cout << "\nReading the Epsilon file" << endl;
string fileName2 = "";
std::ostringstream oss2;
oss2 << "Gen//Seq-TXT//_eps_" << g_input_FileName << "_MSE" << g_qpInit  << ".txt";
fileName2 = oss2.str();
//    std::ifstream epsfile(fileName2);

// Read the MSE file for weighted lambda
oss2.clear(); oss2.str("");
oss2 << "Gen//Seq-TXT//" << g_input_FileName << "_MSE" << g_qpInit  << ".txt";
string fileName3 = oss2.str();
std::ifstream msefile(fileName3);

cout << "Read " << fileName2 << " & " << fileName3 << endl;

Double epsilon = 1;
std::string input;
Double sum_top = 0;
Double sum_bottom = 0;
Double D_i = 0;

Double sum_Di1 = 0;
Double sum_Di2 = 0;
Double sum_sigma = 0;
std::string input2;
Double sigma_i = 0;

if(rpcSlice->getPOC() > 11)
{
    cout << "\nReading the Sigma file" << endl;
    string fileName3 = "";
    std::ostringstream oss3;
    oss3 << "Gen//Seq-TXT//" << g_input_FileName << "_sigma" << g_qpInit  << ".txt";
    fileName3 = oss3.str();
    std::ifstream sigmafile(fileName3);
    
    
    UInt GOP_size = 4;
    UInt rpoc = rpcSlice->getPOC() % GOP_size;
    
    for(UInt eps_i = 0; eps_i < rpcSlice->getPOC(); eps_i++)
    {
        
        std::getline(msefile, input);
        std::getline(sigmafile, input2);
        D_i = atof(input.c_str());
        sigma_i = atof(input2.c_str());
        
        if (eps_i % GOP_size == rpoc) {
            sum_Di1 += D_i;
            
            //                cout << eps_i << "-D_i1111: " << D_i << endl;
            
        }
        
        if (eps_i % GOP_size == rpoc + 1) {
            sum_Di2 += D_i;
            sum_sigma += sigma_i;
            //                cout << eps_i << "-D_i222: " << D_i << ", " << sigma_i << endl;
        }
    }
    
    UInt count = floor(rpcSlice->getPOC()/GOP_size);
    sum_top = sum_Di2 / count;
    sum_bottom = (sum_Di1 + sum_sigma) / count;
    
    
    epsilon = 1 + (sum_top / sum_bottom);
    
    cout << "Epsilon: " << epsilon  << endl;
    
}


////////////// Change Delta QP

// Change Delta QP
// read the file
string fileName2 = "";
std::ostringstream oss2;

// Read the MSE file for weighted lambda
oss2 << "Gen//Seq-TXT//" << g_input_FileName << "_MSE" << g_qpInit  << ".txt";
string fileName3 = oss2.str();
std::ifstream msefile(fileName3);

Double epsilon = 1;
std::string input;
Double sum_top = 0;
Double sum_bottom = 0;
Double D_i = 0;

Double sum_Di1 = 0;
Double sum_Di2 = 0;
Double sum_sigma = 0;
std::string input2;
Double sigma_i = 0;

if(rpcSlice->getPOC() > 11)
{
    cout << "\nReading the Sigma file" << endl;
    string fileName3 = "";
    std::ostringstream oss3;
    oss3 << "Gen//Seq-TXT//" << g_input_FileName << "_sigma" << g_qpInit  << ".txt";
    fileName3 = oss3.str();
    std::ifstream sigmafile(fileName3);
    
    
    UInt GOP_size = 4;
    UInt rpoc = rpcSlice->getPOC() % GOP_size;
    
    // Calculate Epsilon_zero
    Double epsilon_zero = 0;
    Double D_i1_zero = 0;
    Double D_i2_zero = 0;
    Double sigma_zero = 0;
    
    for(UInt eps_i = 0; eps_i < rpcSlice->getPOC(); eps_i++)
    {
        std::getline(msefile, input);
        std::getline(sigmafile, input2);
        D_i = atof(input.c_str());
        sigma_i = atof(input2.c_str());
        
        if(eps_i == 0)
        {
            D_i1_zero = D_i;
        }
        
        if(eps_i == 1)
        {
            D_i2_zero = D_i;
            sigma_zero = sigma_i;
            // Epislon_zero
            epsilon_zero = 1 + (D_i2_zero)/ (D_i1_zero + sigma_zero);
        }
        
        
        if (eps_i != 0 && eps_i % GOP_size == rpoc) {
            sum_Di1 += D_i;
        }
        
        if (eps_i != 0 && eps_i % GOP_size == rpoc + 1) {
            sum_Di2 += D_i;
            sum_sigma += sigma_i;
            //                cout << eps_i << "-D_i222: " << D_i << ", " << sigma_i << endl;
        }
    }
    
    UInt count = floor(rpcSlice->getPOC()/GOP_size);
    sum_top = sum_Di2 / count;
    sum_bottom = (sum_Di1 + sum_sigma) / count;
    
    
    epsilon = 1 + (sum_top / sum_bottom);
    
    cout << "Epsilon: " << epsilon   << ", EpsZero: " << epsilon_zero << endl;
    Double offset = (3*log2(epsilon_zero * QpFact[0]/(epsilon*dQPFactor)));
    cout << "Frame offset: " << int(round(offset)) << endl;
    
    dQP = g_qpInit + int(round(offset)) + Qpoff[GOP_size - 1 - rpcSlice->getPOC() % GOP_size];
    
}


//////////////////// Lagrangian multiplier adaption.

/// Trying to implement the original lagrangian mulitplier adaption
Double epsilon = 1;
std::string input;
Double sigma_i = 0;
Double sum_sigma_i = 0;

if(rpcSlice->getPOC() > 11)
{
    cout << "\nReading the Sigma file" << endl;
    string fileName3 = "";
    std::ostringstream oss3;
    oss3 << "Gen//Seq-TXT//" << g_input_FileName << "_sigma" << g_qpInit  << ".txt";
    fileName3 = oss3.str();
    std::ifstream sigmafile(fileName3);
    
    // Skip until N
    for(UInt eps_i = 0; eps_i < rpcSlice->getPOC(); eps_i++)
    {
        std::getline(sigmafile, input);
        sum_sigma_i += atof(input.c_str());
    }
    
    std::getline(sigmafile, input);
    sigma_i = atof(input.c_str());
    
    sum_sigma_i += atof(input.c_str());
    Double average_sigma_i = sum_sigma_i / (rpcSlice->getPOC() + 1);
    
    //        epsilon = 1 - ( ( dLambda / ( 2 * log(2) * sigma_i ) ) );
    Double x =  dLambda / (2 * log(2));
    Double y = (1 / sigma_i) - (1 / average_sigma_i);
    
    epsilon  = 1 + ( x * y );
    Double phi = 1 / epsilon;
    
    if(epsilon == 0)
    {
        phi = 1;
    }
    
    if (phi < 0.5)
    {
        phi  = 0.5;
    }
    
    if(phi > 2)
    {
        phi = 2;
    }
    
    //        cout << "Old Adaption factor: " << ( ( dLambda / ( 2 * log(2) * sigma_i ) ) ) << endl;
    //        cout << "Sigma_i " << sigma_i << ", average_sigma_i: " << average_sigma_i << endl;
    cout << "x " << x << ", y: " << y << ", x*y: " << (x*y) << ", " << (1 + x*y) << endl;
    cout << "Epsilon: " << epsilon << ", averageOrg: " << average_sigma_i << ", sigma_i: "  << sigma_i << endl;
    cout << "dLambda before: " << dLambda << ", " << " dLambda after: " << (dLambda / epsilon) << endl;
    //        dLambda = dLambda / epsilon;
    dLambda = dLambda * phi;
    
}


///////////// Save the encodeNewP working original defualt
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
    TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
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
    
    TComPicYuv cPicYuvTrueOrg;
    
    // Dady: The difference between the two cases is that one of them is creating with the original source dimensions, the other with the normal source dimensions
    // m_iSourceHeight                  :                source height in pixel (when interlaced = field height)
    // m_iSourceHeightOrg;              :                original source height in pixel (when interlaced = frame height)
    // pcPicYUVTrueOrg                  :                buffer where he stores the YUV components
    
    
    // Hossam: Create a list of picYuvTrueOrg
    TComList<TComPicYuv> cPicYuvTrueOrgBuffer;
    
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
    
    // Hossam: index for debugging reading frames
    Int read_idx = 0;
    
    //Hossam:  it reads frame by frame
    while ( !bEos )
    {
        // get buffers
        xGetBuffer(pcPicYuvRec);
        
        
        cout << "Read Frame " << read_idx++ << endl;
        // Dady: m_InputChromaFormatIDC: Which subsampling I am using 4:0:0? 4:4:4? It is there for high profiles
        // read input YUV file
        // Hossam: pcPicYuvOrg is created and will be filled here
        //        m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, &cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );
        
        
        
        // Hossam: Scene change Get Buffers by passing the original data
        // Passing addresses by reference
        // Now by value
        // Hossam: Change it to match exactly the reconstructed
        //        for (Int h = 0; h < 5; h++)
        //        {
        m_cTVideoIOYuvInputFile.read( pcPicYuvOrg, &cPicYuvTrueOrg, ipCSC, m_aiPad, m_InputChromaFormatIDC );
        cout << "Push the Org picture "  << m_cListPicYuvOrg.size() << endl;
        xGetBufferOrgP(pcPicYuvOrg);
        cout << "Front of Org picture " << m_cListPicYuvOrg.front() << endl;
        
        
        // increase number of received frames
        m_iFrameRcvd++;
        //        } //end for
        
        
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
        
        
        
        // call encoding function for one frame (Hossam: 2nd call only)
        if ( m_isField ){
            cout << "ENCODE GHADA MENNA MAYAR (1) " << endl;
            m_cTEncTop.encode( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded, m_isTopFieldFirst );
        } // end if
        // Hossam: m_cListPicYuvRec: List of reconstruction YUV pictures
        // Hossam: m_cListPicYuvOrg: List of original YUV pictures
        // Hossam: Scene change
        else
        {
            cout << "ENCODE GHADA MENNA MAYAR (2)" << endl;
            
            
            //            for (Int h = 0; h < 5; h++)
            //            {
            //                m_cTEncTop.encodeNew( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
            //                xGetBuffer(pcPicYuvRec);
            
            
            m_cTEncTop.encodeNew( bEos, m_cListPicYuvOrg.popFront(), &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
            
            
            
            
            //            } // end for
            
            //            m_cTEncTop.encodeNew( bEos, flush ? 0 : pcPicYuvOrg, flush ? 0 : &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, m_cListPicYuvOrg, outputAccessUnits, iNumEncoded );
        } // end else
        
        
        // write bistream to file if necessary
        if ( iNumEncoded > 0)
        {
            cout << "xWriteOutput " << read_idx << endl;
            xWriteOutput(bitstreamFile, iNumEncoded, outputAccessUnits);
            outputAccessUnits.clear();
        }
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


////////////////// INIT QP ALGORITHM ////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

if(rpcSlice -> getPOC() == 6 || rpcSlice -> getPOC() == 7)
{
    switch (g_qpInit) {
        case 22:
            //                dQP = g_qpInit + 2;
            //                dQPFactor = g_qpFact;
            break;
        case 27:
            //                dQP = g_qpInit + 7;
            dQP = g_qpInit + 2;
            dQPFactor = 0.20;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 32:
            
            dQP = g_qpInit + 2;
            dQPFactor = 0.12;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 37:
            if (rpcSlice ->getPOC() == 6 ) {
                dQP = g_qpInit - 1;
                dQPFactor = g_qpFact;
                
            }
            else{
                dQP = g_qpInit + 3;
                dQPFactor = 0.11;
                
            }
            
            //                dQPFactor = 0.11;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        default:
            break;
    }// end switch
    
    // Set the QP factor
    //                dQPFactor = g_qpFact;
    
} // end if
else
if(rpcSlice -> getPOC() == 5)
{
    switch (g_qpInit) {
        case 22:
            dQP = g_qpInit + 3;
            dQPFactor = 0.60;
            break;
        case 27:
            //                dQP = g_qpInit + 7;
            dQP = g_qpInit + 3;
            dQPFactor = 0.40;
            //                dQPFactor = 0.20;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 32:
            
            dQP = g_qpInit + 2;
            dQPFactor = 0.20;
            //                dQPFactor = 0.12;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 37:
            dQP = g_qpInit + 2;
            dQPFactor = 0.20;
            //                dQPFactor = 0.11;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        default:
            break;
    }// end switch
    
    // Set the QP factor
    //        dQPFactor = g_qpFact;
} // end if
else
if (rpcSlice->getPOC () == 0) {
    switch (g_qpInit) {
        case 22:
            dQP = g_qpInit;
            break;
        case 27:
            //                dQP = g_qpInit - 2;
            dQP = g_qpInit - 1;
            break;
        case 32:
            
            dQP = g_qpInit - 3;
            break;
        case 37:
            
            dQP = g_qpInit - 3;
            break;
        default:
            break;
    }
}
else if(rpcSlice -> getPOC() % 4 == 0)
{
    switch (g_qpInit) {
        case 22:
            
            dQP = g_qpInit + 5;
            dQPFactor = 0.67;
            break;
        case 27:
            
            dQP = g_qpInit + 5;
            dQPFactor = 0.60;
            break;
        case 32:
            
            dQP = g_qpInit + 1;
            dQPFactor = 0.79;
            break;
        case 37:
            
            dQP = g_qpInit + 1;
            dQPFactor = 0.78;
            break;
        default:
            break;
            // 0.578
            //                dQPFactor = 0.578;
            //                dQPFactor = 0.579;
            //                dQPFactor = 0.580; // -0.007 dB, 0.11%
            //                dQPFactor = 0.581; //
            
            
            
    }// end switch
    
    
    
    //         0.578
    //        dQPFactor = g_qpFact;
    
}
else if(rpcSlice -> getPOC() % 4 == 1)
{
    switch (g_qpInit) {
        case 22:
            //                dQP = g_qpInit + 1;
            break;
        case 27:
            //                dQP = g_qpInit + 7;
            dQP = g_qpInit + 3;
            dQPFactor = 0.20;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 32:
            
            dQP = g_qpInit + 3;
            dQPFactor = 0.12;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 37:
            dQP = g_qpInit + 3;
            dQPFactor = 0.11;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        default:
            break;
    }// end switch
    
    // Set the QP factor
    //        dQPFactor = g_qpFact;
} // end if
else if(rpcSlice -> getPOC() % 4 == 2)
{
    switch (g_qpInit) {
        case 22:
            //                dQP = g_qpInit + 2;
            //                dQPFactor = g_qpFact;
            break;
        case 27:
            //                dQP = g_qpInit + 7;
            dQP = g_qpInit + 2;
            dQPFactor = 0.20;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 32:
            
            dQP = g_qpInit + 2;
            dQPFactor = 0.12;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 37:
            dQP = g_qpInit + 2;
            dQPFactor = 0.11;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        default:
            break;
    }// end switch
    
    // Set the QP factor
    //        dQPFactor = g_qpFact;
} // end if
else if(rpcSlice -> getPOC() % 4 == 3)
{
    switch (g_qpInit) {
        case 22:
            //                dQP = g_qpInit + 3;
            //                dQPFactor = g_qpFact;
            
            break;
        case 27:
            dQP = g_qpInit + 3;
            dQPFactor = 0.20;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 32:
            
            dQP = g_qpInit + 3;
            dQPFactor = 0.12;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        case 37:
            dQP = g_qpInit + 2;
            dQPFactor = 0.11;
            cout << "Qp factor: " << dQPFactor << endl;
            break;
        default:
            break;
    }// end switch
    
    // Set the QP factor
    //        dQPFactor = g_qpFact;
} // end if

// ***COMMENT: Change the predicted and reference frame
// Hossam: test the predicted and reference frame
//    cout << "Changing the code to hardcoded predicted and reference frame effect [P-P case] initSliceNew " << endl;
//    UInt pred_poc = 16;
//    UInt start_poc = 14;
//    UInt end_poc = 16;
//    if(rpcSlice->getPOC() == pred_poc)
//    {
//        dQP = 32;
//    }
////    else if(rpcSlice->getPOC() == (pred_poc+start_poc)/2)
//    else if(rpcSlice->getPOC() == (start_poc+end_poc)/2)
//    {
//        dQP = g_qpRef;
//    }

// Change Delta QP
// Empty

///

//    cout << rpcSlice->getPOC() << ") lambda Cathy: TEncSlice: qpFact, qp, initQP: " << dQPFactor << ", " << dQP << ", " << g_qpInit << ", type: " << rpcSlice->getSliceType() << endl;



////////////////// Approaches for Epsilon
// set the no_frames-1 to 1
//    epsilon_array[progress_in_sliding_window-1] = 1;
epsilon_array[progress_in_sliding_window-1] = 1;

cout << "epsilon of frame-" << (progress_in_sliding_window-1) << ", is " << epsilon_array.at(progress_in_sliding_window-1) << endl;
for (Int i = (progress_in_sliding_window - 2); i >= 0; i--) {
    epsilon_array[i] = 1 + epsilon_array.at(i+1) * mu_ij.at(i);
    
    cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
}

cout << "....ratio all Set ratio between eps 1 and 0.. " << endl;
// set the no_frames-1 to 1
// get epsilon 1
Double fact_ratio = 1.92;
Double fact_ratio2 = 1.526;
Double fact_ratio4 = 1.2;
epsilon_array[1] = (1 + mu_ij.at(0)) / (fact_ratio - 1);
epsilon_array[0] = fact_ratio*epsilon_array[1];

epsilon_array[2] = epsilon_array[0]/fact_ratio2;
epsilon_array[3] = epsilon_array[0]/fact_ratio;
epsilon_array[4] = epsilon_array[0]/fact_ratio4;


cout << "ratio 0 1 " << epsilon_array[0]/epsilon_array[1] << endl;
cout << "ratio 0 2 " << epsilon_array[0]/epsilon_array[2] << endl;
cout << "ratio 0 3 " << epsilon_array[0]/epsilon_array[3] << endl;
cout << "ratio 0 4 " << epsilon_array[0]/epsilon_array[4] << endl;
cout << "\n done ratios" << endl;

cout << "epsilon of frame-" << 0 << ", is " << epsilon_array.at(0) << endl;
cout << "epsilon of frame-" << 1 << ", is " << epsilon_array.at(1) << endl;
cout << "epsilon of frame-" << 2 << ", is " << epsilon_array.at(2) << endl;
cout << "epsilon of frame-" << 3 << ", is " << epsilon_array.at(3) << endl;
cout << "epsilon of frame-" << 4 << ", is " << epsilon_array.at(4) << endl;
for (Int i = 5; i <= (progress_in_sliding_window - 2); i++) {
    epsilon_array[i] = (epsilon_array[i-1] - 1)/(mu_ij.at(i-1));
    cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
}

cout << "ratio 0 i+1 " << epsilon_array[0]/epsilon_array[4] << endl;


cout << "....(2222) first two Set ratio between eps 1 and 0.. " << endl;
// set the no_frames-1 to 1
// get epsilon 1
epsilon_array[1] = (1 + mu_ij.at(0)) / (fact_ratio - 1);
epsilon_array[0] = fact_ratio*epsilon_array[1];

cout << "epsilon of frame-" << 0 << ", is " << epsilon_array.at(0) << endl;
cout << "epsilon of frame-" << 1 << ", is " << epsilon_array.at(1) << endl;

for (Int i = 2; i <= (progress_in_sliding_window - 2); i++) {
    epsilon_array[i] = (epsilon_array[i-1] - 1)/(mu_ij.at(i-1));
    cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
}

cout << "ratio 0 1 " << epsilon_array[0]/epsilon_array[1] << endl;
cout << "ratio 0 2 " << epsilon_array[0]/epsilon_array[2] << endl;
cout << "ratio 0 3 " << epsilon_array[0]/epsilon_array[3] << endl;
cout << "ratio 0 4 " << epsilon_array[0]/epsilon_array[4] << endl;
cout << "\n done ratios" << endl;


cout << "....(33333) last two Set ratio between eps 1 and 0.. " << endl;
// set the no_frames-1 to 1
// get epsilon 1
epsilon_array[progress_in_sliding_window-1] = (1 + mu_ij.at(progress_in_sliding_window-2)) / (fact_ratio - 1);
epsilon_array[progress_in_sliding_window-2] = fact_ratio*epsilon_array[progress_in_sliding_window-1];

cout << "epsilon of frame-" << progress_in_sliding_window-1 << ", is " << epsilon_array.at(progress_in_sliding_window-1) << endl;
cout << "epsilon of frame-" << progress_in_sliding_window-2 << ", is " << epsilon_array.at(progress_in_sliding_window-2) << endl;

for (Int i = progress_in_sliding_window-3; i >= 0; i--) {
    epsilon_array[i] = 1 + epsilon_array.at(i+1) * mu_ij.at(i);
    cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
}

cout << "ratio 0 1 " << epsilon_array[0]/epsilon_array[1] << endl;
cout << "ratio 0 2 " << epsilon_array[0]/epsilon_array[2] << endl;
cout << "ratio 0 3 " << epsilon_array[0]/epsilon_array[3] << endl;
cout << "ratio 0 4 " << epsilon_array[0]/epsilon_array[4] << endl;
cout << "\n done ratios" << endl;

cout << "epsilon of frame-" << 0 << ", is " << epsilon_array.at(0) << endl;
cout << "epsilon of frame-" << 1 << ", is " << epsilon_array.at(1) << endl;
cout << "epsilon of frame-" << 2 << ", is " << epsilon_array.at(2) << endl;
cout << "epsilon of frame-" << 3 << ", is " << epsilon_array.at(3) << endl;
cout << "epsilon of frame-" << 4 << ", is " << epsilon_array.at(4) << endl;


//    cout << "..Set last to zero.... " << endl;
//    // set the no_frames-1 to 1
//    epsilon_array[progress_in_sliding_window-1] = 0;
//
//    cout << "epsilon of frame-" << (progress_in_sliding_window-1) << ", is " << epsilon_array.at(progress_in_sliding_window-1) << endl;
//    for (Int i = (progress_in_sliding_window - 2); i >= 0; i--) {
//        epsilon_array[i] = 1 + epsilon_array.at(i+1) * mu_ij.at(i);
//
//        cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
//    }
//
//
//    cout << "...Set last to 2... " << endl;
//    // set the no_frames-1 to 1
//    epsilon_array[progress_in_sliding_window-1] = 2;
//
//    cout << "epsilon of frame-" << (progress_in_sliding_window-1) << ", is " << epsilon_array.at(progress_in_sliding_window-1) << endl;
//    for (Int i = (progress_in_sliding_window - 2); i >= 0; i--) {
//        epsilon_array[i] = 1 + epsilon_array.at(i+1) * mu_ij.at(i);
//
//        cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
//    }


//    cout << "....Set first to 2.. " << endl;
//    // set the no_frames-1 to 1
//    epsilon_array[0] = 4;
//
//    cout << "epsilon of frame-" << 0 << ", is " << epsilon_array.at(0) << endl;
//    for (Int i = 1; i <= (progress_in_sliding_window - 2); i++) {
////    for (Int i = (progress_in_sliding_window - 2); i >= 0; i--) {
//        epsilon_array[i] = (epsilon_array[i-1] - 1)/(mu_ij.at(i-1));
//
//        cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
//    }

cout << "....ratio all Set ratio between eps 1 and 0.. " << endl;
// set the no_frames-1 to 1
// get epsilon 1
Double fact_ratio = 1.92;
Double fact_ratio2 = 1.526;
Double fact_ratio4 = 1.2;

static bool fst_time = true;
if(fst_time) {
    epsilon_array[1] = (1 + mu_ij.at(0)) / (fact_ratio - 1);
    fst_time = false;
}
epsilon_array[0] = fact_ratio*epsilon_array[1];

epsilon_array[2] = epsilon_array[0]/fact_ratio2;
epsilon_array[3] = epsilon_array[0]/fact_ratio;
epsilon_array[4] = epsilon_array[0]/fact_ratio4;


cout << "ratio 0 1 " << epsilon_array[0]/epsilon_array[1] << endl;
cout << "ratio 0 2 " << epsilon_array[0]/epsilon_array[2] << endl;
cout << "ratio 0 3 " << epsilon_array[0]/epsilon_array[3] << endl;
cout << "ratio 0 4 " << epsilon_array[0]/epsilon_array[4] << endl;
cout << "\n done ratios" << endl;

cout << "epsilon of frame-" << 0 << ", is " << epsilon_array.at(0) << endl;
cout << "epsilon of frame-" << 1 << ", is " << epsilon_array.at(1) << endl;
cout << "epsilon of frame-" << 2 << ", is " << epsilon_array.at(2) << endl;
cout << "epsilon of frame-" << 3 << ", is " << epsilon_array.at(3) << endl;
cout << "epsilon of frame-" << 4 << ", is " << epsilon_array.at(4) << endl;
for (Int i = 5; i <= (progress_in_sliding_window - 2); i++) {
    epsilon_array[i] = (epsilon_array[i-1] - 1)/(mu_ij.at(i-1));
    cout << "epsilon of frame-" << i << ", is " << epsilon_array.at(i) << endl;
}

cout << "ratio 0 i+1 " << epsilon_array[0]/epsilon_array[4] << endl;


/// Last trial for epsilon
//    cout << curr_poc << ") start_loop_from " << start_loop_from << endl;

// Calculate mu(i, j) for every pair i, j till the last one you have
//    for(UInt i = start_loop_from; i < progress_in_sliding_window; i++)

for(UInt i = 1; i < progress_in_sliding_window; i++)
{
    // mu_pred_ref = Dpred / (Dref + sigma^2)
    Double curr_mu_i_j = distortion_in_sliding_window[i] / (distortion_in_sliding_window[i-1] + sigma_squared_in_sliding_window[i]);
    mu_ij.push_back(curr_mu_i_j);
    cout << "curr_mu_i_j for i_pred = " << i << ", j_ref = " << (i-1) << ", is: " << mu_ij.back() << endl;
}

cout << "Done calculate curr mu i, j " << endl;
// display mu_i, j
displayCodingDepList();

// Calculate Epsilon i

// Reset all the epsilon array to 0
epsilon_array.resize(progress_in_sliding_window);
std::fill(epsilon_array.begin(), epsilon_array.end(), 0);


cout << "Current POC is  " << curr_poc << endl;
cout << "Go back p frames to " << curr_poc - g_pFramesSize << endl;
Int last_frame_encoded = progress_in_sliding_window;
// For each frame in P
for (Int i = 0; i < g_pFramesSize; i++) {
    Double eps = 1;
    Double multiplier = 1;
    
    // Loop for p elements
    for(Int j = last_frame_encoded-g_pFramesSize; j < last_frame_encoded; j++)
    {
        Double mu = 0;
        if(j < mu_ij.size())
        {
            mu = mu_ij.at(j);
        }
        
        cout << j <<  ") Loop get mu " << mu << endl;
        
        eps += mu*multiplier;
        
        if(j < mu_ij.size())
        {
            multiplier = mu_ij.at(j);
        }
        else
        {
            multiplier = 1;
        }
        
    }
    cout << "Eps " << eps << endl;
    
}

cout << "&&&&&&&&&&&&" << endl;

for (Int i = 0; i <= progress_in_sliding_window-2; i++) {
    Double eps = 1;
    Double multiplier = 1;
    
    // Loop for p elements
    for(Int j = i; j < i + g_pFramesSize; j++)
    {
        Double mu = 0;
        if(j < mu_ij.size())
        {
            mu = mu_ij.at(j);
        }
        
        cout << j <<  ") Loop get mu " << mu << endl;
        
        eps += mu*multiplier;
        
        if(j < mu_ij.size())
        {
            multiplier = mu_ij.at(j);
        }
        else
        {
            multiplier = 1;
        }
        
    }
    
    epsilon_array[i] = eps;
    cout << "epsilon of frame-" << i  << ", " << i % 4 << ", is " << epsilon_array.at(i) << endl;
}





// Calculate current QP offset
m_pcSliceEncoder->calculateCurrentQPOffset(pcPic, epsilon_array, mu_ij, sigma_squared_in_sliding_window);


// empty up the mu(i, j) -- Dynamic
cout << "Empty the mu(i,j) to recalulate based on my previous QP offset choice " << endl;
mu_ij.clear();


//    exit(0);

// 4 offsets
for(Int i = 1; i < epsilon_array.size(); i++)
{
    Int qp_factor_index = curr_poc % 4 == 0? 3: curr_poc % 4 - 1;
    Double dQP_factor_for_i = QpFact[qp_factor_index];
    
    Double epsilon_i = epsilon_array.at(i);
    
    if(four_epsilons.size() >= 4)
    {
        
        cout << "xXXXXXXXX EEEEEEEE if(four_epsilons >= 4) " << endl;
        four_epsilons.at(i-1) = epsilon_i;
    }
    else{
        four_epsilons.push_back(epsilon_i);
    }
    
    
    Double up = epsilon_zero * dQP_factor_for_zero;
    Double down = epsilon_i * dQP_factor_for_i;
    
    Double deltaQP_i = 3*log2(up/down);
    
    current_qp_offset = round(deltaQP_i);
    
    if(deltaQP_i < 0)
    {
        Double pos_delta = deltaQP_i * -1;
        current_qp_offset = -1*round(pos_delta);
    }
    
    four_offsets.push_back(current_qp_offset);
    
    //        cout << "Zero_qp_fact " << dQP_factor_for_zero << " eps_zero: " << epsilon_zero << ", " << " i_qp_fact: " << dQP_factor_for_i << " eps_i: " <<  epsilon_i << endl;
    //        cout << "Zero_Up parameter: " << up << " down parameter: " << down << endl;
    //        cout << "\n(epsilon_zero/epsilon_i): " << (epsilon_zero/epsilon_i) << endl;
    //        cout << "\n ******QP offset for frame-" << (pcPic->getPOC() + i - 1) << ", " << deltaQP_i << ", " << current_qp_offset  << "\n" << endl;
    
    // calculate
    //        Double cur_lambda = lambdaArr3[qp_factor_index]/epsilon_i;
    //        cout << (pcPic->getPOC() + i - 1) << ") cur_lambda " << cur_lambda << " old lambda " << lambdaArr3[qp_factor_index] << endl;
    //        Double desired_qp = 3 * log2(cur_lambda/dQP_factor_for_i) + 12;
    //        cout << (pcPic->getPOC() + i - 1)  << ") desired_qp: " << desired_qp  << ", fourEps " << four_epsilons.front() << endl;
    
    
    // increment the current poc
    curr_poc++;
}


///// Code adjustment for the first few frames
/// QP factor
// static float QpFact[] ={0.4624, 0.4624, 0.4624, 0.578};
// 0.57, 0.68, 0.85, 0.85, 0.85
//                if(rpcSlice -> getPOC() % 4 == 0)
//                {
//                    switch (g_qpInit) {
//                        case 22:
//                            break;
//                        case 27:
//                            dQP = g_qpInit + 3;
//                            dQPFactor = 0.20;
//                            break;
//                        case 32:
//
//                            dQP = g_qpInit + 3;
//
//
//                            dQPFactor = 0.68;
//
//                            break;
//                        case 37:
//                            dQP = g_qpInit + 3;
//                            dQPFactor = 0.11;
//                            break;
//                        default:
//                            break;
//                    }// end switch
//
//                    // Set the QP factor
//                    //        dQPFactor = g_qpFact;
//                } // end if
//
//                if(rpcSlice -> getPOC() % 4 == 1)
//                {
//                    switch (g_qpInit) {
//                        case 22:
//                            break;
//                        case 27:
//                            dQP = g_qpInit + 3;
//                            dQPFactor = 0.20;
//                            break;
//                        case 32:
//
//                            dQP = g_qpInit + 3;
//
//
//                            dQPFactor = 0.85;
//                            break;
//                        case 37:
//                            dQP = g_qpInit + 3;
//                            dQPFactor = 0.11;
//                            break;
//                        default:
//                            break;
//                    }// end switch
//
//                    // Set the QP factor
//                    //        dQPFactor = g_qpFact;
//                } // end if
//                else if(rpcSlice -> getPOC() % 4 == 2)
//                {
//                    switch (g_qpInit) {
//                        case 22:
//                            break;
//                        case 27:
//                            dQP = g_qpInit + 2;
//                            dQPFactor = 0.20;
//                            break;
//                        case 32:
//                            dQP = g_qpInit + 2;
//
//
//                            dQPFactor = 0.85;
//                            break;
//                        case 37:
//                            dQP = g_qpInit + 2;
//                            dQPFactor = 0.11;
//                            break;
//                        default:
//                            break;
//                    }// end switch
//
//                } // end if
//                else if(rpcSlice -> getPOC() % 4 == 3)
//                {
//                    switch (g_qpInit) {
//                        case 22:
//                            break;
//                        case 27:
//                            dQP = g_qpInit + 3;
//                            dQPFactor = 0.20;
//                            break;
//                        case 32:
//
//                            dQP = g_qpInit + 3;
//
//
//                            dQPFactor = 0.85;
//                            break;
//                        case 37:
//                            dQP = g_qpInit + 2;
//                            dQPFactor = 0.11;
//                            break;
//                        default:
//                            break;
//                    }// end switch
//                } // end if
//                // update qp factor
//                dQP_factor_for_i = dQPFactor;


========= OLD TURN ON AND OFF
// Calculate current epsilon in the mu prediction method
Void TEncGOP::calculateCurrentEpsilonMu(TComPic *pcPic)
{
    Int curr_poc = pcPic->getPOC();
    
    //      displayDAndSigmaSlidingWindowList(progress_in_sliding_window);
    // visit the past
    //    UInt start_loop_from = progress_in_sliding_window - sliding_window_length;
    // Calculate mu(i, j)
    //    cout << "\n calculateCurrentEpsilonMu start " << endl;
    //    cout << " D list length " << distortion_in_sliding_window.size() << endl;
#if GEN_MU_FILES
    Double mult = 1;
#endif
    
    
    UInt last_frame_to_be_encoded = curr_poc + 1;
    
    
    ////////////// Checker loop
    
    cout << "\n------Before isMuMethodON: " << isMuMethodON  << endl;
    
    // initial value of end_prop_window
    UInt start_for_i = m_pcEncTop->getSceneChangeCoder()->getLastSC()==-1? 1:m_pcEncTop->getSceneChangeCoder()->getLastSC()+1;
    UInt last_frame_encoded = curr_poc;
    
    if(last_frame_encoded > start_for_i + 6)
    {
        std::vector<Double> mu_ijChecker;
        
        // e.g compare mu(8,7) with mu(4,3)
        
        cout << "**** Actual Calc == " << endl;
        cout << "actual=" << last_frame_encoded << ", mseACtual=" << distortion_in_sliding_window[last_frame_encoded]
        << ", actualRef=" << last_frame_encoded-1 << ", mseAcRef=" << distortion_in_sliding_window[last_frame_encoded-1] << endl;
        cout << "sigma of ac " << last_frame_encoded << " = "<< sigma_squared_in_sliding_window[last_frame_encoded] << endl;
        
        Double actual_mu_i_j = distortion_in_sliding_window[last_frame_encoded] / (distortion_in_sliding_window[last_frame_encoded-1] + sigma_squared_in_sliding_window[last_frame_encoded]);
        
        Double estimate_mu_i_j = distortion_in_sliding_window[last_frame_encoded-4] / (distortion_in_sliding_window[last_frame_encoded-5] + sigma_squared_in_sliding_window[last_frame_encoded-4]);
        
        //        Double estimate_mu_i_j = distortion_in_sliding_window[last_frame_encoded-4] / (distortion_in_sliding_window[last_frame_encoded-5] + sigma_squared_in_sliding_window[last_frame_encoded]);
        
        cout << "**** Pred Calc == " << endl;
        cout << "estimate=" << last_frame_encoded-4 << ", mseEst=" << distortion_in_sliding_window[last_frame_encoded-4]
        << ", estimateRef=" << last_frame_encoded-5 << ", mseEstRef=" << distortion_in_sliding_window[last_frame_encoded-5] << endl;
        cout << "sigma of ac " << last_frame_encoded-4 << " = "<< sigma_squared_in_sliding_window[last_frame_encoded-4] << endl;
        
        Double mu_distance = 100.0*fabs(estimate_mu_i_j - actual_mu_i_j)/actual_mu_i_j;
        isMuMethodON = mu_distance < MU_METHOD_ACCURACY;
        
        cout << "************Actual: " << actual_mu_i_j << ", estimated: " << estimate_mu_i_j << endl;
        cout << "************Mu distance: " << mu_distance << " isMuMethodON " << isMuMethodON  << endl;
        
    }
    
    //////////////////////
    
    if(isMuMethodON)
    {
        //    cout << "last frame to be encoded " << last_frame_to_be_encoded << endl;
        for(UInt i = last_frame_to_be_encoded - 3; i < last_frame_to_be_encoded; i++)
        {
            //         cout << "pred = " << i << " ref " << i-1 << endl;
            // mu_pred_ref = Dpred / (Dref + sigma^2)
            Double curr_mu_i_j = distortion_in_sliding_window[i] / (distortion_in_sliding_window[i-1] + sigma_squared_in_sliding_window[i]);
            mu_ij.push_back(curr_mu_i_j);
            
#if GEN_MU_FILES
            if (start_prop_window_mu == 88) {
                cout << "\nOUTPUTTING THE mu orig " << endl;
                string fileName2 = "";
                std::ostringstream oss2;
                oss2 << "Gen//Seq-TXT//" << g_input_FileName << "_mu" << g_qpInit  << ".txt";
                fileName2 = oss2.str();
                Char* pYUVFileName2 = fileName2.empty()? NULL: strdup(fileName2.c_str());
                FILE* mse_pFile2 = fopen (pYUVFileName2, "at");
                mult = mult*curr_mu_i_j;
                fprintf(mse_pFile2, "%f\t\t\t\t %f\n", curr_mu_i_j, mult);
                fclose(mse_pFile2);
            }
            
#endif
        } // end for loop
        
        // push the last item (mu(i-4, i-5))
        Double curr_mu_i_j = distortion_in_sliding_window[last_frame_to_be_encoded-4] / (distortion_in_sliding_window[last_frame_to_be_encoded-5] + sigma_squared_in_sliding_window[last_frame_to_be_encoded-4]);
        mu_ij.push_back(curr_mu_i_j);
        
        
        //    cout << "pred = " << last_frame_to_be_encoded-4 << " ref " << last_frame_to_be_encoded-5 << endl;
        
        // Calculate epsilon_i
        Int gop_sz = 4;
        
        // Reset all the epsilon array to 0 -- four epsilons you want to compute
        epsilon_array.resize(gop_sz + 1);
        std::fill(epsilon_array.begin(), epsilon_array.end(), 0);
        
        Double eps = 1.0;
        Double multiplier = 1.0;
        for(UInt j = 0; j < 4; j++)
        {
            eps += multiplier*mu_ij.at(j);
            multiplier = mu_ij.at(j);
        }
        epsilon_array[0] = eps;
        
        
        //    cout << "Empty the mu(i,j) to re-calculate based on my previous QP offset choice " << endl;
        mu_ij.clear();
    } // end if isMuMethodON
    else
    {
        epsilon_array[0] = -1;
    }
    
    
    return;
}



