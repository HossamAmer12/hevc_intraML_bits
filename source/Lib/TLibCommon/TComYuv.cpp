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

/** \file     TComYuv.cpp
    \brief    general YUV buffer class
    \todo     this should be merged with TComPicYuv
*/

#include <stdlib.h>
#include <memory.h>
#include <assert.h>
#include <math.h>

#include "CommonDef.h"
#include "TComYuv.h"
#include "TComInterpolationFilter.h"

//! \ingroup TLibCommon
//! \{

TComYuv::TComYuv()
{
  for(Int comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_apiBuf[comp] = NULL;
  }
}

TComYuv::~TComYuv()
{
}

Void TComYuv::create( UInt iWidth, UInt iHeight, ChromaFormat chromaFormatIDC )
{
  // set width and height
  m_iWidth   = iWidth;
  m_iHeight  = iHeight;
  m_chromaFormatIDC = chromaFormatIDC;

  for(Int ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    // memory allocation
    m_apiBuf[ch]  = (Pel*)xMalloc( Pel, getWidth(ComponentID(ch))*getHeight(ComponentID(ch)) );
  }
}

Void TComYuv::destroy()
{
  // memory free
  for(Int ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    if (m_apiBuf[ch]!=NULL) { xFree( m_apiBuf[ch] ); m_apiBuf[ch] = NULL; }
  }
}

Void TComYuv::clear()
{
  for(Int ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    if (m_apiBuf[ch]!=NULL)
      ::memset( m_apiBuf[ch], 0, ( getWidth(ComponentID(ch)) * getHeight(ComponentID(ch))  )*sizeof(Pel) );
  }
}

Void TComYuv::copyToPicYuv   ( TComPicYuv* pcPicYuvDst, const UInt iCuAddr, const UInt uiAbsZorderIdx, const UInt uiPartDepth, const UInt uiPartIdx ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyToPicComponent  ( ComponentID(ch), pcPicYuvDst, iCuAddr, uiAbsZorderIdx, uiPartDepth, uiPartIdx );
}

Void TComYuv::copyToPicComponent  ( const ComponentID ch, TComPicYuv* pcPicYuvDst, const UInt iCuAddr, const UInt uiAbsZorderIdx, const UInt uiPartDepth, const UInt uiPartIdx ) const
{
  const Int iWidth  = getWidth(ch) >>uiPartDepth;
  const Int iHeight = getHeight(ch)>>uiPartDepth;

//    std::cout << "tstttt 1:: " << iHeight << std::endl;
#if SC_ENABLE_PRINT
    std::cout << "Source:: " << std::endl;
#endif
    
  const Pel* pSrc     = getAddr(ch, uiPartIdx, iWidth);
//     std::cout << "tstttt 2:: " << pSrc << std::endl;

#if SC_ENABLE_PRINT
        std::cout << "Destination:: " << std::endl;
#endif
        Pel* pDst     = pcPicYuvDst->getAddr ( ch, iCuAddr, uiAbsZorderIdx );

//        std::cout << "tstttt 3:: " << pSrc << std::endl;
    
  const UInt  iSrcStride  = getStride(ch);
  const UInt  iDstStride  = pcPicYuvDst->getStride(ch);

  for ( Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}





Void TComYuv::copyFromPicYuv   ( const TComPicYuv* pcPicYuvSrc, const UInt iCuAddr, const UInt uiAbsZorderIdx )
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyFromPicComponent  ( ComponentID(ch), pcPicYuvSrc, iCuAddr, uiAbsZorderIdx );
}

Void TComYuv::copyFromPicComponent  ( const ComponentID ch, const TComPicYuv* pcPicYuvSrc, const UInt iCuAddr, const UInt uiAbsZorderIdx )
{
        Pel* pDst     = getAddr(ch);
  const Pel* pSrc     = pcPicYuvSrc->getAddr ( ch, iCuAddr, uiAbsZorderIdx );

  const UInt iDstStride  = getStride(ch);
  const UInt iSrcStride  = pcPicYuvSrc->getStride(ch);
  const Int  iWidth=getWidth(ch);
  const Int  iHeight=getHeight(ch);

  for (Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}


// Hossam: Use this if you want to print after you are done with the whole CU
Void TComYuv::mockPrintFromSlice (const UInt uiDstPartIdx)  const
{
    for(Int ch=0; ch<getNumberValidComponents(); ch++)
        mockPrintComponentSlice(ComponentID(ch), uiDstPartIdx);
}

// Hossam: Scene change
Void TComYuv::mockPrintComponentSlice ( const ComponentID ch, const UInt uiDstPartIdx) const
{
    // Get the src with Offset
    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
    
    UInt blkX = getBlckX(ch, uiDstPartIdx);
    UInt blkY = getBlckY(ch, uiDstPartIdx);
    UInt offset = getOffset(ch, blkX, blkY);
    
    const UInt iSrcStride  = getStride(ch); // 64
    
    // Print this specific block with width and height
    const Int  iWidth   = getWidth(ch);
    const Int  iHeight = getHeight(ch);
    
    
    for (Int y = 0; y < iHeight; y++ )
    {
        mockPrintComponentLineBlk(ch, uiDstPartIdx, iWidth, iHeight, true, y);
        std::cout << "\n";
        pSrc += iSrcStride;
    }
    
    
    std::cout << "--------------------------------" << std::endl;
}

// Hossam: Scene Change -- Debug copy special
Void TComYuv::mockPrint (const UInt uiDstPartIdx,  UInt srcWidth, UInt srcHeight, Bool isBlock )  const
{
    for(Int ch=0; ch<getNumberValidComponents(); ch++)
        mockPrintComponent(ComponentID(ch), uiDstPartIdx, srcWidth, srcHeight, isBlock);
}

// Hossam: Scene change
Void TComYuv::mockPrintComponent               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock ) const
{
    // Get the src with Offset
    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
    
    UInt blkX = getBlckX(ch, uiDstPartIdx);
    UInt blkY = getBlckY(ch, uiDstPartIdx);
    UInt offset = getOffset(ch, blkX, blkY);
    
    const UInt iSrcStride  = getStride(ch); // 64
    
    // Print this specific block with width and height
    const Int  iWidth   = srcWidth;
    const Int  iHeight = srcHeight;
    
    //    const Int  bufferWidth = getWidth(ch); // 64
    //    const Int  bufferHeight = getHeight(ch); // 64
    
//    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << std::endl;
    
    for (Int y = 0; y < iHeight; y++ )
    {
        mockPrintComponentLineBlk(ch, uiDstPartIdx, srcWidth, srcHeight, true, y);
        std::cout << "\n";
        pSrc += iSrcStride;
    }
    

    std::cout << "--------------------------------" << std::endl;
}


//Void TComYuv::mockPrintComponent               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock ) const
//{
//    // Get the src with Offset
//    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
//    
//    UInt blkX = getBlckX(ch, uiDstPartIdx);
//    UInt blkY = getBlckY(ch, uiDstPartIdx);
//    UInt offset = getOffset(ch, blkX, blkY);
//    
//    const UInt iSrcStride  = getStride(ch); // 64
//    
//    // Print this specific block with width and height
//    const Int  iWidth   = srcWidth;
//    const Int  iHeight = srcHeight;
//    
////    const Int  bufferWidth = getWidth(ch); // 64
////    const Int  bufferHeight = getHeight(ch); // 64
//    
//    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << std::endl;
//    
//   
//    Int moreOffset = 0;
//    Int lines = 0;
////    Int i = offset + moreOffset;
//    if(!isBlock)
//    {
//        for (Int y = iHeight-1; y >= 0; y-- )
//        {
//            Int i = offset + moreOffset;
//            
//            std::cout << "Line " << (lines++) << std::endl;
////            for (Int x = offset + moreOffset; x <= offset + moreOffset + iWidth; x++) {
//            // You already got the src with offset
//            for (Int x = 0; x < iWidth; x++) {
//                //            if (i>=offset + moreOffset && i <= offset + iWidth + moreOffset ) {
//                std::cout << i << ") " << "x: " << x << ", val: " << pSrc[x] << std::endl;
//                //            }
//                
//                //            std:: cout << "i: " << i << ", " << (offset+moreOffset) << ", to: " << (offset+iWidth + moreOffset) << std::endl;
//                i = i + 1;
//            }
//            
//            //        std::cout << "======" << std::endl;
//            pSrc += iSrcStride;
//            moreOffset += iSrcStride;
//        }
//    }
//    else
//    {
//        for (Int y = iHeight-1; y >= 0; y-- )
//        {
//            Int i = offset + moreOffset;
//            for (Int x = offset + moreOffset; x <= offset + moreOffset + iWidth; x++)
//            {
//                if (pSrc[x] >= 0)
//                    std::cout  << "+" << pSrc[x] << " ";
//                else
//                    std::cout  << pSrc[x] << " ";
//            }
//            
//                std::cout << "\n";
//                //            std:: cout << "i: " << i << ", " << (offset+moreOffset) << ", to: " << (offset+iWidth + moreOffset) << std::endl;
//                i = i + 1;
//           
//            
//            //        std::cout << "======" << std::endl;
//            pSrc += iSrcStride;
//            moreOffset += iSrcStride;
//        }
//    
//    }
//        
//    
//    std::cout << "--------------------------------" << std::endl;
//}


// Hossam: Scene change
//Void TComYuv::mockPrintComponentLine               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock, Int myline ) const
//{
//    // Get the src with Offset
//    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
//    
//    UInt blkX = getBlckX(ch, uiDstPartIdx);
//    UInt blkY = getBlckY(ch, uiDstPartIdx);
//    UInt offset = getOffset(ch, blkX, blkY);
//    
//    const UInt iSrcStride  = getStride(ch); // 64
//    
//    // Print this specific block with width and height
//    const Int  iWidth   = srcWidth;
//    const Int  iHeight = srcHeight;
//    
//    //    const Int  bufferWidth = getWidth(ch); // 64
//    //    const Int  bufferHeight = getHeight(ch); // 64
//    
//    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << std::endl;
//    
//    
//    Int moreOffset = 0;
//    Int lines = 0;
//    //    Int i = offset + moreOffset;
//    if(!isBlock)
//    {
//        for (Int y = iHeight-1; y >= 0; y-- )
//        {
//            Int i = offset + moreOffset;
//            
//            std::cout << "Line " << (lines++) << std::endl;
////            for (Int x = offset + moreOffset; x <= offset + moreOffset + iWidth; x++) {
//                //            if (i>=offset + moreOffset && i <= offset + iWidth + moreOffset ) {
//            // You already got the src with offset
//            for (Int x = 0; x < iWidth; x++) {
// 
//               if(myline == lines)
//                std::cout << i << ") " << "x: " << x << ", val: " << pSrc[x] << std::endl;
//                //            }
//                
//                //            std:: cout << "i: " << i << ", " << (offset+moreOffset) << ", to: " << (offset+iWidth + moreOffset) << std::endl;
//                i = i + 1;
//            }
//            
//            //        std::cout << "======" << std::endl;
//            pSrc += iSrcStride;
//            moreOffset += iSrcStride;
//        }
//    }
//    else
//    {
//        Int countLines = 0;
//        for (Int y = iHeight-1; y >= 0; y-- )
//        {
//            Int i = offset + moreOffset;
//            for (Int x = offset + moreOffset; x <= offset + moreOffset + iWidth; x++)
//            {
//               if(countLines == myline)
//               {
//                   if (pSrc[x] >= 0)
//                       std::cout  << "+" << pSrc[x] << " ";
//                   else
//                       std::cout  << pSrc[x] << " ";
//               }
//            }
//            
//            // If I printed my line stop the method
//            if (countLines == myline) {
//                std::cout << "\n--------------------------------\n" << std::endl;
//                return;
//            }
//            
////            std::cout << "\n";
//            //            std:: cout << "i: " << i << ", " << (offset+moreOffset) << ", to: " << (offset+iWidth + moreOffset) << std::endl;
//            i = i + 1;
//            countLines++;
//            
//            
//            //        std::cout << "======" << std::endl;
//            pSrc += iSrcStride;
//            moreOffset += iSrcStride;
//        }
//        
//    }
//    
//    
//    std::cout << "--------------------------------" << std::endl;
//}

// Hossam: Scene change -- Print a specific line
Void TComYuv::mockPrintComponentLine               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock, Int myline ) const
{
    // Get the src with Offset
    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
    
    std:: cout << "\n****MOCK PRINT: " << std::endl;
    std:: cout << "MockPrintComponentLine address BEFORE Move: " << pSrc << std::endl;
    
    UInt blkX = getBlckX(ch, uiDstPartIdx);
    UInt blkY = getBlckY(ch, uiDstPartIdx);
    UInt offset = getOffset(ch, blkX, blkY);
    
    const UInt iSrcStride  = getStride(ch); // 64
    
    // Print this specific block with width and height
    const Int  iWidth   = srcWidth;
    const Int  iHeight = srcHeight;
    
    
    std:: cout << "blkX: " << this->getBlckX(ch, uiDstPartIdx) << std::endl;
    std:: cout << "blkY: " << this->getBlckY(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Offset: " << this->getOffset(ch, this->getBlckX(ch, uiDstPartIdx), this->getBlckY(ch, uiDstPartIdx)) << std::endl;
    std::cout << "Width: " << iWidth << ", Src Height: " << iHeight << std::endl;
    std::cout << "Stride: " << iSrcStride << std::endl;
    std::cout << "Added offset: " <<  myline*iSrcStride << ", for line: " << myline << std::endl;
    
//    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << ", Stride: " << iSrcStride << std::endl;
    
    
    // Move the source pointer to the begining of the line
    pSrc = pSrc + myline*iSrcStride;
    
    std:: cout << "MockPrintComponentLine address AFTER Move: " << pSrc << std::endl;
    for (Int x = 0; x < iWidth; x++)
    {
        if (pSrc[x] >= 0)
            std::cout  << "+" << pSrc[x] << " ";
        else
            std::cout  << pSrc[x] << " ";
    }
    
    std::cout << "\n--------------------------------\n" << std::endl;
}

// Hossam: Scene change
Void TComYuv::mockPrintComponentLine               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock, Int myline )
{
    // Get the src with Offset
    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
    
//    std:: cout << "\n****MOCK PRINT: " << std::endl;
//    std:: cout << "MockPrintComponentLine address BEFORE Move: " << pSrc << std::endl;
    
    UInt blkX = getBlckX(ch, uiDstPartIdx);
    UInt blkY = getBlckY(ch, uiDstPartIdx);
    UInt offset = getOffset(ch, blkX, blkY);
    
    const UInt iSrcStride  = getStride(ch); // 64
    
    // Print this specific block with width and height
    const Int  iWidth   = srcWidth;
    const Int  iHeight = srcHeight;

#if  SC_DEBUG_COPY_SPECIAL
    std:: cout << "blkX: " << this->getBlckX(ch, uiDstPartIdx) << std::endl;
    std:: cout << "blkY: " << this->getBlckY(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Offset: " << this->getOffset(ch, this->getBlckX(ch, uiDstPartIdx), this->getBlckY(ch, uiDstPartIdx)) << std::endl;
    std::cout << "Width: " << iWidth << ", Src Height: " << iHeight << std::endl;
    std::cout << "Stride: " << iSrcStride << std::endl;
    std::cout << "Added offset: " <<  myline*iSrcStride << ", for line: " << myline << std::endl;
#endif
    
    //    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << ", Stride: " << iSrcStride << std::endl;
    
    
    // Move the source pointer to the begining of the line
    pSrc = pSrc + myline*iSrcStride;
    
//    std:: cout << "MockPrintComponentLine address AFTER Move: " << pSrc << std::endl;
    for (Int x = 0; x < iWidth; x++)
    {
        if (pSrc[x] >= 0)
            std::cout  << "+" << pSrc[x] << " ";
        else
            std::cout  << pSrc[x] << " ";
    }
    
    std::cout << "\n--------------------------------\n" << std::endl;
}

Void TComYuv::mockPrintComponentLineBlk               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock, Int myline ) const
{
    // Get the src with Offset
    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
    
    //    std:: cout << "\n****MOCK PRINT: " << std::endl;
    //    std:: cout << "MockPrintComponentLine address BEFORE Move: " << pSrc << std::endl;
    
    UInt blkX = getBlckX(ch, uiDstPartIdx);
    UInt blkY = getBlckY(ch, uiDstPartIdx);
    UInt offset = getOffset(ch, blkX, blkY);
    
    const UInt iSrcStride  = getStride(ch); // 64
    
    // Print this specific block with width and height
    const Int  iWidth   = srcWidth;
    const Int  iHeight = srcHeight;
    

#if  SC_DEBUG_COPY_SPECIAL
    std:: cout << "blkX: " << this->getBlckX(ch, uiDstPartIdx) << std::endl;
    std:: cout << "blkY: " << this->getBlckY(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Offset: " << this->getOffset(ch, this->getBlckX(ch, uiDstPartIdx), this->getBlckY(ch, uiDstPartIdx)) << std::endl;
    std::cout << "Width: " << iWidth << ", Src Height: " << iHeight << std::endl;
    std::cout << "Stride: " << iSrcStride << std::endl;
    std::cout << "Added offset: " <<  myline*iSrcStride << ", for line: " << myline << std::endl;
    
    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << ", Stride: " << iSrcStride << std::endl;
#endif
    
    // Move the source pointer to the begining of the line
    pSrc = pSrc + myline*iSrcStride;
    
    //    std:: cout << "MockPrintComponentLine address AFTER Move: " << pSrc << std::endl;
    for (Int x = 0; x < iWidth; x++)
    {
        if (pSrc[x] >= 0)
            std::cout  << "+" << pSrc[x] << " ";
        else
            std::cout  << pSrc[x] << " ";
    }

}

//Void TComYuv::mockPrintComponentLine               ( const ComponentID ch, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight, Bool isBlock, Int myline )
//{
//    // Get the src with Offset
//    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
//    
//    UInt blkX = getBlckX(ch, uiDstPartIdx);
//    UInt blkY = getBlckY(ch, uiDstPartIdx);
//    UInt offset = getOffset(ch, blkX, blkY);
//    
//    const UInt iSrcStride  = getStride(ch); // 64
//    
//    // Print this specific block with width and height
//    const Int  iWidth   = srcWidth;
//    const Int  iHeight = srcHeight;
//    
//    //    const Int  bufferWidth = getWidth(ch); // 64
//    //    const Int  bufferHeight = getHeight(ch); // 64
//    
//    std::cout << "Mock Print Width: " << iWidth << ", Height: " << iHeight << std::endl;
//    
//    
//    Int moreOffset = 0;
//    Int lines = 0;
//    //    Int i = offset + moreOffset;
//    if(!isBlock)
//    {
//        for (Int y = iHeight-1; y >= 0; y-- )
//        {
//            Int i = offset + moreOffset;
//            
//            std::cout << "Line " << (lines++) << std::endl;
//            //            for (Int x = offset + moreOffset; x <= offset + moreOffset + iWidth; x++) {
//            //            if (i>=offset + moreOffset && i <= offset + iWidth + moreOffset ) {
//            // You already got the src with offset
//            for (Int x = 0; x < iWidth; x++) {
//                
//                if(myline == lines)
//                    std::cout << i << ") " << "x: " << x << ", val: " << pSrc[x] << std::endl;
//                //            }
//                
//                //            std:: cout << "i: " << i << ", " << (offset+moreOffset) << ", to: " << (offset+iWidth + moreOffset) << std::endl;
//                i = i + 1;
//            }
//            
//            //        std::cout << "======" << std::endl;
//            pSrc += iSrcStride;
//            moreOffset += iSrcStride;
//        }
//    }
//    else
//    {
//        Int countLines = 0;
//        for (Int y = iHeight-1; y >= 0; y-- )
//        {
//            Int i = offset + moreOffset;
//            for (Int x = offset + moreOffset; x <= offset + moreOffset + iWidth; x++)
//            {
//                if(countLines == myline)
//                {
//                    if (pSrc[x] >= 0)
//                        std::cout  << "+" << pSrc[x] << " ";
//                    else
//                        std::cout  << pSrc[x] << " ";
//                }
//            }
//            
//            // If I printed my line stop the method
//            if (countLines == myline) {
//                std::cout << "\n--------------------------------\n" << std::endl;
//                return;
//            }
//            
//            //            std::cout << "\n";
//            //            std:: cout << "i: " << i << ", " << (offset+moreOffset) << ", to: " << (offset+iWidth + moreOffset) << std::endl;
//            i = i + 1;
//            countLines++;
//            
//            
//            //        std::cout << "======" << std::endl;
//            pSrc += iSrcStride;
//            moreOffset += iSrcStride;
//        }
//        
//    }
//    
//    
//    std::cout << "--------------------------------" << std::endl;
//}




// Hossam: Scene change
Void TComYuv::print (const UInt uiDstPartIdx )  const
{
    for(Int ch=0; ch<getNumberValidComponents(); ch++)
        printComponent(ComponentID(ch), uiDstPartIdx);
}

// Hossam: Scene change
Void TComYuv::printComponent(const ComponentID ch, const UInt uiDstPartIdx) const
{
    const Pel* pSrc     = getAddr(ch);
   
    const UInt iSrcStride  = getStride(ch);
    const Int  iWidth=getWidth(ch);
    const Int  iHeight=getHeight(ch);
    
    std::cout << "Print Width: " << iWidth << ", Height: " << iHeight << std::endl;
    
    Int i = 0;
    for (Int y = iHeight-1; y >= 0; y-- )
    {
        for (Int x = 0; x < iWidth; x++) {
//            if ( (i>= 2048 && i <= 2048+iWidth) || ((i>= 2048 + iSrcStride && i <= 2048 + iSrcStride + iWidth)) )
            std::cout << (i) << ") " << "x: " << x << ", val: " << pSrc[x] << std::endl;
                i = i + 1;
        }
        
        std::cout << "======" << std::endl;
        pSrc += iSrcStride;
    }
    
    std::cout << "--------------------------------" << std::endl;
}

// Hossam: Scene change
Bool TComYuv::isSame(const UInt uiDstPartIdx, TComYuv* pcYuvDst) const
{
    Bool x = true;
    
    for(Int ch=0; ch<getNumberValidComponents(); ch++)
        x = x & isSameComponent(ComponentID(ch), uiDstPartIdx, pcYuvDst);

    return x;
}

// Hossam: Scene change
Bool TComYuv::isSameComponent(const ComponentID ch, const UInt uiDstPartIdx, TComYuv* pcYuvDst) const
{
    const Pel* pSrc     = getAddr(ch);
    Pel* pDst     = pcYuvDst->getAddr( ch, uiDstPartIdx );
    
    const UInt iSrcStride  = getStride(ch);
    const UInt iDstStride  = pcYuvDst->getStride(ch);
    const Int  iWidth=getWidth(ch);
    const Int  iHeight=getHeight(ch);
    
    
    for (Int y = iHeight-1; y >= 0; y-- )
    {
        for (Int x = 0; x < iWidth; x++) {
            
            if (Int(pSrc[x] != pDst[x])) {
                return false;
            }
        }
        
        pSrc += iSrcStride;
        pDst += iDstStride;
    }
    
    return true;
}


// Hossam: Scene Change
Void TComYuv::copyToTempResidualsBuffer( TComYuv* pcYuvDst, const UInt uiDstPartIdx) const
{
    for(Int ch=0; ch<getNumberValidComponents(); ch++)
        copyToTempResidualsBufferComponent  ( ComponentID(ch), pcYuvDst, uiDstPartIdx );
}

Void TComYuv::copyToTempResidualsBufferComponent ( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiDstPartIdx) const
{
    const Pel* pSrc = getAddr(ch);
//    Pel* pDst  = pcYuvDst->getAddr(ch);

    
    const UInt iSrcStride  = getStride(ch);
//    const UInt iDstStride  = pcYuvDst->getStride(ch);
    const UInt iDstStride  = iSrcStride;
    const Int  iWidth=getWidth(ch);
    const Int  iHeight=getHeight(ch);
    
    Pel* pDst  = pcYuvDst->getAddrWithStride(ch, uiDstPartIdx, iSrcStride);

//    Int blkX = pcYuvDst->getBlckX(ch, uiDstPartIdx);
//    Int blkY = pcYuvDst->getBlckX(ch, uiDstPartIdx);
//   
    
    
    for (Int y = iHeight; y != 0; y-- )
    {
        ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
        pDst += iDstStride;
        pSrc += iSrcStride;
    }
}

// Hossam: Scene change
Void TComYuv::copyToPartYuvSpecial( TComYuv* pcYuvDst, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight ) const
{
    for(Int ch=0; ch<getNumberValidComponents(); ch++)
        copyToPartYuvSpecialComponent  ( ComponentID(ch), pcYuvDst, uiDstPartIdx, srcWidth, srcHeight );
}

Void TComYuv::copyToPartYuvSpecialComponent( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiDstPartIdx, UInt srcWidth, UInt srcHeight ) const
{

//    const Pel* pSrc     = getAddr(ch);

//      std:: cout << "\nCopy Special Component: " << std:: endl;
//    std:: cout << "Special Source: " << std:: endl;
    const Pel* pSrc     = getAddr(ch, uiDstPartIdx);
    
    
//    std:: cout << "copy Special  pSrc address Initial: " << pSrc << std::endl;
//     std:: cout << "Special Destination: " << std:: endl;
    Pel* pDst     = pcYuvDst->getAddr( ch, uiDstPartIdx );
    
//        std:: cout << "copy Special  pDst address Initial: " << pDst << std::endl;
    
    const UInt iSrcStride  = getStride(ch);
    const UInt iDstStride  = pcYuvDst->getStride(ch);
    const Int  iWidth = srcWidth;
    const Int  iHeight = srcHeight;
    
//    const Int  iWidth=getWidth(ch);
//    const Int  iHeight=getHeight(ch);
    
#if  SC_DEBUG_COPY_SPECIAL
    std:: cout << "\n\nSrc blkX: " << this->getBlckX(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Src blkY: " << this->getBlckY(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Destination blkX: " << pcYuvDst->getBlckX(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Destination blkY: " << pcYuvDst->getBlckY(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Destination offset: " << pcYuvDst->getOffset(ch, pcYuvDst->getBlckX(ch, uiDstPartIdx), pcYuvDst->getBlckY(ch, uiDstPartIdx)) << std::endl;
    
    std:: cout << "Source offset: " << this->getOffset(ch, this->getBlckX(ch, uiDstPartIdx), this->getBlckY(ch, uiDstPartIdx)) << std::endl;
    
    std::cout << "Src Width: " << iWidth << ", Src Height: " << iHeight << std::endl;
    
    std::cout << "iSrcStride: " << iSrcStride << std::endl;
    std::cout << "iDstStride: " << iDstStride << std::endl;
    
#endif
    
    for (Int y = iHeight - 1 ; y >= 0; y-- )
    {

#if SC_DEBUG_COPY_SPECIAL
        if (y == 22) {
        
            std::cout << "*****************" << std::endl;
            std::cout << y <<  ") Source: " << std::endl;
            std:: cout << "copy Special  pSrc address : " << pSrc << std::endl;
        
             this->mockPrintComponentLine(COMPONENT_Y, uiDstPartIdx, srcWidth, srcHeight, true, y);
        
            std::cout << y <<  ") Destination Before: " << std::endl;
            pcYuvDst->mockPrintComponentLine(COMPONENT_Y, uiDstPartIdx, srcWidth, srcHeight, true, y);
            std:: cout << "copy Special  pDst address Before: " << pDst << std::endl;
       }
#endif

        
            ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
        
    
        
        pDst += iDstStride;
        pSrc += iSrcStride;
        
    }// end for

#if SC_DEBUG_COPY_SPECIAL
    for (Int y = iHeight - 1 ; y >= 0; y-- )
    {
        if (y == 22) {
            std::cout << y << ") Destination After: " << std::endl;
            pcYuvDst->mockPrintComponentLine(COMPONENT_Y, uiDstPartIdx, srcWidth, srcHeight, true, y);
            std:: cout << "copy Special  pDst address After: " << pSrc << std::endl;
            //        pDst -> mockPrintComponentLine(COMPONENT_Y, uiDstPartIdx, srcWidth, srcHeight, true, y);
            
            std::cout << "*****************\n\n" << std::endl;
        }
    }
#endif
    
}// end copy



Void TComYuv::copyToPartYuv( TComYuv* pcYuvDst, const UInt uiDstPartIdx ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyToPartComponent  ( ComponentID(ch), pcYuvDst, uiDstPartIdx );
}

Void TComYuv::copyToPartComponent( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiDstPartIdx ) const
{
  

    
  const Pel* pSrc     = getAddr(ch);
        Pel* pDst     = pcYuvDst->getAddr( ch, uiDstPartIdx );

  const UInt iSrcStride  = getStride(ch);
  const UInt iDstStride  = pcYuvDst->getStride(ch);
  const Int  iWidth=getWidth(ch);
  const Int  iHeight=getHeight(ch);


#if SC_ENABLE_PRINT
    std:: cout << "\n\nSrc blkX: " << pcYuvDst->getBlckX(ch, 0) << std::endl;
    std:: cout << "Src blkY: " << pcYuvDst->getBlckY(ch, 0) << std::endl;
    std:: cout << "Destination blkX: " << pcYuvDst->getBlckX(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Destination blkY: " << pcYuvDst->getBlckY(ch, uiDstPartIdx) << std::endl;
    std:: cout << "Destination offset: " << pcYuvDst->getOffset(ch, pcYuvDst->getBlckX(ch, uiDstPartIdx), pcYuvDst->getBlckY(ch, uiDstPartIdx)) << std::endl;
    std::cout << "Src Width: " << iWidth << ", Src Height: " << iHeight << std::endl;
    
    std::cout << "iSrcStride: " << iSrcStride << std::endl;
    std::cout << "iDstStride: " << iDstStride << std::endl;
#endif
    
  for (Int y = iHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*iWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyPartToYuv( TComYuv* pcYuvDst, const UInt uiSrcPartIdx ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyPartToComponent  ( ComponentID(ch), pcYuvDst, uiSrcPartIdx );
}

Void TComYuv::copyPartToComponent( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiSrcPartIdx ) const
{
  const Pel* pSrc     = getAddr(ch, uiSrcPartIdx);
        Pel* pDst     = pcYuvDst->getAddr(ch, 0 );

  const UInt  iSrcStride  = getStride(ch);
  const UInt  iDstStride  = pcYuvDst->getStride(ch);

  const UInt uiHeight = pcYuvDst->getHeight(ch);
  const UInt uiWidth = pcYuvDst->getWidth(ch);

  for ( UInt y = uiHeight; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, sizeof(Pel)*uiWidth);
    pDst += iDstStride;
    pSrc += iSrcStride;
  }
}




Void TComYuv::copyPartToPartYuv   ( TComYuv* pcYuvDst, const UInt uiPartIdx, const UInt iWidth, const UInt iHeight ) const
{
  for(Int ch=0; ch<getNumberValidComponents(); ch++)
    copyPartToPartComponent   (ComponentID(ch), pcYuvDst, uiPartIdx, iWidth>>getComponentScaleX(ComponentID(ch)), iHeight>>getComponentScaleY(ComponentID(ch)) );
}

Void TComYuv::copyPartToPartComponent  ( const ComponentID ch, TComYuv* pcYuvDst, const UInt uiPartIdx, const UInt iWidthComponent, const UInt iHeightComponent ) const
{
  const Pel* pSrc =           getAddr(ch, uiPartIdx);
        Pel* pDst = pcYuvDst->getAddr(ch, uiPartIdx);
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller
    return ;
  }

  const UInt  iSrcStride = getStride(ch);
  const UInt  iDstStride = pcYuvDst->getStride(ch);
  for ( UInt y = iHeightComponent; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, iWidthComponent * sizeof(Pel) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}




Void TComYuv::copyPartToPartComponentMxN  ( const ComponentID ch, TComYuv* pcYuvDst, const TComRectangle &rect) const
{
  const Pel* pSrc =           getAddrPix( ch, rect.x0, rect.y0 );
        Pel* pDst = pcYuvDst->getAddrPix( ch, rect.x0, rect.y0 );
  if( pSrc == pDst )
  {
    //th not a good idea
    //th best would be to fix the caller
    return ;
  }

  const UInt  iSrcStride = getStride(ch);
  const UInt  iDstStride = pcYuvDst->getStride(ch);
  const UInt uiHeightComponent=rect.height;
  const UInt uiWidthComponent=rect.width;
  for ( UInt y = uiHeightComponent; y != 0; y-- )
  {
    ::memcpy( pDst, pSrc, uiWidthComponent * sizeof( Pel ) );
    pSrc += iSrcStride;
    pDst += iDstStride;
  }
}




Void TComYuv::addClip( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt uiTrUnitIdx, const UInt uiPartSize )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Int uiPartWidth =uiPartSize>>getComponentScaleX(ch);
    const Int uiPartHeight=uiPartSize>>getComponentScaleY(ch);

    const Pel* pSrc0 = pcYuvSrc0->getAddr(ch, uiTrUnitIdx, uiPartWidth );
    const Pel* pSrc1 = pcYuvSrc1->getAddr(ch, uiTrUnitIdx, uiPartWidth );
          Pel* pDst  = getAddr(ch, uiTrUnitIdx, uiPartWidth );

    const UInt iSrc0Stride = pcYuvSrc0->getStride(ch);
    const UInt iSrc1Stride = pcYuvSrc1->getStride(ch);
    const UInt iDstStride  = getStride(ch);
    const Int clipbd = g_bitDepth[toChannelType(ch)];
#if RExt__O0043_BEST_EFFORT_DECODING
    const Int bitDepthDelta = g_bitDepthInStream[toChannelType(ch)] - g_bitDepth[toChannelType(ch)];
#endif

    for ( Int y = uiPartHeight-1; y >= 0; y-- )
    {
      for ( Int x = uiPartWidth-1; x >= 0; x-- )
      {
#if RExt__O0043_BEST_EFFORT_DECODING
        pDst[x] = Pel(ClipBD<Int>( Int(pSrc0[x]) + rightShiftEvenRounding<Pel>(pSrc1[x], bitDepthDelta), clipbd));
#else
        pDst[x] = Pel(ClipBD<Int>( Int(pSrc0[x]) + Int(pSrc1[x]), clipbd));
#endif
      }
      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst  += iDstStride;
    }
  }
}




Void TComYuv::subtract( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt uiTrUnitIdx, const UInt uiPartSize )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Int uiPartWidth =uiPartSize>>getComponentScaleX(ch);
    const Int uiPartHeight=uiPartSize>>getComponentScaleY(ch);

    const Pel* pSrc0 = pcYuvSrc0->getAddr( ch, uiTrUnitIdx, uiPartWidth );
    const Pel* pSrc1 = pcYuvSrc1->getAddr( ch, uiTrUnitIdx, uiPartWidth );
          Pel* pDst  = getAddr( ch, uiTrUnitIdx, uiPartWidth );

    const Int  iSrc0Stride = pcYuvSrc0->getStride(ch);
    const Int  iSrc1Stride = pcYuvSrc1->getStride(ch);
    const Int  iDstStride  = getStride(ch);

    for (Int y = uiPartHeight-1; y >= 0; y-- )
    {
      for (Int x = uiPartWidth-1; x >= 0; x-- )
      {
        pDst[x] = pSrc0[x] - pSrc1[x];
      }
      pSrc0 += iSrc0Stride;
      pSrc1 += iSrc1Stride;
      pDst  += iDstStride;
    }
  }
}




Void TComYuv::addAvg( const TComYuv* pcYuvSrc0, const TComYuv* pcYuvSrc1, const UInt iPartUnitIdx, const UInt uiWidth, const UInt uiHeight )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const Pel* pSrc0  = pcYuvSrc0->getAddr( ch, iPartUnitIdx );
    const Pel* pSrc1  = pcYuvSrc1->getAddr( ch, iPartUnitIdx );
    Pel* pDst   = getAddr( ch, iPartUnitIdx );

    const UInt  iSrc0Stride = pcYuvSrc0->getStride(ch);
    const UInt  iSrc1Stride = pcYuvSrc1->getStride(ch);
    const UInt  iDstStride  = getStride(ch);
    const Int   clipbd      = g_bitDepth[toChannelType(ch)];
    const Int   shiftNum    = std::max<Int>(2, (IF_INTERNAL_PREC - clipbd)) + 1;
    const Int   offset      = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

    const Int   iWidth      = uiWidth  >> getComponentScaleX(ch);
    const Int   iHeight     = uiHeight >> getComponentScaleY(ch);

    if (iWidth&1)
    {
      assert(0);
      exit(-1);
    }
    else if (iWidth&2)
    {
      for ( Int y = 0; y < iHeight; y++ )
      {
        for (Int x=0 ; x < iWidth; x+=2 )
        {
          pDst[ x + 0 ] = ClipBD( rightShift(( pSrc0[ x + 0 ] + pSrc1[ x + 0 ] + offset ), shiftNum), clipbd );
          pDst[ x + 1 ] = ClipBD( rightShift(( pSrc0[ x + 1 ] + pSrc1[ x + 1 ] + offset ), shiftNum), clipbd );
        }
        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst  += iDstStride;
      }
    }
    else
    {
      for ( Int y = 0; y < iHeight; y++ )
      {
        for (Int x=0 ; x < iWidth; x+=4 )
        {
          pDst[ x + 0 ] = ClipBD( rightShift(( pSrc0[ x + 0 ] + pSrc1[ x + 0 ] + offset ), shiftNum), clipbd );
          pDst[ x + 1 ] = ClipBD( rightShift(( pSrc0[ x + 1 ] + pSrc1[ x + 1 ] + offset ), shiftNum), clipbd );
          pDst[ x + 2 ] = ClipBD( rightShift(( pSrc0[ x + 2 ] + pSrc1[ x + 2 ] + offset ), shiftNum), clipbd );
          pDst[ x + 3 ] = ClipBD( rightShift(( pSrc0[ x + 3 ] + pSrc1[ x + 3 ] + offset ), shiftNum), clipbd );
        }
        pSrc0 += iSrc0Stride;
        pSrc1 += iSrc1Stride;
        pDst  += iDstStride;
      }
    }
  }
}

Void TComYuv::removeHighFreq( const TComYuv* pcYuvSrc, const UInt uiPartIdx, const UInt uiWidth, UInt const uiHeight )
{
  for(Int chan=0; chan<getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
#if !DISABLING_CLIP_FOR_BIPREDME
    const ChannelType chType=toChannelType(ch);
#endif

    const Pel* pSrc  = pcYuvSrc->getAddr(ch, uiPartIdx);
    Pel* pDst  = getAddr(ch, uiPartIdx);

    const Int iSrcStride = pcYuvSrc->getStride(ch);
    const Int iDstStride = getStride(ch);
    const Int iWidth  = uiWidth >>getComponentScaleX(ch);
    const Int iHeight = uiHeight>>getComponentScaleY(ch);

    for ( Int y = iHeight-1; y >= 0; y-- )
    {
      for ( Int x = iWidth-1; x >= 0; x-- )
      {
#if DISABLING_CLIP_FOR_BIPREDME
        pDst[x ] = (2 * pDst[x]) - pSrc[x];
#else
        pDst[x ] = Clip((2 * pDst[x]) - pSrc[x], chType);
#endif
      }
      pSrc += iSrcStride;
      pDst += iDstStride;
    }
  }
}

//! \}
