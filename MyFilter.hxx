#ifndef __itkMyFilter_hxx
#define __itkMyFilter_hxx
 

#include <itkObjectFactory.h>
#include <itkImageRegionIterator.h>
#include <itkImageRegionConstIterator.h>
#include <itkSize.h>
#include <itkIndex.h>
#include <itkConstNeighborhoodIterator.h>
#include <itkVector.h>
#include <cmath>
#include <itkSobelOperator.h>
#include <itkGaussianOperator.h>
#include <itkNeighborhoodInnerProduct.h>
#include <math.h>
#include <itkRescaleIntensityImageFilter.h>

#include "MyFilter.h"

namespace itk
{

template <typename TImage>
void MyFilter<TImage>::SetVariance(float v)
 {
variance=v;
 }

 template <typename TImage>
void MyFilter<TImage>::SetUpperThreshold(int u)
 {
upperThreshold=u;
 }

template <typename TImage>
void MyFilter<TImage>::SetLowerThreshold(int l)
 {
lowerThreshold=l;
 }

template <typename TImage>
void MyFilter<TImage>::GenerateData()
{

  typename TImage::ConstPointer input = this->GetInput();
  typename TImage::Pointer output = this->GetOutput();
  
  this->AllocateOutputs();

  typedef itk::ConstNeighborhoodIterator< TImage > NeighborhoodIteratorType;
  typedef itk::ImageRegionConstIterator< TImage > ConstIteratorType;
  typedef itk::ImageRegionIterator< TImage> IteratorType;
  
typename NeighborhoodIteratorType::RadiusType radius;
radius.Fill(3);
 
typename NeighborhoodIteratorType::OffsetType offset0 = {{0,0}};
//offset horizontal
typename NeighborhoodIteratorType::OffsetType offset1 = {{-1,0}};
typename NeighborhoodIteratorType::OffsetType offset2 = {{-2,0}};
typename NeighborhoodIteratorType::OffsetType offset3 = {{-3,0}};
typename NeighborhoodIteratorType::OffsetType offset4 = {{1,0}};
typename NeighborhoodIteratorType::OffsetType offset5 = {{2,0}};
typename NeighborhoodIteratorType::OffsetType offset6 = {{3,0}};
//offset vertical
typename NeighborhoodIteratorType::OffsetType offset7 = {{0,-1}};
typename NeighborhoodIteratorType::OffsetType offset8 = {{0,-2}};
typename NeighborhoodIteratorType::OffsetType offset9 = {{0,-3}};
typename NeighborhoodIteratorType::OffsetType offset10 = {{0,1}};
typename NeighborhoodIteratorType::OffsetType offset11 = {{0,2}};
typename NeighborhoodIteratorType::OffsetType offset12 = {{0,3}};
//offset diag
typename NeighborhoodIteratorType::OffsetType offset13 = {{1,-1}};
typename NeighborhoodIteratorType::OffsetType offset14 = {{-1,1}};
typename NeighborhoodIteratorType::OffsetType offset15= {{-1,-1}};
typename NeighborhoodIteratorType::OffsetType offset16 = {{1,1}};

typename NeighborhoodIteratorType::OffsetType offsetTab[9]={offset0,offset1,offset2,offset3,offset4,
															offset5,offset6,offset7,offset8};
//First step : Gauss Filter 
//***************************************************************************************************
//***************************************************************************************************															
//Initialisation pointer for gaussian filter
  typename TImage::Pointer outputgaussH=TImage::New();
  outputgaussH->SetRegions(input->GetRequestedRegion());
  outputgaussH->Allocate();

  typename TImage::Pointer outputgaussV=TImage::New();
  outputgaussV->SetRegions(input->GetRequestedRegion());
  outputgaussV->Allocate();


/*
//Gauss horizontal
typedef itk::Vector<float,7> VectorType;
VectorType gaussian;
gaussian[0]=0.006;gaussian[1]=0.061;gaussian[2]=0.242;gaussian[3]=0.383;
gaussian[4]=0.242;gaussian[5]=0.061;gaussian[6]=0.006;

NeighborhoodIteratorType itgaussH( radius, input ,input->GetRequestedRegion() );
IteratorType outgaussH(outputgaussH, input->GetRequestedRegion());

	for (itgaussH.GoToBegin(), outgaussH.GoToBegin(); !itgaussH.IsAtEnd(); ++itgaussH, ++outgaussH)
	{	
		VectorType TabH;
		TabH[0]=itgaussH.GetPixel(offset3);TabH[1]=itgaussH.GetPixel(offset2);TabH[2]=itgaussH.GetPixel(offset1);
		TabH[3]=itgaussH.GetPixel(offset0);TabH[4]=itgaussH.GetPixel(offset4);TabH[5]=itgaussH.GetPixel(offset5);
		TabH[6]=itgaussH.GetPixel(offset6);

		VectorType::ComponentType Convolution=gaussian*TabH;		
		outgaussH.Set(Convolution);

	}*/

  itk::GaussianOperator< float, 2 > gaussianOperatorH;
  gaussianOperatorH.SetDirection(0);
  gaussianOperatorH.SetVariance( variance );
  gaussianOperatorH.CreateDirectional();
  
NeighborhoodIteratorType itgaussH( radius, input ,input->GetRequestedRegion() );
IteratorType outgaussH(outputgaussH, input->GetRequestedRegion());

 itk::NeighborhoodInnerProduct<TImage> innerProduct;

 for (itgaussH.GoToBegin(),outgaussH.GoToBegin(); !itgaussH.IsAtEnd(); ++itgaussH, ++outgaussH)
    {
    outgaussH.Set( innerProduct(itgaussH, gaussianOperatorH ) );
    }


//Gauss vertical
/*NeighborhoodIteratorType itgaussV( radius, input,input->GetRequestedRegion() );
IteratorType outgaussV(outputgaussV, input->GetRequestedRegion());

	for (itgaussV.GoToBegin(), outgaussV.GoToBegin(); !itgaussV.IsAtEnd(); ++itgaussV, ++outgaussV)
	{
		
		VectorType TabV;
		TabV[0]=itgaussV.GetPixel(offset9);TabV[1]=itgaussV.GetPixel(offset8);TabV[2]=itgaussV.GetPixel(offset7);
		TabV[3]=itgaussV.GetPixel(offset0);TabV[4]=itgaussV.GetPixel(offset10);TabV[5]=itgaussV.GetPixel(offset11);
		TabV[6]=itgaussV.GetPixel(offset12);

		VectorType::ComponentType Convolution=gaussian*TabV;		
		outgaussV.Set(Convolution);
	}*/
  itk::GaussianOperator< float, 2 > gaussianOperatorV;
  gaussianOperatorV.SetDirection(1);
  gaussianOperatorV.SetVariance( variance );
  gaussianOperatorV.CreateDirectional();
  
NeighborhoodIteratorType itgaussV( radius, input ,input->GetRequestedRegion() );
IteratorType outgaussV(outputgaussV, input->GetRequestedRegion());

 for (itgaussV.GoToBegin(),outgaussV.GoToBegin(); !itgaussV.IsAtEnd(); ++itgaussV, ++outgaussV)
    {
    outgaussV.Set( innerProduct(itgaussV, gaussianOperatorV ) );
    }
  
//Module of 2 gaussian horizontal and vertical
typename TImage::Pointer outputMod=TImage::New();
 outputMod->SetRegions(input->GetRequestedRegion());
 outputMod->Allocate();

//NeighborhoodIteratorType itmod1( radius, outputgaussH ,input->GetRequestedRegion() );
//NeighborhoodIteratorType itmod2( radius, outputgaussV ,input->GetRequestedRegion() );

ConstIteratorType itmod1( outputgaussH, input->GetRequestedRegion() );
ConstIteratorType itmod2( outputgaussV, input->GetRequestedRegion() );

IteratorType outmod(outputMod, input->GetRequestedRegion());

for (itmod1.GoToBegin(), itmod2.GoToBegin(),outmod.GoToBegin(); !itmod1.IsAtEnd(),!itmod2.IsAtEnd(); ++itmod1,++itmod2 ,++outmod)
{
	outmod.Set(sqrt(itmod1.Get()*itmod1.Get()+itmod2.Get()*itmod2.Get()));
	//std::cout<<sqrt(itmod1.Get()*itmod1.Get()+itmod2.Get()*itmod2.Get())<<std::endl;
}
//***************************************************************************************************
//***************************************************************************************************


//Second step : Finding the intensity gradient of the image 
//***************************************************************************************************
//***************************************************************************************************
//Initialisation pointer for diff Horizontal iteration***********************************************
  typename TImage::Pointer outputdiffH=TImage::New();
  outputdiffH->SetRegions(input->GetRequestedRegion());
  outputdiffH->Allocate();

//Derivative horizontal direction
  itk::SobelOperator<float, 2> sobelOperatorH;
  sobelOperatorH.SetDirection( 0);// Create the operator for the X axis derivative
  sobelOperatorH.CreateDirectional();

  typename NeighborhoodIteratorType::RadiusType radH = sobelOperatorH.GetRadius();
NeighborhoodIteratorType itdiffH( radH, outputMod ,outputMod->GetRequestedRegion() );
IteratorType outdiffH(outputdiffH, outputMod->GetRequestedRegion());


 for (itdiffH.GoToBegin(), outdiffH.GoToBegin(); !itdiffH.IsAtEnd(); ++itdiffH, ++outdiffH)
    {
    outdiffH.Set( innerProduct( itdiffH, sobelOperatorH ) );
    }

//Initialisation pointer for diff vertical iteration**************************************************
  typename TImage::Pointer outputdiffV=TImage::New();
  outputdiffV->SetRegions(input->GetRequestedRegion());
  outputdiffV->Allocate();

 //Derivative vertical direction 
  itk::SobelOperator<float, 2> sobelOperatorV;
  sobelOperatorV.SetDirection( 1);// Create the operator for the Y axis derivative
  sobelOperatorV.CreateDirectional();

typename NeighborhoodIteratorType::RadiusType radV = sobelOperatorV.GetRadius();
NeighborhoodIteratorType itdiffV( radV, outputMod ,outputMod->GetRequestedRegion() );
IteratorType outdiffV(outputdiffV, outputMod->GetRequestedRegion());

 for (itdiffV.GoToBegin(), outdiffV.GoToBegin(); !itdiffV.IsAtEnd(); ++itdiffV, ++outdiffV)
    {
    outdiffV.Set( innerProduct( itdiffV, sobelOperatorV ) );
    }

//Module of 2 derivatives horizontal and vertical
typename TImage::Pointer outputModDeri=TImage::New();
 outputModDeri->SetRegions(input->GetRequestedRegion());
 outputModDeri->Allocate();

NeighborhoodIteratorType itmodDeri1( radV, outputdiffH ,outputMod->GetRequestedRegion() );
NeighborhoodIteratorType itmodDeri2( radV, outputdiffV ,outputMod->GetRequestedRegion() );
IteratorType outModDeri(outputModDeri, outputMod->GetRequestedRegion());

for (itmodDeri1.GoToBegin(), itmodDeri2.GoToBegin(),outModDeri.GoToBegin(); !itmodDeri1.IsAtEnd(),!itmodDeri2.IsAtEnd(); ++itmodDeri1,++itmodDeri2 ,++outModDeri)
{
	outModDeri.Set(sqrt(itmodDeri1.GetPixel(offset0)*itmodDeri1.GetPixel(offset0)+
		itmodDeri2.GetPixel(offset0)*itmodDeri2.GetPixel(offset0)));
}

//Atan2 of the 2 derivative -> Find the edge direction
typename TImage::Pointer outputDirection=TImage::New();
 outputDirection->SetRegions(input->GetRequestedRegion());
 outputDirection->Allocate();

IteratorType outDirection(outputDirection, outputMod->GetRequestedRegion());

for (itmodDeri1.GoToBegin(), itmodDeri2.GoToBegin(),outDirection.GoToBegin(); !itmodDeri1.IsAtEnd(),!itmodDeri2.IsAtEnd(); ++itmodDeri1,++itmodDeri2, ++outDirection)
    {
    outDirection.Set( atan2(itmodDeri1.GetPixel(offset0),itmodDeri2.GetPixel(offset0))*180/3.14 );
  // std::cout<<atan2(itmodDeri1.GetPixel(offset0),itmodDeri2.GetPixel(offset0))*180/3.14 <<std::endl;
    }
//****************************************************************************************************
//****************************************************************************************************


//Third step : Non-maximum suppression
//****************************************************************************************************  
//***************************************************************************************************

typename NeighborhoodIteratorType::RadiusType connectivity;
connectivity.Fill(1);

typename TImage::Pointer outputMaxSup=TImage::New();
 outputMaxSup->SetRegions(input->GetRequestedRegion());
 outputMaxSup->Allocate();

NeighborhoodIteratorType itMaxSup( connectivity, outputModDeri ,outputModDeri->GetRequestedRegion() );
ConstIteratorType itDirection(outputDirection, outputModDeri->GetRequestedRegion());
IteratorType outMaxSup(outputMaxSup, outputModDeri->GetRequestedRegion());

for(itDirection.GoToBegin(),itMaxSup.GoToBegin(),outMaxSup.GoToBegin(); !itDirection.IsAtEnd(),!itMaxSup.IsAtEnd(); ++itDirection,++itMaxSup,++outMaxSup)
{
	if( (itDirection.Get()>-30 && itDirection.Get()<30) || (itDirection.Get()<-150) || (itDirection.Get()>150) )
	{
		if(itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset7) && itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset10))
		{
			outMaxSup.Set(itMaxSup.GetPixel(offset0));
		}
		else
		{
			outMaxSup.Set(0);
		}
	}
	if((itDirection.Get()>60 && itDirection.Get()<120 ) || (itDirection.Get()>-120 && itDirection.Get()<-60))
	{
		if(itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset15) && itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset16))
		{
			outMaxSup.Set(itMaxSup.GetPixel(offset0));
		}
		else
		{
			outMaxSup.Set(0);
		}
	}

	if( (itDirection.Get()>30 && itDirection.Get()<60) || (itDirection.Get()<-120 && itDirection.Get()>-150) )
	{
		if(itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset1) && itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset4))
		{
			outMaxSup.Set(itMaxSup.GetPixel(offset0));
		}
		else
		{
			outMaxSup.Set(0);
		}
	}
	
		if((itDirection.Get()<-30 && itDirection.Get()>-60)||(itDirection.Get()>120 && itDirection.Get()<150))
	{
		if(itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset13) && itMaxSup.GetPixel(offset0)>itMaxSup.GetPixel(offset14))
		{
			outMaxSup.Set(itMaxSup.GetPixel(offset0));
		}
		else
		{
			outMaxSup.Set(0);
		}
	}
}
//***********************************************************************************************************
//***********************************************************************************************************


//Fourth step :: Double threshold
//***********************************************************************************************************
//***********************************************************************************************************

//Initialisation pointer for threshold Horizontal iteration***********************************************
  typename TImage::Pointer outputthreshH=TImage::New();
  outputthreshH->SetRegions(input->GetRequestedRegion());
  outputthreshH->Allocate();

typename NeighborhoodIteratorType::RadiusType radiusthreshold;
radiusthreshold.Fill(1);
NeighborhoodIteratorType itthreshH( radiusthreshold, outputMaxSup ,outputMaxSup->GetRequestedRegion() );
IteratorType outthreshH(outputthreshH, outputMaxSup->GetRequestedRegion());

 for (itthreshH.GoToBegin(), outthreshH.GoToBegin(); !itthreshH.IsAtEnd(); ++itthreshH, ++outthreshH)
    {
    	if(itthreshH.GetPixel(offset0)>upperThreshold)
    	{
    		outthreshH.Set(255);
		}
		else if (itthreshH.GetPixel(offset0)<lowerThreshold)
		{
			outthreshH.Set(0);
		}
		else
		{
			outthreshH.Set(130);
		}
    }

//Initialisation pointer for threshold Vertical iteration***********************************************
  typename TImage::Pointer outputthreshV=TImage::New();
  outputthreshV->SetRegions(input->GetRequestedRegion());
  outputthreshV->Allocate();

NeighborhoodIteratorType itthreshV( radiusthreshold, outputMaxSup ,outputMaxSup->GetRequestedRegion() );
IteratorType outthreshV(outputthreshV, outputMaxSup->GetRequestedRegion());

 for (itthreshV.GoToBegin(), outthreshV.GoToBegin(); !itthreshV.IsAtEnd(); ++itthreshV, ++outthreshV)
    {
    	if(itthreshV.GetPixel(offset0)>upperThreshold)
    	{
    		outthreshV.Set(255);
		}
		else if (itthreshH.GetPixel(offset0)<lowerThreshold)
		{
			outthreshV.Set(0);
		}
		else
		{
			outthreshH.Set(130);
		}
    }

//Module of 2 threshold
typename TImage::Pointer outputthresh=TImage::New();
 outputthresh->SetRegions(input->GetRequestedRegion());
 outputthresh->Allocate();

NeighborhoodIteratorType itthresh1( radV, outputthreshH ,outputMaxSup->GetRequestedRegion() );
NeighborhoodIteratorType itthresh2( radV, outputthreshV ,outputMaxSup->GetRequestedRegion() );
IteratorType outthresh(outputthresh, outputMaxSup->GetRequestedRegion());

for (itthresh1.GoToBegin(), itthresh2.GoToBegin(),outthresh.GoToBegin(); !itthresh1.IsAtEnd(),!itthresh2.IsAtEnd(); ++itthresh1,++itthresh2 ,++outthresh)
{
	outthresh.Set(sqrt(itthresh1.GetPixel(offset0)*itthresh1.GetPixel(offset0)+
		itthresh2.GetPixel(offset0)*itthresh2.GetPixel(offset0)));
} 
//********************************************************************************************************
//********************************************************************************************************

//Fifth step :: Edge tracking 
//********************************************************************************************************
//********************************************************************************************************

NeighborhoodIteratorType ittrack( radius, outputthresh ,outputthresh->GetRequestedRegion() );
IteratorType out(output,outputthresh->GetRequestedRegion() );

//Initialisation label image
for(ittrack.GoToBegin(), out.GoToBegin();!ittrack.IsAtEnd(), !out.IsAtEnd();++ittrack, ++out)
{
	if(ittrack.GetPixel(offset0)>250)
	{
		out.Set(255);
	}
	else if(ittrack.GetPixel(offset0)>100 && ittrack.GetPixel(offset0)<150)
	{
		if(ittrack.GetPixel(offset1)>250 ||  ittrack.GetPixel(offset4)>250 ||ittrack.GetPixel(offset7)>250 ||
		ittrack.GetPixel(offset10)>250 || ittrack.GetPixel(offset13)>250|| ittrack.GetPixel(offset14)>250 ||
		ittrack.GetPixel(offset15)>250 || ittrack.GetPixel(offset16)>250 )
		{
			out.Set(255);
		}
		else
		{
			out.Set(0);
		}
	}
	else
	{
		out.Set(0);
	}
}
//**********************************************************************************************************
//**********************************************************************************************************

}



}// end namespace
 
 
#endif