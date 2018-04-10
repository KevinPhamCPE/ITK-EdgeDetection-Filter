#include "MyFilterAppCLP.h"

#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkSize.h>
#include <itkCastImageFilter.h>
#include <itkRescaleIntensityImageFilter.h>
#include <cstring>

#include <vtkImageViewer2.h>
#include "MyFilter.h"
#include <vtkActor.h>
#include <vtkVersion.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkImageMapper.h>
#include <vtkActor2D.h>
#include <vtkImageSlice.h>

#include <vtkPNGReader.h>

int main(int argc, char *argv[])

{
	PARSE_ARGS;
	
typedef float PixelType;
typedef itk::Image <PixelType,2> ImageType;
typedef itk::MyFilter<ImageType> FilterType;
typedef itk::Image< unsigned char, 2 > FinalImageType;

typedef itk::ImageFileReader< FinalImageType > ReaderType;
typedef itk::ImageFileWriter< FinalImageType > WriterType;
typedef itk::CastImageFilter< FinalImageType, ImageType > CastToRealFilterType;
typedef itk::RescaleIntensityImageFilter< ImageType,FinalImageType > RescaleFilterType;

ReaderType::Pointer reader= ReaderType::New();

WriterType::Pointer writer = WriterType::New();
FilterType::Pointer filter = FilterType::New();

CastToRealFilterType:: Pointer castToRealFilterType= CastToRealFilterType::New();
RescaleFilterType::Pointer rescaleFilterType=RescaleFilterType::New();

reader->SetFileName( InputImage.c_str() );

castToRealFilterType->SetInput(reader->GetOutput());
filter->SetInput(castToRealFilterType->GetOutput());

float var=variance;
filter->SetVariance(var);
int up=upperThreshold;
std::cout<<"UpperThreshold ="<<up<<std::endl;
filter->SetUpperThreshold(up);
int low=lowerThreshold;
std::cout<<"LowerThreshold ="<<low<<std::endl;
filter->SetLowerThreshold(low);

rescaleFilterType->SetInput(filter->GetOutput());

writer->SetInput (rescaleFilterType->GetOutput());
writer->SetFileName(OutputImage.c_str());
writer->Update();

vtkSmartPointer<vtkPNGReader> readerVTK =vtkSmartPointer<vtkPNGReader>::New();
  readerVTK->SetFileName(InputImage.c_str());

vtkSmartPointer<vtkPNGReader> readerVTKout =vtkSmartPointer<vtkPNGReader>::New();
  readerVTKout->SetFileName(OutputImage.c_str());

 // Visualize
  vtkSmartPointer<vtkImageViewer2> imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
  vtkSmartPointer<vtkImageViewer2> imageViewerOut = vtkSmartPointer<vtkImageViewer2>::New();
  imageViewer->SetInputConnection(readerVTK->GetOutputPort());
  imageViewerOut->SetInputConnection(readerVTKout->GetOutputPort());


 	vtkSmartPointer<vtkRenderWindow> renderWindow =vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(800, 400);    

    vtkSmartPointer<vtkRenderWindowInteractor> interactor =vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

// Define viewport ranges
    // (xmin, ymin, xmax, ymax)
    double leftViewport[4] = { 0.0, 0.0, 0.5, 1.0 };
    double rightViewport[4] = { 0.5, 0.0, 1.0, 1.0 };

 // Setup both renderers
    vtkSmartPointer<vtkRenderer> leftRenderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(leftRenderer);
    leftRenderer->SetViewport(leftViewport);

    vtkSmartPointer<vtkRenderer> rightRenderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(rightRenderer);
    rightRenderer->SetViewport(rightViewport);

    imageViewer->SetRenderer(leftRenderer);
    
    leftRenderer->ResetCamera();
	imageViewerOut->SetRenderer(rightRenderer);

    rightRenderer->ResetCamera();
    renderWindow->Render();
    interactor->Start();

  return EXIT_SUCCESS;
}
 
 