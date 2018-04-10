#ifndef __itkMyFilter_h
#define __itkMyFilter_h
 
#include "itkImageToImageFilter.h"
 
namespace itk
{
  template <typename TImage>
class MyFilter:public ImageToImageFilter< TImage, TImage >
{
public:
  /** Standard class typedefs. */
  typedef MyFilter Self;
  typedef ImageToImageFilter< TImage, TImage > Superclass;
  typedef SmartPointer< Self > Pointer;
 
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
 
  /** Run-time type information (and related methods). */
  itkTypeMacro(MyFilter, ImageToImageFilter);
  
  virtual void SetVariance(float v);
  virtual void SetUpperThreshold(int u);
  virtual void SetLowerThreshold(int l);

protected:
  MyFilter(){}
  ~MyFilter(){}
 
  /** Does the real work. */
  virtual void GenerateData();
  

private:
 MyFilter(const Self &); //purposely not implemented
 void operator=(const Self &);  //purposely not implemented
 float variance;
 int upperThreshold, lowerThreshold;
};
} //namespace ITK
 
#ifndef ITK_MANUAL_INSTANTIATION
#include "MyFilter.hxx"
#endif
 
#endif // __itkMyFilter_h