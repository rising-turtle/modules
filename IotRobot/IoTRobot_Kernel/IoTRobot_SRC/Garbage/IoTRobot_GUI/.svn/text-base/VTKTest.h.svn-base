#pragma once
#include <Windows.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
class myVTKApp
{
public:
	myVTKApp(HWND parent,int x,int y,int nWidth,int nHeight);
	~myVTKApp();
private:
	vtkRenderWindow *renwin;
	vtkRenderer *renderer;
	vtkRenderWindowInteractor *iren;
	vtkConeSource *cone;
	vtkPolyDataMapper *coneMapper;
	vtkActor *coneActor;
};
