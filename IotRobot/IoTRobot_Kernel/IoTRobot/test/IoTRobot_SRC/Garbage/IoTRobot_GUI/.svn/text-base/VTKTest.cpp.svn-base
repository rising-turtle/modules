#include "VTKTest.h"

myVTKApp::myVTKApp(HWND parent,int x,int y,int nWidth,int nHeight)
{
	this->renderer=vtkRenderer::New();
	this->renwin=vtkRenderWindow::New();
	this->renwin->AddRenderer(this->renderer);

	this->renwin->SetParentId(parent);
	this->iren=vtkRenderWindowInteractor::New();
	this->iren->SetRenderWindow(this->renwin);

	this->cone=vtkConeSource::New();
	this->cone->SetHeight(3.0);
	this->cone->SetRadius(1.0);
	this->cone->SetResolution(10);
	this->coneMapper=vtkPolyDataMapper::New();
	this->coneMapper->SetInput(this->cone->GetOutput());
	this->coneActor=vtkActor::New();
	this->coneActor->SetMapper(this->coneMapper);

	this->renderer->AddActor(this->coneActor);
	this->renderer->SetBackground(0.2,0.4,0.3);
	this->renwin->SetPosition(x,y);
	this->renwin->SetSize(nWidth,nHeight);
	this->renwin->Render();

}

myVTKApp::~myVTKApp()
{
	renwin->Delete();
	renderer->Delete();
	iren->Delete();
	cone->Delete();
	coneMapper->Delete();
	coneActor->Delete();
}



