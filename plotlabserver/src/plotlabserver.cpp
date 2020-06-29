/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#include <plotlabserver/plotlab.h>
#include <plotlablib/zmqobjectsink.h>
#include <plotlablib/plcommands.h>
#include <unistd.h>

using namespace DLR::PlotLab;
using namespace DLR_TS::PlotLab;
bool queryZMQInterface(ZMQObjectSink<PLComPaint>* m_paint,ZMQObjectSink<PLComOther>* m_other, ZMQObjectSink<PLComView>* m_view)
{
	bool received_commands_in_this_run = false;
	std::vector<std::vector<std::pair<std::string,PlotObject*>>> addobjects;
	std::vector<std::vector<std::pair<std::string,LinePlot*>>> appendobjects;
	addobjects.resize(10);
	appendobjects.resize(10);

	while(m_paint->has_data())
	{
		PLComPaint* p = m_paint->pop_data();
		switch(p->comtype)
		{
		case PLComPaint::line:
			{
				PlotObject* obj = new LinePlot(&p->X[0],&p->Y[0],&p->Z[0],p->size);
				obj->parseOptions(p->options);
				addobjects[p->target].push_back(std::make_pair(p->hashtag,obj));
			}
			break;
		case PLComPaint::append:
			{
				LinePlot* obj = new LinePlot(&p->X[0],&p->Y[0],&p->Z[0],p->size);
				obj->parseOptions(p->options);
				appendobjects[p->target].push_back(std::make_pair(p->hashtag,obj));
			}
			break;
		case PLComPaint::patch:
			{
				PlotObject* obj = new PatchPlot(&p->X[0],&p->Y[0],&p->Z[0],p->size);
				obj->parseOptions(p->options);
				addobjects[p->target].push_back(std::make_pair(p->hashtag,obj));
			}
			break;
		case PLComPaint::text:
			{
				PlotObject* obj = new TextPlot(p->X[0],p->Y[0],p->Z[0],std::string(p->options));
				addobjects[p->target].push_back(std::make_pair(p->hashtag,obj));
			}
			break;
		case PLComPaint::texture:
			{
				Figure* f = Figure::getFigure(p->target);
				PlotObject* obj = new TexturePlot(f->getTextureCache(),p->options,p->X[0],p->Y[0],p->Z[0],p->X[1],p->Y[1],p->Z[1]);
				addobjects[p->target].push_back(std::make_pair(p->hashtag,obj));
			}
			break;
		}
		received_commands_in_this_run = true;
		delete p;
	}
	
	for(unsigned int i=0;i<addobjects.size();i++)
	{
		Figure::getFigure(i)->add(addobjects[i]);
	}
	for(unsigned int i=0;i<appendobjects.size();i++)
	{
		Figure::getFigure(i)->append(appendobjects[i]);
	}
	while(m_view->has_data())
	{
		
		PLComView* v = m_view->pop_data();
		switch(v->viewType)
		{
		case PLComView::ViewPosOnly:
			Figure::getFigure(v->target)->setViewPortOffsets(v->targetX,v->targetY,0.0,0.0,false);
			break;
		case PLComView::ViewPosOrientation:
			Figure::getFigure(v->target)->setViewPortOffsets(v->targetX,v->targetY,v->orientDeg,0.0,false);
			break;
		case PLComView::ViewPosOrientZoom:
			Figure::getFigure(v->target)->setViewPortOffsets(v->targetX,v->targetY,v->orientDeg,v->zoom,false);
			break;
		case PLComView::ViewPosZoom:
			Figure::getFigure(v->target)->setViewPortOffsets(v->targetX,v->targetY,0.0,v->zoom,false);
			break;
		case PLComView::disable:
			Figure::getFigure(v->target)->setViewPortOffsets(0.0,0.0,0.0,0.0,true);
		}
	}


	while(m_other->has_data())
	{
		PLComOther* p = m_other->pop_data();

		switch(p->comtype)
		{
		case PLComOther::clear:
			Figure::getFigure(p->target)->clear();
			break;
		case PLComOther::erase:
			Figure::getFigure(p->target)->erase(p->hashtag);
			break;
		case PLComOther::erase_similar:
			Figure::getFigure(p->target)->erase_similar(p->hashtag);
			break;
		case PLComOther::hide:
			Figure::getFigure(p->target)->hide();
			break;
		case PLComOther::hideAxis:
			Figure::getFigure(p->target)->setShowAxis(0);
			break;
		case PLComOther::hideGrid:
			Figure::getFigure(p->target)->setShowGrid(0);
			break;
		case PLComOther::show:
			Figure::getFigure(p->target)->show();
			break;
		case PLComOther::showAxis:
			Figure::getFigure(p->target)->setShowAxis(1);
			break;
		case PLComOther::showGrid:
			Figure::getFigure(p->target)->setShowGrid(1);
			break;
		case PLComOther::xlabel:
			Figure::getFigure(p->target)->setXLabel(p->options);
			break;
		case PLComOther::ylabel:
			Figure::getFigure(p->target)->setYLabel(p->options);
			break;
		case PLComOther::zlabel:
			Figure::getFigure(p->target)->setZLabel(p->options);
			break;
		case PLComOther::title:
			Figure::getFigure(p->target)->setTitle(p->options);
			break;
		case PLComOther::name:
			Figure::getFigure(p->target)->setName(p->options);
			break;
		}

		delete p;
		received_commands_in_this_run = true;
	}


	return received_commands_in_this_run;
}


int main(int argc,char **argv)
{
	ZMQObjectSink<DLR_TS::PlotLab::PLComPaint>* m_paint;
	ZMQObjectSink<DLR_TS::PlotLab::PLComOther>* m_other;
	ZMQObjectSink<DLR_TS::PlotLab::PLComView>*  m_view;
	for (short i=0; i < argc; i++)
	{
		if (strcmp(argv[i], "NOHTTP")==0)
		{
			DLR::PlotLab::Figure::loadFromHTTP = false;
		}
	}

	DLR::PlotLab::Figure::init();


	zmq::context_t* context = new zmq::context_t(1);
	m_paint = new ZMQObjectSink<PLComPaint>(12345);
	m_other = new ZMQObjectSink<PLComOther>(12346);
	m_view  = new ZMQObjectSink<PLComView>(12347);
	std::cout<<"listening on port 12345, 12346 and 12347\n";


	std::cout<<"\n\n\n";

	std::string rotor[]={"|","/","-","\\"};
	int count = 0;

	for(;;)
	{
		if(queryZMQInterface(m_paint,m_other,m_view))
		{
			count++;
			if(count>3)count=0;
			std::cout<<"\b"<<rotor[count];
		}

		usleep(100);
	}
}

