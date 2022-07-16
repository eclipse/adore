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
#pragma once
#include "if_plotlab/plot_border.h"
#include <adore/env/map/map_border_management.h>
#include <adore/if_xodr/xodr2borderbased.h>
#include <adore/if_r2s/r2s2borderbased.h>
#include <adore/mad/centerline.h>
#include <string>
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <fstream>

namespace adore
{
namespace apps
{

/**
 * Read one or more map files supported by ADORe, create Border description, compute PlainXML nodes and edges, write PlainXML files.
 * PlainXML can be used to create SUMO network defintion files. 
 */
namespace BorderType = adore::env::BorderBased::BorderType;
class PlainXMLExporter
{
    private:    
    DLR_TS::PlotLab::FigureStubFactory* figure_factory_;
    DLR_TS::PlotLab::FigureStubZMQ* figure_;
    adore::env::BorderBased::BorderSet globalSet_;
    adore::if_xodr::BorderIDTranslation idTranslation_;
    double min_length_start_;//minimum length of a start edge. a start edge is an edge which has no predecessor
    double maxDistMeter_;
    double maxDistRel_;
    double min_length_;//minimum length for an edge
    double max_length_;//maximum length for an edge before splitting into subcomponents
    bool lateral_aggregate_;//determine whether all borders adjacent to the left are aggregated into one edge
    bool use_constant_width_;
    double constant_width_;
    bool use_lane_shape_;//if true, write shape (=geometry) for each lane

    private:
    typedef adore::env::BorderBased::Border Border;
    typedef adore::env::BorderBased::BorderID BorderID;

    struct ConnectionStep;
    struct Node
    {
        int id_;
        std::unordered_set<Border*> incoming_edge_; //baseline borders ending at node
        std::unordered_set<Border*> outgoing_edge_; //baseline borders starting at node
        std::unordered_set<ConnectionStep*> start_connections_; //connection elements, which start at node
        std::unordered_set<ConnectionStep*> end_connections_; //connection elements, which end at node
        adoreMatrix<double,3,1> position_; //average of all start/end positions
    };
    struct Edge
    {
        std::vector<Border*> represents_;//borders represented by baseline, represents_[0] is the baseline border
        int number_of_lanes_;//number of lanes
        double lane_width_;//average width of lanes
        std::vector<int> ids_;//for splitting of edges, ids of first to last sub-edge are stored in this list
        std::vector<int> split_node_ids_;//for splitting of edges, ids of interim nodes
        std::vector<double> s_;//s values of baseline for intervals, given in percentage of border length
    };
    struct Connection
    {
        Border* from_;//baseline border
        Border* to_;//baseline border
        std::vector<std::pair<int,int>> from_lane_to_lane_; //lane number pairs
    };

    struct ConnectionStep;
    struct ConnectionStep
    {
        Border* connection_border_; //the border used by the step
        Border* edge_border0_;   //if connection step starts at junction fringe => edge_border0_!=0
        Border* edge_border1_;   //if connection step ends at junction fringe => edge_border1_!=0
        ConnectionStep* predecessor_;//if connection step does not start at junction fringe => predecessor!=0
        int depth_;
    };

    std::unordered_set<Node*> nodes_;
    std::unordered_map<Border*,Edge> edges_;//all edges: baseline + a number of dependent borders
    std::unordered_map<Border*,Node*> leads_to_;//baseline border leads to node id
    std::unordered_map<Border*,Node*> originates_at_;//baseline border originates at node id
    std::unordered_map<Border*,Border*> represented_by_;//a border belongs to a baseline border
    std::list<ConnectionStep*> open_connections_;//the open set for connection search
    std::vector<ConnectionStep*> closed_connections_;//the closed set for connection search
    std::unordered_map<std::string,std::set<Node*>> merge_nodes_;//the nodes which shall be merged into one intersection, according to xodr connecting road information and junction id
    std::unordered_map<Node*,int> node_clusters_;//maps from node to its cluster id
    int cluster_count = 0;//id of the next cluster
    std::unordered_set<Edge*> deleted_edges_;//edges, which were filtered out and deleted due to some sort of deficiency

    private:
    bool enable_plotting_;
    public:
    PlainXMLExporter()
    {
        figure_factory_ = 0;
        figure_ = 0;
        enable_plotting_ = false;
        min_length_start_ = 1.0;
        min_length_ = 0.1;
        max_length_ = 5.0;
        maxDistMeter_ = 1.0;
        maxDistRel_ = 0.2;
        lateral_aggregate_ = true;
        use_constant_width_ = true;
        constant_width_ = 3.0;
        use_lane_shape_ = true;
    }
    void run(int argc,char **argv)
    {
        std::cout<<"PlainXMLExporter------------------------------------------------------------------"<<std::endl;
        std::cout<<"usage: PlainXMLExporter [plot] infile1[,transform] [infile2[,transform]] ... outfile"<<std::endl;
        std::cout<<"* will create files outfile.nod.xml, outfile.edg.xml and outfile.con.xml"<<std::endl;
        std::cout<<"* will plot with plotlab, if first argument [plot] is given."<<std::endl;
        std::cout<<"Create net.xml for SUMO as follows:"<<std::endl;
        std::cout<<"./netconvert --node-files=[outfile].nod.xml --edge-files=[outfile].edg.xml --connection-files=[outfile].con.xml --output-file=[outfile].net.xml  -R --geometry.remove.width-tolerance 1 --junctions.join --junctions.join-dist 10 -l [outfile].txt "<<std::endl;
        std::cout<<"----------------------------------------------------------------------------------"<<std::endl;

        std::string output_prefix(argv[argc-1]);

        for(int i=1;i<argc-1;i++)
        {
            std::string trackConfig(argv[i]);

            if(std::strcmp(trackConfig.c_str(),"plot")==0)
            {
                enable_plotting_ = true;
                continue;
            }

            /* reading of single track configuration, comma separated */
            bool transform = false;
            std::string filename = "";
            {
                std::string token = "";
                std::stringstream trackConfigStream(trackConfig);
                while(std::getline(trackConfigStream,token,','))
                {
                    if(token.compare("transform")==0)
                    {
                        transform = true;
                    }
                    else
                    {
                        filename = token;
                    }
                }
            }

            std::cout<<"loading track: "<<filename.c_str()<<", "<<(transform?"with":"without")<<" transformation"<<std::endl;
            adore::env::BorderBased::BorderSet partialSet;        
            if(filename.substr(filename.length()-4,std::string::npos).compare("xodr")==0)
            {
                std::cout<<"OpenDrive loader..."<<std::endl;
                try
                {
                    adore::if_xodr::XODR2BorderBasedConverter converter;
                    converter.convert(filename.c_str(),&partialSet,transform,&idTranslation_);
                }
                catch(const std::exception& e)
                {
                    std::cout<<"Could not parse file"<<std::endl;
                    std::cout << e.what() << '\n';
                    return;
                }
            }
            else if(filename.substr(filename.length()-3,std::string::npos).compare("r2s")==0)
            {
                std::cout<<"Road2Simulation simple feature loader..."<<std::endl;
                try
                {
                    if_r2s::R2S2BorderBasedConverter converter;
                    converter.convert(filename+"r",filename+"l", partialSet);                    
                }
                catch(const std::exception& e)
                {
                    std::cout<<"Could not parse file"<<std::endl;
                    std::cout << e.what() << '\n';
                    return;
                }
                
            }
            else
            {
            	std::cout<<"unknown file ending: "<<filename<<std::endl;
    	     }
            /* add partial map to global map */
            auto its = partialSet.getAllBorders();
            for(;its.first!=its.second;its.first++)
            {
                globalSet_.insert_border(its.first->second);
            }
            partialSet.setIsOwner(false);
            

    
        }

        std::cout<<"...successfully loaded all files"<<std::endl;
        if(enable_plotting_)
        {
            figure_factory_ = new DLR_TS::PlotLab::FigureStubFactory();
            figure_ = (DLR_TS::PlotLab::FigureStubZMQ*) figure_factory_->createFigureStub(1);
            figure_->show();
            PLOT::plotBorderSet(globalSet_,figure_);
        }

        std::cout<<"converting to node/edge format..."<<std::endl;
        
        std::unordered_set<BorderType::TYPE> valid_types;
        valid_types.emplace(BorderType::DRIVING);

        //create edges: find all baseline borders in globalSet
        std::cout<<"creating edges"<<std::endl;
        for(auto it=globalSet_.getAllBorders();it.first!=it.second;it.first++)
        {
            Border* border = it.first->second;
            Border* left = globalSet_.getLeftNeighbor(border);
            Border* right = globalSet_.getRightNeighbor(border);

            //reasons for exclusion of border
            if(represented_by_.find(border)!=represented_by_.end())continue;//already represented
            if(valid_types.find(border->m_type)==valid_types.end())continue;//not a valid type
            if(lateral_aggregate_ && right!=0 && right->m_type==border->m_type)continue;//not a baseline border
            if(left==0)continue;//this border is a wierd singleton
            if(idTranslation_.isInJunction(border->m_id))continue;//inside junction: use as connection
            if(border->getLength()<min_length_)continue;//border is too short

            //this is a valid baseline border: create edge
            Edge e;
            left = border;
            if(lateral_aggregate_) 
            {
                do
                {
                    Border* nextleft = globalSet_.getLeftNeighbor(left);
                    if(nextleft!=0 && left->m_type==border->m_type)
                    {
                        e.represents_.push_back(left);
                        represented_by_.emplace(left,border);
                        left = nextleft;
                    }
                    else
                    {
                        left = 0;
                    }                
                }while(left!=0);
            }
            else
            {
                //border only represents itself
                e.represents_.push_back(border);
                represented_by_.emplace(border,border);
            }
            
            e.number_of_lanes_ = e.represents_.size();
            double dx = e.represents_[0]->m_id.m_first.m_X -  e.represents_[e.number_of_lanes_-1]->m_left->m_first.m_X;
            double dy = e.represents_[0]->m_id.m_first.m_Y -  e.represents_[e.number_of_lanes_-1]->m_left->m_first.m_Y;
            e.lane_width_ = std::sqrt(dx*dx+dy*dy)/(double)e.number_of_lanes_;
            edges_.emplace(border,e);
        }

        //create atomic from-to-links
        std::cout<<"creating atomic nodes"<<std::endl;
        for(auto it=globalSet_.getAllBorders();it.first!=it.second;it.first++)
        {
            Border* border = it.first->second;

            //reasons for exclusion of border
            if(represented_by_.find(border)==represented_by_.end())continue;//previously not selected

            for(auto successors = globalSet_.getSuccessors(border);
                successors.current()!=successors.end();
                successors.current()++)
            {
                Border* next = successors.current()->second;
                if(border->isContinuousPredecessorOf(next,maxDistMeter_,maxDistRel_))
                {
                    //resons for exclusion of next
                    if(represented_by_.find(next)==represented_by_.end()
                    && edges_.find(next)==edges_.end())continue;//previously not selected

                    //there is a link here
                    Node* n = new Node();
                    n->incoming_edge_.emplace(border);
                    n->outgoing_edge_.emplace(next);
                    leads_to_.emplace(border,n);
                    originates_at_.emplace(next,n);
                    nodes_.emplace(n);
                }
            }
        }


        //merge nodes, which share a baseline link
        std::cout<<"aggregating nodes"<<std::endl;
        bool changed = true;
        std::unordered_set<Node*> node_done_;
        while(changed)
        {
            changed = false;
            std::unordered_set<Node*> equivalent_nodes;
            for(auto it = nodes_.begin();it!=nodes_.end();it++)
            {
                Node* n = *it;
                if(node_done_.find(n)==node_done_.end())
                {
                    node_done_.emplace(n);
                    //incoming nodes
                    for(auto it2 = n->incoming_edge_.begin();it2!=n->incoming_edge_.end();it2++)
                    {
                        Border* border = *it2;
                        Border* baseline = represented_by_[border];
                        auto it3 = leads_to_.find(baseline);
                        if(it3!=leads_to_.end() && it3->second!=n)
                        {
                            equivalent_nodes.emplace(it3->second);
                        }
                    }
                    //outgoing nodes
                    for(auto it2 = n->outgoing_edge_.begin();it2!=n->outgoing_edge_.end();it2++)
                    {
                        Border* border = *it2;
                        Border* baseline = represented_by_[border];
                        auto it3 = originates_at_.find(baseline);
                        if(it3!=originates_at_.end() && it3->second!=n)
                        {
                            equivalent_nodes.emplace(it3->second);
                        }
                    }
                    //if there are equivalent nodes, process those
                    for(auto it2 = equivalent_nodes.begin();it2!=equivalent_nodes.end();it2++)
                    {
                        Node* m = *it2;
                        //copy incoming and outgoing borders
                        for(auto it3 = m->incoming_edge_.begin();it3!=m->incoming_edge_.end();it3++)
                        {
                            n->incoming_edge_.emplace(*it3);
                            leads_to_[*it3] = n;//replace m with n
                        }
                        for(auto it3 = m->outgoing_edge_.begin();it3!=m->outgoing_edge_.end();it3++)
                        {
                            n->outgoing_edge_.emplace(*it3);
                            originates_at_[*it3] = n;//replace m with n
                        }
                        delete m;
                    }
                    if(equivalent_nodes.size()>0)
                    {
                        changed = true;
                        break;
                    }
                }
            }
            for(auto it=equivalent_nodes.begin();it!=equivalent_nodes.end();it++)
            {
                nodes_.erase(*it);//remove the equivalent nodes from the nodes_ set
            }
        }

        //create dead-end nodes for edges, which have no start/end
        for(auto it = edges_.begin();it!=edges_.end();it++)
        {
            Border* baseline = it->first;
            Edge* e = &it->second;
            if(originates_at_.find(baseline)==originates_at_.end())
            {
                Node* n = new Node();
                nodes_.emplace(n);
                for(auto it2 = e->represents_.begin();it2!=e->represents_.end();it2++)
                {
                    n->outgoing_edge_.emplace(*it2);
                    originates_at_.emplace(*it2,n);
                }
            }
            if(leads_to_.find(baseline)==leads_to_.end())
            {
                Node* n = new Node();
                nodes_.emplace(n);
                for(auto it2 = e->represents_.begin();it2!=e->represents_.end();it2++)
                {
                    n->incoming_edge_.emplace(*it2);
                    leads_to_.emplace(*it2,n);
                }
            }
        }

        //find connections
        for(auto it=globalSet_.getAllBorders();it.first!=it.second;it.first++)
        {
            Border* border = it.first->second;
            Border* left = globalSet_.getLeftNeighbor(border);

            //reasons for exclusion of border
            if(represented_by_.find(border)!=represented_by_.end())continue;//already represented
            if(valid_types.find(border->m_type)==valid_types.end())continue;//not a valid type
            if(left==0)continue;//this border is a wierd singleton
            if(!idTranslation_.isInJunction(border->m_id))continue;//inside junction: use as connection
            std::string junctionID = idTranslation_.getJunctionID(border->m_id);

            //walk through predecessors and see whether the junction border can be connected to a regular border
            for(auto it2 = globalSet_.getPredecessors(border);it2.current()!=it2.end();it2.current()++)
            {
                Border* pre = it2.current()->second;
                if(represented_by_.find(pre)==represented_by_.end())continue;
                if(!pre->isContinuousPredecessorOf(border,maxDistMeter_,maxDistRel_))continue;
                //junction border has a known edge as predecessor
                ConnectionStep* c = new ConnectionStep();
                c->connection_border_ = border;
                c->edge_border0_ = pre;
                c->edge_border1_ = 0;
                c->predecessor_ = 0;
                c->depth_ = 0;
                open_connections_.push_back(c);

                //remember junctionid for node merging
                Node* n = leads_to_[pre];
                if(merge_nodes_.find(junctionID)==merge_nodes_.end())
                {
                    std::set<Node*> empty_set;
                    merge_nodes_.emplace(junctionID,empty_set);
                }
                merge_nodes_[junctionID].emplace(n);
            }
        }

        //breadth-first search for connections
        int maxDepth = 15;
        while(open_connections_.size()>0)
        {
            ConnectionStep* c = open_connections_.front();
            open_connections_.pop_front();
            closed_connections_.push_back(c);
            std::vector<ConnectionStep*> children;
            //std::cout<<"open list size: "<<open_connections_.size()<<", depth="<<c->depth_<<std::endl;

            //search for connected edges
            for(auto it = globalSet_.getSuccessors(c->connection_border_);it.current()!=it.end();it.current()++)
            {
                Border* next = it.current()->second;
                if(c->connection_border_->isContinuousPredecessorOf(next,maxDistMeter_,maxDistRel_))
                {
                    if(represented_by_.find(next)!=represented_by_.end())
                    {
                        //found a goal
                        c->edge_border1_ = next;
                        //std::cout<<"found a goal"<<std::endl;

                        //connect start and end node, if they are not already in the same cluster
                        Node* n1 = originates_at_[represented_by_[next]];
                        ConnectionStep* c0 = c;
                        while(c0->predecessor_!=0)c0=c0->predecessor_;
                        Node* n0 = leads_to_[represented_by_[c0->edge_border0_]];
                        n0->start_connections_.emplace(c0);
                        n1->end_connections_.emplace(c);

                        auto it0 = node_clusters_.find(n0);
                        auto it1 = node_clusters_.find(n1);
                        if(it0==node_clusters_.end() && it1==node_clusters_.end())
                        {
                            node_clusters_.emplace(n0,cluster_count);
                            node_clusters_.emplace(n1,cluster_count);
                            cluster_count ++;
                        }
                        else if(it0==node_clusters_.end())
                        {
                            node_clusters_.emplace(n0,it1->second);
                        }
                        else if(it1==node_clusters_.end())
                        {
                            node_clusters_.emplace(n1,it0->second);
                        }
                        else
                        {
                            int replace = it1->second;
                            int by = it0->second;
                            for(auto it2=node_clusters_.begin();it2!=node_clusters_.end();it2++)
                            {
                                if(it2->second==replace)node_clusters_[it2->first]=by;                                
                            }
                        }
                        break;
                    }
                    else if(c->depth_<maxDepth)
                    {
                        //found a child
                        ConnectionStep* cc = new ConnectionStep();
                        cc->connection_border_ = next;
                        cc->edge_border0_ = 0;
                        cc->edge_border1_ = 0;
                        cc->predecessor_ = c;
                        cc->depth_ = c->depth_ + 1;
                        children.push_back(cc);
                    }                    
                }
            }

            if(c->edge_border1_==0)
            {
                for(auto cc:children)
                {
                    open_connections_.push_back(cc);
                }
            }
            else
            {
                for(auto cc:children)
                {
                    delete cc;
                }
            }
        }

        //cut edges which are too long
        int edgeid = 0;//start counting edge ids here
        int nodeid = 0;//start counting node ids here: intermediate nodes in edges
        for(auto it = edges_.begin();it!=edges_.end();it++)
        {
            Edge* e = &it->second;
            Border* b = it->first;
            e->ids_.push_back(edgeid++);
            e->s_.push_back(0.0);
            for(double s = max_length_;s + max_length_<=b->getLength();s+=max_length_)
            {
                e->ids_.push_back(edgeid++);
                e->s_.push_back(s/b->getLength());
                e->split_node_ids_.push_back(nodeid++);
            }
            e->s_.push_back(1.0);
        }


        //write nodes file
        std::cout<<"Writing to file "<<output_prefix<<".nod.xml ..."<<std::endl;
        try
        {
            std::ofstream nodfile;
            nodfile.open (output_prefix + ".nod.xml");
            nodfile.setf(std::ios_base::fixed, std::ios_base::floatfield);
            nodfile.precision(2);
            nodfile << "<nodes>"<<std::endl;
            for(auto it = nodes_.begin();it!=nodes_.end();it++)
            {
                Node* n = *it;
                if(node_clusters_.find(n)==node_clusters_.end())//=>independet node
                {
                    n->id_ = nodeid++;
                    double x = 0.0;
                    double y = 0.0;
                    int count = 0;
                    //compute center of incoming/outgoing
                    for(auto it2 = n->incoming_edge_.begin();it2!=n->incoming_edge_.end();it2++)
                    {
                        Border* b = *it2;
                        x+=b->m_id.m_last.m_X;
                        y+=b->m_id.m_last.m_Y;
                        count++;
                    }
                    for(auto it2 = n->outgoing_edge_.begin();it2!=n->outgoing_edge_.end();it2++)
                    {
                        Border* b = *it2;
                        x+=b->m_id.m_first.m_X;
                        y+=b->m_id.m_first.m_Y;
                        count++;
                    }
                    nodfile << "\t<node id=\""<<n->id_<<"\" "
                            << "x=\""<<x/(double)count<<"\" "
                            << "y=\""<<y/(double)count<<"\" "
                            << "type=\"priority\" />"
                            << std::endl;
                }
            }
            nodfile << std::endl;

            //write the node clusters
            std::cout<<"writing "<<cluster_count<<" joined nodes"<<std::endl;
            for(int i=0;i<cluster_count;i++)
            {
                int nodes_in_cluster = 0;
                double x = 0.0;
                double y = 0.0;
                int connectors_in_cluster = 0;
                nodeid++;

                for(auto it = node_clusters_.begin();it!=node_clusters_.end();it++)
                {
                    if(it->second==i)
                    {
                        Node* n = it->first;
                        nodes_in_cluster ++;
                        n->id_ = nodeid;

                        //compute center of incoming/outgoing
                        for(auto it2 = n->incoming_edge_.begin();it2!=n->incoming_edge_.end();it2++)
                        {
                            Border* b = *it2;
                            x+=b->m_id.m_last.m_X;
                            y+=b->m_id.m_last.m_Y;
                            connectors_in_cluster++;
                        }

                        for(auto it2 = n->outgoing_edge_.begin();it2!=n->outgoing_edge_.end();it2++)
                        {
                            Border* b = *it2;
                            x+=b->m_id.m_first.m_X;
                            y+=b->m_id.m_first.m_Y;
                            connectors_in_cluster++;
                        }
                    }
                }
                if(nodes_in_cluster>0)
                {
                    nodfile << "\t<node id=\""<<nodeid<<"\" "
                            << "x=\""<<x/(double)connectors_in_cluster<<"\" "
                            << "y=\""<<y/(double)connectors_in_cluster<<"\" "
                            << "type=\"priority\" />"
                            << std::endl;
                }
            }

            //write the subnodes of all edges
            for(auto it=edges_.begin();it!=edges_.end();it++)
            {
                Edge* e = &it->second;
                Border* baseline = it->first;
                for(int i=0;i<e->split_node_ids_.size();i++)
                {
                    double seval = e->s_[i+1]*baseline->getLength()+baseline->m_path->limitLo();
                    // double seval = adore::mad::bound(baseline->m_path->limitLo(),e->s_[i+1]+baseline->m_path->limitLo(),baseline->m_path->limitHi());
                    auto p = baseline->m_path->f(seval);
                    int id = e->split_node_ids_[i];
                    nodfile << "\t<node id=\""<<id<<"\" "
                            << "x=\""<< (p(0)) <<"\" "
                            << "y=\""<< (p(1)) <<"\" "
                            << "type=\"priority\" />"
                            << std::endl;
                }
            }

            nodfile << "</nodes>"<<std::endl;
            nodfile.close();       
            std::cout<<"done"<<std::endl;     
        }
        catch(const std::exception& e)
        {
            std::cout << e.what() << '\n';
            std::cout<<"Failed to write to file."<<std::endl;
        }

        // write the .edg.xml file
        std::cout<<"Writing to file "<<output_prefix<<".edg.xml ..."<<std::endl;
        try
        {
            adore::mad::LLinearPiecewiseFunctionM<double,3> lane_buffer;
            std::ofstream edgfile;
            edgfile.open (output_prefix + ".edg.xml");
            edgfile.setf(std::ios_base::fixed, std::ios_base::floatfield);
            edgfile.precision(2);
            edgfile << "<edges>"<<std::endl;

            for(auto it = edges_.begin();it!=edges_.end();it++)
            {
                Border* baseline = it->first;
                Edge* e = &it->second;

                //spreadType=right => represent the left of the border array
                Border* leftest = globalSet_.getLeftNeighbor(e->represents_.back());
                Border* rightest = e->represents_.front();
                if(leftest==0)
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" has no left hand border - skipping"<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }
                if(leftest->m_path==0)
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" has no left hand path - skipping"<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }
                if(baseline==0)
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" has no baseline - skipping"<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }
                if(originates_at_.find(baseline)==originates_at_.end())
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" has no baseline link to from-node"<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }
                if(leads_to_.find(baseline)==leads_to_.end())
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" has no baseline link to to-node"<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }
                Node* n0 = originates_at_[baseline];
                Node* n1 = leads_to_[baseline];
                if(     n0->incoming_edge_.size()==0 && n0->end_connections_.size()==0
                    &&  n1->outgoing_edge_.size()==0 && n1->start_connections_.size()==0
                    &&  node_clusters_.find(n0)==node_clusters_.end()
                    &&  node_clusters_.find(n1)==node_clusters_.end() )
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" starts and ends at unconnected nodes."<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }
                if( baseline->getLength()<min_length_start_  
                    && n0->incoming_edge_.size()==0 && n0->end_connections_.size()==0
                    && node_clusters_.find(n0)==node_clusters_.end() )
                {
                    std::cout<<"Edge with id "<<e->ids_.front()<<" is a start edge and very short."<<std::endl;
                    deleted_edges_.emplace(e);
                    continue;
                }

                for(int i=0;i<e->ids_.size();i++)
                {
                    
                    int eid = e->ids_[i];
                    int fromid = (i==0?originates_at_[baseline]->id_:e->split_node_ids_[i-1]);
                    int toid = (i+1==e->ids_.size()?leads_to_[baseline]->id_:e->split_node_ids_[i]);

                    if(fromid==toid)
                    {
                        std::cout<<"Edge with id "<<eid<<" has fromid="<<fromid<<" and toid="<<toid<<". "<<"It's index is "<<i<<", with a total of "<<e->ids_.size()<<"."<<std::endl;
                        deleted_edges_.emplace(e);
                        continue;
                    }

                    if(use_lane_shape_)
                    {
                        edgfile << "\t<edge id=\""<< eid <<"\" "
                                << "numLanes=\""<<e->represents_.size()<<"\" " 
                                << "from=\""<< fromid <<"\" "
                                << "to=\""<< toid << "\" >\n";

                        for(int k=0;k<e->represents_.size();k++)
                        {
                            Border* right = e->represents_[k];
                            Border* left = globalSet_.getLeftNeighbor(right);
                            int point_count = adore::mad::computeCenterline(*left->m_path,*right->m_path,lane_buffer);
                            lane_buffer.getData()(0,0) = 0.0;//start value
                            for(int l=1;l<point_count;l++)
                            {
                                double d = 0.0;
                                double dd = 0.0;
                                for(int m=1;m<=3;m++)
                                {
                                    d = lane_buffer.getData()(m,l)-lane_buffer.getData()(m,l-1);
                                    dd+=d*d;
                                }
                                lane_buffer.getData()(0,l) = (std::sqrt)(dd) + lane_buffer.getData()(0,l-1);
                            }
                            double lane_length = lane_buffer.getData()(0,point_count-1);


                            std::stringstream laneid;
                            laneid<<eid<<"_"<<k;
                            
                            edgfile << "\t\t<lane index=\""
                                    // << laneid.str()  //id?
                                    << k                //or index?
                                    << "\" shape=\"";

                            //write the shape
                            int dim = 2;//output number of dimensions
                            int n=0;static const int N=500;
                            double X[N];
                            double Y[N];
                            for(double  s=e->s_[i]*lane_length;
                                        s<e->s_[i+1]*lane_length;
                                        s+=1.0)
                            {
                                // double seval = adore::mad::bound(leftest->m_path->limitLo(),s+leftest->m_path->limitLo(),leftest->m_path->limitHi());
                                auto p = lane_buffer.f(s);
                                if(n<N)
                                {
                                    X[n] = p(0);
                                    Y[n] = p(1);
                                    n++;
                                }
                                for(int j=0;j<dim;j++)
                                {
                                    edgfile<<p(j)<<(j<dim-1?",":" ");
                                }
                            }
                            //last point
                            auto p = lane_buffer.f(e->s_[i+1]*lane_length);
                            if(n<N)
                            {
                                X[n] = p(0);
                                Y[n] = p(1);
                                n++;
                            }
                            //put shape in edge file
                            for(int j=0;j<dim;j++)
                            {
                                edgfile<<p(j)<<(j<dim-1?",":"");
                            }
                            //plot the shape
                            if(enable_plotting_)
                            {
                                figure_->plot("lane_shape_"+laneid.str(),X,Y,2.0,n,"LineWidth=1;LineColor=0,0,1");
                                X[1]=X[n-1];
                                Y[1]=Y[n-1];
                                figure_->plot("lane_shape_"+laneid.str()+"/marker",X,Y,2.0,2,"LineStyle=none;MarkerSize=9;LineColor=0,0,1");
                            }
                            //compute width at beginning of subsegment
                            double seval = e->s_[i]*left->getLength()+left->m_path->limitLo();
                            auto pleft = left->m_path->f(seval);
                            seval = e->s_[i]*right->getLength()+right->m_path->limitLo();
                            auto pright = right->m_path->f(seval);
                            double dx = pleft(0)-pright(0);
                            double dy = pleft(1)-pright(1);
                            double width = (std::sqrt)(dx*dx+dy*dy);

                            edgfile <<"\" width=\""
                                    <<width
                                    << "\"/>\n";
                        }

                        edgfile << "\t</edge>\n";
                    }
                    else
                    {
                        
                        edgfile << "\t<edge id=\""<< eid <<"\" "
                                << "from=\""<< fromid <<"\" "
                                << "to=\""<< toid << "\" "
                                << "spreadType=\"right\" "
                                << "shape=\"";

                        //write the shape
                        int dim = 2;//output number of dimensions
                        for(double s=e->s_[i]*leftest->getLength()+leftest->m_path->limitLo();
                                s<e->s_[i+1]*leftest->getLength()+leftest->m_path->limitLo();
                                s+=1.0)
                        {
                            // double seval = adore::mad::bound(leftest->m_path->limitLo(),s+leftest->m_path->limitLo(),leftest->m_path->limitHi());
                            auto p = leftest->m_path->f(s);
                            for(int j=0;j<dim;j++)
                            {
                                edgfile<<p(j)<<(j<dim-1?",":" ");
                            }
                        }
                        double seval = e->s_[i+1]*leftest->getLength()+leftest->m_path->limitLo();
                        // double seval = adore::mad::bound(leftest->m_path->limitLo(),e->s_[i+1]+leftest->m_path->limitLo(),leftest->m_path->limitHi());
                        auto p = leftest->m_path->f(seval);
                        for(int j=0;j<dim;j++)
                        {
                            edgfile<<p(j)<<(j<dim-1?",":" ");
                        }
                        //compute width at beginning of subsegment
                        seval = e->s_[i]*leftest->getLength()+leftest->m_path->limitLo();
                        // seval = adore::mad::bound(leftest->m_path->limitLo(),e->s_[i]+leftest->m_path->limitLo(),leftest->m_path->limitHi());
                        auto pleft = leftest->m_path->f(seval);
                        seval = e->s_[i]*rightest->getLength()+rightest->m_path->limitLo();
                        // seval = adore::mad::bound(rightest->m_path->limitLo(),e->s_[i]+rightest->m_path->limitLo(),rightest->m_path->limitHi());
                        auto pright = rightest->m_path->f(seval);
                        double dx = pleft(0)-pright(0);
                        double dy = pleft(1)-pright(1);
                        double avrgwidth = std::sqrt(dx*dx+dy*dy)/(double)e->number_of_lanes_;

                        if(use_constant_width_)avrgwidth = constant_width_;

                        edgfile <<"\" numLanes=\""
                                <<e->number_of_lanes_
                                <<"\" width=\""
                                <<avrgwidth
                                <<"\"/>"<<std::endl;
                    }
                }
            }

            edgfile << "</edges>"<<std::endl;
            edgfile.close();       
            std::cout<<"done"<<std::endl;     
        }
        catch(const std::exception& e)
        {
            std::cout << e.what() << '\n';
            std::cout<<"Failed to write to file."<<std::endl;
        }

        // write the .con.xml file
        std::cout<<"Writing to file "<<output_prefix<<".con.xml ..."<<std::endl;
        try
        {
            std::ofstream confile;
            confile.open (output_prefix + ".con.xml");
            confile.setf(std::ios_base::fixed, std::ios_base::floatfield);
            confile.precision(2);

            confile << "<connections>"<<std::endl;
            int connection_count = 0;
            for(auto it = closed_connections_.begin();it!=closed_connections_.end();it++)
            {
                ConnectionStep* c = *it;
                if(c->edge_border1_!=0)//goal edge
                {
                    connection_count ++;
                    Border* b1 = c->edge_border1_;
                    Border* baseline1 = represented_by_[b1];
                    Edge* e1 = &edges_[baseline1];
                    int l1 = 0;
                    for(int i=0;i<e1->represents_.size();i++)
                    {
                        if(e1->represents_[i]==b1)
                        {
                            //l1 = e1->represents_.size()-i-1;
                            l1 = i;
                            break;
                        }
                    }

                    Edge* e0 = 0;
                    Border* b0 = 0;
                    Border* baseline0;
                    int l0;

                    std::vector<adoreMatrix<double,3,1>> shape;
                    while(c!=0)
                    {
                        adore::mad::LLinearPiecewiseFunctionM<double,3> path;
                        Border* right = c->connection_border_;
                        Border* left = globalSet_.getLeftNeighbor(right);
                        int point_count = adore::mad::computeCenterline(*left->m_path,*right->m_path,path);
                        auto p = path.getData();
                        for(int i=point_count-1;i>=0;i--)
                        {
                            shape.push_back(dlib::subm(p,dlib::range(1,3),dlib::range(i,i)));                            
                        }
                        if(c->edge_border0_!=0)
                        {
                            b0 = c->edge_border0_;
                            baseline0 = represented_by_[b0];
                            e0 = &edges_[baseline0];
                            for(int i=0;i<e0->represents_.size();i++)
                            {
                                if(e0->represents_[i]==b0)
                                {
                                    // l0 = e0->represents_.size()-i-1;
                                    l0 = i;
                                    break;
                                }
                            }
                            c = 0;
                            
                        }
                        else
                        {
                            c = c->predecessor_;
                        }
                        
                    }
                    if( deleted_edges_.find(e0)!=deleted_edges_.end()
                     || deleted_edges_.find(e1)!=deleted_edges_.end())
                    {
                        std::cout<<"Connection from "<<e0->ids_.back()<<" to "<<e1->ids_.front()<<" not written, as one of the edges was deleted."<<std::endl;
                        continue;
                    }
                    double front_dx = shape.back()(0)-(b0->m_id.m_last.m_X+b0->m_left->m_last.m_X)*0.5;
                    double front_dy = shape.back()(1)-(b0->m_id.m_last.m_Y+b0->m_left->m_last.m_Y)*0.5;
                    double back_dx = (b1->m_id.m_first.m_X+b1->m_left->m_first.m_X)*0.5 - shape.front()(0);
                    double back_dy = (b1->m_id.m_first.m_Y+b1->m_left->m_first.m_Y)*0.5 - shape.front()(1);

                    if(front_dx*front_dx+front_dy*front_dy>25.0)
                    {
                        int asdf=0;
                    }
                    if(back_dx*back_dx+back_dy*back_dy>25.0)
                    {
                        int asdf=0;
                    }

                    confile<<"\t <connection "
                            <<"from=\""<<e0->ids_.back()<<"\" " 
                            <<"to=\""<<e1->ids_.front()<<"\" "
                            <<"fromLane=\""<<l0<<"\" "
                            <<"toLane=\""<<l1<<"\" "
                            <<"shape=\"";
                    for(auto it2=shape.rbegin();it2!=shape.rend();it2++)
                    {
                        confile<<(*it2)(0)<<","<<(*it2)(1)<<" ";
                    }
                    confile<<"\"/>"<<std::endl;

                    if(enable_plotting_)
                    {
                        int k=0;static const int N=200;
                        double X[N];
                        double Y[N];
                        for(int i=0;i<shape.size();i++)
                        {
                            X[k]=shape[i](0);
                            Y[k]=shape[i](1);
                            k++;
                            if(k==N||i+1==shape.size())
                            {
                                std::stringstream ss;
                                ss<<"connection/"<<connection_count<<"/"<<i;
                                figure_->plot(ss.str(),X,Y,2.0,k,"LineWidth=1;LineColor=1,0,0");
                                X[0]=X[k-1];
                                Y[0]=Y[k-1];
                                k=1;                                
                            }                            
                        }
                        std::stringstream ss;
                        ss<<"connection/"<<connection_count<<"/markers";
                        X[0] = shape.front()(0);
                        Y[0] = shape.front()(1);
                        X[1] = shape.back()(0);
                        Y[1] = shape.back()(1);
                        figure_->plot(ss.str(),X,Y,2.0,2,"LineStyle=none;MarkerSize=9;LineColor=1,0,0");


                        //plot connection to b0
                        X[0] = (b0->m_id.m_last.m_X+b0->m_left->m_last.m_X)*0.5;
                        Y[0] = (b0->m_id.m_last.m_Y+b0->m_left->m_last.m_Y)*0.5;
                        X[1] = shape.back()(0);
                        Y[1] = shape.back()(1);
                        figure_->plot(ss.str()+"frontend",X,Y,2.0,2,"LineWidth=5;LineColor=0,0.7,0;MarkerSize=9");


                        //plot connection to b1
                        X[0] = shape.front()(0);
                        Y[0] = shape.front()(1);
                        X[1] = (b1->m_id.m_first.m_X+b1->m_left->m_first.m_X)*0.5;
                        Y[1] = (b1->m_id.m_first.m_Y+b1->m_left->m_first.m_Y)*0.5;
                        figure_->plot(ss.str()+"backend",X,Y,2.0,2,"LineWidth=5;LineColor=0.7,0,0.7;MarkerSize=9");

                    }

                }
            }
            confile << "</connections>"<<std::endl;
        }
        catch(const std::exception& e)
        {
            std::cout << e.what() << '\n';
            std::cout<<"Failed to write to file."<<std::endl;
        }


 
        std::cin.get();
    }
};

}
}
