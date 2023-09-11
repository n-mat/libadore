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
 *   Reza Dariani - initial API and implementation
 ********************************************************************************/

#pragma once
#include <adore/fun/search_grid.h>
#include <adore/fun/A_star.h>
#include <adore/fun/node.h>
#include <adore/fun/dubins_curve.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <adore/fun/tree_builder.h>
//#include <adore/mad/catmull_rom_splines.h>
#include <adore/fun/trajectory_smoothing.h>
#include <adore/fun/setpointrequest.h>

namespace adore
{
	namespace fun
	{
	   /**
	   *  ----------------------
	   */
       	class Hybrid_A_Star
		{
            private:
            long double pi;
            TrajectorySmoothing* smoothing;
            std::string RED= "LineStyle=none;PointSize=4;LineColor=1,0,0";
            static const int nH_Type = 3;  //non holonomic
            static const int H_Type = 2;  //holonomic
            GRID<adore::fun::Node<H_Type,int>> H_GRID;
             adore::fun::TreeBuilder<nH_Type,double> Tree;
            //ArrayFormGrid<adore::fun::Node<H_Type,int>> HA_GRID; //array format
            DubinsCurve dubins;
            A_Star a_star;
            long double HeadingResolutionRad;
            int iteration;
            adore::fun::SetPointRequest spr;
            struct compare
            {
                bool operator()(Node<nH_Type,double>* n1, Node<nH_Type,double>* n2) const
                {
                    return n1->C > n2->C;
                }
            };
            public:

            Hybrid_A_Star(TrajectorySmoothing* smoothing)
            {
                this->smoothing = smoothing;
                pi = 3.141592653589793;


            }
            void setSize(int Width, int Length)
            {
                H_GRID.resize(Width, Length);
            }
            bool plan(GRID<Node<nH_Type,double>>* grid,adore::env::OccupanyGrid* og, adore::fun::CollisionCheckOffline* cco,Node<nH_Type,double>* Start, Node<nH_Type,double>* End,int  HeadingResolution, int MAX_ITERATION , double vehicleWidth, double vehicleLength)
            {   
                Tree.init();
                iteration = 0;   
                HeadingResolutionRad =  double(HeadingResolution) * pi/180.0000000000000;             
                grid->initialize(); 
                double H = cost2go(Start,End,&H_GRID,og);   
                Start->set_H(H); Start->isOpen = true; Start->isClosed = false;
                //https://www.boost.org/doc/libs/1_53_0/doc/html/boost/heap/fibonacci_heap.html
                boost::heap::fibonacci_heap<Node<nH_Type,double>*, boost::heap::compare<compare>> heap;
                heap.push(Start);
                grid->replace(Start,HeadingResolutionRad);
                int cc = 0;
                std::vector<double> plot_x, plot_y;

                std::vector<std::stringstream > ss;
                while(!heap.empty())
                {
                    std::cout << "in while loop" << std::endl;
                    Node<nH_Type,double>* predecessor_node = heap.top();



         



                    if(grid->isClosed(predecessor_node,HeadingResolutionRad))
                    {
                        std::cout << "   grid closed" << std::endl;
                        heap.pop();
                        continue;
                    }
                    else if(grid->isOpen(predecessor_node,HeadingResolutionRad))
                    {
                        std::cout << "   grid open" << std::endl;
                        grid->set_closed(predecessor_node,HeadingResolutionRad);
                        heap.pop();
                        if(predecessor_node->isEqual(End,HeadingResolutionRad) || iteration > MAX_ITERATION)
                        {
                            std::cout << "   grid equal" << std::endl;
                            Tree.push_p(predecessor_node);
                            std::cout<<"build_tree"<<std::endl;
                            Tree.build(Start,End,vehicleWidth, vehicleLength);
                            std::cout << "smoothing"<<std::endl;
                            smoothing->get_pre_trajectory(og, &Tree.tree, &dubins.optPath.curve, vehicleWidth, vehicleLength, spr);
                                                     
                            std::cout<<"\nEnd is reached"<<std::endl;;
                            return true;
                        } //end is reached, or max iteration
                        if(predecessor_node->isCloseTo(End,50.0))
                        {
                            std::cout << "   close to" << std::endl;
                            dubins.plan(predecessor_node,End,og, cco, HeadingResolutionRad);
                            if(dubins.isCollisionFree) 
                            {
                                std::cout << "   dubins is collision free" << std::endl;
                            Tree.push_p(predecessor_node);
                            std::cout << "build tree"<<std::endl;
                            Tree.build(Start,predecessor_node,vehicleWidth, vehicleLength);
                            std::cout << "do smoothing"<<std::endl; 
                            smoothing->get_pre_trajectory(og, &Tree.tree, &dubins.optPath.curve, vehicleWidth, vehicleLength, spr);
                            std::cout << "return true"<<std::endl;
                                return true;
                            }                           
                        }//close to end
                        evaluateSuccessors(predecessor_node,grid,&H_GRID,og,cco,End,&heap,HeadingResolutionRad);
                    }
                    iteration++;
                    cc++;
                }
                std::cout<<"\nGOAL IS UNREACHABLE";
                iteration++;
                return false;

            }
            adore::fun::SetPointRequest* getSetPointRequest()
            {
                return &spr;
            }

            private:

            void evaluateSuccessors(Node<nH_Type,double>* node, GRID<Node<nH_Type,double>>* grid,GRID<Node<H_Type,int>>* h_grid, adore::env::OccupanyGrid* og,adore::fun::CollisionCheckOffline* cco, Node<nH_Type,double>* End,boost::heap::fibonacci_heap<Node<nH_Type,double>*, boost::heap::compare<compare>>* heap, double HeadingResolutionRad)
            {
                double G ;
                std::vector<Node< 3,  double>*> successors_nh;
                successors_nh = node->updateSuccessors3D(og, cco ,HeadingResolutionRad);
               for (int i=0; i<successors_nh.size(); i++)
                {
                    //std::cout<<"\n"<<i<<"\t"<<!grid->isClosed(successors_nh[i],HeadingResolutionRad) <<"\t"<<node->hasEqualIndex(successors_nh[i],HeadingResolutionRad);
                    if(!grid->isClosed(successors_nh[i],HeadingResolutionRad) ||  node->hasEqualIndex(successors_nh[i],HeadingResolutionRad))
                    {
                        G = successors_nh[i]->get_G(node);
                        //std::cout<<"\t"<<!grid->isOpen(successors_nh[i],HeadingResolutionRad)<<"\t"<<G<<"\t"<<grid->get_G(successors_nh[i],HeadingResolutionRad) ;
                        if(!grid->isOpen(successors_nh[i],HeadingResolutionRad) || G < grid->get_G(successors_nh[i],HeadingResolutionRad) || node->hasEqualIndex(successors_nh[i],HeadingResolutionRad))
                        {
                            successors_nh[i]->set_H(cost2go(successors_nh[i],End, h_grid,og));
                            successors_nh[i]->update_C();
                            if(node->hasEqualIndex(successors_nh[i],HeadingResolutionRad) && successors_nh[i]->C > node->C + 0.01)
                            {
                                continue;
                            }
                            else if(node->hasEqualIndex(successors_nh[i],HeadingResolutionRad) && successors_nh[i]->C < node->C + 0.01)
                            {
                            }
                           Tree.push_p(node);
                           Tree.push_s(successors_nh[i]);                           
                            successors_nh[i]->isOpen = true;
                            successors_nh[i]->isClosed = false;
                            grid->replace(successors_nh[i],HeadingResolutionRad);
                            heap->push(successors_nh[i]);

                        }
                    }
                } //iterating the successors   

            }

            double cost2go (Node<nH_Type,double>* Current, Node<nH_Type,double>* End,GRID<adore::fun::Node<H_Type,int>> * grid,adore::env::OccupanyGrid* og)
            {
                double dubinsPathLength = 0.0;
                double nonHolonomicPath = 0.0;
                dubinsPathLength = dubins.plan(Current,End, og);
                nonHolonomicPath = a_star.plan(grid ,og, Current->nH2H() , End->nH2H()); 
                std::cout << "dubins length "<< std::to_string(dubinsPathLength) << " nonholonomic "<< std::to_string(nonHolonomicPath)<<std::endl;
                return std::max(dubinsPathLength,nonHolonomicPath);
            }

        };
    }
}