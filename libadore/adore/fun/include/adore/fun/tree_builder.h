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
#include <boost/container/vector.hpp>
#include <adore/fun/node.h>
//#include <adore/env/map/occupancy_grid.h>

namespace adore
{
	namespace fun
	{
	   /**
	   *  ----------------------
	   */
       template <int TYPE, typename T>
       	class TreeBuilder
		{
		private:
        std::string RED= "LineColor=1.,0.,0.;LineWidth=3";
        std::string GRAY= "LineColor=0.7,0.7,0.7;LineWidth=1";

        public:
        struct Node_Lite
        {
            double x;
            double y;
            double psi;
            double s;   //used only for smoothing
        };
        typedef boost::container::vector<Node_Lite> TrajectoryVector;
        TrajectoryVector p;
        TrajectoryVector s;
        TrajectoryVector tree;

        TreeBuilder()
        {

        }
        void init()
        {
            p.clear();
            s.clear();  
            tree.clear();          
        }
        void push_p(Node<TYPE,T>* node )
        {
            Node_Lite tmp;
            tmp.x = node->x;
            tmp.y = node->y;
            tmp.psi = node->psi;
            p.push_back(tmp);
        }
        void push_s(Node<TYPE,T>* node )
        {
            Node_Lite tmp;
            tmp.x = node->x;
            tmp.y = node->y;
            tmp.psi = node->psi;
            s.push_back(tmp);
        }   
        void build(Node<TYPE,T>* Start, Node<TYPE,T>* End,double vehicleWidth, double vehicleLength)
        {
            int index = 0;
            std::cout << "node lite init"<<std::endl;
            Node_Lite search;

            std::cout << "endx"<<std::endl;
            search.x = End->x;
            search.y = End->y;
            search.psi = End->psi;

            std::cout << "pushback"<<std::endl;
            tree.push_back(search);
            int starting_index = s.size();

            std::cout << "starting index "<< std::to_string(starting_index)<<std::endl;
            while(true)
            {
                for(int i=starting_index; i>=0 && s.size() > 0; --i)
                {
                    std::cout << "        in for"<<std::endl;
                    if(s[i].x == search.x && s[i].y == search.y)
                    {
                    std::cout << "           in if"<<std::endl;
                        index = i;
                        starting_index = i;
                        search.x = p[i].x;
                        search.y = p[i].y;
                        search.psi = p[i].psi;
                        tree.push_back(search);
                        break;
                    }
                }
                if(search.x == Start->x && search.y == Start->y)
                {
                    break;
                }
            }//while

            std::cout << "reverse"<<std::endl;
            std::reverse(tree.begin(), tree.end());
            std::cout<<"\nTree size: "<<tree.size();

            
        }     

        };
    }
}