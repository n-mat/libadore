/********************************************************************************
 * Copyright (C) 2017-2023 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Daniel Heß - a request to plan a trajectory
 *   Matthias Nichting
 *********************************************************************************/
#pragma once
#include <adore/fun/setpointrequest.h>

namespace adore
{
namespace fun
{
struct PlanningRequest
{
    long long int iteration;        /**<the planning iteration*/
    double t_planning_start;        /**<plannning start time*/
    double t_planning_end;          /**<terminate planning for given iteration at this time*/
    double t_emergency_start;       /**<*/
    double v_ref_percentage_of_v_max; /**<request a plan with specific percentage of v_max as reference velocity*/
    bool allow_for_cooperation;  
    SetPoint initial_state;          /**<the initial state of the plan*/
    PlanningRequest()
    :iteration(0),t_planning_start(0.0),t_planning_end(0.0),t_emergency_start(0.0),v_ref_percentage_of_v_max(0.0),allow_for_cooperation(false)
    {}
};  
}
}
