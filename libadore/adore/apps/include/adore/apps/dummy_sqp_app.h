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
 *   Matthias Nichting
 ********************************************************************************/

#pragma once
#include <Hessian.h>

namespace adore
{
    namespace apps
    {
        
        class DummySQP
        {
          private:
            
          public:
            DummySQP()
            {
                std::cout << "Hessian.h included"<<std::endl;
                Hessian hessian;
                std::cout << "Hessian object created"<<std::endl;
            }

            ~DummySQP()
            {}

            void run()
            {
                std::cout << "app is running" << std::endl;
            }
        };
    }  // namespace apps
}  // namespace adore
