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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#pragma once

namespace adore
{
	namespace params
	{

		/**
		 * @brief abstract class for communication parameters
		 *
		 */
		class APCommunication 
		{
			public:	
				/// get the station id of the vehicle for v2x communication
				virtual double get_stationID() const = 0;
		};
	}  // namespace params
}  // namespace adore
