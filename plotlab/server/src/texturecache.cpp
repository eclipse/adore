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
#include <plotlabserver/texturecache.h>


std::size_t DLR::PlotLab::TextureCacheC::write_callback(char *ptr, std::size_t size, std::size_t nmemb, void *stream)
{
	size_t written;
	written = fwrite(ptr, size, nmemb, (FILE*)stream);
	return written;
}
