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
#include <unordered_map>
#include "stb_image.h"
#include <iostream>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <string>
#include <curl/curl.h>
#include <string>
#include <sstream>
#include <fstream>

namespace DLR
{
	namespace PlotLab
	{
		namespace TextureCacheC
		{
			std::size_t write_callback(char *ptr, std::size_t size, std::size_t nmemb, void *stream);
		}
		/**
		 *	TextureCache loads and manages textures for OpenGL
		 *  - A note on libcurl for web-based textures with Dominion under Windows: 
		 *	- Go to D:\Dominion\User\Extern\curl-7.60.0
		 *  - unzip the provided zip file and copy content to main folder
		 *  - start "D:\Dominion\User\Extern\cmake-3.8.2_win32_x86\bin\cmake-gui.exe"
			- Source code directory:	this, "D:\Dominion\User\Extern\curl-7.60.0"
			- Build directory:			"D:\Dominion\User\Extern\curl-7.60.0/build"
			- Configure option:			Visual Studio 10 2010
			- Generate option(s):		"CURL_ZLIB"
			- Builds (in VS):			"libcurl" (Debug, Release)
			- open "D:\Dominion\User\Extern\curl-7.60.0\build\lib\libcurl.vcxproj"
			- build ALL_BUILD
		*  -> https://curl.haxx.se/libcurl/c/https.html

		*	DLLS:
		*   - curl may be built against zlib: copy libcurl(-d).dll and zlib1.dll to working directory
		*
		 */
		class TextureCache
		{
		private:
			std::unordered_map<std::string,GLuint> m_idmap;
			CURL* m_curl;
			std::hash<std::string> hasher;
			bool m_loadfromHTTP;

			GLuint fromFile(const std::string& filename)
			{
				GLuint texture;
				int width, height, nrChannels;
				unsigned char* data = stbi_load(filename.c_str(),&width,&height,&nrChannels,0);
				if( data )
				{
					glGenTextures(1, &texture);
					glBindTexture(GL_TEXTURE_2D, texture);
					// set the texture wrapping/filtering options (on the currently bound texture object)
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

					if( nrChannels==3 )
					{
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
					}
					else
					{
						if( nrChannels==4 )
						{
							glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
						}
					}
					stbi_image_free(data);
				}
				else
				{
					std::cerr << "Failed to load texture " << filename.c_str() << std::endl;
					auto it = m_idmap.find("container.jpg");//default
					if(it!=m_idmap.end())
					{
						texture = it->second;
					}
					else
					{
						texture = 0;
					}
				}
				m_idmap.insert(std::make_pair(filename,texture));
				return texture;
			}

			GLuint fromHTTP(const std::string& url)
			{
				bool file_exists = false;
				GLuint texture;
				std::string unavailable = "grass.jpg";

				std::string filename;
				std::size_t bboxpos = url.find("&bbox=");
				std::size_t widthpos = url.find("&width=");
				//gis url: https://maps.aim.ts.dlr.de/geoserver/bs.amt/wms?service=WMS&version=1.1.0&request=GetMap&layers=bs.amt:DLK_2017&styles=&bbox=594999.975,5782000.025,612999.9749999995,5802000.025&width=691&height=768&srs=EPSG:25832&format=image%2Fpng
				if(bboxpos == std::string::npos || widthpos == std::string::npos )// an arbitrary url
				{
					//compute hash id of url
					std::stringstream urlhash;
					urlhash.str("");
					urlhash<<hasher(url);
					filename = urlhash.str()+".jpg";
				}
				else
				{
					int namestart = bboxpos+6;
					int nameend = widthpos-1;
					std::stringstream s;
					s.str("");
					s<<url.substr(namestart,nameend-namestart+1)<<".jpg";
					filename = s.str();
				}



				{// test whether file has been previously cached
					std::ifstream ifile(filename);
					file_exists = file_exists || ifile.is_open();
				}

				if(!file_exists && m_curl && m_loadfromHTTP)
				{
					FILE *f = fopen(filename.c_str(), "wb");
					curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, TextureCacheC::write_callback);
					curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, f);
					curl_easy_setopt(m_curl, CURLOPT_URL, url.c_str());
					curl_easy_setopt(m_curl, CURLOPT_SSL_VERIFYPEER, 0L);
					curl_easy_setopt(m_curl, CURLOPT_SSL_VERIFYHOST, 0L);
					curl_easy_setopt(m_curl, CURLOPT_TIMEOUT, 5L);
					CURLcode res = curl_easy_perform(m_curl);
					fclose(f);

					if(res == CURLE_OK)
					{
						file_exists = true;
					}
					else
					{
						fprintf(stderr, "curl_easy_perform() failed: %s\n",curl_easy_strerror(res));
					}
				}

				if( file_exists )
				{
					texture = fromFile(filename);
				}
				else
				{
					auto it = m_idmap.find(unavailable);
					if( it==m_idmap.end() )
					{
						texture = fromFile(unavailable);
					}
					else
					{
						texture = it->second;
					}
				}
				m_idmap.insert(std::make_pair(url,texture));
				return texture;
			}


		public:
			TextureCache()
			{
				curl_global_init(CURL_GLOBAL_DEFAULT);
				m_curl = curl_easy_init();
				//getGLTexture("container.jpg");
				m_loadfromHTTP = true;
			}
			void setLoadFromHTTP(bool value)
			{
				m_loadfromHTTP = value;
			}
			virtual ~TextureCache()
			{
				//curl_easy_cleanup(m_curl);
			}
			/**
			 *	getGLTexture: retrieves an image from a specified file, saves it for future use and returns an index of the image
			 */
			GLuint getGLTexture(const std::string& name)
			{
				auto it = m_idmap.find(name);
				if( it==m_idmap.end() )
				{
					if( name.find("http://") == 0 || name.find("https://") == 0 )
					{
						return fromHTTP(name);
					}
					else
					{
						return fromFile(name);
					}
				}
				else
				{
					return it->second;
				}
			}
		};
	}
}
