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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once

// #include <istream>
#include <fstream>
#include <locale>
#include <vector>

namespace adore
{
	namespace mad
	{
        namespace CsvReader
        {
            /**
             * @brief a helper construct to facilitate reading csv files
             * 
             */
            struct csv_reader_facet: std::ctype<char> {

                csv_reader_facet(): std::ctype<char>(get_table()) {}

                static std::ctype_base::mask const* get_table() {
                    static std::vector<std::ctype_base::mask> 
                        rc(table_size, std::ctype_base::mask());

                    rc['\n'] = std::ctype_base::space;
                    rc['\r'] = std::ctype_base::space;
                    rc[','] = std::ctype_base::space;
                    rc[' '] = std::ctype_base::space;
                    rc[';'] = std::ctype_base::space;
                    rc['\t'] = std::ctype_base::space;
                    return &rc[0];
                }
            };

            /**
             * @brief A simple csv reader function which just gets all data T from a file as a vector of T,
             * usual separators like comma semicolon and tab are ignored during read. data vector is flat.
             * 
             * @tparam T 
             */
            template<typename T>
            static std::vector<T> get_data(std::string filepath)
            {
                std::vector<T> result;
                try
                {
                    std::ifstream inputfile;
                    inputfile.open(filepath);
                    if(inputfile.is_open())
                    {
                        std::string firstline;
                        std::getline(inputfile,firstline);
                        inputfile.imbue(std::locale(std::locale(),new csv_reader_facet()));
                        int count = 0;
                        T value;
                        while(inputfile >> value)
                        {
                            result.push_back(value);
                        }
                    }
                    else
                    {
                        std::cerr << "ERROR opening csv file " << filepath << '\n';
                    }
                    
                }
                catch(const std::exception& e)
                {
                    std::cerr << "ERROR processing csv file " << filepath << '\n' << e.what() << '\n';
                }
                return result;
            }

        };
    }
}