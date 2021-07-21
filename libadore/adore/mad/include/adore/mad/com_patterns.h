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
 *   Matthias Nichting - added class AFeedWithCallback
 ********************************************************************************/

#pragma once
#include <functional>
#include <string>

namespace adore
{
    namespace mad
    {
        /**
         * A communication pattern, which allows a client to read all messages received since last queried.
         */
        template<typename T>
        class AFeed
        {
            public:
            /**
             * hasNext indicates whether there is more data to read
             */
            virtual bool hasNext() const =0;
            /**
             * getNext reads the next data element
             */
            virtual void getNext(T& value)=0; 
            /**
             * getLatest reads the latest data element and discards all previous
             */
            virtual void getLatest(T& value)=0;
        };
        template<typename T>
        class AFeedWithCallback
        {
            public:
            /**
             * hasNext indicates whether there is more data to read
             */
            virtual bool hasNext() const =0;
            virtual void setCallback(std::function<void()> fcn) = 0;
            /**
             * getNext reads the next data element
             */
            virtual void getNext(T& value)=0; 
            /**
             * getLatest reads the latest data element and discards all previous
             */
            virtual void getLatest(T& value)=0;
        };

        /**
         * A communication pattern, which allows a client to read the last message received.
         */
        template<typename T>
        class AReader
        {
            public:
            /**
             * hasData indicates whether the data has been initialized with a first data item
             */
            virtual bool hasData() const=0;
            /**
             * hasUpdate indicates whether the data item was updated since last getdata
             */
            virtual bool hasUpdate() const=0;
            /**
             * getData returns the latest data item
             */
            virtual void getData(T& value) =0;
            /**
             * describes the implementation of the reader
             * @return default implementation only returns best guess for type T
             */
            virtual std::string getDesc()
            {
                return std::string(typeid(T).name());
            }
        };

        /**
         * A communication pattern which allows a client to send messages.
         */
        template<typename T>
        class AWriter
        {
            public:
            /**
             * canWriteMore indicates whether more data can be written
             */
            virtual bool canWriteMore() const =0;
            /**
             * write sends out data value
             */
            virtual void write(const T& value)=0;
             /**
             * describes the implementation of the writer
             * @return default implementation only returns best guess for type T
             */
            virtual std::string getDesc()
            {
                return std::string(typeid(T).name());
            }
            /**
             * return the number number of subscribers/readers
             */
            virtual uint32_t getNumberOfSubscribers() const
            {
                return 0;
            }
        };
        
        /**
         * @brief Observer pattern to manage feed data in a storage class
         * 
         * @tparam T 
         */
        template<typename T>
        class ALocalObserver
        {
            protected:
            AFeed<T> * feed_;

            public:
            /**
             * @brief retrieve new objects
             * 
             */
            virtual void update() = 0;
            
            /**
             * @brief discard data outside of radius
             * 
             * @param x 
             * @param y 
             * @param z 
             * @param radius 
             */
            virtual void discard_radius_based(double x, double y, double z, double radius) = 0;
            
            public:
            
            /**
             * @brief add source feed to observer
             * 
             * @param feed 
             */
            virtual void addFeed(AFeed<T> * feed) = 0;
        };
    }
}