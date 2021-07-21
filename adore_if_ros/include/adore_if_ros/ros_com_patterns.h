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

#include <adore/mad/com_patterns.h>
#include <ros/ros.h>
#include <list>
#include <string>


namespace adore
{
    namespace if_ROS
    {
        /**
         * ROS specific implementation of the AFeed communication pattern.
         */
        template<class T,class TMSG,class CONVERTER>
        class Feed:public adore::mad::AFeed<T>
        {
            private:
            std::list<T> data_;
            ros::Subscriber subscriber_;
            CONVERTER converter_;
            public:
            void receive(TMSG msg)
            {
                data_.emplace_back();
                converter_(msg,data_.back());
            }
            Feed(ros::NodeHandle* n,const std::string& topic,int qsize)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                subscriber_ = n->subscribe(topic,qsize,&Feed<T,TMSG,CONVERTER>::receive,this,ros::TransportHints().tcpNoDelay(no_delay));
            }
            virtual bool hasNext() const override
            {
                return data_.size()>0;
            }
            virtual void getNext(T& value) override
            {
                value = data_.front();
                data_.pop_front();
            }
            virtual void getLatest(T& value) override
            {
                value = data_.back();
                data_.clear();
            }
        };

        template<class T,class TMSG,class CONVERTER>
        class FeedWithCallback:public adore::mad::AFeedWithCallback<T>
        {
            private:
            std::list<T> data_;
            ros::Subscriber subscriber_;
            CONVERTER converter_;
            std::function<void()> fcn_;
            bool has_fcn_;
            public:
            void receive(TMSG msg)
            {
                data_.emplace_back();
                converter_(msg,data_.back());
                if (has_fcn_) fcn_();
            }
            FeedWithCallback(ros::NodeHandle* n,const std::string& topic,int qsize)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                subscriber_ = n->subscribe(topic,qsize,&FeedWithCallback<T,TMSG,CONVERTER>::receive,this,ros::TransportHints().tcpNoDelay(no_delay));
                has_fcn_ = false;
            }
            virtual void setCallback(std::function<void()> fcn) override
            {
                fcn_ = fcn;
                has_fcn_ = true;
            }
            virtual bool hasNext() const override
            {
                return data_.size()>0;
            }
            virtual void getNext(T& value) override
            {
                value = data_.front();
                data_.pop_front();
            }
            virtual void getLatest(T& value) override
            {
                value = data_.back();
                data_.clear();
            }
        };
        /**
         * ROS specific implementation of the AReader communication pattern.
         */
        template<class T,class TMSG,class CONVERTER>
        class Reader:public adore::mad::AReader<T>
        {
            private:
            bool initialized_;
            bool changed_;
            T data_;
            ros::Subscriber subscriber_;
            void receive(TMSG msg)
            {
                CONVERTER()(msg,&data_);
                initialized_=true;
                changed_=true;
            }
            public:
            Reader(ros::NodeHandle* n,const std::string& topic,int qsize)
                :initialized_(false),changed_(false)
            {
                bool no_delay;
                n->param("/tcp_no_delay", no_delay, false);
                subscriber_ = n->subscribe(topic,qsize,&Reader<T,TMSG,CONVERTER>::receive,this,ros::TransportHints().tcpNoDelay(no_delay));
            }
            virtual bool hasData() const override
            {
                return initialized_;
            }
            virtual bool hasUpdate() const override
            {
                return changed_;
            }
            virtual void getData(T& value) override
            {
                value = data_;
                changed_ = false;
            }
        };

        /**
         * ROS specific implementation of the AWriter communication pattern.
         */
        template<class T,class TMSG,class CONVERTER>
        class Writer:public adore::mad::AWriter<T>
        {
            private:
            ros::Publisher publisher_;
            public:
            Writer(ros::NodeHandle* n,const std::string& topic,int qsize)
            {
               publisher_ =  n->advertise<TMSG>(topic,qsize);
            }
            virtual bool canWriteMore() const override
            {
                return true;//@TODO: check if this is a correct assumption
            }
            virtual void write(const T& value)
            {   
                publisher_.publish(CONVERTER()(value));
            }
            virtual uint32_t getNumberOfSubscribers() const override
            {   
                return publisher_.getNumSubscribers();
            }
        };

        /**
         * ROS specific parameter retrieval
         */
        class ROSParam
        {
            protected:
                std::string prefix_;
                ros::NodeHandle n_;
                ROSParam(ros::NodeHandle n, std::string prefix):prefix_(prefix),n_(n){}
            public:
                template<typename T>
                void get(const std::string & name,T& result)const
                {
                    if(!n_.getParamCached(name,result))
                    {
                        ROS_INFO_STREAM("No parameter named " << name << " in launch-file. Hardcoded default value used.");
                    }
                }
        };
    }
}