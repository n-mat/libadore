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
 *   Daniel Heß - initial API and implementation
 *   Matthias Nichting
 ********************************************************************************/
#pragma once
#include <adore/view/agap.h>
#include <adore/view/alane.h>
#include <adore/params/ap_tactical_planner.h>
#include <adore/params/ap_trajectory_generation.h>
#include <adore/params/ap_vehicle.h>

namespace adore
{
    namespace env
    {
        /**
         * @brief defines a gap for testing purposes, which never has lead or chase vehicles.
         */
        class EMLC_Gap : public adore::view::AGap
        {
            adore::view::ALane* target_;
            adore::view::ALane* source_;
            bool chase_exists_;
            unsigned int chase_id_;
            double chase_t0_;
            double chase_v0_;
            double chase_s0_;
            bool front_exists_;
            unsigned int front_id_;
            double front_t0_;
            double front_v0_;
            double front_s0_;

          public:
            EMLC_Gap(adore::view::ALane* target, adore::view::ALane* source)
              : target_(target), source_(source), chase_exists_(false)
            {
            }
            virtual adore::view::AGap::EGapState getState(double s, double t) override
            {
                if (s + 5.0 < getChaseProgress(t, s - 100.0))
                {
                    return adore::view::AGap::OPENING;
                }
                return adore::view::AGap::OPEN;
            }
            virtual double getLeadProgress(double t, double guard) override
            {
                return guard;
            }
            virtual double getChaseProgress(double t, double guard) override
            {
                if (chase_exists_)
                {
                    return chase_s0_ + (t - chase_t0_) * chase_v0_;
                }
                return guard;
            }
            virtual double getFrontProgress(double t, double guard)
            {
                if (front_exists_)
                {
                    return front_s0_ + (t - front_t0_) * front_v0_;
                }
                return guard;
            }
            void setChase(int id)
            {
                chase_exists_ = false;
                auto olt = target_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == id)
                    {
                        chase_exists_ = true;
                        chase_id_ = id;
                        chase_t0_ = a.getObservationTime();
                        chase_s0_ = a.getCurrentProgress() + 0.5 * a.getLength() + 1.0;
                        chase_v0_ = a.getCurrentSpeed();
                    }
                }
            }
            void setFront(int id)
            {
                front_exists_ = false;
                auto olt = source_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == id)
                    {
                        front_exists_ = true;
                        front_id_ = id;
                        front_t0_ = a.getObservationTime();
                        front_s0_ = a.getCurrentProgress() - 0.5 * a.getLength() + 1.0;
                        front_v0_ = a.getCurrentSpeed();
                    }
                }
            }
        };

        /**
         * @brief defines a gap for testing purposes, which never has lead or chase vehicles.
         */
        class SGap : public adore::view::AGap
        {
            adore::view::ALane* target_;
            adore::view::ALane* source_;
            bool chase_exists_;
            unsigned int chase_id_;
            double chase_t0_;
            double chase_v0_;
            double chase_s0_;
            bool lead_exists_;
            unsigned int lead_id_;
            double lead_t0_;
            double lead_v0_;
            double lead_s0_,lead_a0_,chase_a0_;
            std::pair<double,double> end_of_source_;
            bool chase_initialized_,lead_initialized_;
            bool chase_manually_deactivated_;
            double amin_,front_s_buffer_;
            adore::params::APTacticalPlanner* ptac_;

          public:
            SGap(adore::view::ALane* target, adore::view::ALane* source, adore::params::APTacticalPlanner* ptac,double x_end, double y_end)
              : target_(target), source_(source), ptac_(ptac), chase_exists_(false), lead_exists_(false), chase_initialized_(false), lead_initialized_(false), chase_manually_deactivated_(false)
            {
                end_of_source_.first = x_end;
                end_of_source_.second = y_end;
            }
            virtual adore::view::AGap::EGapState getState(double s, double t) override
            {
                double send,nend;
                source_->toRelativeCoordinates(end_of_source_.first,end_of_source_.second,send,nend);
                if (s>send || s > getLeadProgress(t,s-100))
                {
                    return adore::view::AGap::CLOSED;
                }
                if (s < getChaseProgress(t, s -1.0))
                {
                    return adore::view::AGap::OPENING;
                }
                return adore::view::AGap::OPEN;
            }
            virtual double getLeadProgress(double t, double guard) override
            {
                if (lead_exists_)
                {   
                    double t_standstill = std::max(lead_v0_,0.0) / -amin_ + lead_t0_;
                    t = std::min(t,t_standstill);//predict no further then until standstill
                    return lead_s0_ + (t - lead_t0_) * lead_v0_ + 0.5 * amin_ * (t - lead_t0_) * (t - lead_t0_)-front_s_buffer_;
                }
                return guard;
            }
            virtual double getChaseProgress(double t, double guard) override
            {
                if (chase_exists_ && !chase_manually_deactivated_)
                {
                    double t_standstill = std::max(chase_v0_,0.0) / -amin_ + chase_t0_;
                    t = std::min(t,t_standstill);//predict no further then until standstill
                    return chase_s0_ + (t - chase_t0_) * chase_v0_ + 0.5 * amin_ * (t - chase_t0_) * (t - chase_t0_);
                }
                return guard;
            }
            virtual double getChaseProgressWithoutBrake(double t, double guard)
            {
                if (chase_exists_ && !chase_manually_deactivated_)
                {
                    return chase_s0_ + (t - chase_t0_) * chase_v0_ + 0.5 * chase_a0_ * (t - chase_t0_) * (t - chase_t0_);
                }
                return guard;
            }

            int getChaseID()
            {
                return chase_exists_ ? chase_id_ : -1;
            }
            void setChase(int id)
            {
                chase_initialized_ = true;
                chase_id_ = id;
                updateChase();
            }
            void setLead(int id)
            {
                lead_initialized_ = true;
                lead_id_ = id;
                updateLead();
            }
            void updateLead()
            {
                lead_exists_ = false;
                if(!lead_initialized_)return;
                auto olt = target_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == lead_id_)
                    {
                        lead_exists_ = true;
                        lead_t0_ = a.getObservationTime();
                        lead_s0_ = a.getCurrentProgress() - 0.5 * a.getLength();
                        lead_v0_ = a.getCurrentSpeed();
                        lead_a0_ = 0;//a.getCurrentAcceleration();
                    }
                }
            }
            void deactivateChase()
            {
                chase_manually_deactivated_=true;
            }
            void updateChase()
            {
                chase_exists_ = false;
                if(!chase_initialized_)return;
                auto olt = target_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == chase_id_)
                    {
                        chase_exists_ = true;
                        chase_manually_deactivated_=false;
                        chase_t0_ = a.getObservationTime();
                        chase_s0_ = a.getCurrentProgress() + 0.5 * a.getLength();
                        chase_v0_ = a.getCurrentSpeed();
                        chase_a0_ = -0.5;//a.getCurrentAcceleration();
                        }
                }
            }
            void update()
            {

          amin_ = ptac_->getAssumedNominalAccelerationMinimum();
          front_s_buffer_ = ptac_->getFrontSGap();
                updateLead();
                updateChase();
            }
        };
        class StdEgoProgressGap : public adore::view::AGap
        {
            adore::view::ALane* target_;
            adore::view::ALane* source_;
            bool chase_exists_;
            int chase_id_;
            double chase_t0_;
            double chase_v0_;
            double chase_s0_;
            bool lead_exists_;
            int lead_id_;
            double lead_t0_;
            double lead_v0_;
            double lead_s0_,lead_a0_,chase_a0_;
            bool end_of_source_exists_;
            std::pair<double,double> end_of_source_;
            bool chase_initialized_,lead_initialized_;
            double amin_,front_s_buffer_;
            adore::params::APTacticalPlanner* ptac_;

          public:
            StdEgoProgressGap(adore::view::ALane* target, adore::view::ALane* source, adore::params::APTacticalPlanner* ptac,double x_end, double y_end)
              : target_(target), source_(source), ptac_(ptac), chase_exists_(false), lead_exists_(false), chase_initialized_(false), lead_initialized_(false)
            {
                end_of_source_exists_ = true;
                end_of_source_.first = x_end;
                end_of_source_.second = y_end;
            }
            virtual adore::view::AGap::EGapState getState(double s, double t) override
            {
                double send,nend;
                source_->toRelativeCoordinates(end_of_source_.first,end_of_source_.second,send,nend);
                if (s>send || s > getLeadProgress(t,s-100))
                {
                    return adore::view::AGap::CLOSED;
                }
                if (s - front_s_buffer_ * 0.5 < getChaseProgress(t, s -1.0))
                {
                    return adore::view::AGap::OPENING;
                }
                return adore::view::AGap::OPEN;
            }
            virtual double getLeadProgress(double t, double guard) override
            {
                if (lead_exists_)
                {   
                    double t_standstill = std::max(lead_v0_,0.0) / -amin_ + lead_t0_;
                    t = std::min(t,t_standstill);//predict no further then until standstill
                    return lead_s0_ + (t - lead_t0_) * lead_v0_ + 0.5 * amin_ * (t - lead_t0_) * (t - lead_t0_)-front_s_buffer_;
                }
                return guard;
            }
            virtual double getChaseProgress(double t, double guard) override
            {
                if (chase_exists_)
                {
                    double t_standstill = std::max(chase_v0_,0.0) / -amin_ + chase_t0_;
                    t = std::min(t,t_standstill);//predict no further then until standstill
                    return chase_s0_ + (t - chase_t0_) * chase_v0_ + 0.5 * amin_ * (t - chase_t0_) * (t - chase_t0_);
                }
                return guard;
            }
            
            int getChaseID()
            {
                return chase_exists_ ? chase_id_ : -1;
            }
            void setChase(int id)
            {
                chase_initialized_ = true;
                chase_id_ = id;
            }
            void setLead(int id)
            {
                lead_initialized_ = true;
                lead_id_ = id;
            }
            void updateLead()
            {
                lead_exists_ = false;
                if(!lead_initialized_)return;
                auto olt = target_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == lead_id_)
                    {
                        lead_exists_ = true;
                        lead_t0_ = a.getObservationTime();
                        lead_s0_ = a.getCurrentProgress() - 0.5 * a.getLength();
                        lead_v0_ = a.getCurrentSpeed();
                        lead_a0_ = a.getCurrentAcceleration();
                    }
                }
            }
           
            void updateChase()
            {
                chase_exists_ = false;
                if(!chase_initialized_)return;
                auto olt = target_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == chase_id_)
                    {
                        chase_exists_ = true;
                        chase_t0_ = a.getObservationTime();
                        chase_s0_ = a.getCurrentProgress() + 0.5 * a.getLength();
                        chase_v0_ = a.getCurrentSpeed();
                        chase_a0_ = a.getCurrentAcceleration();
                        }
                }
            }
            void findLeadAndChase(double x, double y, double t)
            {
                double s, n;
                source_->toRelativeCoordinates(x, y, s, n);
                auto olt = target_->getOnLaneTraffic();
                auto a = olt.begin();
                setLead(-1);
                for ( ; a != olt.end(); ++a)
                {
                    if (a->getCurrentProgress() + (t - a->getObservationTime()) * a->getCurrentSpeed() - a->getLength() > s)
                    {
                        setLead(a->getTrackingID());
                        break;
                    }
                }
                // there is no lead, but there is a chase:
                if (a == olt.end() && a != olt.begin())
                {
                    --a;
                    setChase(a->getTrackingID());
                }
                // there is a lead and a chase
                else if (a != olt.begin())
                {
                    --a;
                    setChase(a->getTrackingID());
                }
                // there is no chase or no chase and no lead
                else
                {
                    setChase(-1);
                }
            }
            void update(double x, double y, double t)
            {
                amin_ = ptac_->getAssumedNominalAccelerationMinimum();
                front_s_buffer_ = ptac_->getFrontSGap();
                findLeadAndChase(x,y,t);
                updateLead();
                updateChase();
            }
        };

        /**
         * @brief defines a gap for testing purposes, which never has lead or chase vehicles.
         */
        class SGap_second : public adore::view::AGap
        {
            double t_; //time of last update
            bool cooperation_params_freezed_;
            double cooperation_time_;
            adore::view::ALane* target_;
            adore::view::ALane* source_;
            bool chase_exists_;
            unsigned int chase_id_;
            unsigned int chase_v2xid_;
            double chase_t0_;
            double chase_v0_;
            double chase_s0_;
            bool lead_exists_;
            unsigned int lead_id_;
            double lead_t0_;
            double lead_v0_;
            double lead_s0_,lead_a0_,chase_a0_;
            std::pair<double,double> end_of_source_;
            std::pair<double,double> start_of_merge_area_;
            bool chase_initialized_,lead_initialized_;
            bool chase_manually_deactivated_;
            double amin_,front_s_buffer_;
            adore::params::APTacticalPlanner* ptac_;

            adore::params::APTrajectoryGeneration* pgen_;
            adore::params::APVehicle* pveh_;
            double target_x_t0_, target_y_t0_;
            double to_front_, to_rear_;

          public:
            SGap_second(adore::view::ALane* target, adore::view::ALane* source, adore::params::APTacticalPlanner* ptac, adore::params::APVehicle* pveh, adore::params::APTrajectoryGeneration* pgen, double x_end, double y_end, double x_start, double y_start)
              : cooperation_params_freezed_(false), target_(target), source_(source), ptac_(ptac), pveh_(pveh), pgen_(pgen), chase_exists_(false), lead_exists_(false), chase_initialized_(false), lead_initialized_(false), chase_manually_deactivated_(false)
            {
                end_of_source_.first = x_end;
                end_of_source_.second = y_end;
                start_of_merge_area_.first = x_start;
                start_of_merge_area_.second = y_start;
            }
            virtual adore::view::AGap::EGapState getState(double s, double t) override
            {
                double send,nend;
                double sstart,nstart;
                source_->toRelativeCoordinates(end_of_source_.first,end_of_source_.second,send,nend);
                source_->toRelativeCoordinates(start_of_merge_area_.first,start_of_merge_area_.second,sstart,nstart);

                if(s+to_front_<sstart)
                {
                    return adore::view::AGap::OPENING;
                }
                if (s+to_front_>send || s+to_front_ > getLeadProgress(t,s+100))
                {
                    return adore::view::AGap::CLOSED;
                }
                if (s -to_rear_< getChaseProgress(t, s -100.0))
                {
                    return adore::view::AGap::OPENING;
                }
                return adore::view::AGap::OPEN;
            }
            virtual double getLeadProgress(double t, double guard) override
            {
                if (lead_exists_)
                {   
                    //double t_standstill = std::max(lead_v0_,0.0) / -amin_ + lead_t0_;
                    //t = std::min(t,t_standstill);//predict no further then until standstill
                    //return lead_s0_ + (t - lead_t0_) * lead_v0_ + 0.5 * amin_ * (t - lead_t0_) * (t - lead_t0_)-front_s_buffer_;
                    return lead_s0_ + (t-lead_t0_) * lead_v0_;
                }
                return guard;
            }
            virtual double getChaseProgress(double t, double guard) override
            {
                return getCoopChaseProgress(t,guard);
                if (chase_exists_ && !chase_manually_deactivated_)
                {
                    //double t_standstill = std::max(chase_v0_,0.0) / -amin_ + chase_t0_;
                    //t = std::min(t,t_standstill);//predict no further then until standstill
                    //return chase_s0_ + (t - chase_t0_) * chase_v0_ + 0.5 * amin_ * (t - chase_t0_) * (t - chase_t0_);
                    return chase_s0_ + (t-chase_t0_) * chase_v0_;
                }
                return guard;
            }
            virtual double getCoopChaseProgress(double t, double guard) 
            {
                if (chase_exists_)
                {
                    double result = 0.0;
                    //double t_standstill = std::max(chase_v0_,0.0) / -amin_ + chase_t0_;
                    //t = std::min(t,t_standstill);//predict no further then until standstill
                    //return chase_s0_ + (t - chase_t0_) * chase_v0_ + 0.5 * amin_ * (t - chase_t0_) * (t - chase_t0_);
                    double t_decelerate = std::min(std::min(t,cooperation_time_) - chase_t0_,chase_v0_/-chase_a0_);
                    // todo:double t_decelerate = std::min(t - chase_t0_,chase_v0_/-chase_a0_);

                    double tdiff = std::min(t,cooperation_time_) - chase_t0_;

                    result = result + chase_s0_ + (t_decelerate) * chase_v0_ + 0.5 * chase_a0_ * t_decelerate * t_decelerate;

                    // if chase_t0_ +t_decelerate is lower than t_cooperation_time, there is no distance driven in between t and t_cooperation_time!

                    if (t> cooperation_time_)
                    {
                        double target_v = std::max(0.0,chase_v0_ + (cooperation_time_ - chase_t0_) * chase_a0_);
                        result = result + target_v * (t-cooperation_time_);
                    }
                    return result;
                }
                return guard;
            }
            virtual double getChaseProgressWithoutBrake(double t, double guard)
            {
                if (chase_exists_ && !chase_manually_deactivated_)
                {
                    return chase_s0_ + (t - chase_t0_) * chase_v0_ + 0.5 * chase_a0_ * (t - chase_t0_) * (t - chase_t0_);
                }
                return guard;
            }

            int getChaseID()
            {
                return chase_exists_ ? chase_id_ : -1;
            }
            int getLeadID()
            {
                return lead_exists_ ? lead_id_ : -1;
            }
            void setChase(int id)
            {
                std::cout << "SET CHASE TO " <<id << std::endl;

                chase_initialized_ = true;
                chase_id_ = id;
                if(cooperation_params_freezed_)
                throw std::logic_error("params freezed while setting chase");
                updateChase();
            }
            void noLead()
            {
                setLead(-1);
            }
            void noChase()
            {
                setChase(-1);
            }
            void setLead(int id)
            {
                lead_initialized_ = true;
                lead_id_ = id;
                updateLead();
            }
            void updateLead()
            {
                lead_exists_ = false;
                if(!lead_initialized_)return;
                auto olt = target_->getOnLaneTraffic();
                for (auto& a : olt)
                {
                    if (a.getTrackingID() == lead_id_)
                    {
                        lead_exists_ = true;
                        lead_t0_ = a.getObservationTime();
                        lead_s0_ = a.getCurrentProgress() - 0.5 * a.getLength();
                        lead_v0_ = a.getCurrentSpeed();
                        lead_a0_ = 0;//a.getCurrentAcceleration();
                    }
                }
            }
            void deactivateChase()
            {
                chase_manually_deactivated_=true;
            }
            void updateChase()
            {
                std::cout << "UPDATE CHASE "<<chase_id_ <<std::endl;
                chase_exists_ = false;
                if(!chase_initialized_)return;

                if (!cooperation_params_freezed_) // true
                {
                    auto olt = target_->getOnLaneTraffic();
                    for (auto& a : olt)
                    {
                        if (a.getTrackingID() == chase_id_)
                        {
                            std::cout << "      found chase "<< std::endl;
                            chase_v2xid_ = a.getV2XStationID();
                            std::cout << "        v2xid "<<chase_v2xid_ << std::endl;
                            chase_exists_ = true;
                            chase_manually_deactivated_=false;
                            chase_t0_ = a.getObservationTime();
                            chase_s0_ = a.getCurrentProgress() + 0.5 * a.getLength();
                            chase_v0_ = a.getCurrentSpeed();
                            chase_a0_ = -1;

                            //todo: move the following to the trafficobjectclass
                            double z;
                            target_->toEucledianCoordinates(chase_s0_,0.5 * target_->getOffsetOfRightBorder(chase_s0_) + 0.5* target_->getOffsetOfLeftBorder(chase_s0_),target_x_t0_,target_y_t0_,z);
                            std::cout << "     x " << target_x_t0_ << "     y "<< target_y_t0_ << std::endl;
                        }
                    }
                }
                else
                {
                    double n;
                    target_->toRelativeCoordinates(target_x_t0_,target_y_t0_,chase_s0_,n);
                    chase_exists_ = true;
                }
            }
            int getChaseV2XID()
            {
                return chase_v2xid_;
            }
            void update( double t , double s = 0)
            {
                t_ = t;
                to_rear_ = pveh_->get_d() + pgen_->get_rho();
                to_front_ = pveh_->get_a() + pveh_->get_b() + pveh_->get_c() - pgen_->get_rho();

                amin_ = ptac_->getAssumedNominalAccelerationMinimum();
                front_s_buffer_ = ptac_->getFrontSGap();
                updateLead();
                updateChase();
                if(!cooperation_params_freezed_)
                {
                    cooperation_time_ = t_+5.0;
                }
                std::cout << "t "<< t_ << " chase " << getChaseProgress(t_,0) << " lead " << getLeadProgress(t_,1000)<< " ego " << s << " freeze "<< cooperation_params_freezed_ << std::endl;
            }
            void freeze_cooperation_parameters()
            {
                cooperation_params_freezed_ = true;
            }
            void unfreeze_cooperation_parameters()
            {
                cooperation_params_freezed_ = false;
            }
            bool is_freezed()
            {
                return cooperation_params_freezed_;
            }
            void get_cooperation_target(unsigned int & id, double & targetx, double & targety, double & targetv, double & targettime)
            {
                std::cout << "GET COOPERATION TARGET" << std::endl;
                std::cout << "       chaseID and chase V2xid: "<< chase_id_ << " , "<<chase_v2xid_ << std::endl;


                double n,s,z;
                target_->toRelativeCoordinates(target_x_t0_,target_y_t0_,s,n);
                double tdiff= std::min (cooperation_time_-chase_t0_, - chase_v0_ / chase_a0_);
                s = s + tdiff * chase_v0_ + 0.5 * chase_a0_ * tdiff * tdiff;

                target_->toEucledianCoordinates(s,n,targetx,targety,z);
                targetv = chase_v0_ + chase_a0_ * tdiff;
                targettime = cooperation_time_;
                id = chase_id_;
            }
        };


    }  // namespace env
}  // namespace adore
