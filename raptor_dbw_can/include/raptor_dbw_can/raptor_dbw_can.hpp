// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \brief This file defines the RaptorDbwCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file raptor_dbw_can.hpp
 */

#ifndef RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_
#define RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_

#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// ROS messages
#include "can_msgs/msg/frame.hpp"
#include "raptor_dbw_msgs/msg/dbw_misc.hpp"
#include "raptor_dbw_msgs/msg/dbw_accelpdlreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_steeringreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_brakereport.hpp"
#include "raptor_dbw_msgs/msg/dbw_prndreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_wheelpositionreport.hpp"
#include "raptor_dbw_msgs/msg/akit_accelpdlrequest.hpp"
#include "raptor_dbw_msgs/msg/akit_globalenbl.hpp"
#include "raptor_dbw_msgs/msg/akit_otheractuators.hpp"
#include "raptor_dbw_msgs/msg/akit_steeringrequest.hpp"
#include "raptor_dbw_msgs/msg/akit_brakerequest.hpp"
#include "raptor_dbw_msgs/msg/brake_pressure_report.hpp"
#include "raptor_dbw_msgs/msg/akit_prndrequest.hpp"
#include "raptor_dbw_msgs/msg/dbw_driverinputs.hpp"
#include "raptor_dbw_msgs/msg/dbw_tirepressreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_vinreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_reserved1.hpp"
#include "raptor_dbw_msgs/msg/dbw_imureport.hpp"
#include "raptor_dbw_msgs/msg/dbw_wheelspeedreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_radarsonar.hpp"
#include "raptor_dbw_msgs/msg/dbw_faulttext.hpp"
#include "raptor_dbw_msgs/msg/dbw_lowvoltsysreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_brakereport2.hpp"
#include "raptor_dbw_msgs/msg/dbw_steeringreport2.hpp"
#include "raptor_dbw_msgs/msg/dbw_otheractuatorsreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_faultactionsreport.hpp"
#include "raptor_dbw_msgs/msg/dbw_gpsreference.hpp"
#include "raptor_dbw_msgs/msg/dbw_gpsremainder.hpp"
#include "raptor_dbw_msgs/msg/brake_control.hpp"
#include "raptor_dbw_msgs/msg/brake_position_report.hpp"

#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

#include "raptor_dbw_can/dispatch.hpp"

using can_msgs::msg::Frame;
using NewEagle::DbcMessage;

using raptor_dbw_msgs::msg::DbwMisc;
using raptor_dbw_msgs::msg::DbwAccelpdlreport;
using raptor_dbw_msgs::msg::DbwSteeringreport;
using raptor_dbw_msgs::msg::DbwBrakereport;
using raptor_dbw_msgs::msg::DbwPrndreport;
using raptor_dbw_msgs::msg::DbwWheelpositionreport;
using raptor_dbw_msgs::msg::AkitAccelpdlrequest;
using raptor_dbw_msgs::msg::AkitGlobalenbl;
using raptor_dbw_msgs::msg::AkitOtheractuators;
using raptor_dbw_msgs::msg::AkitSteeringrequest;
using raptor_dbw_msgs::msg::AkitBrakerequest;
using raptor_dbw_msgs::msg::BrakePressureReport;
using raptor_dbw_msgs::msg::AkitPrndrequest;
using raptor_dbw_msgs::msg::DbwDriverinputs;
using raptor_dbw_msgs::msg::DbwTirepressreport;
using raptor_dbw_msgs::msg::DbwVinreport;
using raptor_dbw_msgs::msg::DbwReserved1;
using raptor_dbw_msgs::msg::DbwImureport;
using raptor_dbw_msgs::msg::DbwWheelspeedreport;
using raptor_dbw_msgs::msg::DbwRadarsonar;
using raptor_dbw_msgs::msg::DbwFaulttext;
using raptor_dbw_msgs::msg::DbwLowvoltsysreport;
using raptor_dbw_msgs::msg::DbwBrakereport2;
using raptor_dbw_msgs::msg::DbwSteeringreport2;
using raptor_dbw_msgs::msg::DbwOtheractuatorsreport;
using raptor_dbw_msgs::msg::DbwFaultactionsreport;
using raptor_dbw_msgs::msg::DbwGpsreference;
using raptor_dbw_msgs::msg::DbwGpsremainder;
using raptor_dbw_msgs::msg::BrakeControl;
using raptor_dbw_msgs::msg::BrakePositionReport;

namespace raptor_dbw_can
{
class RaptorDbwCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 */
    explicit RaptorDbwCAN(const rclcpp::NodeOptions & options);

private:

/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
    void recvCAN(const Frame::SharedPtr msg);

    void recvDbwMisc(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwAccelpdlreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwSteeringreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwBrakereport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwPrndreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwWheelpositionreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvAkitOtheractuators(const Frame::SharedPtr msg, DbcMessage * message);
	void recvBrakePressureReport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwDriverinputs(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwTirepressreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwVinreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwReserved1(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwImureport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwWheelspeedreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwRadarsonar(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwFaulttext(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwLowvoltsysreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwBrakereport2(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwSteeringreport2(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwOtheractuatorsreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwFaultactionsreport(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwGpsreference(const Frame::SharedPtr msg, DbcMessage * message);
	void recvDbwGpsremainder(const Frame::SharedPtr msg, DbcMessage * message);
	void recvBrakeControl(const Frame::SharedPtr msg, DbcMessage * message);
	void recvBrakePositionReport(const Frame::SharedPtr msg, DbcMessage * message);

    void recvAkitAccelpdlrequest(const AkitAccelpdlrequest::SharedPtr msg);
	void recvAkitGlobalenbl(const AkitGlobalenbl::SharedPtr msg);
	void recvAkitSteeringrequest(const AkitSteeringrequest::SharedPtr msg);
	void recvAkitBrakerequest(const AkitBrakerequest::SharedPtr msg);
	void recvAkitPrndrequest(const AkitPrndrequest::SharedPtr msg);

    std::uint8_t vehicle_number_;

    // Parameters from launch
    std::string dbc_file_;
    float max_steer_angle_;
    bool publish_my_laps_;

    rclcpp::Subscription<DbwMisc>::SharedPtr subDbwMisc_;
	rclcpp::Subscription<DbwAccelpdlreport>::SharedPtr subDbwAccelpdlreport_;
	rclcpp::Subscription<DbwSteeringreport>::SharedPtr subDbwSteeringreport_;
	rclcpp::Subscription<DbwBrakereport>::SharedPtr subDbwBrakereport_;
	rclcpp::Subscription<DbwPrndreport>::SharedPtr subDbwPrndreport_;
	rclcpp::Subscription<DbwWheelpositionreport>::SharedPtr subDbwWheelpositionreport_;
	rclcpp::Subscription<AkitAccelpdlrequest>::SharedPtr subAkitAccelpdlrequest_;
	rclcpp::Subscription<AkitGlobalenbl>::SharedPtr subAkitGlobalenbl_;
	rclcpp::Subscription<AkitOtheractuators>::SharedPtr subAkitOtheractuators_;
	rclcpp::Subscription<AkitSteeringrequest>::SharedPtr subAkitSteeringrequest_;
	rclcpp::Subscription<AkitBrakerequest>::SharedPtr subAkitBrakerequest_;
	rclcpp::Subscription<BrakePressureReport>::SharedPtr subBrakePressureReport_;
	rclcpp::Subscription<AkitPrndrequest>::SharedPtr subAkitPrndrequest_;
	rclcpp::Subscription<DbwDriverinputs>::SharedPtr subDbwDriverinputs_;
	rclcpp::Subscription<DbwTirepressreport>::SharedPtr subDbwTirepressreport_;
	rclcpp::Subscription<DbwVinreport>::SharedPtr subDbwVinreport_;
	rclcpp::Subscription<DbwReserved1>::SharedPtr subDbwReserved1_;
	rclcpp::Subscription<DbwImureport>::SharedPtr subDbwImureport_;
	rclcpp::Subscription<DbwWheelspeedreport>::SharedPtr subDbwWheelspeedreport_;
	rclcpp::Subscription<DbwRadarsonar>::SharedPtr subDbwRadarsonar_;
	rclcpp::Subscription<DbwFaulttext>::SharedPtr subDbwFaulttext_;
	rclcpp::Subscription<DbwLowvoltsysreport>::SharedPtr subDbwLowvoltsysreport_;
	rclcpp::Subscription<DbwBrakereport2>::SharedPtr subDbwBrakereport2_;
	rclcpp::Subscription<DbwSteeringreport2>::SharedPtr subDbwSteeringreport2_;
	rclcpp::Subscription<DbwOtheractuatorsreport>::SharedPtr subDbwOtheractuatorsreport_;
	rclcpp::Subscription<DbwFaultactionsreport>::SharedPtr subDbwFaultactionsreport_;
	rclcpp::Subscription<DbwGpsreference>::SharedPtr subDbwGpsreference_;
	rclcpp::Subscription<DbwGpsremainder>::SharedPtr subDbwGpsremainder_;
	rclcpp::Subscription<BrakeControl>::SharedPtr subBrakeControl_;
	rclcpp::Subscription<BrakePositionReport>::SharedPtr subBrakePositionReport_;
    rclcpp::Subscription<Frame>::SharedPtr sub_can_;

    rclcpp::Publisher<DbwMisc>::SharedPtr pubDbwMisc_;
	rclcpp::Publisher<DbwAccelpdlreport>::SharedPtr pubDbwAccelpdlreport_;
	rclcpp::Publisher<DbwSteeringreport>::SharedPtr pubDbwSteeringreport_;
	rclcpp::Publisher<DbwBrakereport>::SharedPtr pubDbwBrakereport_;
	rclcpp::Publisher<DbwPrndreport>::SharedPtr pubDbwPrndreport_;
	rclcpp::Publisher<DbwWheelpositionreport>::SharedPtr pubDbwWheelpositionreport_;
	rclcpp::Publisher<AkitAccelpdlrequest>::SharedPtr pubAkitAccelpdlrequest_;
	rclcpp::Publisher<AkitGlobalenbl>::SharedPtr pubAkitGlobalenbl_;
	rclcpp::Publisher<AkitOtheractuators>::SharedPtr pubAkitOtheractuators_;
	rclcpp::Publisher<AkitSteeringrequest>::SharedPtr pubAkitSteeringrequest_;
	rclcpp::Publisher<AkitBrakerequest>::SharedPtr pubAkitBrakerequest_;
	rclcpp::Publisher<BrakePressureReport>::SharedPtr pubBrakePressureReport_;
	rclcpp::Publisher<AkitPrndrequest>::SharedPtr pubAkitPrndrequest_;
	rclcpp::Publisher<DbwDriverinputs>::SharedPtr pubDbwDriverinputs_;
	rclcpp::Publisher<DbwTirepressreport>::SharedPtr pubDbwTirepressreport_;
	rclcpp::Publisher<DbwVinreport>::SharedPtr pubDbwVinreport_;
	rclcpp::Publisher<DbwReserved1>::SharedPtr pubDbwReserved1_;
	rclcpp::Publisher<DbwImureport>::SharedPtr pubDbwImureport_;
	rclcpp::Publisher<DbwWheelspeedreport>::SharedPtr pubDbwWheelspeedreport_;
	rclcpp::Publisher<DbwRadarsonar>::SharedPtr pubDbwRadarsonar_;
	rclcpp::Publisher<DbwFaulttext>::SharedPtr pubDbwFaulttext_;
	rclcpp::Publisher<DbwLowvoltsysreport>::SharedPtr pubDbwLowvoltsysreport_;
	rclcpp::Publisher<DbwBrakereport2>::SharedPtr pubDbwBrakereport2_;
	rclcpp::Publisher<DbwSteeringreport2>::SharedPtr pubDbwSteeringreport2_;
	rclcpp::Publisher<DbwOtheractuatorsreport>::SharedPtr pubDbwOtheractuatorsreport_;
	rclcpp::Publisher<DbwFaultactionsreport>::SharedPtr pubDbwFaultactionsreport_;
	rclcpp::Publisher<DbwGpsreference>::SharedPtr pubDbwGpsreference_;
	rclcpp::Publisher<DbwGpsremainder>::SharedPtr pubDbwGpsremainder_;
	rclcpp::Publisher<BrakeControl>::SharedPtr pubBrakeControl_;
	rclcpp::Publisher<BrakePositionReport>::SharedPtr pubBrakePositionReport_;
    rclcpp::Publisher<Frame>::SharedPtr pub_can_;

    NewEagle::Dbc dbc_;
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__RAPTOR_DBW_CAN_HPP_

