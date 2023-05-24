// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include <cmath>
#include <algorithm>
#include <string>

#include "raptor_dbw_can/raptor_dbw_can.hpp"

using std::chrono::duration;

namespace raptor_dbw_can
{

static constexpr uint64_t MS_IN_SEC = 1000;

RaptorDbwCAN::RaptorDbwCAN(const rclcpp::NodeOptions & options)
: Node("raptor_dbw_can_node", options)
{

    dbc_file_ = declare_parameter<std::string>("dbc_file", "");

    pub_can_ = this->create_publisher<Frame>(
        "can_rx", 20
    );
    pubDbwMisc_ = this->create_publisher<DbwMisc>("dbw_misc", rclcpp::SensorDataQoS());
	pubDbwAccelpdlreport_ = this->create_publisher<DbwAccelpdlreport>("dbw_accelpdlreport", rclcpp::SensorDataQoS());
	pubDbwSteeringreport_ = this->create_publisher<DbwSteeringreport>("dbw_steeringreport", rclcpp::SensorDataQoS());
	pubDbwBrakereport_ = this->create_publisher<DbwBrakereport>("dbw_brakereport", rclcpp::SensorDataQoS());
	pubDbwPrndreport_ = this->create_publisher<DbwPrndreport>("dbw_prndreport", rclcpp::SensorDataQoS());
	pubDbwWheelpositionreport_ = this->create_publisher<DbwWheelpositionreport>("dbw_wheelpositionreport", rclcpp::SensorDataQoS());
	pubAkitOtheractuators_ = this->create_publisher<AkitOtheractuators>("akit_otheractuators", rclcpp::SensorDataQoS());
	pubBrakePressureReport_ = this->create_publisher<BrakePressureReport>("brake_pressure_report", rclcpp::SensorDataQoS());
	pubDbwDriverinputs_ = this->create_publisher<DbwDriverinputs>("dbw_driverinputs", rclcpp::SensorDataQoS());
	pubDbwTirepressreport_ = this->create_publisher<DbwTirepressreport>("dbw_tirepressreport", rclcpp::SensorDataQoS());
	pubDbwVinreport_ = this->create_publisher<DbwVinreport>("dbw_vinreport", rclcpp::SensorDataQoS());
	pubDbwReserved1_ = this->create_publisher<DbwReserved1>("dbw_reserved1", rclcpp::SensorDataQoS());
	pubDbwImureport_ = this->create_publisher<DbwImureport>("dbw_imureport", rclcpp::SensorDataQoS());
	pubDbwWheelspeedreport_ = this->create_publisher<DbwWheelspeedreport>("dbw_wheelspeedreport", rclcpp::SensorDataQoS());
	pubDbwRadarsonar_ = this->create_publisher<DbwRadarsonar>("dbw_radarsonar", rclcpp::SensorDataQoS());
	pubDbwFaulttext_ = this->create_publisher<DbwFaulttext>("dbw_faulttext", rclcpp::SensorDataQoS());
	pubDbwLowvoltsysreport_ = this->create_publisher<DbwLowvoltsysreport>("dbw_lowvoltsysreport", rclcpp::SensorDataQoS());
	pubDbwBrakereport2_ = this->create_publisher<DbwBrakereport2>("dbw_brakereport2", rclcpp::SensorDataQoS());
	pubDbwSteeringreport2_ = this->create_publisher<DbwSteeringreport2>("dbw_steeringreport2", rclcpp::SensorDataQoS());
	pubDbwOtheractuatorsreport_ = this->create_publisher<DbwOtheractuatorsreport>("dbw_otheractuatorsreport", rclcpp::SensorDataQoS());
	pubDbwFaultactionsreport_ = this->create_publisher<DbwFaultactionsreport>("dbw_faultactionsreport", rclcpp::SensorDataQoS());
	pubDbwGpsreference_ = this->create_publisher<DbwGpsreference>("dbw_gpsreference", rclcpp::SensorDataQoS());
	pubDbwGpsremainder_ = this->create_publisher<DbwGpsremainder>("dbw_gpsremainder", rclcpp::SensorDataQoS());
	pubBrakeControl_ = this->create_publisher<BrakeControl>("brake_control", rclcpp::SensorDataQoS());
	pubBrakePositionReport_ = this->create_publisher<BrakePositionReport>("brake_position_report", rclcpp::SensorDataQoS());

    subAkitAccelpdlrequest_ = this->create_subscription<AkitAccelpdlrequest>("akit_accelpdlrequest", rclcpp::SensorDataQoS(), std::bind(&RaptorDbwCAN::recvAkitAccelpdlrequest, this, std::placeholders::_1));
	subAkitGlobalenbl_ = this->create_subscription<AkitGlobalenbl>("akit_globalenbl", rclcpp::SensorDataQoS(), std::bind(&RaptorDbwCAN::recvAkitGlobalenbl, this, std::placeholders::_1));
	subAkitSteeringrequest_ = this->create_subscription<AkitSteeringrequest>("akit_steeringrequest", rclcpp::SensorDataQoS(), std::bind(&RaptorDbwCAN::recvAkitSteeringrequest, this, std::placeholders::_1));
	subAkitBrakerequest_ = this->create_subscription<AkitBrakerequest>("akit_brakerequest", rclcpp::SensorDataQoS(), std::bind(&RaptorDbwCAN::recvAkitBrakerequest, this, std::placeholders::_1));
	subAkitPrndrequest_ = this->create_subscription<AkitPrndrequest>("akit_prndrequest", rclcpp::SensorDataQoS(), std::bind(&RaptorDbwCAN::recvAkitPrndrequest, this, std::placeholders::_1));
    sub_can_ = this->create_subscription<Frame>(
        "can_tx", 500,
        std::bind(&RaptorDbwCAN::recvCAN, this, std::placeholders::_1)
    );

    dbc_ = NewEagle::DbcBuilder().NewDbc(dbc_file_);
}

#define RECV_DBC(handler) \
    message = dbc_.GetMessageById(id); \
    if (msg->dlc >= message->GetDlc()) {message->SetFrame(msg); handler(msg, message);}

void RaptorDbwCAN::recvCAN(const Frame::SharedPtr msg)
{
    NewEagle::DbcMessage * message = nullptr;
    if (!msg->is_rtr && !msg->is_error) {
        auto id = msg->id;
        switch (id) {
            case ID_DBW_MISC:
				RECV_DBC(recvDbwMisc);
				break;
			case ID_DBW_ACCELPDLREPORT:
				RECV_DBC(recvDbwAccelpdlreport);
				break;
			case ID_DBW_STEERINGREPORT:
				RECV_DBC(recvDbwSteeringreport);
				break;
			case ID_DBW_BRAKEREPORT:
				RECV_DBC(recvDbwBrakereport);
				break;
			case ID_DBW_PRNDREPORT:
				RECV_DBC(recvDbwPrndreport);
				break;
			case ID_DBW_WHEELPOSITIONREPORT:
				RECV_DBC(recvDbwWheelpositionreport);
				break;
			case ID_AKIT_OTHERACTUATORS:
				RECV_DBC(recvAkitOtheractuators);
				break;
			case ID_BRAKE_PRESSURE_REPORT:
				RECV_DBC(recvBrakePressureReport);
				break;
			case ID_DBW_DRIVERINPUTS:
				RECV_DBC(recvDbwDriverinputs);
				break;
			case ID_DBW_TIREPRESSREPORT:
				RECV_DBC(recvDbwTirepressreport);
				break;
			case ID_DBW_VINREPORT:
				RECV_DBC(recvDbwVinreport);
				break;
			case ID_DBW_RESERVED1:
				RECV_DBC(recvDbwReserved1);
				break;
			case ID_DBW_IMUREPORT:
				RECV_DBC(recvDbwImureport);
				break;
			case ID_DBW_WHEELSPEEDREPORT:
				RECV_DBC(recvDbwWheelspeedreport);
				break;
			case ID_DBW_RADARSONAR:
				RECV_DBC(recvDbwRadarsonar);
				break;
			case ID_DBW_FAULTTEXT:
				RECV_DBC(recvDbwFaulttext);
				break;
			case ID_DBW_LOWVOLTSYSREPORT:
				RECV_DBC(recvDbwLowvoltsysreport);
				break;
			case ID_DBW_BRAKEREPORT2:
				RECV_DBC(recvDbwBrakereport2);
				break;
			case ID_DBW_STEERINGREPORT2:
				RECV_DBC(recvDbwSteeringreport2);
				break;
			case ID_DBW_OTHERACTUATORSREPORT:
				RECV_DBC(recvDbwOtheractuatorsreport);
				break;
			case ID_DBW_FAULTACTIONSREPORT:
				RECV_DBC(recvDbwFaultactionsreport);
				break;
			case ID_DBW_GPSREFERENCE:
				RECV_DBC(recvDbwGpsreference);
				break;
			case ID_DBW_GPSREMAINDER:
				RECV_DBC(recvDbwGpsremainder);
				break;
			case ID_BRAKE_CONTROL:
				RECV_DBC(recvBrakeControl);
				break;
			case ID_BRAKE_POSITION_REPORT:
				RECV_DBC(recvBrakePositionReport);
				break;
            default:
                break;
        }
    }
}

void RaptorDbwCAN::recvDbwMisc(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwMisc out;
	out.stamp = msg->header.stamp;

	out.dbw_miscbywireenabled = message->GetSignal("DBW_MiscByWireEnabled")->GetResult();
	out.dbw_miscfault = message->GetSignal("DBW_MiscFault")->GetResult();
	out.dbw_miscakitcommfault = message->GetSignal("DBW_MiscAKitCommFault")->GetResult();
	out.dbw_miscbywireready = message->GetSignal("DBW_MiscByWireReady")->GetResult();
	out.dbw_miscdriveractivity = message->GetSignal("DBW_MiscDriverActivity")->GetResult();
	out.dbw_vehreadytodrive = message->GetSignal("DBW_VehReadyToDrive")->GetResult();
	out.dbw_miscvehiclespeed = message->GetSignal("DBW_MiscVehicleSpeed")->GetResult();
	out.dbw_miscfuellvl = message->GetSignal("DBW_MiscFuelLvl")->GetResult();
	out.dbw_softwarebuildnumber = message->GetSignal("DBW_SoftwareBuildNumber")->GetResult();
	out.dbw_ambienttemp = message->GetSignal("DBW_AmbientTemp")->GetResult();

	pubDbwMisc_->publish(out);
}

void RaptorDbwCAN::recvDbwAccelpdlreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwAccelpdlreport out;
	out.stamp = msg->header.stamp;

	out.dbw_accelpdldriverinput = message->GetSignal("DBW_AccelPdlDriverInput")->GetResult();
	out.dbw_accelpdlposnfdbck = message->GetSignal("DBW_AccelPdlPosnFdbck")->GetResult();
	out.dbw_accelpcnttorqueactual = message->GetSignal("DBW_AccelPcntTorqueActual")->GetResult();
	out.dbw_accelctrltype = message->GetSignal("DBW_AccelCtrlType")->GetResult();
	out.dbw_accelpdlenabled = message->GetSignal("DBW_AccelPdlEnabled")->GetResult();
	out.dbw_accelpdlignoredriver = message->GetSignal("DBW_AccelPdlIgnoreDriver")->GetResult();
	out.dbw_accelpdldriveractivity = message->GetSignal("DBW_AccelPdlDriverActivity")->GetResult();
	out.dbw_accelpdlfault = message->GetSignal("DBW_AccelPdlFault")->GetResult();
	out.dbw_accelpdlfault_ch1 = message->GetSignal("DBW_AccelPdlFault_Ch1")->GetResult();
	out.dbw_accelpdlfault_ch2 = message->GetSignal("DBW_AccelPdlFault_Ch2")->GetResult();
	out.dbw_accelpdlrollingcntr = message->GetSignal("DBW_AccelPdlRollingCntr")->GetResult();

	pubDbwAccelpdlreport_->publish(out);
}

void RaptorDbwCAN::recvDbwSteeringreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwSteeringreport out;
	out.stamp = msg->header.stamp;

	out.dbw_steeringwhlangleact = message->GetSignal("DBW_SteeringWhlAngleAct")->GetResult();
	out.dbw_steeringwhlangledes = message->GetSignal("DBW_SteeringWhlAngleDes")->GetResult();
	out.dbw_steeringwhlpcnttrqcmd = message->GetSignal("DBW_SteeringWhlPcntTrqCmd")->GetResult();
	out.dbw_steeringctrltype = message->GetSignal("DBW_SteeringCtrlType")->GetResult();
	out.dbw_steeringenabled = message->GetSignal("DBW_SteeringEnabled")->GetResult();
	out.dbw_steeringdriveractivity = message->GetSignal("DBW_SteeringDriverActivity")->GetResult();
	out.dbw_steeringfault = message->GetSignal("DBW_SteeringFault")->GetResult();
	out.dbw_overheatpreventmode = message->GetSignal("DBW_OverheatPreventMode")->GetResult();
	out.dbw_steeringoverheatwarning = message->GetSignal("DBW_SteeringOverheatWarning")->GetResult();
	out.dbw_steeringrollingcntr = message->GetSignal("DBW_SteeringRollingCntr")->GetResult();

	pubDbwSteeringreport_->publish(out);
}

void RaptorDbwCAN::recvDbwBrakereport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwBrakereport out;
	out.stamp = msg->header.stamp;

	out.dbw_brakepdldriverinput = message->GetSignal("DBW_BrakePdlDriverInput")->GetResult();
	out.dbw_brakepdlposnfdbck = message->GetSignal("DBW_BrakePdlPosnFdbck")->GetResult();
	out.dbw_brakepcnttorqueactual = message->GetSignal("DBW_BrakePcntTorqueActual")->GetResult();
	out.dbw_brakectrltype = message->GetSignal("DBW_BrakeCtrlType")->GetResult();
	out.dbw_brakeenabled = message->GetSignal("DBW_BrakeEnabled")->GetResult();
	out.dbw_brakedriveractivity = message->GetSignal("DBW_BrakeDriverActivity")->GetResult();
	out.dbw_brakefault = message->GetSignal("DBW_BrakeFault")->GetResult();
	out.dbw_brakeparkingbrkstatus = message->GetSignal("DBW_BrakeParkingBrkStatus")->GetResult();
	out.dbw_brakerollingcntr = message->GetSignal("DBW_BrakeRollingCntr")->GetResult();
	out.dbw_brakeinterventionactv = message->GetSignal("DBW_BrakeInterventionActv")->GetResult();
	out.dbw_brakeinterventionready = message->GetSignal("DBW_BrakeInterventionReady")->GetResult();

	pubDbwBrakereport_->publish(out);
}

void RaptorDbwCAN::recvDbwPrndreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwPrndreport out;
	out.stamp = msg->header.stamp;

	out.dbw_prndstateactual = message->GetSignal("DBW_PrndStateActual")->GetResult();
	out.dbw_transcurgear = message->GetSignal("DBW_TransCurGear")->GetResult();
	out.dbw_prndstatereject = message->GetSignal("DBW_PrndStateReject")->GetResult();
	out.dbw_prndmismatchflash = message->GetSignal("DBW_PrndMismatchFlash")->GetResult();
	out.dbw_prnddriveractivity = message->GetSignal("DBW_PrndDriverActivity")->GetResult();
	out.dbw_prndfault = message->GetSignal("DBW_PrndFault")->GetResult();
	out.dbw_prndctrlenabled = message->GetSignal("DBW_PrndCtrlEnabled")->GetResult();

	pubDbwPrndreport_->publish(out);
}

void RaptorDbwCAN::recvDbwWheelpositionreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwWheelpositionreport out;
	out.stamp = msg->header.stamp;

	out.dbw_whlpulsecnt_fl = message->GetSignal("DBW_WhlPulseCnt_FL")->GetResult();
	out.dbw_whlpulsecnt_fr = message->GetSignal("DBW_WhlPulseCnt_FR")->GetResult();
	out.dbw_whlpulsecnt_rl = message->GetSignal("DBW_WhlPulseCnt_RL")->GetResult();
	out.dbw_whlpulsecnt_rr = message->GetSignal("DBW_WhlPulseCnt_RR")->GetResult();
	out.dbw_whlpulsesperrev = message->GetSignal("DBW_WhlPulsesPerRev")->GetResult();

	pubDbwWheelpositionreport_->publish(out);
}

void RaptorDbwCAN::recvAkitOtheractuators(const Frame::SharedPtr msg, DbcMessage * message)
{
	AkitOtheractuators out;
	out.stamp = msg->header.stamp;

	out.akit_doorlockreq = message->GetSignal("AKit_DoorLockReq")->GetResult();
	out.akit_turnsignalreq = message->GetSignal("AKit_TurnSignalReq")->GetResult();
	out.akit_rightreardoorreq = message->GetSignal("AKit_RightRearDoorReq")->GetResult();
	out.akit_leftreardoorreq = message->GetSignal("AKit_LeftRearDoorReq")->GetResult();
	out.akit_hornreq = message->GetSignal("AKit_HornReq")->GetResult();
	out.akit_blockbasiccruisectrlbtns = message->GetSignal("AKit_BlockBasicCruiseCtrlBtns")->GetResult();
	out.akit_blockadapcruisectrlbtns = message->GetSignal("AKit_BlockAdapCruiseCtrlBtns")->GetResult();
	out.akit_blockturnsigstalkinpts = message->GetSignal("AKit_BlockTurnSigStalkInpts")->GetResult();
	out.akit_frontwiperreq = message->GetSignal("AKit_FrontWiperReq")->GetResult();
	out.akit_rearwiperreq = message->GetSignal("AKit_RearWiperReq")->GetResult();
	out.akit_ignitionreq = message->GetSignal("AKit_IgnitionReq")->GetResult();
	out.akit_highbeamreq = message->GetSignal("AKit_HighBeamReq")->GetResult();
	out.akit_lowbeamreq = message->GetSignal("AKit_LowBeamReq")->GetResult();
	out.akit_liftgatedoorreq = message->GetSignal("AKit_LiftgateDoorReq")->GetResult();
	out.akit_otherrollingcntr = message->GetSignal("AKit_OtherRollingCntr")->GetResult();
	out.akit_otherchecksum = message->GetSignal("AKit_OtherChecksum")->GetResult();

	pubAkitOtheractuators_->publish(out);
}

void RaptorDbwCAN::recvBrakePressureReport(const Frame::SharedPtr msg, DbcMessage * message)
{
	BrakePressureReport out;
	out.stamp = msg->header.stamp;

	out.brake_pressure_fdbk_front = message->GetSignal("brake_pressure_fdbk_front")->GetResult();
	out.brake_pressure_fdbk_rear = message->GetSignal("brake_pressure_fdbk_rear")->GetResult();
	out.brk_pressure_fdbk_counter = message->GetSignal("brk_pressure_fdbk_counter")->GetResult();

	// out.brake_pressure_fdbk_rear = (2.59 * ((out.brake_pressure_fdbk_rear + 2341)/4.67)) - 1293;
	out.brake_pressure_fdbk_front = ((out.brake_pressure_fdbk_rear + 2341)/4.67);
	// out.brake_pressure_fdbk_rear = (2.56 * ((out.brake_pressure_fdbk_rear + 2341)/4.67)) - 1179;

	pubBrakePressureReport_->publish(out);
}

void RaptorDbwCAN::recvDbwDriverinputs(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwDriverinputs out;
	out.stamp = msg->header.stamp;

	out.dbw_occupanydoororhoodajar = message->GetSignal("DBW_OccupAnyDoorOrHoodAjar")->GetResult();
	out.dbw_occupanyairbagdeployed = message->GetSignal("DBW_OccupAnyAirbagDeployed")->GetResult();
	out.dbw_occupanyseatbeltunbuckled = message->GetSignal("DBW_OccupAnySeatbeltUnbuckled")->GetResult();
	out.dbw_drvinputstrwhlbtna = message->GetSignal("DBW_DrvInputStrWhlBtnA")->GetResult();
	out.dbw_drvinputstrwhlbtnb = message->GetSignal("DBW_DrvInputStrWhlBtnB")->GetResult();
	out.dbw_drvinputstrwhlbtnc = message->GetSignal("DBW_DrvInputStrWhlBtnC")->GetResult();
	out.dbw_drvinputstrwhlbtnd = message->GetSignal("DBW_DrvInputStrWhlBtnD")->GetResult();
	out.dbw_drvinputstrwhlbtne = message->GetSignal("DBW_DrvInputStrWhlBtnE")->GetResult();
	out.dbw_drvinptcruiseresumebtn = message->GetSignal("DBW_DrvInptCruiseResumeBtn")->GetResult();
	out.dbw_drvinptcruisecancelbtn = message->GetSignal("DBW_DrvInptCruiseCancelBtn")->GetResult();
	out.dbw_drvinptcruiseaccelbtn = message->GetSignal("DBW_DrvInptCruiseAccelBtn")->GetResult();
	out.dbw_drvinptcruisedecelbtn = message->GetSignal("DBW_DrvInptCruiseDecelBtn")->GetResult();
	out.dbw_drvinptcruiseonoffbtn = message->GetSignal("DBW_DrvInptCruiseOnOffBtn")->GetResult();
	out.dbw_drvinptacconoffbtn = message->GetSignal("DBW_DrvInptAccOnOffBtn")->GetResult();
	out.dbw_drvinptaccincdistbtn = message->GetSignal("DBW_DrvInptAccIncDistBtn")->GetResult();
	out.dbw_drvinptaccdecdistbtn = message->GetSignal("DBW_DrvInptAccDecDistBtn")->GetResult();
	out.dbw_drvinpthibeam = message->GetSignal("DBW_DrvInptHiBeam")->GetResult();
	out.dbw_drvinptturnsignal = message->GetSignal("DBW_DrvInptTurnSignal")->GetResult();
	out.dbw_drvinptwiper = message->GetSignal("DBW_DrvInptWiper")->GetResult();

	pubDbwDriverinputs_->publish(out);
}

void RaptorDbwCAN::recvDbwTirepressreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwTirepressreport out;
	out.stamp = msg->header.stamp;

	out.dbw_tirepressrr = message->GetSignal("DBW_TirePressRR")->GetResult();
	out.dbw_tirepressrl = message->GetSignal("DBW_TirePressRL")->GetResult();
	out.dbw_tirepressfr = message->GetSignal("DBW_TirePressFR")->GetResult();
	out.dbw_tirepressfl = message->GetSignal("DBW_TirePressFL")->GetResult();

	pubDbwTirepressreport_->publish(out);
}

void RaptorDbwCAN::recvDbwVinreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwVinreport out;
	out.stamp = msg->header.stamp;

	out.dbw_vinmultiplexor = message->GetSignal("DBW_VinMultiplexor")->GetResult();
	out.dbw_vindigit_01 = message->GetSignal("DBW_VinDigit_01")->GetResult();
	out.dbw_vindigit_08 = message->GetSignal("DBW_VinDigit_08")->GetResult();
	out.dbw_vindigit_15 = message->GetSignal("DBW_VinDigit_15")->GetResult();
	out.dbw_vindigit_02 = message->GetSignal("DBW_VinDigit_02")->GetResult();
	out.dbw_vindigit_09 = message->GetSignal("DBW_VinDigit_09")->GetResult();
	out.dbw_vindigit_16 = message->GetSignal("DBW_VinDigit_16")->GetResult();
	out.dbw_vindigit_03 = message->GetSignal("DBW_VinDigit_03")->GetResult();
	out.dbw_vindigit_10 = message->GetSignal("DBW_VinDigit_10")->GetResult();
	out.dbw_vindigit_17 = message->GetSignal("DBW_VinDigit_17")->GetResult();
	out.dbw_vindigit_04 = message->GetSignal("DBW_VinDigit_04")->GetResult();
	out.dbw_vindigit_11 = message->GetSignal("DBW_VinDigit_11")->GetResult();
	out.dbw_vindigit_05 = message->GetSignal("DBW_VinDigit_05")->GetResult();
	out.dbw_vindigit_12 = message->GetSignal("DBW_VinDigit_12")->GetResult();
	out.dbw_vindigit_06 = message->GetSignal("DBW_VinDigit_06")->GetResult();
	out.dbw_vindigit_13 = message->GetSignal("DBW_VinDigit_13")->GetResult();
	out.dbw_vindigit_07 = message->GetSignal("DBW_VinDigit_07")->GetResult();
	out.dbw_vindigit_14 = message->GetSignal("DBW_VinDigit_14")->GetResult();

	pubDbwVinreport_->publish(out);
}

void RaptorDbwCAN::recvDbwReserved1(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwReserved1 out;
	out.stamp = msg->header.stamp;

	

	pubDbwReserved1_->publish(out);
}

void RaptorDbwCAN::recvDbwImureport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwImureport out;
	out.stamp = msg->header.stamp;

	out.dbw_imuyawrate = message->GetSignal("DBW_ImuYawRate")->GetResult();
	out.dbw_imuaccelx = message->GetSignal("DBW_ImuAccelX")->GetResult();
	out.dbw_imuaccely = message->GetSignal("DBW_ImuAccelY")->GetResult();

	pubDbwImureport_->publish(out);
}

void RaptorDbwCAN::recvDbwWheelspeedreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwWheelspeedreport out;
	out.stamp = msg->header.stamp;

	out.dbw_whlspd_fl = message->GetSignal("DBW_WhlSpd_FL")->GetResult();
	out.dbw_whlspd_fr = message->GetSignal("DBW_WhlSpd_FR")->GetResult();
	out.dbw_whlspd_rl = message->GetSignal("DBW_WhlSpd_RL")->GetResult();
	out.dbw_whlspd_rr = message->GetSignal("DBW_WhlSpd_RR")->GetResult();

	pubDbwWheelspeedreport_->publish(out);
}

void RaptorDbwCAN::recvDbwRadarsonar(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwRadarsonar out;
	out.stamp = msg->header.stamp;

	out.dbw_reserved2 = message->GetSignal("DBW_Reserved2")->GetResult();
	out.dbw_sonararcnumrr = message->GetSignal("DBW_SonarArcNumRR")->GetResult();
	out.dbw_sonararcnumrl = message->GetSignal("DBW_SonarArcNumRL")->GetResult();
	out.dbw_sonararcnumrc = message->GetSignal("DBW_SonarArcNumRC")->GetResult();
	out.dbw_sonararcnumfr = message->GetSignal("DBW_SonarArcNumFR")->GetResult();
	out.dbw_sonararcnumfl = message->GetSignal("DBW_SonarArcNumFL")->GetResult();
	out.dbw_sonararcnumfc = message->GetSignal("DBW_SonarArcNumFC")->GetResult();
	out.dbw_sonarreardist = message->GetSignal("DBW_SonarRearDist")->GetResult();
	out.dbw_reserved3 = message->GetSignal("DBW_Reserved3")->GetResult();
	out.dbw_sonarvld = message->GetSignal("DBW_SonarVld")->GetResult();

	pubDbwRadarsonar_->publish(out);
}

void RaptorDbwCAN::recvDbwFaulttext(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwFaulttext out;
	out.stamp = msg->header.stamp;

	out.flttxt_pagenum = message->GetSignal("FltTxt_PageNum")->GetResult();
	out.flttxt_faultid = message->GetSignal("FltTxt_FaultId")->GetResult();
	out.flttxt_faultsetruntime = message->GetSignal("FltTxt_FaultSetRuntime")->GetResult();
	out.flttxt_char01 = message->GetSignal("FltTxt_Char01")->GetResult();
	out.flttxt_char08 = message->GetSignal("FltTxt_Char08")->GetResult();
	out.flttxt_char15 = message->GetSignal("FltTxt_Char15")->GetResult();
	out.flttxt_char22 = message->GetSignal("FltTxt_Char22")->GetResult();
	out.flttxt_char29 = message->GetSignal("FltTxt_Char29")->GetResult();
	out.flttxt_char36 = message->GetSignal("FltTxt_Char36")->GetResult();
	out.flttxt_char43 = message->GetSignal("FltTxt_Char43")->GetResult();
	out.flttxt_char02 = message->GetSignal("FltTxt_Char02")->GetResult();
	out.flttxt_char09 = message->GetSignal("FltTxt_Char09")->GetResult();
	out.flttxt_char16 = message->GetSignal("FltTxt_Char16")->GetResult();
	out.flttxt_char23 = message->GetSignal("FltTxt_Char23")->GetResult();
	out.flttxt_char30 = message->GetSignal("FltTxt_Char30")->GetResult();
	out.flttxt_char37 = message->GetSignal("FltTxt_Char37")->GetResult();
	out.flttxt_char44 = message->GetSignal("FltTxt_Char44")->GetResult();
	out.flttxt_char03 = message->GetSignal("FltTxt_Char03")->GetResult();
	out.flttxt_char10 = message->GetSignal("FltTxt_Char10")->GetResult();
	out.flttxt_char17 = message->GetSignal("FltTxt_Char17")->GetResult();
	out.flttxt_char24 = message->GetSignal("FltTxt_Char24")->GetResult();
	out.flttxt_char31 = message->GetSignal("FltTxt_Char31")->GetResult();
	out.flttxt_char38 = message->GetSignal("FltTxt_Char38")->GetResult();
	out.flttxt_char45 = message->GetSignal("FltTxt_Char45")->GetResult();
	out.flttxt_currentruntime = message->GetSignal("FltTxt_CurrentRuntime")->GetResult();
	out.flttxt_char04 = message->GetSignal("FltTxt_Char04")->GetResult();
	out.flttxt_char11 = message->GetSignal("FltTxt_Char11")->GetResult();
	out.flttxt_char18 = message->GetSignal("FltTxt_Char18")->GetResult();
	out.flttxt_char25 = message->GetSignal("FltTxt_Char25")->GetResult();
	out.flttxt_char32 = message->GetSignal("FltTxt_Char32")->GetResult();
	out.flttxt_char39 = message->GetSignal("FltTxt_Char39")->GetResult();
	out.flttxt_char46 = message->GetSignal("FltTxt_Char46")->GetResult();
	out.flttxt_numpages = message->GetSignal("FltTxt_NumPages")->GetResult();
	out.flttxt_char05 = message->GetSignal("FltTxt_Char05")->GetResult();
	out.flttxt_char12 = message->GetSignal("FltTxt_Char12")->GetResult();
	out.flttxt_char19 = message->GetSignal("FltTxt_Char19")->GetResult();
	out.flttxt_char26 = message->GetSignal("FltTxt_Char26")->GetResult();
	out.flttxt_char33 = message->GetSignal("FltTxt_Char33")->GetResult();
	out.flttxt_char40 = message->GetSignal("FltTxt_Char40")->GetResult();
	out.flttxt_char47 = message->GetSignal("FltTxt_Char47")->GetResult();
	out.flttxt_faultstatus = message->GetSignal("FltTxt_FaultStatus")->GetResult();
	out.flttxt_char06 = message->GetSignal("FltTxt_Char06")->GetResult();
	out.flttxt_char13 = message->GetSignal("FltTxt_Char13")->GetResult();
	out.flttxt_char20 = message->GetSignal("FltTxt_Char20")->GetResult();
	out.flttxt_char27 = message->GetSignal("FltTxt_Char27")->GetResult();
	out.flttxt_char34 = message->GetSignal("FltTxt_Char34")->GetResult();
	out.flttxt_char41 = message->GetSignal("FltTxt_Char41")->GetResult();
	out.flttxt_char48 = message->GetSignal("FltTxt_Char48")->GetResult();
	out.flttxt_crc = message->GetSignal("FltTxt_CRC")->GetResult();
	out.flttxt_char07 = message->GetSignal("FltTxt_Char07")->GetResult();
	out.flttxt_char14 = message->GetSignal("FltTxt_Char14")->GetResult();
	out.flttxt_char21 = message->GetSignal("FltTxt_Char21")->GetResult();
	out.flttxt_char28 = message->GetSignal("FltTxt_Char28")->GetResult();
	out.flttxt_char35 = message->GetSignal("FltTxt_Char35")->GetResult();
	out.flttxt_char42 = message->GetSignal("FltTxt_Char42")->GetResult();
	out.flttxt_char49 = message->GetSignal("FltTxt_Char49")->GetResult();

	pubDbwFaulttext_->publish(out);
}

void RaptorDbwCAN::recvDbwLowvoltsysreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwLowvoltsysreport out;
	out.stamp = msg->header.stamp;

	out.dbw_lvdbwbattvlt = message->GetSignal("DBW_LvDbwBattVlt")->GetResult();
	out.dbw_lvvehbattvlt = message->GetSignal("DBW_LvVehBattVlt")->GetResult();
	out.dbw_lvbattcurr = message->GetSignal("DBW_LvBattCurr")->GetResult();
	out.dbw_lvdcdccurr = message->GetSignal("DBW_LvDcdcCurr")->GetResult();
	out.dbw_lvalternatorcurr = message->GetSignal("DBW_LvAlternatorCurr")->GetResult();
	out.dbw_lvinvtrcontactorcmd = message->GetSignal("DBW_LvInvtrContactorCmd")->GetResult();

	pubDbwLowvoltsysreport_->publish(out);
}

void RaptorDbwCAN::recvDbwBrakereport2(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwBrakereport2 out;
	out.stamp = msg->header.stamp;

	out.dbw_brakepress_bar = message->GetSignal("DBW_BrakePress_bar")->GetResult();
	out.dbw_roadslopeestimate = message->GetSignal("DBW_RoadSlopeEstimate")->GetResult();
	out.dbw_speedsetpt = message->GetSignal("DBW_SpeedSetpt")->GetResult();

	pubDbwBrakereport2_->publish(out);
}

void RaptorDbwCAN::recvDbwSteeringreport2(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwSteeringreport2 out;
	out.stamp = msg->header.stamp;

	out.dbw_steeringvehcurvatureact = message->GetSignal("DBW_SteeringVehCurvatureAct")->GetResult();
	out.dbw_pacificasteertrq = message->GetSignal("DBW_PacificaSteerTrq")->GetResult();
	out.dbw_steertrq_motor = message->GetSignal("DBW_SteerTrq_Motor")->GetResult();
	out.dbw_steertrq_driver = message->GetSignal("DBW_SteerTrq_Driver")->GetResult();
	out.dbw_steertrq_driverexpectedvalue = message->GetSignal("DBW_SteerTrq_DriverExpectedValue")->GetResult();

	pubDbwSteeringreport2_->publish(out);
}

void RaptorDbwCAN::recvDbwOtheractuatorsreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwOtheractuatorsreport out;
	out.stamp = msg->header.stamp;

	out.dbw_doorlockstate = message->GetSignal("DBW_DoorLockState")->GetResult();
	out.dbw_turnsignalstate = message->GetSignal("DBW_TurnSignalState")->GetResult();
	out.dbw_turnsignalsyncbit = message->GetSignal("DBW_TurnSignalSyncBit")->GetResult();
	out.dbw_rightreardoorstate = message->GetSignal("DBW_RightRearDoorState")->GetResult();
	out.dbw_leftreardoorstate = message->GetSignal("DBW_LeftRearDoorState")->GetResult();
	out.dbw_hornstate = message->GetSignal("DBW_HornState")->GetResult();
	out.dbw_frontwiperstate = message->GetSignal("DBW_FrontWiperState")->GetResult();
	out.dbw_rearwiperstate = message->GetSignal("DBW_RearWiperState")->GetResult();
	out.dbw_ignitionstate = message->GetSignal("DBW_IgnitionState")->GetResult();
	out.dbw_highbeamstate = message->GetSignal("DBW_HighBeamState")->GetResult();
	out.dbw_lowbeamstate = message->GetSignal("DBW_LowBeamState")->GetResult();
	out.dbw_liftgatedoorstate = message->GetSignal("DBW_LiftgateDoorState")->GetResult();
	out.dbw_otheractrollingcntr = message->GetSignal("DBW_OtherActRollingCntr")->GetResult();

	pubDbwOtheractuatorsreport_->publish(out);
}

void RaptorDbwCAN::recvDbwFaultactionsreport(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwFaultactionsreport out;
	out.stamp = msg->header.stamp;

	out.dbw_fltact_autondsblnobrakes = message->GetSignal("DBW_FltAct_AutonDsblNoBrakes")->GetResult();
	out.dbw_fltact_autondsblapplybrakes = message->GetSignal("DBW_FltAct_AutonDsblApplyBrakes")->GetResult();
	out.dbw_fltact_cangatewaydsbl = message->GetSignal("DBW_FltAct_CANGatewayDsbl")->GetResult();
	out.dbw_fltact_invtrcntctrdsbl = message->GetSignal("DBW_FltAct_InvtrCntctrDsbl")->GetResult();
	out.dbw_fltact_prevententerautonmode = message->GetSignal("DBW_FltAct_PreventEnterAutonMode")->GetResult();
	out.dbw_fltact_warndriveronly = message->GetSignal("DBW_FltAct_WarnDriverOnly")->GetResult();
	out.dbw_fltact_chime_fcwbeeps = message->GetSignal("DBW_FltAct_Chime_FcwBeeps")->GetResult();

	pubDbwFaultactionsreport_->publish(out);
}

void RaptorDbwCAN::recvDbwGpsreference(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwGpsreference out;
	out.stamp = msg->header.stamp;

	out.dbw_gpsreflat = message->GetSignal("DBW_GpsRefLat")->GetResult();
	out.dbw_gpsreflong = message->GetSignal("DBW_GpsRefLong")->GetResult();

	pubDbwGpsreference_->publish(out);
}

void RaptorDbwCAN::recvDbwGpsremainder(const Frame::SharedPtr msg, DbcMessage * message)
{
	DbwGpsremainder out;
	out.stamp = msg->header.stamp;

	out.dbw_gpsremainderlat = message->GetSignal("DBW_GpsRemainderLat")->GetResult();
	out.dbw_gpsremainderlong = message->GetSignal("DBW_GpsRemainderLong")->GetResult();

	pubDbwGpsremainder_->publish(out);
}

void RaptorDbwCAN::recvBrakeControl(const Frame::SharedPtr msg, DbcMessage * message)
{
	BrakeControl out;
	out.stamp = msg->header.stamp;

	out.position_command = message->GetSignal("Position_Command")->GetResult();
	out.datatype = message->GetSignal("Datatype")->GetResult();
	out.autoreply_flag = message->GetSignal("Autoreply_Flag")->GetResult();
	out.confirmation_flag = message->GetSignal("Confirmation_Flag")->GetResult();
	out.dpos_low = message->GetSignal("DPOS_LOW")->GetResult();
	out.dpos_hi = message->GetSignal("DPOS_HI")->GetResult();
	out.motor_enable = message->GetSignal("Motor_Enable")->GetResult();
	out.clutch_enable = message->GetSignal("Clutch_Enable")->GetResult();

	pubBrakeControl_->publish(out);
}

void RaptorDbwCAN::recvBrakePositionReport(const Frame::SharedPtr msg, DbcMessage * message)
{
	BrakePositionReport out;
	out.stamp = msg->header.stamp;

	out.messagetype = message->GetSignal("MessageType")->GetResult();
	out.confirmationflag = message->GetSignal("ConfirmationFlag")->GetResult();
	out.autoreplyflag = message->GetSignal("AutoReplyFlag")->GetResult();
	out.datatype = message->GetSignal("DataType")->GetResult();
	out.shaftextension = message->GetSignal("ShaftExtension")->GetResult();

	pubBrakePositionReport_->publish(out);
}

void RaptorDbwCAN::recvAkitAccelpdlrequest(const AkitAccelpdlrequest::SharedPtr msg)
{
	NewEagle::DbcMessage * message = dbc_.GetMessageById(ID_AKIT_ACCELPDLREQUEST);

	message->GetSignal("AKit_AccelPdlReq")->SetResult(msg->akit_accelpdlreq);
	message->GetSignal("AKit_AccelPcntTorqueReq")->SetResult(msg->akit_accelpcnttorquereq);
	message->GetSignal("AKit_SpeedReq")->SetResult(msg->akit_speedreq);
	message->GetSignal("AKit_SpeedModeAccelLim")->SetResult(msg->akit_speedmodeaccellim);
	message->GetSignal("AKit_SpeedModePosJerkLim")->SetResult(msg->akit_speedmodeposjerklim);
	message->GetSignal("AKit_SpeedModeRoadSlope")->SetResult(msg->akit_speedmoderoadslope);
	message->GetSignal("AKit_AccelReqType")->SetResult(msg->akit_accelreqtype);
	message->GetSignal("AKit_AccelPdlRollingCntr")->SetResult(msg->akit_accelpdlrollingcntr);
	message->GetSignal("AKit_AccelPdlEnblReq")->SetResult(msg->akit_accelpdlenblreq);
	message->GetSignal("Akit_AccelPdlIgnoreDriverOvrd")->SetResult(msg->akit_accelpdlignoredriverovrd);
	message->GetSignal("AKit_AccelPdlChecksum")->SetResult(msg->akit_accelpdlchecksum);

	Frame frame = message->GetFrame();
	pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAkitGlobalenbl(const AkitGlobalenbl::SharedPtr msg)
{
	NewEagle::DbcMessage * message = dbc_.GetMessageById(ID_AKIT_GLOBALENBL);

	message->GetSignal("AKit_GlobalByWireEnblReq")->SetResult(msg->akit_globalbywireenblreq);
	message->GetSignal("AKit_EnblJoystickLimits")->SetResult(msg->akit_enbljoysticklimits);
	message->GetSignal("AKit_SoftwareBuildNumber")->SetResult(msg->akit_softwarebuildnumber);
	message->GetSignal("AKit_GlobalEnblRollingCntr")->SetResult(msg->akit_globalenblrollingcntr);
	message->GetSignal("Akit_GlobalEnblChecksum")->SetResult(msg->akit_globalenblchecksum);

	Frame frame = message->GetFrame();
	pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAkitSteeringrequest(const AkitSteeringrequest::SharedPtr msg)
{
	NewEagle::DbcMessage * message = dbc_.GetMessageById(ID_AKIT_STEERINGREQUEST);

	message->GetSignal("AKit_SteeringWhlAngleReq")->SetResult(msg->akit_steeringwhlanglereq);
	message->GetSignal("AKit_SteeringWhlPcntTrqReq")->SetResult(msg->akit_steeringwhlpcnttrqreq);
	message->GetSignal("AKit_SteeringVehCurvatureReq")->SetResult(msg->akit_steeringvehcurvaturereq);
	message->GetSignal("AKit_SteeringWhlAngleVelocityLim")->SetResult(msg->akit_steeringwhlanglevelocitylim);
	message->GetSignal("AKit_SteeringReqType")->SetResult(msg->akit_steeringreqtype);
	message->GetSignal("AKit_SteerRollingCntr")->SetResult(msg->akit_steerrollingcntr);
	message->GetSignal("AKit_SteerCtrlEnblReq")->SetResult(msg->akit_steerctrlenblreq);
	message->GetSignal("AKit_SteeringWhlIgnoreDriverOvrd")->SetResult(msg->akit_steeringwhlignoredriverovrd);
	message->GetSignal("AKit_SteeringChecksum")->SetResult(msg->akit_steeringchecksum);

	Frame frame = message->GetFrame();
	pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAkitBrakerequest(const AkitBrakerequest::SharedPtr msg)
{
	NewEagle::DbcMessage * message = dbc_.GetMessageById(ID_AKIT_BRAKEREQUEST);

	message->GetSignal("AKit_BrakePedalReq")->SetResult(msg->akit_brakepedalreq);
	message->GetSignal("AKit_BrakePcntTorqueReq")->SetResult(msg->akit_brakepcnttorquereq);
	message->GetSignal("AKit_ParkingBrkReq")->SetResult(msg->akit_parkingbrkreq);
	message->GetSignal("AKit_SpeedModeDecelLim")->SetResult(msg->akit_speedmodedecellim);
	message->GetSignal("AKit_SpeedModeNegJerkLim")->SetResult(msg->akit_speedmodenegjerklim);
	message->GetSignal("AKit_BrakeCtrlReqType")->SetResult(msg->akit_brakectrlreqtype);
	message->GetSignal("AKit_BrakeRollingCntr")->SetResult(msg->akit_brakerollingcntr);
	message->GetSignal("AKit_BrakeCtrlEnblReq")->SetResult(msg->akit_brakectrlenblreq);
	message->GetSignal("AKit_BrakeChecksum")->SetResult(msg->akit_brakechecksum);

	Frame frame = message->GetFrame();
	pub_can_->publish(frame);
}

void RaptorDbwCAN::recvAkitPrndrequest(const AkitPrndrequest::SharedPtr msg)
{
	NewEagle::DbcMessage * message = dbc_.GetMessageById(ID_AKIT_PRNDREQUEST);

	message->GetSignal("AKit_PrndStateReq")->SetResult(msg->akit_prndstatereq);
	message->GetSignal("AKit_PrndRollingCntr")->SetResult(msg->akit_prndrollingcntr);
	message->GetSignal("AKit_PrndCtrlEnblReq")->SetResult(msg->akit_prndctrlenblreq);
	message->GetSignal("AKit_PrndChecksum")->SetResult(msg->akit_prndchecksum);

	Frame frame = message->GetFrame();
	pub_can_->publish(frame);
}

}  // namespace raptor_dbw_can

