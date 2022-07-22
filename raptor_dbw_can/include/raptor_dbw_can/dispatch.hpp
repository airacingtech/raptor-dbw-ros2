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

#ifndef RAPTOR_DBW_CAN__DISPATCH_HPP_
#define RAPTOR_DBW_CAN__DISPATCH_HPP_

namespace raptor_dbw_can
{

/** \brief Enumeration of CAN message IDs */
enum ListMessageIDs
{
    ID_DBW_MISC = 0x1f01,
	ID_DBW_ACCELPDLREPORT = 0x1f02,
	ID_DBW_STEERINGREPORT = 0x1f03,
	ID_DBW_BRAKEREPORT = 0x1f04,
	ID_DBW_PRNDREPORT = 0x1f05,
	ID_DBW_WHEELPOSITIONREPORT = 0x1f06,
	ID_AKIT_ACCELPDLREQUEST = 0x2f02,
	ID_AKIT_GLOBALENBL = 0x2f01,
	ID_AKIT_OTHERACTUATORS = 0x2f06,
	ID_AKIT_STEERINGREQUEST = 0x2f03,
	ID_AKIT_BRAKEREQUEST = 0x2f04,
	ID_AKIT_PRNDREQUEST = 0x2f05,
	ID_DBW_DRIVERINPUTS = 0x1f0f,
	ID_DBW_TIREPRESSREPORT = 0x1f07,
	ID_DBW_VINREPORT = 0x1f08,
	ID_DBW_RESERVED1 = 0x1f09,
	ID_DBW_IMUREPORT = 0x1f0a,
	ID_DBW_WHEELSPEEDREPORT = 0x1f0b,
	ID_DBW_RADARSONAR = 0x1f10,
	ID_DBW_FAULTTEXT = 0x070f,
	ID_DBW_LOWVOLTSYSREPORT = 0x1f11,
	ID_DBW_BRAKEREPORT2 = 0x1f12,
	ID_DBW_STEERINGREPORT2 = 0x1f13,
	ID_DBW_OTHERACTUATORSREPORT = 0x1f14,
	ID_DBW_FAULTACTIONSREPORT = 0x1f15,
	ID_DBW_GPSREFERENCE = 0x1f16,
	ID_DBW_GPSREMAINDER = 0x1f17,
	ID_DBW_EXITREPORT = 0x1f24,
	ID_DBW_RESERVED_1F20 = 0x1f20,
	ID_DBW_RESERVED_1F21 = 0x1f21,
	ID_DBW_RESERVED_1F22 = 0x1f22,
	ID_DBW_RESERVED_1F23 = 0x1f23,
	ID_AKIT_RESERVED_2F07 = 0x2f07,
	ID_AKIT_RESERVED_2F08 = 0x2f08,
	ID_AKIT_RESERVED_2F09 = 0x2f09,
	ID_DBW_RESERVED_1F25 = 0x1f25,
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DISPATCH_HPP_