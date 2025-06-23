/////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright [2022] [Hesai Technology Co., Ltd] 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @file
 * <b>HESAI Plugin for DriveWorks: Lidar Sensor UDP Parser</b>
 *
 * @b Description: This file defines the udp parser for Pandar64.
 */

#ifndef UDP64_PARSER_H_
#define UDP64_PARSER_H_

// Unit of azimuth in UDP packet 1/100
#define HS_LIDAR_P64_AZIMUTH_UNIT_UDP (100)
#define HS_LIDAR_P64_LASER_NUM (64)

#include "GeneralParser.h"
#include "HsLidar64.h"

// For Pandar64
class Udp64_Parser : public GeneralParser {
 public:
  Udp64_Parser();
  virtual ~Udp64_Parser();

  dwStatus GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) override;
  
  virtual dwStatus ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                   dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) override;

  int16_t GetVecticalAngle(int channel) override;

 private:
  // to be updated by the UDP packet
  int m_nLaserNum = HS_LIDAR_P64_LASER_NUM;
  // block number in a UDP packet
  int m_nBlockNum = 6;
  // unit of azimuth angle from UDP packet
  const int m_nAziUnitUDP = HS_LIDAR_P64_AZIMUTH_UNIT_UDP;

  unsigned long GetDataBodySize(const HS_LIDAR_HEADER_64 *pHeader);
  int64_t GetMicroLidarTimeU64(const uint8_t* utc, int size, uint32_t timestamp) const;
};

#endif  // UDP64_PARSER_H_
