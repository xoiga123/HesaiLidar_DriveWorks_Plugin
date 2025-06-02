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
 * <b>HESAI Plugin for DriveWorks: Lidar Sensor UDP Protocol</b>
 *
 * @b Description: This file defines the udp parser for Pandar64.
 */

#ifndef HS_LIDAR_64_H
#define HS_LIDAR_64_H

#include <LidarProtocolHeader.h>
#include <LidarStatusInfoMe.h>

#ifdef _MSC_VER
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((packed))
#endif

struct HS_LIDAR_BODY_AZIMUTH_64 {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return little_to_native(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_ME_V4: azimuth:%u\n", GetAzimuth());
  }
} PACKED;

struct HS_LIDAR_BODY_CHN_UNIT_64 {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;

  uint16_t GetDistance() const { return little_to_native(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_64:\n");
    printf("Dist:%u, Reflectivity: %u\n", GetDistance(), GetReflectivity());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_64:\n");
  }
} PACKED;

struct HS_LIDAR_TAIL_64 {
  // shutdown flag
  static const uint8_t kNormalOp = 0x00;
  static const uint8_t kHighTemp = 0x01;

  // return mode
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;

  uint8_t m_u8Reserved1[5];
  uint8_t m_u8ShutdownFlag;
  uint8_t m_u8Reserved2[2];
  uint16_t m_u16MotorSpeed;
  uint32_t m_u32Timestamp;
  uint8_t m_u8ReturnMode;
  uint8_t m_u8FactoryInfo;  // 0x42 (or 0x43)
  uint8_t m_u8UTC[6];

  uint8_t GetShutdownFlag() const { return m_u8ShutdownFlag; }
  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  uint16_t GetMotorSpeed() const { return little_to_native(m_u16MotorSpeed); }
  uint32_t GetTimestamp() const { return little_to_native(m_u32Timestamp); }

  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsDualReturn() const { return m_u8ReturnMode == kDualReturn; }

  int64_t GetMicroLidarTimeU64() const {
    if (m_u8UTC[0] != 0) {
			struct tm t = {0};
			t.tm_year = m_u8UTC[0] + 100;
			if (t.tm_year >= 200) {
				t.tm_year -= 100;
			}
			t.tm_mon = m_u8UTC[1] - 1;
			t.tm_mday = m_u8UTC[2];
			t.tm_hour = m_u8UTC[3];
			t.tm_min = m_u8UTC[4];
			t.tm_sec = m_u8UTC[5];
			t.tm_isdst = 0;
			return (mktime(&t)) * 1000000 + GetTimestamp();
		}
		else {
      uint32_t utc_time_big = *(uint32_t*)(&m_u8UTC[0] + 2);
      int64_t unix_second = ((utc_time_big >> 24) & 0xff) |
              ((utc_time_big >> 8) & 0xff00) |
              ((utc_time_big << 8) & 0xff0000) |
              ((utc_time_big << 24));
      return unix_second * 1000000 + GetTimestamp();
		}
  }

  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }
  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_ME_V4:\n");
    printf("shutDown:%d, motorSpeed:%u, timestamp:%u, returnMode:0x%02x, "
           "factoryInfo:0x%02x, utc:%u %u %u %u %u %u\n",
           GetShutdownFlag(), GetMotorSpeed(), GetTimestamp(), GetReturnMode(),
           GetFactoryInfo(), GetUTCData(0), GetUTCData(1), GetUTCData(2),
           GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }
} PACKED;

struct HS_LIDAR_TAIL_SEQ_NUM_64 {
  uint32_t m_u32SeqNum;

  uint32_t GetSeqNum() const { return little_to_native(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void CalPktLoss() const {
    static uint32_t u32StartSeqNum = m_u32SeqNum;
    static uint32_t u32LastSeqNum = m_u32SeqNum;
    static uint32_t u32LossCount = 0;
    static uint32_t u32StartTime = GetMicroTickCount();

    if (m_u32SeqNum - u32LastSeqNum - 1 > 0) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
    }

    // print log every 10s
    if (GetMicroTickCount() - u32StartTime >= 10 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount, 
          m_u32SeqNum - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
    }

    u32LastSeqNum = m_u32SeqNum;
  }

  void Print() const { 
    printf("HS_LIDAR_TAIL_SEQ_NUM_64:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} PACKED;

struct HS_LIDAR_HEADER_64 {
  static const uint16_t kDelimiter = 0xffee;

  uint16_t m_u16Delimiter;
  uint8_t m_u8LaserNum;  // 0x40 (64 channels)
  uint8_t m_u8BlockNum;  // 0x06 (6 blocks per packet)
  uint8_t m_u8Reserved1;
  uint8_t m_u8DistUnit;
  uint16_t m_u8Reserved2;

  bool IsValidDelimiter() const {
    return little_to_native(m_u16Delimiter) == kDelimiter;
  }
  uint8_t GetDelimiter() const { return little_to_native(m_u16Delimiter); }
  uint8_t GetLaserNum() const { return m_u8LaserNum; }
  uint8_t GetBlockNum() const { return m_u8BlockNum; }
  double GetDistUnit() const { return m_u8DistUnit / 1000.f; }

  uint16_t GetPacketSize(bool udpSequence = false) const {
    return sizeof(HS_LIDAR_HEADER_64) +
           (sizeof(HS_LIDAR_BODY_AZIMUTH_64) +
           sizeof(HS_LIDAR_BODY_CHN_UNIT_64) * GetLaserNum()
           ) * GetBlockNum() +
           sizeof(HS_LIDAR_TAIL_64) +
           (udpSequence ? sizeof(HS_LIDAR_TAIL_SEQ_NUM_64) : 0);
  }
} PACKED;

#endif
