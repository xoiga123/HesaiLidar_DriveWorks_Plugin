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

#include <iostream>
#include "Udp64_Parser.h"

Udp64_Parser::Udp64_Parser() {}

Udp64_Parser::~Udp64_Parser() { 
  // printf("release general parser\n"); 
}

dwStatus Udp64_Parser::GetDecoderConstants(_dwSensorLidarDecoder_constants* constants) {
    // printf("GetDecoderConstants: \n");
    // Each packet contains 1194 bytes for each Pandar64 UDP
    constants->maxPayloadSize = 1300;
    // Packet nums per second, send one packet per 0.1 degree for Pandar64. 360/0.1*10 = 36000, it takes 0.1s
    // Dual return means two block in one packet have the same timestamp
    // !std::bad_alloc happens if value is too small, narrow memory 900000 450000 ok, but 90000 fails? 
    // !Fault parameter to avoid dw printing packet dropping 12, real value = 36000
    constants->properties.packetsPerSecond = 12 / (m_bIsDualReturn ? 1 : 2);
    // !Influence the display of point cloud, 36000 * 64 * 2 = 4608000
    // Error occurs if normal size, DW_OUT_OF_BOUNDS: RenderEngine::Buffer too small for requested layout
    constants->properties.pointsPerSecond = 4608000 / (m_bIsDualReturn ? 1 : 2);
    // 10Hz 10 circle per second, 20Hz the numbers of packets halve
    constants->properties.spinFrequency = m_u16SpinSpeed / 60.0f;
    // Will be override by dw, depends on the packets received in practice
    // constants->properties.packetsPerSpin = 900;
    // constants->properties.pointsPerSpin = 230400;
    // blockNum * laserNum = 6 * 64
    constants->properties.pointsPerPacket = 384;
    
    constants->properties.pointStride = 8;
    constants->properties.horizontalFOVStart = deg2Rad(0);
    constants->properties.horizontalFOVEnd = deg2Rad(360);
    constants->properties.numberOfRows = m_nLaserNum;
    // From Pandar128 manual or correction file to take the first and last value
    constants->properties.verticalFOVStart = deg2Rad(-14);
    constants->properties.verticalFOVEnd = deg2Rad(26);
    for (int i = 0; i < m_nLaserNum; i++) {
        if(m_bGetCorrectionFile == true && m_vEleCorrection.empty() == false) {
            constants->properties.verticalAngles[i] = deg2Rad(m_vEleCorrection[i] / m_iAziCorrUnit);
            // printf("verticalAngles: %f \n", constants->properties.verticalAngles[i]);
        }
    }

    // printLidarProperty(&constants->properties);
    return DW_SUCCESS;
}

dwStatus Udp64_Parser::ParserOnePacket(dwLidarDecodedPacket *output, const uint8_t *buffer, const size_t length, \
                                  dwLidarPointXYZI* pointXYZI, dwLidarPointRTHI* pointRTHI) {
  if (buffer[0] != 0xEE || buffer[1] != 0xFF || length < 0) {
    printf("Udp64_Parser: ParserOnePacket, invalid packet %x %x\n", buffer[0], buffer[1]);
    return DW_FAILURE;
  }
  const HS_LIDAR_HEADER_64 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_64 *>(&(buffer[0]));
  // pHeader->Print();
  // point to azimuth of udp start block
  const HS_LIDAR_BODY_AZIMUTH_64 *pAzimuth =
      reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_64 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_64));
  int32_t azimuth = pAzimuth->GetAzimuth();
  m_nBlockNum = pHeader->GetBlockNum();
  m_nLaserNum = pHeader->GetLaserNum();
  // pAzimuth->Print();
  
  const auto *pTail = reinterpret_cast<const HS_LIDAR_TAIL_64 *>(
      (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_64) +
      GetDataBodySize(pHeader));
  // pTail->Print();
  m_u16SpinSpeed = pTail->m_u16MotorSpeed;
  m_bIsDualReturn = pTail->IsDualReturn();
  output->duration =  pTail->GetMicroLidarTimeU64() - 100000;
  output->hostTimestamp = 0;
  output->maxPoints = m_nBlockNum * m_nLaserNum;
  if (m_vEleCorrection.empty() == true) {
    // printf("Udp64_Parser: ParserOnePacket, no calibration string loaded Error \n");
    return DW_FAILURE;
  }
  output->maxVerticalAngleRad = m_vEleCorrection[m_nLaserNum - 1] / 1000 / 180 * M_PI;
  output->minVerticalAngleRad = m_vEleCorrection[0] / 1000 / 180 * M_PI;
  output->nPoints = m_nBlockNum * m_nLaserNum;
  // scanComplete must be filled or it cracks
  output->scanComplete = false;
  // output->sensorTimestamp = GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());
  output->sensorTimestamp = this->GetMicroLidarTimeU64(pTail->m_u8UTC, 6, pTail->GetTimestamp());

  int index = 0;
  float minAzimuth = -361;
  float maxAzimuth = 361;
  for (int blockID = 0; blockID < m_nBlockNum; blockID++) {
    // point to channel unit addr
    const HS_LIDAR_BODY_CHN_UNIT_64 *pChnUnit =
        reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_64 *>(
            (const unsigned char *)pAzimuth + sizeof(HS_LIDAR_BODY_AZIMUTH_64));
    // pAzimuth->Print();
    azimuth = pAzimuth->GetAzimuth();
    // point to next block azimuth addr
    pAzimuth = reinterpret_cast<const HS_LIDAR_BODY_AZIMUTH_64 *>(
        (const unsigned char *)pAzimuth +
        sizeof(HS_LIDAR_BODY_AZIMUTH_64) +
        sizeof(HS_LIDAR_BODY_CHN_UNIT_64) * m_nLaserNum);
    for (int laserID = 0; laserID < m_nLaserNum; laserID++) {
      int32_t elevation = this->m_vEleCorrection[laserID];
      elevation = (360000 + elevation) % 360000;  //TODO No need
      int32_t aziCorr = this->CalibrateAzimuth(azimuth, laserID);

      double distance = static_cast<double>(pChnUnit->GetDistance()) * pHeader->GetDistUnit();
      uint8_t intensity = pChnUnit->GetReflectivity();
      this->ComputeDwPoint(pointXYZI[index], pointRTHI[index], distance, elevation, aziCorr, intensity);
      // PrintDwPoint(&pointXYZI[index]);
      ++ index;
      pChnUnit = pChnUnit + 1;
      // pChnUnit->Print();
    }  // iterate laserId

    if (IsNeedFrameSplit(azimuth)) {
      output->scanComplete = true;
    }
    this->m_u16LastAzimuth = azimuth;
    if (blockID == 0) minAzimuth = azimuth;
    else maxAzimuth = azimuth;
  }  // iterate block
  
  output->maxHorizontalAngleRad = this->deg2Rad(maxAzimuth / m_nAziUnitUDP);
  output->minHorizontalAngleRad = this->deg2Rad(minAzimuth / m_nAziUnitUDP);
  output->pointsRTHI = pointRTHI;
  output->pointsXYZI = pointXYZI;
  // PrintDwPoint(&pointXYZI[index-2]);

  return DW_SUCCESS;
}

int16_t Udp64_Parser::GetVecticalAngle(int channel) {
  if (channel < 0 || channel >= HS_LIDAR_P64_LASER_NUM) {
    printf("GetVecticalAngle: channel id not in range 0-%d \n", HS_LIDAR_P64_LASER_NUM-1);
    return -1;
  }
  return m_vEleCorrection[channel];
}

unsigned long Udp64_Parser::GetDataBodySize(const HS_LIDAR_HEADER_64 *pHeader) {
  unsigned long bodySize = (sizeof(HS_LIDAR_BODY_AZIMUTH_64) + sizeof(HS_LIDAR_BODY_CHN_UNIT_64) *
                            pHeader->GetLaserNum()) * pHeader->GetBlockNum();
  return bodySize;
}

int64_t Udp64_Parser::GetMicroLidarTimeU64(const uint8_t* utc, int size, uint32_t timestamp) const {
  if (size != 6) {
    printf("GetMicroLidarTimeU64: array utc size is not 6 Error\n");
    return -1;
  }

  if (utc[0] != 0) {
    struct tm t = {0};
    t.tm_year = utc[0] + 100;
    t.tm_mon = utc[1] - 1;
    t.tm_mday = utc[2];
    t.tm_hour = utc[3];
    t.tm_min = utc[4];
    t.tm_sec = utc[5];
    t.tm_isdst = 0;
    return (mktime(&t)) * 1000000 + timestamp;
  }
  else {
    uint32_t utc_time_big = *(uint32_t*)(&utc[0] + 2);
    int unix_second = ((utc_time_big >> 24) & 0xff) |
            ((utc_time_big >> 8) & 0xff00) |
            ((utc_time_big << 8) & 0xff0000) |
            ((utc_time_big << 24));
    return unix_second * 1000000 + timestamp;
  }
}
