// Copyright (c) 2020 New Eagle, All rights reserved.
// Copyright (c) 2022 U Power Robotics USA
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

#ifndef DBCMESSAGE_HPP_
#define DBCMESSAGE_HPP_

#include <can_dbc_parser/DbcSignal.hpp>

#include <map>
#include <string>

namespace CanDbcParser
{
struct DbcMessageComment
{
  uint32_t Id;
  std::string Comment;
};

struct CanFrame
{
  uint32_t id;
  bool is_extended;
  bool is_fd_frame;
  uint8_t dlc;
  std::array<uint8_t, 64> data;
};

enum IdType
{
  STD = 0,
  EXT = 1
};

static uint8_t SizeToDlc(uint8_t size)
{
  if (size < 8) {
    return size;
  } else if (size < 13) {
    return 9;
  } else if (size < 17) {
    return 10;
  } else if (size < 21) {
    return 11;
  } else if (size < 25) {
    return 12;
  } else if (size < 33) {
    return 13;
  } else if (size < 49) {
    return 14;
  } else {
    return 15;
  }
}

static uint8_t DlcToSize(uint8_t dlc)
{
  if (dlc < 8) {
    return dlc;
  } else if (dlc == 9) {
    return 12;
  } else if (dlc == 10) {
    return 18;
  } else if (dlc == 11) {
    return 20;
  } else if (dlc == 12) {
    return 24;
  } else if (dlc == 13) {
    return 32;
  } else if (dlc == 14) {
    return 48;
  } else if (dlc == 15) {
    return 64;
  } else {
    return 0;
  }
}

class DbcMessage
{
public:
  DbcMessage();
  DbcMessage(
    uint8_t dlc,
    uint32_t id,
    IdType idType,
    std::string name,
    uint32_t rawId
  );

  uint8_t GetDlc();
  uint32_t GetId();
  IdType GetIdType();
  std::string GetName();
  CanFrame GetFrame();
  uint32_t GetSignalCount();
  void SetFrame(CanFrame & msg);
  void AddSignal(std::string signalName, CanDbcParser::DbcSignal signal);
  CanDbcParser::DbcSignal * GetSignal(std::string signalName);
  void SetRawText(std::string rawText);
  uint32_t GetRawId();
  void SetComment(CanDbcParser::DbcMessageComment comment);
  std::map<std::string, CanDbcParser::DbcSignal> * GetSignals();
  bool AnyMultiplexedSignals();

private:
  std::map<std::string, CanDbcParser::DbcSignal> _signals;
  uint8_t _data[64];
  uint8_t _dlc;
  uint32_t _id;
  IdType _idType;
  std::string _name;
  uint32_t _rawId;
  CanDbcParser::DbcMessageComment _comment;
};
}  // namespace CanDbcParser

#endif  // DBCMESSAGE_HPP_
