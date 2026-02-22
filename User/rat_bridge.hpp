#pragma once

#include <cstdint>
#include <cstring>

#include "rat.h"
#include "rat_gen.h"
#include "rat_types.h"

namespace RatBridge
{

namespace detail
{
constexpr uint8_t kControlPacketId = 0x00u;
constexpr uint8_t kSchemaHelloOpcode = 0x01u;
constexpr uint8_t kSchemaChunkOpcode = 0x02u;
constexpr uint8_t kSchemaCommitOpcode = 0x03u;
constexpr uint8_t kSchemaVersion = 1u;
constexpr uint16_t kSchemaChunkMax = 96u;

inline void WriteU16Le(uint8_t* dst, uint16_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8u) & 0xFFu);
}

inline void WriteU32Le(uint8_t* dst, uint32_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8u) & 0xFFu);
  dst[2] = static_cast<uint8_t>((value >> 16u) & 0xFFu);
  dst[3] = static_cast<uint8_t>((value >> 24u) & 0xFFu);
}

inline void WriteU64Le(uint8_t* dst, uint64_t value)
{
  for (uint32_t index = 0u; index < 8u; ++index)
  {
    dst[index] = static_cast<uint8_t>((value >> (8u * index)) & 0xFFu);
  }
}

inline void ResetMainUpRingBuffer()
{
  RatRttRingBuffer& ring = _SEGGER_RTT.up[RAT_CTX_MAIN];
  if (ring.pBuffer == nullptr || ring.size == 0u)
  {
    return;
  }

  std::memset(ring.pBuffer, 0, ring.size);
  ring.wr = 0u;
  ring.rd = 0u;
}

inline int EmitControl(const void* payload, uint32_t payload_len)
{
  return rat_emit(kControlPacketId, payload, payload_len, false);
}

}  // namespace detail

inline void SyncSchema()
{
  constexpr uint32_t schema_len = static_cast<uint32_t>(RAT_GEN_SCHEMA_LEN);
  if (schema_len == 0u)
  {
    return;
  }

  const auto* schema_bytes = static_cast<const uint8_t*>(RAT_GEN_SCHEMA_BYTES);
  const uint64_t schema_hash = static_cast<uint64_t>(RAT_GEN_SCHEMA_HASH);

  uint8_t hello[18] = {0u};
  hello[0] = detail::kSchemaHelloOpcode;
  hello[1] = 'R';
  hello[2] = 'A';
  hello[3] = 'T';
  hello[4] = 'S';
  hello[5] = detail::kSchemaVersion;
  detail::WriteU32Le(&hello[6], schema_len);
  detail::WriteU64Le(&hello[10], schema_hash);
  (void)detail::EmitControl(hello, static_cast<uint32_t>(sizeof(hello)));

  uint32_t offset = 0u;
  while (offset < schema_len)
  {
    const uint32_t remaining = schema_len - offset;
    const uint16_t chunk_len = static_cast<uint16_t>(
        (remaining > detail::kSchemaChunkMax) ? detail::kSchemaChunkMax : remaining);

    uint8_t chunk[7u + detail::kSchemaChunkMax] = {0u};
    chunk[0] = detail::kSchemaChunkOpcode;
    detail::WriteU32Le(&chunk[1], offset);
    detail::WriteU16Le(&chunk[5], chunk_len);
    std::memcpy(&chunk[7], &schema_bytes[offset], chunk_len);
    (void)detail::EmitControl(chunk, static_cast<uint32_t>(7u + chunk_len));

    offset += static_cast<uint32_t>(chunk_len);
  }

  uint8_t commit[9] = {0u};
  commit[0] = detail::kSchemaCommitOpcode;
  detail::WriteU64Le(&commit[1], schema_hash);
  (void)detail::EmitControl(commit, static_cast<uint32_t>(sizeof(commit)));
}

inline void Init()
{
  rat_init();

  // librat 默认会发 legacy RATI 控制帧；这里清空后改为发 runtime schema 控制帧。
  detail::ResetMainUpRingBuffer();
  SyncSchema();
}

}  // namespace RatBridge
