// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_DESTROYVEHICLE_VENERIS_COMMUNICATIONS_H_
#define FLATBUFFERS_GENERATED_DESTROYVEHICLE_VENERIS_COMMUNICATIONS_H_

#include "flatbuffers/flatbuffers.h"

namespace Veneris {
namespace Communications {

struct DestroyVehicle;

struct DestroyVehicle FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_ID = 4
  };
  int32_t id() const {
    return GetField<int32_t>(VT_ID, 0);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<int32_t>(verifier, VT_ID) &&
           verifier.EndTable();
  }
};

struct DestroyVehicleBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_id(int32_t id) {
    fbb_.AddElement<int32_t>(DestroyVehicle::VT_ID, id, 0);
  }
  explicit DestroyVehicleBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  DestroyVehicleBuilder &operator=(const DestroyVehicleBuilder &);
  flatbuffers::Offset<DestroyVehicle> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<DestroyVehicle>(end);
    return o;
  }
};

inline flatbuffers::Offset<DestroyVehicle> CreateDestroyVehicle(
    flatbuffers::FlatBufferBuilder &_fbb,
    int32_t id = 0) {
  DestroyVehicleBuilder builder_(_fbb);
  builder_.add_id(id);
  return builder_.Finish();
}

inline const Veneris::Communications::DestroyVehicle *GetDestroyVehicle(const void *buf) {
  return flatbuffers::GetRoot<Veneris::Communications::DestroyVehicle>(buf);
}

inline const Veneris::Communications::DestroyVehicle *GetSizePrefixedDestroyVehicle(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<Veneris::Communications::DestroyVehicle>(buf);
}

inline bool VerifyDestroyVehicleBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<Veneris::Communications::DestroyVehicle>(nullptr);
}

inline bool VerifySizePrefixedDestroyVehicleBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<Veneris::Communications::DestroyVehicle>(nullptr);
}

inline void FinishDestroyVehicleBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Veneris::Communications::DestroyVehicle> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedDestroyVehicleBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Veneris::Communications::DestroyVehicle> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace Communications
}  // namespace Veneris

#endif  // FLATBUFFERS_GENERATED_DESTROYVEHICLE_VENERIS_COMMUNICATIONS_H_
