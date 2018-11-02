// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_REMOVEDYNAMICMESHGROUP_VENERIS_COMMUNICATIONS_H_
#define FLATBUFFERS_GENERATED_REMOVEDYNAMICMESHGROUP_VENERIS_COMMUNICATIONS_H_

#include "flatbuffers/flatbuffers.h"

#include "BaseTypes_generated.h"

namespace Veneris {
namespace Communications {

struct RemoveDynamicMeshGroup;

struct RemoveDynamicMeshGroup FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
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

struct RemoveDynamicMeshGroupBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_id(int32_t id) {
    fbb_.AddElement<int32_t>(RemoveDynamicMeshGroup::VT_ID, id, 0);
  }
  explicit RemoveDynamicMeshGroupBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  RemoveDynamicMeshGroupBuilder &operator=(const RemoveDynamicMeshGroupBuilder &);
  flatbuffers::Offset<RemoveDynamicMeshGroup> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<RemoveDynamicMeshGroup>(end);
    return o;
  }
};

inline flatbuffers::Offset<RemoveDynamicMeshGroup> CreateRemoveDynamicMeshGroup(
    flatbuffers::FlatBufferBuilder &_fbb,
    int32_t id = 0) {
  RemoveDynamicMeshGroupBuilder builder_(_fbb);
  builder_.add_id(id);
  return builder_.Finish();
}

inline const Veneris::Communications::RemoveDynamicMeshGroup *GetRemoveDynamicMeshGroup(const void *buf) {
  return flatbuffers::GetRoot<Veneris::Communications::RemoveDynamicMeshGroup>(buf);
}

inline const Veneris::Communications::RemoveDynamicMeshGroup *GetSizePrefixedRemoveDynamicMeshGroup(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<Veneris::Communications::RemoveDynamicMeshGroup>(buf);
}

inline bool VerifyRemoveDynamicMeshGroupBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<Veneris::Communications::RemoveDynamicMeshGroup>(nullptr);
}

inline bool VerifySizePrefixedRemoveDynamicMeshGroupBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<Veneris::Communications::RemoveDynamicMeshGroup>(nullptr);
}

inline void FinishRemoveDynamicMeshGroupBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Veneris::Communications::RemoveDynamicMeshGroup> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedRemoveDynamicMeshGroupBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Veneris::Communications::RemoveDynamicMeshGroup> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace Communications
}  // namespace Veneris

#endif  // FLATBUFFERS_GENERATED_REMOVEDYNAMICMESHGROUP_VENERIS_COMMUNICATIONS_H_
