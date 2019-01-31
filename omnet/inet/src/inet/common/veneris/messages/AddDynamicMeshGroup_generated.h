// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_ADDDYNAMICMESHGROUP_VENERIS_COMMUNICATIONS_H_
#define FLATBUFFERS_GENERATED_ADDDYNAMICMESHGROUP_VENERIS_COMMUNICATIONS_H_

#include "flatbuffers/flatbuffers.h"

#include "BaseTypes_generated.h"

namespace Veneris {
namespace Communications {

struct AddDynamicMeshGroup;

struct AddDynamicMeshGroup FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
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

struct AddDynamicMeshGroupBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_id(int32_t id) {
    fbb_.AddElement<int32_t>(AddDynamicMeshGroup::VT_ID, id, 0);
  }
  explicit AddDynamicMeshGroupBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  AddDynamicMeshGroupBuilder &operator=(const AddDynamicMeshGroupBuilder &);
  flatbuffers::Offset<AddDynamicMeshGroup> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<AddDynamicMeshGroup>(end);
    return o;
  }
};

inline flatbuffers::Offset<AddDynamicMeshGroup> CreateAddDynamicMeshGroup(
    flatbuffers::FlatBufferBuilder &_fbb,
    int32_t id = 0) {
  AddDynamicMeshGroupBuilder builder_(_fbb);
  builder_.add_id(id);
  return builder_.Finish();
}

inline const Veneris::Communications::AddDynamicMeshGroup *GetAddDynamicMeshGroup(const void *buf) {
  return flatbuffers::GetRoot<Veneris::Communications::AddDynamicMeshGroup>(buf);
}

inline const Veneris::Communications::AddDynamicMeshGroup *GetSizePrefixedAddDynamicMeshGroup(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<Veneris::Communications::AddDynamicMeshGroup>(buf);
}

inline bool VerifyAddDynamicMeshGroupBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<Veneris::Communications::AddDynamicMeshGroup>(nullptr);
}

inline bool VerifySizePrefixedAddDynamicMeshGroupBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<Veneris::Communications::AddDynamicMeshGroup>(nullptr);
}

inline void FinishAddDynamicMeshGroupBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Veneris::Communications::AddDynamicMeshGroup> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedAddDynamicMeshGroupBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<Veneris::Communications::AddDynamicMeshGroup> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace Communications
}  // namespace Veneris

#endif  // FLATBUFFERS_GENERATED_ADDDYNAMICMESHGROUP_VENERIS_COMMUNICATIONS_H_