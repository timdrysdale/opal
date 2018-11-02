// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_BASETYPES_VENERIS_COMMUNICATIONS_H_
#define FLATBUFFERS_GENERATED_BASETYPES_VENERIS_COMMUNICATIONS_H_

#include "flatbuffers/flatbuffers.h"

namespace Veneris {
namespace Communications {

struct Vec3;

struct Vec4;

struct Matrix4x4;

struct MaterialEMP;

enum VenerisMessageTypes {
  VenerisMessageTypes_Reserved = 0,
  VenerisMessageTypes_Create = 1,
  VenerisMessageTypes_Destroy = 2,
  VenerisMessageTypes_VehicleState = 3,
  VenerisMessageTypes_ExternalTime = 4,
  VenerisMessageTypes_End = 5,
  VenerisMessageTypes_UseOpal = 6,
  VenerisMessageTypes_FinishOpalContext = 7,
  VenerisMessageTypes_StaticMesh = 8,
  VenerisMessageTypes_AddDynamicMeshGroup = 9,
  VenerisMessageTypes_RemoveDynamicMeshGroup = 10,
  VenerisMessageTypes_UpdateTransformInGroup = 11,
  VenerisMessageTypes_FinishDynamicMeshGroup = 12,
  VenerisMessageTypes_DynamicMesh = 13,
  VenerisMessageTypes_MIN = VenerisMessageTypes_Reserved,
  VenerisMessageTypes_MAX = VenerisMessageTypes_DynamicMesh
};

inline const VenerisMessageTypes (&EnumValuesVenerisMessageTypes())[14] {
  static const VenerisMessageTypes values[] = {
    VenerisMessageTypes_Reserved,
    VenerisMessageTypes_Create,
    VenerisMessageTypes_Destroy,
    VenerisMessageTypes_VehicleState,
    VenerisMessageTypes_ExternalTime,
    VenerisMessageTypes_End,
    VenerisMessageTypes_UseOpal,
    VenerisMessageTypes_FinishOpalContext,
    VenerisMessageTypes_StaticMesh,
    VenerisMessageTypes_AddDynamicMeshGroup,
    VenerisMessageTypes_RemoveDynamicMeshGroup,
    VenerisMessageTypes_UpdateTransformInGroup,
    VenerisMessageTypes_FinishDynamicMeshGroup,
    VenerisMessageTypes_DynamicMesh
  };
  return values;
}

inline const char * const *EnumNamesVenerisMessageTypes() {
  static const char * const names[] = {
    "Reserved",
    "Create",
    "Destroy",
    "VehicleState",
    "ExternalTime",
    "End",
    "UseOpal",
    "FinishOpalContext",
    "StaticMesh",
    "AddDynamicMeshGroup",
    "RemoveDynamicMeshGroup",
    "UpdateTransformInGroup",
    "FinishDynamicMeshGroup",
    "DynamicMesh",
    nullptr
  };
  return names;
}

inline const char *EnumNameVenerisMessageTypes(VenerisMessageTypes e) {
  const size_t index = static_cast<int>(e);
  return EnumNamesVenerisMessageTypes()[index];
}

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) Vec3 FLATBUFFERS_FINAL_CLASS {
 private:
  float x_;
  float y_;
  float z_;

 public:
  Vec3() {
    memset(this, 0, sizeof(Vec3));
  }
  Vec3(float _x, float _y, float _z)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)) {
  }
  float x() const {
    return flatbuffers::EndianScalar(x_);
  }
  float y() const {
    return flatbuffers::EndianScalar(y_);
  }
  float z() const {
    return flatbuffers::EndianScalar(z_);
  }
};
FLATBUFFERS_STRUCT_END(Vec3, 12);

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) Vec4 FLATBUFFERS_FINAL_CLASS {
 private:
  float x_;
  float y_;
  float z_;
  float w_;

 public:
  Vec4() {
    memset(this, 0, sizeof(Vec4));
  }
  Vec4(float _x, float _y, float _z, float _w)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)),
        w_(flatbuffers::EndianScalar(_w)) {
  }
  float x() const {
    return flatbuffers::EndianScalar(x_);
  }
  float y() const {
    return flatbuffers::EndianScalar(y_);
  }
  float z() const {
    return flatbuffers::EndianScalar(z_);
  }
  float w() const {
    return flatbuffers::EndianScalar(w_);
  }
};
FLATBUFFERS_STRUCT_END(Vec4, 16);

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) Matrix4x4 FLATBUFFERS_FINAL_CLASS {
 private:
  Vec4 r0_;
  Vec4 r1_;
  Vec4 r2_;
  Vec4 r3_;

 public:
  Matrix4x4() {
    memset(this, 0, sizeof(Matrix4x4));
  }
  Matrix4x4(const Vec4 &_r0, const Vec4 &_r1, const Vec4 &_r2, const Vec4 &_r3)
      : r0_(_r0),
        r1_(_r1),
        r2_(_r2),
        r3_(_r3) {
  }
  const Vec4 &r0() const {
    return r0_;
  }
  const Vec4 &r1() const {
    return r1_;
  }
  const Vec4 &r2() const {
    return r2_;
  }
  const Vec4 &r3() const {
    return r3_;
  }
};
FLATBUFFERS_STRUCT_END(Matrix4x4, 64);

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) MaterialEMP FLATBUFFERS_FINAL_CLASS {
 private:
  float a_;
  float b_;
  float c_;
  float d_;

 public:
  MaterialEMP() {
    memset(this, 0, sizeof(MaterialEMP));
  }
  MaterialEMP(float _a, float _b, float _c, float _d)
      : a_(flatbuffers::EndianScalar(_a)),
        b_(flatbuffers::EndianScalar(_b)),
        c_(flatbuffers::EndianScalar(_c)),
        d_(flatbuffers::EndianScalar(_d)) {
  }
  float a() const {
    return flatbuffers::EndianScalar(a_);
  }
  float b() const {
    return flatbuffers::EndianScalar(b_);
  }
  float c() const {
    return flatbuffers::EndianScalar(c_);
  }
  float d() const {
    return flatbuffers::EndianScalar(d_);
  }
};
FLATBUFFERS_STRUCT_END(MaterialEMP, 16);

}  // namespace Communications
}  // namespace Veneris

#endif  // FLATBUFFERS_GENERATED_BASETYPES_VENERIS_COMMUNICATIONS_H_
