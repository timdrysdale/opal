// <auto-generated>
//  automatically generated by the FlatBuffers compiler, do not modify
// </auto-generated>

namespace Veneris.Communications
{

using global::System;
using global::FlatBuffers;

public struct MaterialEMP : IFlatbufferObject
{
  private Struct __p;
  public ByteBuffer ByteBuffer { get { return __p.bb; } }
  public void __init(int _i, ByteBuffer _bb) { __p.bb_pos = _i; __p.bb = _bb; }
  public MaterialEMP __assign(int _i, ByteBuffer _bb) { __init(_i, _bb); return this; }

  public float A { get { return __p.bb.GetFloat(__p.bb_pos + 0); } }
  public float B { get { return __p.bb.GetFloat(__p.bb_pos + 4); } }
  public float C { get { return __p.bb.GetFloat(__p.bb_pos + 8); } }
  public float D { get { return __p.bb.GetFloat(__p.bb_pos + 12); } }

  public static Offset<MaterialEMP> CreateMaterialEMP(FlatBufferBuilder builder, float A, float B, float C, float D) {
    builder.Prep(4, 16);
    builder.PutFloat(D);
    builder.PutFloat(C);
    builder.PutFloat(B);
    builder.PutFloat(A);
    return new Offset<MaterialEMP>(builder.Offset);
  }
};


}