// <auto-generated>
//  automatically generated by the FlatBuffers compiler, do not modify
// </auto-generated>

namespace Veneris.Communications
{

using global::System;
using global::FlatBuffers;

public struct UpdateTransformInGroup : IFlatbufferObject
{
  private Table __p;
  public ByteBuffer ByteBuffer { get { return __p.bb; } }
  public static UpdateTransformInGroup GetRootAsUpdateTransformInGroup(ByteBuffer _bb) { return GetRootAsUpdateTransformInGroup(_bb, new UpdateTransformInGroup()); }
  public static UpdateTransformInGroup GetRootAsUpdateTransformInGroup(ByteBuffer _bb, UpdateTransformInGroup obj) { return (obj.__assign(_bb.GetInt(_bb.Position) + _bb.Position, _bb)); }
  public void __init(int _i, ByteBuffer _bb) { __p.bb_pos = _i; __p.bb = _bb; }
  public UpdateTransformInGroup __assign(int _i, ByteBuffer _bb) { __init(_i, _bb); return this; }

  public int Id { get { int o = __p.__offset(4); return o != 0 ? __p.bb.GetInt(o + __p.bb_pos) : (int)0; } }
  public Matrix4x4? Transform { get { int o = __p.__offset(6); return o != 0 ? (Matrix4x4?)(new Matrix4x4()).__assign(o + __p.bb_pos, __p.bb) : null; } }

  public static void StartUpdateTransformInGroup(FlatBufferBuilder builder) { builder.StartObject(2); }
  public static void AddId(FlatBufferBuilder builder, int id) { builder.AddInt(0, id, 0); }
  public static void AddTransform(FlatBufferBuilder builder, Offset<Matrix4x4> transformOffset) { builder.AddStruct(1, transformOffset.Value, 0); }
  public static Offset<UpdateTransformInGroup> EndUpdateTransformInGroup(FlatBufferBuilder builder) {
    int o = builder.EndObject();
    return new Offset<UpdateTransformInGroup>(o);
  }
  public static void FinishUpdateTransformInGroupBuffer(FlatBufferBuilder builder, Offset<UpdateTransformInGroup> offset) { builder.Finish(offset.Value); }
  public static void FinishSizePrefixedUpdateTransformInGroupBuffer(FlatBufferBuilder builder, Offset<UpdateTransformInGroup> offset) { builder.FinishSizePrefixed(offset.Value); }
};


}
