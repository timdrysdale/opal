// <auto-generated>
//  automatically generated by the FlatBuffers compiler, do not modify
// </auto-generated>

namespace Veneris.Communications
{

using global::System;
using global::FlatBuffers;

public struct CreateVehicle : IFlatbufferObject
{
  private Table __p;
  public ByteBuffer ByteBuffer { get { return __p.bb; } }
  public static CreateVehicle GetRootAsCreateVehicle(ByteBuffer _bb) { return GetRootAsCreateVehicle(_bb, new CreateVehicle()); }
  public static CreateVehicle GetRootAsCreateVehicle(ByteBuffer _bb, CreateVehicle obj) { return (obj.__assign(_bb.GetInt(_bb.Position) + _bb.Position, _bb)); }
  public void __init(int _i, ByteBuffer _bb) { __p.bb_pos = _i; __p.bb = _bb; }
  public CreateVehicle __assign(int _i, ByteBuffer _bb) { __init(_i, _bb); return this; }

  public uint Id { get { int o = __p.__offset(4); return o != 0 ? __p.bb.GetUint(o + __p.bb_pos) : (uint)0; } }
  public Vec3? Position { get { int o = __p.__offset(6); return o != 0 ? (Vec3?)(new Vec3()).__assign(o + __p.bb_pos, __p.bb) : null; } }
  public float Radius { get { int o = __p.__offset(8); return o != 0 ? __p.bb.GetFloat(o + __p.bb_pos) : (float)0.0f; } }

  public static void StartCreateVehicle(FlatBufferBuilder builder) { builder.StartObject(3); }
  public static void AddId(FlatBufferBuilder builder, uint id) { builder.AddUint(0, id, 0); }
  public static void AddPosition(FlatBufferBuilder builder, Offset<Vec3> positionOffset) { builder.AddStruct(1, positionOffset.Value, 0); }
  public static void AddRadius(FlatBufferBuilder builder, float radius) { builder.AddFloat(2, radius, 0.0f); }
  public static Offset<CreateVehicle> EndCreateVehicle(FlatBufferBuilder builder) {
    int o = builder.EndObject();
    return new Offset<CreateVehicle>(o);
  }
  public static void FinishCreateVehicleBuffer(FlatBufferBuilder builder, Offset<CreateVehicle> offset) { builder.Finish(offset.Value); }
  public static void FinishSizePrefixedCreateVehicleBuffer(FlatBufferBuilder builder, Offset<CreateVehicle> offset) { builder.FinishSizePrefixed(offset.Value); }
};


}