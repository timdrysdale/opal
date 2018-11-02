// <auto-generated>
//  automatically generated by the FlatBuffers compiler, do not modify
// </auto-generated>

namespace Veneris.Communications
{

using global::System;
using global::FlatBuffers;

public struct ExternalTime : IFlatbufferObject
{
  private Table __p;
  public ByteBuffer ByteBuffer { get { return __p.bb; } }
  public static ExternalTime GetRootAsExternalTime(ByteBuffer _bb) { return GetRootAsExternalTime(_bb, new ExternalTime()); }
  public static ExternalTime GetRootAsExternalTime(ByteBuffer _bb, ExternalTime obj) { return (obj.__assign(_bb.GetInt(_bb.Position) + _bb.Position, _bb)); }
  public void __init(int _i, ByteBuffer _bb) { __p.bb_pos = _i; __p.bb = _bb; }
  public ExternalTime __assign(int _i, ByteBuffer _bb) { __init(_i, _bb); return this; }

  public float Time { get { int o = __p.__offset(4); return o != 0 ? __p.bb.GetFloat(o + __p.bb_pos) : (float)0.0f; } }

  public static Offset<ExternalTime> CreateExternalTime(FlatBufferBuilder builder,
      float time = 0.0f) {
    builder.StartObject(1);
    ExternalTime.AddTime(builder, time);
    return ExternalTime.EndExternalTime(builder);
  }

  public static void StartExternalTime(FlatBufferBuilder builder) { builder.StartObject(1); }
  public static void AddTime(FlatBufferBuilder builder, float time) { builder.AddFloat(0, time, 0.0f); }
  public static Offset<ExternalTime> EndExternalTime(FlatBufferBuilder builder) {
    int o = builder.EndObject();
    return new Offset<ExternalTime>(o);
  }
  public static void FinishExternalTimeBuffer(FlatBufferBuilder builder, Offset<ExternalTime> offset) { builder.Finish(offset.Value); }
  public static void FinishSizePrefixedExternalTimeBuffer(FlatBufferBuilder builder, Offset<ExternalTime> offset) { builder.FinishSizePrefixed(offset.Value); }
};


}
