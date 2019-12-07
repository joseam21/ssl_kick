// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: grSim_Replacement.proto

#ifndef PROTOBUF_INCLUDED_grSim_5fReplacement_2eproto
#define PROTOBUF_INCLUDED_grSim_5fReplacement_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3007000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3007001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_grSim_5fReplacement_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_grSim_5fReplacement_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[3]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_grSim_5fReplacement_2eproto();
class grSim_BallReplacement;
class grSim_BallReplacementDefaultTypeInternal;
extern grSim_BallReplacementDefaultTypeInternal _grSim_BallReplacement_default_instance_;
class grSim_Replacement;
class grSim_ReplacementDefaultTypeInternal;
extern grSim_ReplacementDefaultTypeInternal _grSim_Replacement_default_instance_;
class grSim_RobotReplacement;
class grSim_RobotReplacementDefaultTypeInternal;
extern grSim_RobotReplacementDefaultTypeInternal _grSim_RobotReplacement_default_instance_;
namespace google {
namespace protobuf {
template<> ::grSim_BallReplacement* Arena::CreateMaybeMessage<::grSim_BallReplacement>(Arena*);
template<> ::grSim_Replacement* Arena::CreateMaybeMessage<::grSim_Replacement>(Arena*);
template<> ::grSim_RobotReplacement* Arena::CreateMaybeMessage<::grSim_RobotReplacement>(Arena*);
}  // namespace protobuf
}  // namespace google

// ===================================================================

class grSim_RobotReplacement :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:grSim_RobotReplacement) */ {
 public:
  grSim_RobotReplacement();
  virtual ~grSim_RobotReplacement();

  grSim_RobotReplacement(const grSim_RobotReplacement& from);

  inline grSim_RobotReplacement& operator=(const grSim_RobotReplacement& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  grSim_RobotReplacement(grSim_RobotReplacement&& from) noexcept
    : grSim_RobotReplacement() {
    *this = ::std::move(from);
  }

  inline grSim_RobotReplacement& operator=(grSim_RobotReplacement&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const grSim_RobotReplacement& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const grSim_RobotReplacement* internal_default_instance() {
    return reinterpret_cast<const grSim_RobotReplacement*>(
               &_grSim_RobotReplacement_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(grSim_RobotReplacement* other);
  friend void swap(grSim_RobotReplacement& a, grSim_RobotReplacement& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline grSim_RobotReplacement* New() const final {
    return CreateMaybeMessage<grSim_RobotReplacement>(nullptr);
  }

  grSim_RobotReplacement* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<grSim_RobotReplacement>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const grSim_RobotReplacement& from);
  void MergeFrom(const grSim_RobotReplacement& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(grSim_RobotReplacement* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double x = 1;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // required double y = 2;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // required double dir = 3;
  bool has_dir() const;
  void clear_dir();
  static const int kDirFieldNumber = 3;
  double dir() const;
  void set_dir(double value);

  // required uint32 id = 4;
  bool has_id() const;
  void clear_id();
  static const int kIdFieldNumber = 4;
  ::google::protobuf::uint32 id() const;
  void set_id(::google::protobuf::uint32 value);

  // required bool yellowteam = 5;
  bool has_yellowteam() const;
  void clear_yellowteam();
  static const int kYellowteamFieldNumber = 5;
  bool yellowteam() const;
  void set_yellowteam(bool value);

  // @@protoc_insertion_point(class_scope:grSim_RobotReplacement)
 private:
  class HasBitSetters;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  double x_;
  double y_;
  double dir_;
  ::google::protobuf::uint32 id_;
  bool yellowteam_;
  friend struct ::TableStruct_grSim_5fReplacement_2eproto;
};
// -------------------------------------------------------------------

class grSim_BallReplacement :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:grSim_BallReplacement) */ {
 public:
  grSim_BallReplacement();
  virtual ~grSim_BallReplacement();

  grSim_BallReplacement(const grSim_BallReplacement& from);

  inline grSim_BallReplacement& operator=(const grSim_BallReplacement& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  grSim_BallReplacement(grSim_BallReplacement&& from) noexcept
    : grSim_BallReplacement() {
    *this = ::std::move(from);
  }

  inline grSim_BallReplacement& operator=(grSim_BallReplacement&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const grSim_BallReplacement& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const grSim_BallReplacement* internal_default_instance() {
    return reinterpret_cast<const grSim_BallReplacement*>(
               &_grSim_BallReplacement_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(grSim_BallReplacement* other);
  friend void swap(grSim_BallReplacement& a, grSim_BallReplacement& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline grSim_BallReplacement* New() const final {
    return CreateMaybeMessage<grSim_BallReplacement>(nullptr);
  }

  grSim_BallReplacement* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<grSim_BallReplacement>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const grSim_BallReplacement& from);
  void MergeFrom(const grSim_BallReplacement& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(grSim_BallReplacement* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double x = 1;
  bool has_x() const;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // required double y = 2;
  bool has_y() const;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // required double vx = 3;
  bool has_vx() const;
  void clear_vx();
  static const int kVxFieldNumber = 3;
  double vx() const;
  void set_vx(double value);

  // required double vy = 4;
  bool has_vy() const;
  void clear_vy();
  static const int kVyFieldNumber = 4;
  double vy() const;
  void set_vy(double value);

  // @@protoc_insertion_point(class_scope:grSim_BallReplacement)
 private:
  class HasBitSetters;

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  friend struct ::TableStruct_grSim_5fReplacement_2eproto;
};
// -------------------------------------------------------------------

class grSim_Replacement :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:grSim_Replacement) */ {
 public:
  grSim_Replacement();
  virtual ~grSim_Replacement();

  grSim_Replacement(const grSim_Replacement& from);

  inline grSim_Replacement& operator=(const grSim_Replacement& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  grSim_Replacement(grSim_Replacement&& from) noexcept
    : grSim_Replacement() {
    *this = ::std::move(from);
  }

  inline grSim_Replacement& operator=(grSim_Replacement&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const grSim_Replacement& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const grSim_Replacement* internal_default_instance() {
    return reinterpret_cast<const grSim_Replacement*>(
               &_grSim_Replacement_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(grSim_Replacement* other);
  friend void swap(grSim_Replacement& a, grSim_Replacement& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline grSim_Replacement* New() const final {
    return CreateMaybeMessage<grSim_Replacement>(nullptr);
  }

  grSim_Replacement* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<grSim_Replacement>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const grSim_Replacement& from);
  void MergeFrom(const grSim_Replacement& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(grSim_Replacement* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .grSim_RobotReplacement robots = 2;
  int robots_size() const;
  void clear_robots();
  static const int kRobotsFieldNumber = 2;
  ::grSim_RobotReplacement* mutable_robots(int index);
  ::google::protobuf::RepeatedPtrField< ::grSim_RobotReplacement >*
      mutable_robots();
  const ::grSim_RobotReplacement& robots(int index) const;
  ::grSim_RobotReplacement* add_robots();
  const ::google::protobuf::RepeatedPtrField< ::grSim_RobotReplacement >&
      robots() const;

  // optional .grSim_BallReplacement ball = 1;
  bool has_ball() const;
  void clear_ball();
  static const int kBallFieldNumber = 1;
  const ::grSim_BallReplacement& ball() const;
  ::grSim_BallReplacement* release_ball();
  ::grSim_BallReplacement* mutable_ball();
  void set_allocated_ball(::grSim_BallReplacement* ball);

  // @@protoc_insertion_point(class_scope:grSim_Replacement)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::grSim_RobotReplacement > robots_;
  ::grSim_BallReplacement* ball_;
  friend struct ::TableStruct_grSim_5fReplacement_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// grSim_RobotReplacement

// required double x = 1;
inline bool grSim_RobotReplacement::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void grSim_RobotReplacement::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double grSim_RobotReplacement::x() const {
  // @@protoc_insertion_point(field_get:grSim_RobotReplacement.x)
  return x_;
}
inline void grSim_RobotReplacement::set_x(double value) {
  _has_bits_[0] |= 0x00000001u;
  x_ = value;
  // @@protoc_insertion_point(field_set:grSim_RobotReplacement.x)
}

// required double y = 2;
inline bool grSim_RobotReplacement::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void grSim_RobotReplacement::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double grSim_RobotReplacement::y() const {
  // @@protoc_insertion_point(field_get:grSim_RobotReplacement.y)
  return y_;
}
inline void grSim_RobotReplacement::set_y(double value) {
  _has_bits_[0] |= 0x00000002u;
  y_ = value;
  // @@protoc_insertion_point(field_set:grSim_RobotReplacement.y)
}

// required double dir = 3;
inline bool grSim_RobotReplacement::has_dir() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void grSim_RobotReplacement::clear_dir() {
  dir_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double grSim_RobotReplacement::dir() const {
  // @@protoc_insertion_point(field_get:grSim_RobotReplacement.dir)
  return dir_;
}
inline void grSim_RobotReplacement::set_dir(double value) {
  _has_bits_[0] |= 0x00000004u;
  dir_ = value;
  // @@protoc_insertion_point(field_set:grSim_RobotReplacement.dir)
}

// required uint32 id = 4;
inline bool grSim_RobotReplacement::has_id() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void grSim_RobotReplacement::clear_id() {
  id_ = 0u;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::google::protobuf::uint32 grSim_RobotReplacement::id() const {
  // @@protoc_insertion_point(field_get:grSim_RobotReplacement.id)
  return id_;
}
inline void grSim_RobotReplacement::set_id(::google::protobuf::uint32 value) {
  _has_bits_[0] |= 0x00000008u;
  id_ = value;
  // @@protoc_insertion_point(field_set:grSim_RobotReplacement.id)
}

// required bool yellowteam = 5;
inline bool grSim_RobotReplacement::has_yellowteam() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void grSim_RobotReplacement::clear_yellowteam() {
  yellowteam_ = false;
  _has_bits_[0] &= ~0x00000010u;
}
inline bool grSim_RobotReplacement::yellowteam() const {
  // @@protoc_insertion_point(field_get:grSim_RobotReplacement.yellowteam)
  return yellowteam_;
}
inline void grSim_RobotReplacement::set_yellowteam(bool value) {
  _has_bits_[0] |= 0x00000010u;
  yellowteam_ = value;
  // @@protoc_insertion_point(field_set:grSim_RobotReplacement.yellowteam)
}

// -------------------------------------------------------------------

// grSim_BallReplacement

// required double x = 1;
inline bool grSim_BallReplacement::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void grSim_BallReplacement::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline double grSim_BallReplacement::x() const {
  // @@protoc_insertion_point(field_get:grSim_BallReplacement.x)
  return x_;
}
inline void grSim_BallReplacement::set_x(double value) {
  _has_bits_[0] |= 0x00000001u;
  x_ = value;
  // @@protoc_insertion_point(field_set:grSim_BallReplacement.x)
}

// required double y = 2;
inline bool grSim_BallReplacement::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void grSim_BallReplacement::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline double grSim_BallReplacement::y() const {
  // @@protoc_insertion_point(field_get:grSim_BallReplacement.y)
  return y_;
}
inline void grSim_BallReplacement::set_y(double value) {
  _has_bits_[0] |= 0x00000002u;
  y_ = value;
  // @@protoc_insertion_point(field_set:grSim_BallReplacement.y)
}

// required double vx = 3;
inline bool grSim_BallReplacement::has_vx() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void grSim_BallReplacement::clear_vx() {
  vx_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double grSim_BallReplacement::vx() const {
  // @@protoc_insertion_point(field_get:grSim_BallReplacement.vx)
  return vx_;
}
inline void grSim_BallReplacement::set_vx(double value) {
  _has_bits_[0] |= 0x00000004u;
  vx_ = value;
  // @@protoc_insertion_point(field_set:grSim_BallReplacement.vx)
}

// required double vy = 4;
inline bool grSim_BallReplacement::has_vy() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void grSim_BallReplacement::clear_vy() {
  vy_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline double grSim_BallReplacement::vy() const {
  // @@protoc_insertion_point(field_get:grSim_BallReplacement.vy)
  return vy_;
}
inline void grSim_BallReplacement::set_vy(double value) {
  _has_bits_[0] |= 0x00000008u;
  vy_ = value;
  // @@protoc_insertion_point(field_set:grSim_BallReplacement.vy)
}

// -------------------------------------------------------------------

// grSim_Replacement

// optional .grSim_BallReplacement ball = 1;
inline bool grSim_Replacement::has_ball() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void grSim_Replacement::clear_ball() {
  if (ball_ != nullptr) ball_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::grSim_BallReplacement& grSim_Replacement::ball() const {
  const ::grSim_BallReplacement* p = ball_;
  // @@protoc_insertion_point(field_get:grSim_Replacement.ball)
  return p != nullptr ? *p : *reinterpret_cast<const ::grSim_BallReplacement*>(
      &::_grSim_BallReplacement_default_instance_);
}
inline ::grSim_BallReplacement* grSim_Replacement::release_ball() {
  // @@protoc_insertion_point(field_release:grSim_Replacement.ball)
  _has_bits_[0] &= ~0x00000001u;
  ::grSim_BallReplacement* temp = ball_;
  ball_ = nullptr;
  return temp;
}
inline ::grSim_BallReplacement* grSim_Replacement::mutable_ball() {
  _has_bits_[0] |= 0x00000001u;
  if (ball_ == nullptr) {
    auto* p = CreateMaybeMessage<::grSim_BallReplacement>(GetArenaNoVirtual());
    ball_ = p;
  }
  // @@protoc_insertion_point(field_mutable:grSim_Replacement.ball)
  return ball_;
}
inline void grSim_Replacement::set_allocated_ball(::grSim_BallReplacement* ball) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete ball_;
  }
  if (ball) {
    ::google::protobuf::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ball = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, ball, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  ball_ = ball;
  // @@protoc_insertion_point(field_set_allocated:grSim_Replacement.ball)
}

// repeated .grSim_RobotReplacement robots = 2;
inline int grSim_Replacement::robots_size() const {
  return robots_.size();
}
inline void grSim_Replacement::clear_robots() {
  robots_.Clear();
}
inline ::grSim_RobotReplacement* grSim_Replacement::mutable_robots(int index) {
  // @@protoc_insertion_point(field_mutable:grSim_Replacement.robots)
  return robots_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::grSim_RobotReplacement >*
grSim_Replacement::mutable_robots() {
  // @@protoc_insertion_point(field_mutable_list:grSim_Replacement.robots)
  return &robots_;
}
inline const ::grSim_RobotReplacement& grSim_Replacement::robots(int index) const {
  // @@protoc_insertion_point(field_get:grSim_Replacement.robots)
  return robots_.Get(index);
}
inline ::grSim_RobotReplacement* grSim_Replacement::add_robots() {
  // @@protoc_insertion_point(field_add:grSim_Replacement.robots)
  return robots_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::grSim_RobotReplacement >&
grSim_Replacement::robots() const {
  // @@protoc_insertion_point(field_list:grSim_Replacement.robots)
  return robots_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_grSim_5fReplacement_2eproto
